#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_transport.h"
#include "esp_transport_tcp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_heap_caps.h"

#include <stdatomic.h>

#include "global_state.h"
#include "protocol_coordinator.h"
#include "stratum_v1_task.h"
#include "stratum_api.h"
#include "stratum_v2_task.h"
#include "connect.h"
#include "system.h"

#include <string.h>

// Internal coordinator states
typedef enum {
    COORD_STATE_IDLE = 0,
    COORD_STATE_RUNNING_PRIMARY,
    COORD_STATE_RUNNING_FALLBACK,
    // Entered when consecutive pool failures hit the configured threshold.
    // No protocol task is running; the coordinator probes pools periodically
    // and resumes mining as soon as one is reachable. While in this state,
    // pools_unavailable is true so power management cuts ASIC power.
    COORD_STATE_PAUSED,
} coordinator_state_t;

// Internal event types
typedef enum {
    COORD_EVENT_PROTOCOL_FAILED = 0,
    COORD_EVENT_PROTOCOL_SUCCESS,
    COORD_EVENT_V1_TASK_EXITED,
    COORD_EVENT_V2_TASK_EXITED,
    COORD_EVENT_POOL_CONFIG_CHANGED,
} coordinator_event_t;

#define TRANSPORT_TIMEOUT_MS 5000
#define HEARTBEAT_INTERVAL_MS 60000
#define INITIAL_HEARTBEAT_DELAY_MS 10000
// While paused (all pools unreachable), probe again on this cadence.
#define RECOVERY_PROBE_INTERVAL_MS 30000
#define BUFFER_SIZE 1024

static const char *TAG = "protocol_coordinator";

static GlobalState *s_global_state = NULL;
static coordinator_state_t s_state = COORD_STATE_IDLE;
static QueueHandle_t s_event_queue = NULL;
static volatile bool s_v1_should_shutdown = false;
static volatile bool s_v2_should_shutdown = false;
static atomic_bool s_pool_config_changed = ATOMIC_VAR_INIT(false);

// Protocol tracking
static stratum_protocol_t s_primary_protocol;
static stratum_protocol_t s_fallback_protocol;
static stratum_protocol_t s_running_protocol;
static bool s_heartbeat_enabled = false;

// Number of consecutive pools (primary and/or fallback) that have exhausted
// their retry budget without a successful setup. When this reaches
// pool_failure_threshold(), we enter COORD_STATE_PAUSED and set
// pools_unavailable so power management cuts ASIC power.
// Reset on COORD_EVENT_PROTOCOL_SUCCESS.
static int s_consecutive_pool_failures = 0;

static void enter_paused_state(GlobalState *gs);

void protocol_coordinator_init(GlobalState *gs)
{
    s_global_state = gs;
    s_event_queue = xQueueCreate(8, sizeof(coordinator_event_t));
    s_v1_should_shutdown = false;
    s_v2_should_shutdown = false;
    s_heartbeat_enabled = false;
    s_consecutive_pool_failures = 0;
    atomic_store_explicit(&s_pool_config_changed, false,
                          memory_order_release);
}

void protocol_coordinator_notify_failure(void)
{
    coordinator_event_t evt = COORD_EVENT_PROTOCOL_FAILED;
    if (s_event_queue) {
        (void)xQueueSend(s_event_queue, &evt, portMAX_DELAY);
    }
}

void protocol_coordinator_notify_success(void)
{
    coordinator_event_t evt = COORD_EVENT_PROTOCOL_SUCCESS;
    if (s_event_queue) {
        xQueueSend(s_event_queue, &evt, 0);
    }
}

void protocol_coordinator_notify_pool_config_changed(void)
{
    bool already_pending = atomic_exchange_explicit(
        &s_pool_config_changed, true, memory_order_acq_rel);
    if (already_pending) {
        return;
    }

    // The flag is authoritative: even if the wake event cannot be queued, a
    // full queue already guarantees the coordinator will wake and observe it.
    coordinator_event_t evt = COORD_EVENT_POOL_CONFIG_CHANGED;
    if (s_event_queue) {
        (void)xQueueSend(s_event_queue, &evt, 0);
    }
}

bool protocol_coordinator_v1_should_shutdown(void)
{
    return s_v1_should_shutdown;
}

void protocol_coordinator_v1_exited(void)
{
    coordinator_event_t evt = COORD_EVENT_V1_TASK_EXITED;
    if (s_event_queue) {
        (void)xQueueSend(s_event_queue, &evt, portMAX_DELAY);
    }
}

bool protocol_coordinator_v2_should_shutdown(void)
{
    return s_v2_should_shutdown;
}

void protocol_coordinator_v2_exited(void)
{
    coordinator_event_t evt = COORD_EVENT_V2_TASK_EXITED;
    if (s_event_queue) {
        (void)xQueueSend(s_event_queue, &evt, portMAX_DELAY);
    }
}

static void reset_share_stats(GlobalState *gs)
{
    for (int i = 0; i < gs->SYSTEM_MODULE.rejected_reason_stats_count; i++) {
        gs->SYSTEM_MODULE.rejected_reason_stats[i].count = 0;
        gs->SYSTEM_MODULE.rejected_reason_stats[i].message[0] = '\0';
    }
    gs->SYSTEM_MODULE.rejected_reason_stats_count = 0;
    gs->SYSTEM_MODULE.shares_accepted = 0;
    gs->SYSTEM_MODULE.shares_rejected = 0;
    gs->SYSTEM_MODULE.work_received = 0;
}

static bool has_fallback_pool(GlobalState *gs)
{
    uint16_t sec_idx = gs->SYSTEM_MODULE.secondary_pool_index;
    PoolConfig pool = {0};
    if (!SYSTEM_get_pool_config_snapshot(gs, sec_idx, &pool)) {
        return false;
    }
    bool configured = pool.url != NULL && pool.url[0] != '\0';
    SYSTEM_release_pool_config_snapshot(&pool);
    return configured;
}

// Start the V1 stratum task (for primary V1 or fallback)
static bool start_v1_task(GlobalState *gs)
{
    s_v1_should_shutdown = false;
    if (xTaskCreateWithCaps(stratum_v1_task, "stratum v1", 8192, (void *)gs, 5, NULL, MALLOC_CAP_SPIRAM) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create V1 stratum task");
        return false;
    }
    return true;
}

// Start the V2 stratum task
static bool start_v2_task(GlobalState *gs)
{
    s_v2_should_shutdown = false;
    if (xTaskCreateWithCaps(stratum_v2_task, "stratum v2", 12288, (void *)gs, 5, NULL, MALLOC_CAP_SPIRAM) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create V2 stratum task");
        return false;
    }
    return true;
}

// Start a task for the given protocol
static bool start_protocol_task(GlobalState *gs, stratum_protocol_t protocol)
{
    if (protocol == STRATUM_PROTOCOL_V2) {
        return start_v2_task(gs);
    }
    return start_v1_task(gs);
}

// Tell the V1 task to shut down and wait for it to exit.
// Only closes the transport socket to unblock V1's recv — does NOT destroy it.
// The V1 task handles its own full cleanup (destroy, queue clear) on exit.
static bool stop_v1_task(GlobalState *gs)
{
    s_v1_should_shutdown = true;

    // Close transport to unblock V1's blocked recv(), serialized with share TX.
    stratum_v1_interrupt_connection(gs);

    coordinator_event_t evt;
    for (int i = 0; i < 100; i++) {
        if (xQueueReceive(s_event_queue, &evt, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (evt == COORD_EVENT_V1_TASK_EXITED || evt == COORD_EVENT_PROTOCOL_FAILED) {
                ESP_LOGI(TAG, "V1 task exited cleanly");
                return true;
            }
        }
    }
    ESP_LOGE(TAG,
             "V1 task did not exit within timeout; restarting to avoid overlapping protocol tasks");
    esp_restart();
    return false;
}

// Tell the V2 task to shut down and wait for it to exit.
// Only closes the transport socket to unblock V2's recv — does NOT destroy it.
// The V2 task handles its own full cleanup (destroy, noise ctx, queue clear) on exit.
static bool stop_v2_task(GlobalState *gs)
{
    s_v2_should_shutdown = true;

    // Close transport to unblock V2's blocked recv(), serialized with share TX.
    stratum_v2_interrupt_connection(gs);

    coordinator_event_t evt;
    for (int i = 0; i < 100; i++) {
        if (xQueueReceive(s_event_queue, &evt, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (evt == COORD_EVENT_V2_TASK_EXITED || evt == COORD_EVENT_PROTOCOL_FAILED) {
                ESP_LOGI(TAG, "V2 task exited cleanly");
                return true;
            }
        }
    }
    ESP_LOGE(TAG,
             "V2 task did not exit within timeout; restarting to avoid overlapping protocol tasks");
    esp_restart();
    return false;
}

// Stop the currently running protocol task
static bool stop_running_task(GlobalState *gs)
{
    if (s_running_protocol == STRATUM_PROTOCOL_V2) {
        return stop_v2_task(gs);
    }
    return stop_v1_task(gs);
}

// TCP connect probe (used for SV2 — full noise handshake is too expensive)
static bool probe_pool_sv2(const char *url, uint16_t port)
{
    if (url == NULL || url[0] == '\0' || port == 0) return false;

    esp_transport_handle_t probe = esp_transport_tcp_init();
    if (!probe) return false;

    esp_err_t err = esp_transport_connect(probe, url, port, TRANSPORT_TIMEOUT_MS);
    esp_transport_close(probe);
    esp_transport_destroy(probe);

    return (err == ESP_OK);
}

// Subscribe/authorize probe for V1 — succeeds only if the pool responds with
// a mining.notify line, confirming it's actually serving work.
static bool probe_pool_v1(GlobalState *gs, const char *url, uint16_t port,
                          tls_mode tls, char *cert, const char *user, const char *pass)
{
    if (url == NULL || url[0] == '\0' || port == 0) return false;

    esp_transport_handle_t transport = STRATUM_V1_transport_init(tls, cert);
    if (!transport) return false;

    esp_err_t err = esp_transport_connect(transport, url, port, TRANSPORT_TIMEOUT_MS);
    if (err != ESP_OK) {
        esp_transport_close(transport);
        esp_transport_destroy(transport);
        return false;
    }

    int send_uid = 1;
    STRATUM_V1_subscribe(transport, send_uid++, gs->DEVICE_CONFIG.family.asic.name);
    STRATUM_V1_authorize(transport, send_uid++, user, pass);

    char recv_buffer[BUFFER_SIZE];
    memset(recv_buffer, 0, BUFFER_SIZE);
    int bytes_received = esp_transport_read(transport, recv_buffer, BUFFER_SIZE - 1, TRANSPORT_TIMEOUT_MS);

    esp_transport_close(transport);
    esp_transport_destroy(transport);

    return (bytes_received > 0 && strstr(recv_buffer, "mining.notify") != NULL);
}

// Probe a pool using the appropriate protocol for it.
static bool probe_pool(GlobalState *gs, bool use_fallback)
{
    uint16_t idx = use_fallback ? gs->SYSTEM_MODULE.secondary_pool_index : gs->SYSTEM_MODULE.primary_pool_index;
    PoolConfig pool = {0};
    if (!SYSTEM_get_pool_config_snapshot(gs, idx, &pool)) {
        ESP_LOGE(TAG, "Unable to snapshot %s pool for probe",
                 use_fallback ? "fallback" : "primary");
        return false;
    }

    bool reachable;
    if (pool.protocol == STRATUM_PROTOCOL_V2) {
        reachable = probe_pool_sv2(pool.url, pool.port);
    } else {
        reachable = probe_pool_v1(gs, pool.url, pool.port, pool.tls,
                                  pool.cert, pool.user, pool.pass);
    }

    SYSTEM_release_pool_config_snapshot(&pool);
    return reachable;
}

// Switch from primary to fallback pool.
// The failed task has already exited (it sent PROTOCOL_FAILED then deleted itself).
static void switch_to_fallback(GlobalState *gs)
{
    SYSTEM_clean_jobs_queue(gs);
    reset_share_stats(gs);

    gs->SYSTEM_MODULE.is_using_fallback = true;
    gs->stratum_protocol = s_fallback_protocol;
    s_running_protocol = s_fallback_protocol;
    s_state = COORD_STATE_RUNNING_FALLBACK;

    ESP_LOGI(TAG, "Switching to fallback pool (%s)",
             s_fallback_protocol == STRATUM_PROTOCOL_V2 ? STRATUM_V2 : STRATUM_V1);

    if (!start_protocol_task(gs, s_fallback_protocol)) {
        enter_paused_state(gs);
        return;
    }

    // Only enable heartbeat if this was an automatic failover (not user choice)
    s_heartbeat_enabled = !gs->SYSTEM_MODULE.use_fallback_stratum;
}

// Switch from fallback back to primary pool.
// Must stop the running fallback task first.
static void switch_to_primary(GlobalState *gs)
{
    ESP_LOGI(TAG, "Primary pool is back! Switching from fallback.");

    if (!stop_running_task(gs)) {
        // Do not publish a replacement transport while the old task can still
        // wake up and tear it down. The heartbeat will retry the switch after
        // the old task reports its exit.
        ESP_LOGW(TAG,
                 "Deferring primary-pool switch until the current protocol task exits");
        return;
    }

    SYSTEM_clean_jobs_queue(gs);
    reset_share_stats(gs);

    gs->SYSTEM_MODULE.is_using_fallback = false;
    gs->stratum_protocol = s_primary_protocol;
    s_running_protocol = s_primary_protocol;
    s_state = COORD_STATE_RUNNING_PRIMARY;

    if (!start_protocol_task(gs, s_primary_protocol)) {
        enter_paused_state(gs);
        return;
    }

    s_heartbeat_enabled = false;
}

// Non-blocking heartbeat probe — called when the heartbeat timer expires
static void do_heartbeat_probe(GlobalState *gs)
{
    // Never auto-switch back if user explicitly chose fallback
    if (gs->SYSTEM_MODULE.use_fallback_stratum) {
        s_heartbeat_enabled = false;
        return;
    }

    if (!wifi_is_connected()) {
        return;
    }

    ESP_LOGD(TAG, "Heartbeat: probing primary pool");

    if (probe_pool(gs, /*use_fallback=*/false)) {
        switch_to_primary(gs);
    } else {
        ESP_LOGD(TAG, "Primary pool still unreachable");
    }
}

// Number of consecutive pool failures that triggers entering the paused state.
// With both primary and fallback configured we tolerate one failure per pool;
// with only one pool configured we pause on its first exhaustion.
static int pool_failure_threshold(GlobalState *gs)
{
    return has_fallback_pool(gs) ? 2 : 1;
}

// All configured pools have exhausted retries. Set pools_unavailable so power
// management cuts ASIC power, and park the coordinator until a probe succeeds.
static void enter_paused_state(GlobalState *gs)
{
    SYSTEM_clean_jobs_queue(gs);
    s_state = COORD_STATE_PAUSED;
    gs->SYSTEM_MODULE.pools_unavailable = true;
    s_heartbeat_enabled = false;
    ESP_LOGW(TAG, "All configured pools unreachable, pausing mining to conserve power.");
}

// Resume mining on the given pool after a successful recovery probe.
// Used only from the paused state — caller must have already verified the
// pool is reachable.
static void resume_on_pool(GlobalState *gs, bool use_fallback)
{
    s_consecutive_pool_failures = 0;
    gs->SYSTEM_MODULE.pools_unavailable = false;
    gs->SYSTEM_MODULE.is_using_fallback = use_fallback;

    stratum_protocol_t proto = use_fallback ? s_fallback_protocol : s_primary_protocol;
    gs->stratum_protocol = proto;
    s_running_protocol = proto;
    s_state = use_fallback ? COORD_STATE_RUNNING_FALLBACK : COORD_STATE_RUNNING_PRIMARY;

    SYSTEM_clean_jobs_queue(gs);
    reset_share_stats(gs);

    ESP_LOGI(TAG, "Pool recovery: %s pool reachable, resuming mining (%s)",
             use_fallback ? "fallback" : "primary",
             proto == STRATUM_PROTOCOL_V2 ? STRATUM_V2 : STRATUM_V1);

    if (!start_protocol_task(gs, proto)) {
        enter_paused_state(gs);
        return;
    }

    // Only run the auto-switch-back heartbeat for *automatic* failovers
    // (user did not explicitly choose the fallback pool).
    s_heartbeat_enabled = use_fallback && !gs->SYSTEM_MODULE.use_fallback_stratum;
}

// Probe pools while paused. Tries the user-preferred pool first, then the other.
// Resumes mining as soon as one is reachable.
static void try_resume_from_paused(GlobalState *gs)
{
    if (!wifi_is_connected()) {
        return;
    }

    bool prefer_fallback = gs->SYSTEM_MODULE.use_fallback_stratum && has_fallback_pool(gs);

    if (prefer_fallback) {
        if (probe_pool(gs, /*use_fallback=*/true)) {
            resume_on_pool(gs, /*use_fallback=*/true);
            return;
        }
        if (probe_pool(gs, /*use_fallback=*/false)) {
            resume_on_pool(gs, /*use_fallback=*/false);
            return;
        }
    } else {
        if (probe_pool(gs, /*use_fallback=*/false)) {
            resume_on_pool(gs, /*use_fallback=*/false);
            return;
        }
        if (has_fallback_pool(gs) && probe_pool(gs, /*use_fallback=*/true)) {
            resume_on_pool(gs, /*use_fallback=*/true);
            return;
        }
    }

    ESP_LOGD(TAG, "Recovery probe: no pool reachable, staying paused");
}

static bool refresh_pool_protocols(GlobalState *gs)
{
    uint16_t prim_idx = gs->SYSTEM_MODULE.primary_pool_index;
    uint16_t sec_idx = gs->SYSTEM_MODULE.secondary_pool_index;
    PoolConfig primary = {0};
    PoolConfig fallback = {0};

    if (!SYSTEM_get_pool_config_snapshot(gs, prim_idx, &primary) ||
        !SYSTEM_get_pool_config_snapshot(gs, sec_idx, &fallback)) {
        ESP_LOGE(TAG, "Unable to snapshot pool protocols");
        SYSTEM_release_pool_config_snapshot(&primary);
        SYSTEM_release_pool_config_snapshot(&fallback);
        return false;
    }

    s_primary_protocol = primary.protocol;
    s_fallback_protocol = fallback.protocol;
    SYSTEM_release_pool_config_snapshot(&primary);
    SYSTEM_release_pool_config_snapshot(&fallback);
    return true;
}

static void apply_pool_config_change(GlobalState *gs)
{
    coordinator_state_t old_state = s_state;

    if (old_state == COORD_STATE_RUNNING_PRIMARY ||
        old_state == COORD_STATE_RUNNING_FALLBACK) {
        if (!stop_running_task(gs)) {
            return;
        }
    }

    SYSTEM_clean_jobs_queue(gs);
    reset_share_stats(gs);
    s_consecutive_pool_failures = 0;

    if (!refresh_pool_protocols(gs)) {
        enter_paused_state(gs);
        return;
    }

    if (old_state == COORD_STATE_PAUSED) {
        // A user has supplied new pool data while mining was parked. Probe it
        // immediately instead of waiting for the ordinary 30-second cadence.
        try_resume_from_paused(gs);
        return;
    }

    bool use_fallback = gs->SYSTEM_MODULE.is_using_fallback;
    stratum_protocol_t protocol = use_fallback
                                      ? s_fallback_protocol
                                      : s_primary_protocol;
    gs->stratum_protocol = protocol;
    gs->SYSTEM_MODULE.pools_unavailable = false;
    s_running_protocol = protocol;
    s_state = use_fallback ? COORD_STATE_RUNNING_FALLBACK
                           : COORD_STATE_RUNNING_PRIMARY;
    s_heartbeat_enabled = use_fallback &&
                          !gs->SYSTEM_MODULE.use_fallback_stratum;

    ESP_LOGI(TAG, "Pool configuration changed; restarting %s session (%s)",
             use_fallback ? "fallback" : "primary",
             protocol == STRATUM_PROTOCOL_V2 ? STRATUM_V2 : STRATUM_V1);
    if (!start_protocol_task(gs, protocol)) {
        enter_paused_state(gs);
    }
}

// Handle an event from the event queue
static void handle_event(GlobalState *gs, coordinator_event_t evt)
{
    switch (evt) {
        case COORD_EVENT_PROTOCOL_FAILED: {
            if (s_state == COORD_STATE_PAUSED) {
                // Stray failure from a task that exited after we already paused — ignore.
                break;
            }
            s_consecutive_pool_failures++;
            int threshold = pool_failure_threshold(gs);
            ESP_LOGW(TAG, "Protocol failure reported (state=%d, failures=%d/%d)",
                     s_state, s_consecutive_pool_failures, threshold);

            if (s_consecutive_pool_failures >= threshold) {
                enter_paused_state(gs);
                break;
            }

            // Below threshold — try the other pool. This only fires when a
            // fallback exists (otherwise threshold=1 and we paused above).
            if (s_state == COORD_STATE_RUNNING_PRIMARY) {
                switch_to_fallback(gs);
            } else if (s_state == COORD_STATE_RUNNING_FALLBACK) {
                ESP_LOGI(TAG, "Fallback failed, trying primary");
                SYSTEM_clean_jobs_queue(gs);
                reset_share_stats(gs);
                gs->SYSTEM_MODULE.is_using_fallback = false;
                gs->stratum_protocol = s_primary_protocol;
                s_running_protocol = s_primary_protocol;
                s_state = COORD_STATE_RUNNING_PRIMARY;
                if (!start_protocol_task(gs, s_primary_protocol)) {
                    enter_paused_state(gs);
                }
                s_heartbeat_enabled = false;
            }
            break;
        }

        case COORD_EVENT_PROTOCOL_SUCCESS:
            if (s_consecutive_pool_failures > 0 || gs->SYSTEM_MODULE.pools_unavailable) {
                ESP_LOGI(TAG, "Pool connection succeeded — clearing failure state");
            }
            s_consecutive_pool_failures = 0;
            gs->SYSTEM_MODULE.pools_unavailable = false;
            break;

        case COORD_EVENT_V1_TASK_EXITED:
        case COORD_EVENT_V2_TASK_EXITED:
            // These come from clean coordinator-requested shutdowns (via stop functions).
            // They're consumed by stop_v1_task/stop_v2_task during switch_to_primary.
            // If we receive one here unexpectedly, just log it.
            ESP_LOGI(TAG, "Task exited event received (evt=%d, state=%d)", evt, s_state);
            break;

        case COORD_EVENT_POOL_CONFIG_CHANGED:
            break;
    }
}

void protocol_coordinator_task(void *pvParameters)
{
    GlobalState *gs = (GlobalState *)pvParameters;

    if (!refresh_pool_protocols(gs)) {
        enter_paused_state(gs);
    } else if (gs->SYSTEM_MODULE.is_using_fallback) {
        // User explicitly selected fallback — use fallback protocol
        gs->stratum_protocol = s_fallback_protocol;
        s_running_protocol = s_fallback_protocol;
        s_state = COORD_STATE_RUNNING_FALLBACK;
        if (!start_protocol_task(gs, s_fallback_protocol)) {
            enter_paused_state(gs);
        }
        // User chose fallback, no heartbeat
        s_heartbeat_enabled = false;
    } else {
        s_running_protocol = s_primary_protocol;
        s_state = COORD_STATE_RUNNING_PRIMARY;
        if (!start_protocol_task(gs, s_primary_protocol)) {
            enter_paused_state(gs);
        }
    }

    ESP_LOGI(TAG, "Protocol coordinator started (primary: %s, fallback: %s, state: %d)",
             s_primary_protocol == STRATUM_PROTOCOL_V2 ? STRATUM_V2 : STRATUM_V1,
             s_fallback_protocol == STRATUM_PROTOCOL_V2 ? STRATUM_V2 : STRATUM_V1,
             s_state);

    // Heartbeat initial delay state — give fallback connection time to establish
    // before probing primary pool
    bool heartbeat_initial_delay = false;
    int64_t heartbeat_delay_start = 0;

    // Main non-blocking event loop
    while (1) {
        if (atomic_exchange_explicit(&s_pool_config_changed, false,
                                     memory_order_acq_rel)) {
            bool was_heartbeat_enabled = s_heartbeat_enabled;
            apply_pool_config_change(gs);
            if (s_heartbeat_enabled && !was_heartbeat_enabled) {
                heartbeat_initial_delay = true;
                heartbeat_delay_start = esp_timer_get_time();
            }
            continue;
        }

        coordinator_event_t evt;
        TickType_t wait;
        if (s_state == COORD_STATE_PAUSED) {
            wait = pdMS_TO_TICKS(RECOVERY_PROBE_INTERVAL_MS);
        } else if (s_heartbeat_enabled) {
            wait = pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS);
        } else {
            wait = portMAX_DELAY;
        }

        bool was_heartbeat_enabled = s_heartbeat_enabled;

        if (xQueueReceive(s_event_queue, &evt, wait) == pdTRUE) {
            handle_event(gs, evt);

            // Detect heartbeat disabled→enabled transition, reset initial delay
            if (s_heartbeat_enabled && !was_heartbeat_enabled) {
                heartbeat_initial_delay = true;
                heartbeat_delay_start = esp_timer_get_time();
            }
        } else if (s_state == COORD_STATE_PAUSED) {
            // Recovery probe — try to bring a pool back online.
            try_resume_from_paused(gs);
        } else if (s_heartbeat_enabled) {
            // Timeout expired — time for a heartbeat probe
            if (heartbeat_initial_delay) {
                int64_t elapsed_ms = (esp_timer_get_time() - heartbeat_delay_start) / 1000;
                if (elapsed_ms < INITIAL_HEARTBEAT_DELAY_MS) {
                    continue;
                }
                heartbeat_initial_delay = false;
            }
            do_heartbeat_probe(gs);
        }
    }

    vTaskDelete(NULL);
}
