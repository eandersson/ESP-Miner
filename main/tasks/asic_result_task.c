#include <lwip/tcpip.h>

#include "system.h"
#include <string.h>
#include <stdlib.h>
#include <stdatomic.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "utils.h"
#include "global_state.h"
#include "mining.h"
#include "stratum_api.h"
#include "stratum_v1_task.h"
#include "stratum_v2_task.h"
#include "stratum_socket.h"
#include "sv2_protocol.h"
#include "hashrate_monitor_task.h"
#include "asic.h"
#include "freertos/task.h"
#include "scoreboard.h"
#include "self_test.h"
#include "asic_init.h"
#include "asic_result_task.h"
#include "freertos/queue.h"

static const char *TAG = "asic_result";

#define ASIC_NONCE_QUEUE_LENGTH 64
#define STRATUM_V1_SHARE_QUEUE_LENGTH 64
#define ASIC_RESULT_DEDUP_CACHE_SIZE 64
#define ASIC_RETIRED_JOB_GRACE_MIN_US 250000
#define ASIC_RETIRED_JOB_GRACE_MAX_US 5000000

typedef struct
{
    task_result result;
    bm_job *job;
    bm_job *alternate_job;
    int64_t job_dispatch_us;
    int64_t alternate_job_dispatch_us;
    uint32_t generation;
    stratum_protocol_t protocol;
} queued_asic_result_t;

typedef struct
{
    bm_job *job;
    char *user;
    uint32_t generation;
    uint32_t nonce;
    uint32_t version_bits;
    double difficulty;
    uint64_t result_timestamp_us;
} queued_v1_share_t;

typedef struct
{
    uint64_t job_fingerprint;
    uint32_t generation;
    uint32_t nonce;
    uint32_t rolled_version;
    uint8_t job_id;
    bool valid;
} recent_asic_result_t;

static QueueHandle_t asic_nonce_queue;
static QueueHandle_t stratum_v1_share_queue;
static atomic_uint_fast32_t job_generation;
static atomic_uint_fast32_t pool_generation;
static atomic_uint_fast32_t enqueued_result_count;
static atomic_uint_fast32_t processed_result_count;
static atomic_uint_fast32_t dropped_result_count;
static atomic_uint_fast32_t register_result_count;
static atomic_uint_fast32_t invalid_job_count;
static atomic_uint_fast32_t ambiguous_job_count;
static atomic_uint_fast32_t stale_result_count;
static atomic_uint_fast32_t duplicate_result_count;
static atomic_uint_fast32_t metadata_failure_count;
static atomic_uint_fast32_t queue_high_watermark;
static atomic_uint_fast32_t max_queue_latency_ms;
static atomic_uint_fast32_t queued_share_count;
static atomic_uint_fast32_t submitted_share_count;
static atomic_uint_fast32_t dropped_share_count;
static atomic_uint_fast32_t stale_share_count;
static atomic_uint_fast32_t share_queue_high_watermark;
static recent_asic_result_t recent_results[ASIC_RESULT_DEDUP_CACHE_SIZE];
static size_t recent_result_index;

static void free_queued_result(queued_asic_result_t *queued_result)
{
    if (queued_result == NULL) {
        return;
    }
    if (queued_result->job != NULL) {
        release_bm_job(queued_result->job);
        queued_result->job = NULL;
    }
    if (queued_result->alternate_job != NULL) {
        release_bm_job(queued_result->alternate_job);
        queued_result->alternate_job = NULL;
    }
}

static void free_queued_v1_share(queued_v1_share_t *share)
{
    if (share == NULL) {
        return;
    }
    if (share->job != NULL) {
        release_bm_job(share->job);
        share->job = NULL;
    }
    free(share->user);
    share->user = NULL;
}

static void drain_result_queue(void)
{
    if (asic_nonce_queue == NULL) {
        return;
    }

    queued_asic_result_t queued_result;
    while (xQueueReceive(asic_nonce_queue, &queued_result, 0) == pdTRUE) {
        free_queued_result(&queued_result);
    }
}

static void drain_share_queue(void)
{
    if (stratum_v1_share_queue == NULL) {
        return;
    }

    queued_v1_share_t share;
    while (xQueueReceive(stratum_v1_share_queue, &share, 0) == pdTRUE) {
        free_queued_v1_share(&share);
    }
}

static void update_atomic_max(atomic_uint_fast32_t *maximum, uint32_t value)
{
    uint_fast32_t current = atomic_load(maximum);
    while (value > current &&
           !atomic_compare_exchange_weak(maximum, &current, value)) {
    }
}

static uint64_t fingerprint_bytes(uint64_t hash, const void *data, size_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    for (size_t i = 0; i < length; i++) {
        hash ^= bytes[i];
        hash *= UINT64_C(1099511628211);
    }
    return hash;
}

static uint64_t fingerprint_job(const bm_job *job)
{
    uint64_t hash = UINT64_C(14695981039346656037);
    hash = fingerprint_bytes(hash, &job->ntime, sizeof(job->ntime));
    hash = fingerprint_bytes(hash, &job->version, sizeof(job->version));
    hash = fingerprint_bytes(hash, &job->target, sizeof(job->target));
    if (job->jobid != NULL) {
        hash = fingerprint_bytes(hash, job->jobid, strlen(job->jobid));
    }
    if (job->extranonce2 != NULL) {
        hash = fingerprint_bytes(hash, job->extranonce2, strlen(job->extranonce2));
    }
    return hash;
}

static bool is_duplicate_result(const queued_asic_result_t *queued_result)
{
    const task_result *result = &queued_result->result;
    uint64_t job_fingerprint = fingerprint_job(queued_result->job);

    for (size_t i = 0; i < ASIC_RESULT_DEDUP_CACHE_SIZE; i++) {
        const recent_asic_result_t *recent = &recent_results[i];
        if (recent->valid &&
            recent->generation == queued_result->generation &&
            recent->job_id == result->job_id &&
            recent->nonce == result->nonce &&
            recent->rolled_version == result->rolled_version &&
            recent->job_fingerprint == job_fingerprint) {
            return true;
        }
    }

    recent_results[recent_result_index] = (recent_asic_result_t) {
        .job_fingerprint = job_fingerprint,
        .generation = queued_result->generation,
        .nonce = result->nonce,
        .rolled_version = result->rolled_version,
        .job_id = result->job_id,
        .valid = true,
    };
    recent_result_index =
        (recent_result_index + 1) % ASIC_RESULT_DEDUP_CACHE_SIZE;
    return false;
}

static uint32_t rolled_version_for_job(GlobalState *GLOBAL_STATE,
                                       const task_result *result,
                                       const bm_job *job)
{
    if (GLOBAL_STATE->DEVICE_CONFIG.family.asic.id == BM1397) {
        uint32_t rolled_version = job->version;
        for (uint8_t i = 0; i < result->version_rolling_index; i++) {
            rolled_version = increment_bitmask(rolled_version,
                                               job->version_mask);
        }
        return rolled_version;
    }

    // ASICs return the values of the rolled mask bits, not an OR-only delta.
    // Clear the negotiated field first so a returned zero can clear a base
    // version bit as required by BIP310.
    return (job->version & ~job->version_mask) |
           (result->version_bits & job->version_mask);
}

static int64_t retired_job_grace_us(GlobalState *GLOBAL_STATE)
{
    double interval_us =
        ASIC_get_asic_job_frequency_ms(GLOBAL_STATE) * 1000.0;
    if (!(interval_us > 0.0)) {
        return ASIC_RETIRED_JOB_GRACE_MIN_US;
    }

    double grace_us = interval_us * 2.0 +
                      ASIC_RETIRED_JOB_GRACE_MIN_US;
    if (grace_us < ASIC_RETIRED_JOB_GRACE_MIN_US) {
        return ASIC_RETIRED_JOB_GRACE_MIN_US;
    }
    if (grace_us > ASIC_RETIRED_JOB_GRACE_MAX_US) {
        return ASIC_RETIRED_JOB_GRACE_MAX_US;
    }
    return (int64_t)grace_us;
}

static bool result_after_dispatch(const task_result *result,
                                  int64_t dispatch_us)
{
    return dispatch_us > 0 &&
           result->timestamp_us >= (uint64_t)dispatch_us;
}

static bool resolve_delayed_job_result(GlobalState *GLOBAL_STATE,
                                       queued_asic_result_t *queued_result,
                                       double *resolved_difficulty)
{
    if (GLOBAL_STATE == NULL || queued_result == NULL ||
        resolved_difficulty == NULL) {
        return false;
    }

    bm_job *current = queued_result->job;
    bm_job *retired = queued_result->alternate_job;

    double ticket_difficulty =
        (double)GLOBAL_STATE->DEVICE_CONFIG.family.asic.difficulty;
    uint32_t current_version = 0;
    uint32_t retired_version = 0;
    double current_difficulty = 0.0;
    double retired_difficulty = 0.0;
    bool current_valid = false;
    bool retired_valid = false;

    if (current != NULL && result_after_dispatch(
            &queued_result->result, queued_result->job_dispatch_us)) {
        current_version = rolled_version_for_job(
            GLOBAL_STATE, &queued_result->result, current);
        current_difficulty = test_nonce_value(
            current, queued_result->result.nonce, current_version);
        current_valid = current_difficulty >= ticket_difficulty;
    }

    if (retired != NULL && result_after_dispatch(
            &queued_result->result,
            queued_result->alternate_job_dispatch_us)) {
        int64_t grace_anchor_us = queued_result->job_dispatch_us > 0
                                      ? queued_result->job_dispatch_us
                                      : queued_result->alternate_job_dispatch_us;
        uint64_t valid_until_us =
            (uint64_t)grace_anchor_us +
            (uint64_t)retired_job_grace_us(GLOBAL_STATE);
        if (queued_result->result.timestamp_us <= valid_until_us) {
            retired_version = rolled_version_for_job(
                GLOBAL_STATE, &queued_result->result, retired);
            retired_difficulty = test_nonce_value(
                retired, queued_result->result.nonce, retired_version);
            retired_valid = retired_difficulty >= ticket_difficulty;
        }
    }

    if (current_valid == retired_valid) {
        // Either candidate could be a random match, or neither reproduces the
        // ASIC ticket. Do not guess and send a potentially invalid share.
        return false;
    }

    if (retired_valid) {
        release_bm_job(current);
        queued_result->job = retired;
        queued_result->alternate_job = NULL;
        queued_result->result.rolled_version = retired_version;
        *resolved_difficulty = retired_difficulty;
    } else {
        release_bm_job(retired);
        queued_result->alternate_job = NULL;
        queued_result->result.rolled_version = current_version;
        *resolved_difficulty = current_difficulty;
    }
    return true;
}

static void log_result_metrics(void)
{
    static int64_t last_log_us;
    int64_t now_us = esp_timer_get_time();
    if (last_log_us != 0 && now_us - last_log_us < 60000000) {
        return;
    }
    last_log_us = now_us;

    asic_rx_stats_t rx_stats;
    serial_stats_t serial_stats;
    get_work_rx_stats(&rx_stats);
    SERIAL_get_stats(&serial_stats);
    ESP_LOGI(TAG,
             "RX metrics: frames=%lu crc=%lu discarded=%lu timeouts=%lu uart_err=%lu "
             "registers=%lu nonces_queued=%lu processed=%lu nonce_dropped=%lu "
             "stale=%lu duplicate=%lu invalid=%lu ambiguous=%lu metadata_err=%lu "
             "nonce_peak=%lu latency_max=%lums shares_queued=%lu submitted=%lu "
             "share_dropped=%lu share_stale=%lu share_peak=%lu",
             (unsigned long)rx_stats.frames_received,
             (unsigned long)rx_stats.crc_errors,
             (unsigned long)rx_stats.discarded_bytes,
             (unsigned long)rx_stats.timeouts,
             (unsigned long)rx_stats.uart_errors,
             (unsigned long)atomic_load(&register_result_count),
             (unsigned long)atomic_load(&enqueued_result_count),
             (unsigned long)atomic_load(&processed_result_count),
             (unsigned long)atomic_load(&dropped_result_count),
             (unsigned long)atomic_load(&stale_result_count),
             (unsigned long)atomic_load(&duplicate_result_count),
             (unsigned long)atomic_load(&invalid_job_count),
             (unsigned long)atomic_load(&ambiguous_job_count),
             (unsigned long)atomic_load(&metadata_failure_count),
             (unsigned long)atomic_load(&queue_high_watermark),
             (unsigned long)atomic_load(&max_queue_latency_ms),
             (unsigned long)atomic_load(&queued_share_count),
             (unsigned long)atomic_load(&submitted_share_count),
             (unsigned long)atomic_load(&dropped_share_count),
             (unsigned long)atomic_load(&stale_share_count),
             (unsigned long)atomic_load(&share_queue_high_watermark));
    ESP_LOGI(TAG,
             "UART metrics: tx_fail=%lu tx_partial=%lu rx_fail=%lu rx_peak=%lu "
             "fifo_ovf=%lu buffer_full=%lu parity=%lu frame=%lu baud_fail=%lu",
             (unsigned long)serial_stats.tx_failures,
             (unsigned long)serial_stats.tx_partial_writes,
             (unsigned long)serial_stats.rx_failures,
             (unsigned long)serial_stats.rx_buffer_high_watermark,
             (unsigned long)serial_stats.fifo_overflows,
             (unsigned long)serial_stats.buffer_full_events,
             (unsigned long)serial_stats.parity_errors,
             (unsigned long)serial_stats.frame_errors,
             (unsigned long)serial_stats.baud_failures);
}

esp_err_t ASIC_result_task_init(void)
{
    if (asic_nonce_queue != NULL && stratum_v1_share_queue != NULL) {
        return ESP_OK;
    }

    asic_nonce_queue = xQueueCreate(ASIC_NONCE_QUEUE_LENGTH,
                                    sizeof(queued_asic_result_t));
    stratum_v1_share_queue = xQueueCreate(STRATUM_V1_SHARE_QUEUE_LENGTH,
                                          sizeof(queued_v1_share_t));
    if (asic_nonce_queue == NULL || stratum_v1_share_queue == NULL) {
        if (asic_nonce_queue != NULL) {
            vQueueDelete(asic_nonce_queue);
            asic_nonce_queue = NULL;
        }
        if (stratum_v1_share_queue != NULL) {
            vQueueDelete(stratum_v1_share_queue);
            stratum_v1_share_queue = NULL;
        }
        ESP_LOGE(TAG, "Failed to create ASIC nonce/share queues");
        return ESP_ERR_NO_MEM;
    }

    atomic_init(&job_generation, 1);
    atomic_init(&pool_generation, 1);
    atomic_init(&enqueued_result_count, 0);
    atomic_init(&processed_result_count, 0);
    atomic_init(&dropped_result_count, 0);
    atomic_init(&register_result_count, 0);
    atomic_init(&invalid_job_count, 0);
    atomic_init(&ambiguous_job_count, 0);
    atomic_init(&stale_result_count, 0);
    atomic_init(&duplicate_result_count, 0);
    atomic_init(&metadata_failure_count, 0);
    atomic_init(&queue_high_watermark, 0);
    atomic_init(&max_queue_latency_ms, 0);
    atomic_init(&queued_share_count, 0);
    atomic_init(&submitted_share_count, 0);
    atomic_init(&dropped_share_count, 0);
    atomic_init(&stale_share_count, 0);
    atomic_init(&share_queue_high_watermark, 0);
    memset(recent_results, 0, sizeof(recent_results));
    recent_result_index = 0;
    return ESP_OK;
}

void ASIC_result_task_invalidate_jobs(void)
{
    atomic_fetch_add(&job_generation, 1);
    drain_result_queue();
    drain_share_queue();
}

void ASIC_result_task_invalidate_pool_jobs(void)
{
    atomic_fetch_add(&pool_generation, 1);
    ASIC_result_task_invalidate_jobs();
}

uint32_t ASIC_result_task_get_job_generation(void)
{
    return (uint32_t)atomic_load(&job_generation);
}

uint32_t ASIC_result_task_get_pool_generation(void)
{
    return (uint32_t)atomic_load(&pool_generation);
}

void ASIC_result_task_get_stats(asic_result_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }
    *stats = (asic_result_stats_t) {
        .registers_processed = (uint32_t)atomic_load(&register_result_count),
        .nonces_enqueued = (uint32_t)atomic_load(&enqueued_result_count),
        .nonces_processed = (uint32_t)atomic_load(&processed_result_count),
        .nonces_dropped = (uint32_t)atomic_load(&dropped_result_count),
        .nonce_queue_high_watermark =
            (uint32_t)atomic_load(&queue_high_watermark),
        .nonce_max_latency_ms =
            (uint32_t)atomic_load(&max_queue_latency_ms),
        .invalid_jobs = (uint32_t)atomic_load(&invalid_job_count),
        .ambiguous_jobs = (uint32_t)atomic_load(&ambiguous_job_count),
        .stale_results = (uint32_t)atomic_load(&stale_result_count),
        .duplicate_results = (uint32_t)atomic_load(&duplicate_result_count),
        .shares_enqueued = (uint32_t)atomic_load(&queued_share_count),
        .shares_submitted = (uint32_t)atomic_load(&submitted_share_count),
        .shares_dropped = (uint32_t)atomic_load(&dropped_share_count),
        .stale_shares = (uint32_t)atomic_load(&stale_share_count),
        .share_queue_high_watermark =
            (uint32_t)atomic_load(&share_queue_high_watermark),
    };
}

void ASIC_result_task_reset(void)
{
    reset_work_rx_parser();
    ASIC_result_task_invalidate_jobs();
}

void ASIC_result_rx_task(void *pvParameters)
{
    GlobalState *GLOBAL_STATE = (GlobalState *)pvParameters;

    while (1)
    {
        // Check if ASIC is initialized before trying to process work
        if (!asic_lifecycle_is_running(GLOBAL_STATE)) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            log_result_metrics();
            continue;
        }

        task_result *asic_result = ASIC_process_work(GLOBAL_STATE);

        if (asic_result == NULL) {
            log_result_metrics();
            continue;
        }

        if (asic_result->register_type != REGISTER_INVALID) {
            // Register traffic is frequent on multi-chip boards and must never
            // consume nonce capacity. Updating the small monitor structure is
            // bounded and avoids queueing dozens of telemetry frames per poll.
            hashrate_monitor_register_read(
                GLOBAL_STATE, asic_result->register_type, asic_result->asic_nr,
                asic_result->value, asic_result->timestamp_us);
            atomic_fetch_add(&register_result_count, 1);
            log_result_metrics();
            continue;
        }

        queued_asic_result_t queued_result = {
            .result = *asic_result,
            .job = NULL,
            .alternate_job = NULL,
            .job_dispatch_us = 0,
            .alternate_job_dispatch_us = 0,
        };
        uint8_t job_id = asic_result->job_id;

        // Snapshot both the current and immediately retired owners of this
        // wire ID. Refcounts make them immutable after the short slot lock.
        pthread_mutex_lock(&GLOBAL_STATE->valid_jobs_lock);
        if (GLOBAL_STATE->valid_jobs != NULL &&
            GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs != NULL &&
            GLOBAL_STATE->ASIC_TASK_MODULE.active_job_dispatch_us != NULL &&
            GLOBAL_STATE->valid_jobs[job_id] != 0) {
            queued_result.job =
                GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs[job_id];
            if (queued_result.job != NULL) {
                queued_result.job_dispatch_us =
                    GLOBAL_STATE->ASIC_TASK_MODULE
                        .active_job_dispatch_us[job_id];
                retain_bm_job(queued_result.job);
            }
        }
        if (GLOBAL_STATE->ASIC_TASK_MODULE.retired_jobs != NULL &&
            GLOBAL_STATE->ASIC_TASK_MODULE.retired_job_dispatch_us != NULL) {
            queued_result.alternate_job =
                GLOBAL_STATE->ASIC_TASK_MODULE.retired_jobs[job_id];
            if (queued_result.alternate_job != NULL &&
                queued_result.alternate_job != queued_result.job) {
                queued_result.alternate_job_dispatch_us =
                    GLOBAL_STATE->ASIC_TASK_MODULE
                        .retired_job_dispatch_us[job_id];
                retain_bm_job(queued_result.alternate_job);
            } else {
                queued_result.alternate_job = NULL;
            }
        }
        queued_result.generation = (uint32_t)atomic_load(&job_generation);
        queued_result.protocol = GLOBAL_STATE->stratum_protocol;
        pthread_mutex_unlock(&GLOBAL_STATE->valid_jobs_lock);

        if (queued_result.job == NULL && queued_result.alternate_job == NULL) {
            atomic_fetch_add(&invalid_job_count, 1);
            ESP_LOGD(TAG, "Invalid job nonce found, 0x%02X", job_id);
            continue;
        }

        if (xQueueSend(asic_nonce_queue, &queued_result, 0) != pdTRUE) {
            free_queued_result(&queued_result);
            uint32_t dropped = (uint32_t)atomic_fetch_add(&dropped_result_count, 1) + 1;
            // Log at powers of two so a stalled consumer cannot create a log storm.
            if ((dropped & (dropped - 1)) == 0) {
                ESP_LOGW(TAG, "ASIC nonce queue full; dropped %lu result(s)",
                         (unsigned long)dropped);
            }
        } else {
            atomic_fetch_add(&enqueued_result_count, 1);
            update_atomic_max(&queue_high_watermark,
                              (uint32_t)uxQueueMessagesWaiting(asic_nonce_queue));
        }
        log_result_metrics();
    }
}

static bool enqueue_v1_share(GlobalState *GLOBAL_STATE,
                             const queued_asic_result_t *queued_result,
                             uint32_t version_bits, double difficulty)
{
    uint16_t active_idx = GLOBAL_STATE->SYSTEM_MODULE.is_using_fallback
                              ? GLOBAL_STATE->SYSTEM_MODULE.secondary_pool_index
                              : GLOBAL_STATE->SYSTEM_MODULE.primary_pool_index;
    const char *active_user =
        GLOBAL_STATE->SYSTEM_MODULE.pools[active_idx].user;
    queued_v1_share_t share = {
        .job = queued_result->job,
        .user = active_user != NULL ? strdup(active_user) : NULL,
        .generation = queued_result->generation,
        .nonce = queued_result->result.nonce,
        .version_bits = version_bits,
        .difficulty = difficulty,
        .result_timestamp_us = queued_result->result.timestamp_us,
    };

    if (share.job == NULL || share.user == NULL) {
        free(share.user);
        atomic_fetch_add(&dropped_share_count, 1);
        return false;
    }
    retain_bm_job(share.job);

    if (xQueueSend(stratum_v1_share_queue, &share, 0) != pdTRUE) {
        free_queued_v1_share(&share);
        uint32_t dropped =
            (uint32_t)atomic_fetch_add(&dropped_share_count, 1) + 1;
        if ((dropped & (dropped - 1)) == 0) {
            ESP_LOGE(TAG,
                     "Stratum V1 share queue full; dropped %lu valid share(s)",
                     (unsigned long)dropped);
        }
        return false;
    }

    atomic_fetch_add(&queued_share_count, 1);
    update_atomic_max(
        &share_queue_high_watermark,
        (uint32_t)uxQueueMessagesWaiting(stratum_v1_share_queue));
    return true;
}

void ASIC_result_task(void *pvParameters)
{
    GlobalState *GLOBAL_STATE = (GlobalState *)pvParameters;
    queued_asic_result_t queued_result;

    while (1)
    {
        if (xQueueReceive(asic_nonce_queue, &queued_result, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        task_result *asic_result = &queued_result.result;
        atomic_fetch_add(&processed_result_count, 1);

        uint64_t latency_us = (uint64_t)esp_timer_get_time() - asic_result->timestamp_us;
        uint32_t latency_ms = latency_us / 1000 > UINT32_MAX
                                  ? UINT32_MAX
                                  : (uint32_t)(latency_us / 1000);
        update_atomic_max(&max_queue_latency_ms, latency_ms);

        if (queued_result.generation != (uint32_t)atomic_load(&job_generation)) {
            atomic_fetch_add(&stale_result_count, 1);
            free_queued_result(&queued_result);
            continue;
        }

        double nonce_diff = 0.0;
        if (!resolve_delayed_job_result(GLOBAL_STATE, &queued_result,
                                        &nonce_diff)) {
            atomic_fetch_add(&ambiguous_job_count, 1);
            ESP_LOGD(TAG, "Unable to uniquely resolve delayed job 0x%02X",
                     asic_result->job_id);
            free_queued_result(&queued_result);
            continue;
        }

        if (queued_result.job->jobid == NULL ||
            queued_result.job->extranonce2 == NULL) {
            atomic_fetch_add(&metadata_failure_count, 1);
            ESP_LOGE(TAG, "Missing immutable metadata for job 0x%02X",
                     asic_result->job_id);
            free_queued_result(&queued_result);
            continue;
        }

        if (is_duplicate_result(&queued_result)) {
            atomic_fetch_add(&duplicate_result_count, 1);
            free_queued_result(&queued_result);
            continue;
        }

        bm_job *active_job = queued_result.job;

        if (GLOBAL_STATE->SELF_TEST_MODULE.is_active) {
            self_test_record_nonce(GLOBAL_STATE, nonce_diff);
            free_queued_result(&queued_result);
            continue;
        }

        uint32_t version_bits =
            queued_result.protocol == STRATUM_PROTOCOL_V1
                ? (asic_result->rolled_version & active_job->version_mask)
                : (asic_result->rolled_version ^ active_job->version);
        if (queued_result.generation != (uint32_t)atomic_load(&job_generation)) {
            atomic_fetch_add(&stale_result_count, 1);
            free_queued_result(&queued_result);
            continue;
        }

        double required_share_difficulty = active_job->pool_diff;
        if (queued_result.protocol == STRATUM_PROTOCOL_V1) {
            required_share_difficulty =
                mining_v1_effective_share_difficulty(
                    active_job->pool_diff,
                    stratum_v1_get_current_difficulty(GLOBAL_STATE));
        }

        if (nonce_diff >= required_share_difficulty)
        {
            if (queued_result.protocol == STRATUM_PROTOCOL_V2) {
                // SV2: submit with binary protocol
                int ret;
                uint32_t sv2_job_id = (uint32_t)strtoul(active_job->jobid, NULL, 10);

                if (stratum_v2_is_extended_channel(GLOBAL_STATE)) {
                    // SV2 spec: extranonce_size is the miner's rollable portion.
                    // The pool prepends its extranonce_prefix separately.
                    size_t en2_hex_len = strlen(active_job->extranonce2);
                    if ((en2_hex_len & 1U) != 0 ||
                        en2_hex_len / 2U > 32U) {
                        ESP_LOGW(TAG, "Invalid SV2 extranonce metadata length %u",
                                 (unsigned int)en2_hex_len);
                        free_queued_result(&queued_result);
                        continue;
                    }
                    uint8_t en2_len = (uint8_t)(en2_hex_len / 2U);
                    uint8_t extranonce_2[32];
                    if (hex2bin(active_job->extranonce2, extranonce_2,
                                en2_len) != en2_len) {
                        ESP_LOGW(TAG, "Invalid SV2 extranonce metadata");
                        free_queued_result(&queued_result);
                        continue;
                    }
                    ret = stratum_v2_submit_share_extended(GLOBAL_STATE, sv2_job_id,
                                                           asic_result->nonce,
                                                           active_job->ntime,
                                                           asic_result->rolled_version,
                                                           extranonce_2, en2_len,
                                                           queued_result.generation);
                } else {
                    ret = stratum_v2_submit_share(GLOBAL_STATE, sv2_job_id,
                                                   asic_result->nonce,
                                                   active_job->ntime,
                                                   asic_result->rolled_version,
                                                   queued_result.generation);
                }

                if (ret == STRATUM_SOCKET_WRITE_TRUNCATED) {
                    ESP_LOGE(TAG, "Partial SV2 frame written; dropping connection");
                    stratum_v2_interrupt_connection(GLOBAL_STATE);
                } else if (ret < 0) {
                    ESP_LOGW(TAG, "Failed to submit SV2 share (ret=%d, errno=%d: %s)",
                             ret, errno, strerror(errno));
                }
            } else {
                // Network backpressure must not block nonce validation or allow
                // register traffic to evict valid shares. A dedicated bounded
                // worker owns V1 submissions and rechecks the generation.
                if (!enqueue_v1_share(GLOBAL_STATE, &queued_result,
                                      version_bits, nonce_diff)) {
                    ESP_LOGW(TAG, "Unable to queue valid Stratum V1 share");
                }
            }
        }

        //log the ASIC response
        ESP_LOGD(TAG, "ID: %s, ASIC nr: %d, Core: %d/%d, ver: %08" PRIX32 " Nonce %08" PRIX32 " diff %.1f of %g.", active_job->jobid, asic_result->asic_nr, asic_result->core_id, asic_result->small_core_id, asic_result->rolled_version, asic_result->nonce, nonce_diff, required_share_difficulty);

        SYSTEM_notify_found_nonce(GLOBAL_STATE, nonce_diff, active_job->target);

        scoreboard_add(&GLOBAL_STATE->SYSTEM_MODULE.scoreboard, nonce_diff, active_job->jobid, active_job->extranonce2, active_job->ntime, asic_result->nonce, version_bits);

        free_queued_result(&queued_result);
    }
}

void ASIC_v1_share_submit_task(void *pvParameters)
{
    GlobalState *GLOBAL_STATE = (GlobalState *)pvParameters;
    queued_v1_share_t share;

    while (1) {
        if (xQueueReceive(stratum_v1_share_queue, &share, portMAX_DELAY) !=
            pdTRUE) {
            continue;
        }

        if (share.generation != (uint32_t)atomic_load(&job_generation)) {
            atomic_fetch_add(&stale_share_count, 1);
            free_queued_v1_share(&share);
            continue;
        }

        taskENTER_CRITICAL(&GLOBAL_STATE->stratum_mux);
        int uid = GLOBAL_STATE->send_uid++;
        taskEXIT_CRITICAL(&GLOBAL_STATE->stratum_mux);

        uint64_t sent_time_us = 0;
        int ret = stratum_v1_submit_share_safe(
            GLOBAL_STATE, share.generation, uid, share.user,
            share.job->jobid, share.job->extranonce2, share.job->ntime,
            share.nonce, share.job->version_rolling_enabled,
            share.version_bits, share.difficulty, share.job->pool_diff,
            &sent_time_us);

        if (ret == STRATUM_V1_SUBMIT_FILTERED) {
            atomic_fetch_add(&dropped_share_count, 1);
            ESP_LOGD(TAG,
                     "Filtered queued V1 share below updated pool difficulty");
        } else if (ret == STRATUM_SOCKET_WRITE_TRUNCATED) {
            ESP_LOGE(TAG,
                     "Partial share written to socket; dropping connection");
            stratum_v1_interrupt_connection(GLOBAL_STATE);
        } else if (ret < 0) {
            ESP_LOGW(TAG,
                     "Unable to write share to socket (ret=%d, errno=%d: %s)",
                     ret, errno, strerror(errno));
        } else {
            atomic_fetch_add(&submitted_share_count, 1);
            if (sent_time_us >= share.result_timestamp_us) {
                float process_time =
                    (sent_time_us - share.result_timestamp_us) / 1000.0f;
                GLOBAL_STATE->SYSTEM_MODULE.process_time = process_time;
                ESP_LOGD(TAG, "Processing time: %0.1f ms", process_time);
            }
        }

        free_queued_v1_share(&share);
    }
}
