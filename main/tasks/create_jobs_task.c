#include <sys/time.h>
#include <limits.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "global_state.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_psram.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mining.h"
#include "miner_job.h"
#include "asic.h"
#include "system.h"
#include "asic_result_task.h"
#include "asic_init.h"
#include "asic_reset.h"
#include "serial.h"
#include "vcore.h"
#include "utils.h"

static const char *TAG = "create_jobs_task";

#define MAX_EXTRANONCE2_LEN 32
#define MAX_EXTRANONCE2_STR (MAX_EXTRANONCE2_LEN * 2 + 1)
#define JOB_SCHEDULER_IDLE_POLL_MS 100
#define JOB_DISPATCH_RETRY_US 10000
#define JOB_ASIC_NOT_READY_RETRY_US 100000
#define JOB_VERSION_MASK_RETRY_US 1000000
#define MICROSECONDS_PER_MILLISECOND 1000

// Scheduler state. The stratum clients publish templates into the shared
// miner_job ring; the scheduler copies each one out under the ring lock, so
// the template it rolls can never change underneath it.
typedef struct
{
    miner_job_t work;      // active template (private copy)
    miner_job_t incoming;  // copy-out buffer for the newest publication
    bool has_work;
    uint32_t work_seq;     // changes with every accepted template
    uint32_t work_pool_generation;
    bool coinbase_decoded;

    uint64_t extranonce_2;
    bool extranonce_exhausted;
    uint32_t version_cursor;  // host-side version rolling (BM1397)
    uint32_t ntime_offset;

    bool has_dispatched;
    bool dispatch_pending;
    int64_t residency_deadline_us;
    int64_t retry_not_before_us;
    uint32_t last_dispatched_job_generation;
    uint32_t prepared_job_generation;

    uint32_t applied_version_mask;
    int64_t mask_retry_not_before_us;

    bm_job *prepared_job;
    uint32_t prepared_seq;
    uint64_t prepared_extranonce;
    uint32_t prepared_version_cursor;
    uint32_t prepared_ntime_offset;
    uint32_t prepared_pool_generation;
} job_scheduler_t;

static job_scheduler_t s_scheduler;

static void scheduler_fail_closed(GlobalState *GLOBAL_STATE,
                                  const char *message)
{
    GLOBAL_STATE->SYSTEM_MODULE.hardware_fault = true;
    snprintf(GLOBAL_STATE->SYSTEM_MODULE.hardware_fault_msg,
             sizeof(GLOBAL_STATE->SYSTEM_MODULE.hardware_fault_msg), "%s",
             message);

    // The ASIC was already initialized before this task was created. If its
    // scheduler cannot start, immediately revoke RUNNING and remove power so
    // the rest of the system cannot present an idle chip as mining-ready.
    asic_lifecycle_set(GLOBAL_STATE, ASIC_LIFECYCLE_STOPPING);
    pthread_mutex_lock(&GLOBAL_STATE->asic_command_lock);
    (void)SERIAL_pause_tx(0);
    if (asic_hold_reset_low() != ESP_OK) {
        ESP_LOGE(TAG, "Unable to hold ASIC reset after scheduler failure");
    }
    pthread_mutex_unlock(&GLOBAL_STATE->asic_command_lock);
    if (VCORE_set_voltage(GLOBAL_STATE, 0.0f) != ESP_OK) {
        ESP_LOGE(TAG, "Unable to disable VCORE after scheduler failure");
    }
    GLOBAL_STATE->POWER_MANAGEMENT_MODULE.expected_hashrate = 0.0f;
    asic_lifecycle_set(GLOBAL_STATE, ASIC_LIFECYCLE_STOPPED);
}

static int64_t get_job_interval_us(GlobalState *GLOBAL_STATE)
{
    double interval_ms = ASIC_get_asic_job_frequency_ms(GLOBAL_STATE);
    if (!(interval_ms > 0.0)) {
        interval_ms = 1.0;
    }

    double interval_us = interval_ms * MICROSECONDS_PER_MILLISECOND;
    if (interval_us >= (double)INT64_MAX) {
        return INT64_MAX;
    }

    // Preserve the sub-millisecond part of the model calculation. The
    // scheduler keeps this absolute deadline even though the notification
    // wait itself has millisecond resolution.
    return (int64_t)(interval_us + 0.5);
}

static int deadline_wait_ms(int64_t deadline_us, int64_t now_us)
{
    if (deadline_us <= now_us) {
        return 0;
    }

    int64_t remaining_us = deadline_us - now_us;
    int64_t remaining_ms =
        (remaining_us + MICROSECONDS_PER_MILLISECOND - 1) /
        MICROSECONDS_PER_MILLISECOND;
    return remaining_ms > INT_MAX ? INT_MAX : (int)remaining_ms;
}

static bool hardware_rolls_versions(GlobalState *GLOBAL_STATE)
{
    return GLOBAL_STATE->DEVICE_CONFIG.family.asic.hardware_version_rolling;
}

// Whether the active template can produce another distinct header. Rollable
// work advances extranonce2; on BM1397 other work advances the host version
// cursor. Any other template is sent once and the ASIC rolls it itself.
static bool work_is_refreshable(GlobalState *GLOBAL_STATE, const job_scheduler_t *scheduler)
{
    if (miner_job_is_rollable(&scheduler->work)) {
        return !scheduler->extranonce_exhausted;
    }
    return !hardware_rolls_versions(GLOBAL_STATE);
}

// Midstates per BM1397 work packet: a one-bit or empty mask cannot supply the
// four distinct versions of the four-midstate format.
static uint8_t software_midstates(GlobalState *GLOBAL_STATE, uint32_t version_mask)
{
    uint8_t configured = GLOBAL_STATE->DEVICE_CONFIG.family.asic.software_midstates;
    if (configured == 0) {
        return 0;
    }
    size_t count = version_mask != 0 ? version_mask_midstate_count(version_mask) : 1;
    return count < configured ? (uint8_t)count : configured;
}

static uint32_t increment_version_cursor(uint32_t cursor, uint32_t base_version,
                                         uint32_t version_mask)
{
    uint32_t rolled = increment_bitmask(cursor, version_mask);

    // Preserve the pool-owned version bits so carry from a sparse or
    // exhausted mask wraps only within the negotiated rollable space.
    return (base_version & ~version_mask) | (rolled & version_mask);
}

static bm_job *build_job(GlobalState *GLOBAL_STATE, const miner_job_t *job,
                         uint64_t extranonce_2, uint32_t version_cursor,
                         uint32_t ntime_offset)
{
    char extranonce_2_str[MAX_EXTRANONCE2_STR] = "";
    uint8_t merkle_root[32];

    if (job->type == JOB_TYPE_SV2_STANDARD) {
        memcpy(merkle_root, job->merkle_root, sizeof(merkle_root));
    } else {
        uint8_t extranonce_2_len = job->extranonce2_len;
        if (extranonce_2_len > MAX_EXTRANONCE2_LEN ||
            !extranonce_2_generate(extranonce_2, extranonce_2_len,
                                   extranonce_2_str, sizeof(extranonce_2_str))) {
            ESP_LOGE(TAG, "Unable to encode %u-byte extranonce2 for job %s",
                     (unsigned int)extranonce_2_len, job->job_id);
            return NULL;
        }

        // Same byte order extranonce_2_generate() encodes into the hex string.
        uint8_t extranonce_2_bin[MAX_EXTRANONCE2_LEN] = {0};
        size_t copy_len = extranonce_2_len < sizeof(extranonce_2) ? extranonce_2_len : sizeof(extranonce_2);
        memcpy(extranonce_2_bin, &extranonce_2, copy_len);

        uint8_t coinbase_tx_hash[32];
        calculate_coinbase_tx_hash_bin(job->coinbase_prefix, job->coinbase_prefix_len,
                                       job->extranonce1, job->extranonce1_len,
                                       extranonce_2_bin, extranonce_2_len,
                                       job->coinbase_suffix, job->coinbase_suffix_len,
                                       coinbase_tx_hash);
        calculate_merkle_root_hash(coinbase_tx_hash,
                                   (const uint8_t (*)[32])job->merkle_path,
                                   job->merkle_path_count, merkle_root);
    }

    bm_job *next_job = allocate_bm_job(job->job_id, extranonce_2_str);
    if (next_job == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for new job");
        return NULL;
    }

    uint32_t version = job->version;
    if (!hardware_rolls_versions(GLOBAL_STATE) && !miner_job_is_rollable(job)) {
        version = version_cursor;
    }
    construct_bm_job_from_miner_job(job, version, merkle_root, job->version_mask,
                                    job->pool_diff,
                                    software_midstates(GLOBAL_STATE, job->version_mask),
                                    next_job);
    // ntime is outside the midstate, so the offset needs no midstate rebuild.
    next_job->ntime += ntime_offset;
    return next_job;
}

static bool send_job(GlobalState *GLOBAL_STATE, bm_job *job,
                     uint32_t expected_generation)
{
    if (job == NULL) {
        return false;
    }
    if (!asic_lifecycle_is_running(GLOBAL_STATE)) {
        ESP_LOGW(TAG, "ASIC not initialized, skipping job send");
        release_bm_job(job);
        return false;
    }
    if (!ASIC_send_work(GLOBAL_STATE, job, expected_generation)) {
        release_bm_job(job);
        return false;
    }
    return true;
}

static void discard_prepared_job(job_scheduler_t *scheduler)
{
    if (scheduler->prepared_job != NULL) {
        release_bm_job(scheduler->prepared_job);
        scheduler->prepared_job = NULL;
    }
}

static void reset_dispatch_state(job_scheduler_t *scheduler)
{
    scheduler->has_dispatched = false;
    scheduler->dispatch_pending = false;
    scheduler->residency_deadline_us = 0;
    scheduler->retry_not_before_us = 0;
    scheduler->last_dispatched_job_generation = 0;
    scheduler->prepared_job_generation = 0;
}

static void drop_work(job_scheduler_t *scheduler)
{
    discard_prepared_job(scheduler);
    scheduler->has_work = false;
    scheduler->work_pool_generation = ASIC_result_task_get_pool_generation();
    scheduler->extranonce_2 = 0;
    scheduler->extranonce_exhausted = false;
    scheduler->version_cursor = 0;
    scheduler->ntime_offset = 0;
    reset_dispatch_state(scheduler);
}

// Build the next header while the ASIC scans the current one, so the next
// deadline only has to enqueue the UART packet.
static void prepare_next_job(GlobalState *GLOBAL_STATE, job_scheduler_t *scheduler)
{
    if (scheduler->prepared_job != NULL || !scheduler->has_work ||
        !work_is_refreshable(GLOBAL_STATE, scheduler) ||
        scheduler->work_pool_generation != ASIC_result_task_get_pool_generation()) {
        return;
    }

    bm_job *candidate = build_job(GLOBAL_STATE, &scheduler->work, scheduler->extranonce_2,
                                  scheduler->version_cursor, scheduler->ntime_offset);
    if (candidate == NULL) {
        return;
    }
    if (scheduler->work_pool_generation != ASIC_result_task_get_pool_generation()) {
        release_bm_job(candidate);
        return;
    }
    scheduler->prepared_job = candidate;
    scheduler->prepared_seq = scheduler->work_seq;
    scheduler->prepared_extranonce = scheduler->extranonce_2;
    scheduler->prepared_version_cursor = scheduler->version_cursor;
    scheduler->prepared_ntime_offset = scheduler->ntime_offset;
    scheduler->prepared_pool_generation = scheduler->work_pool_generation;
}

static bool prepared_job_matches(const job_scheduler_t *scheduler)
{
    return scheduler->prepared_job != NULL &&
           scheduler->prepared_seq == scheduler->work_seq &&
           scheduler->prepared_extranonce == scheduler->extranonce_2 &&
           scheduler->prepared_version_cursor == scheduler->version_cursor &&
           scheduler->prepared_ntime_offset == scheduler->ntime_offset &&
           scheduler->prepared_pool_generation == scheduler->work_pool_generation;
}

// Reset everything but the work buffers; the task owns this state for its
// lifetime.
static void reset_scheduler(job_scheduler_t *scheduler)
{
    discard_prepared_job(scheduler);
    miner_job_t work = scheduler->work;
    miner_job_t incoming = scheduler->incoming;
    memset(scheduler, 0, sizeof(*scheduler));
    scheduler->work.coinbase_prefix = work.coinbase_prefix;
    scheduler->work.coinbase_suffix = work.coinbase_suffix;
    scheduler->incoming.coinbase_prefix = incoming.coinbase_prefix;
    scheduler->incoming.coinbase_suffix = incoming.coinbase_suffix;
}

// A template that can never produce a header is rejected once instead of
// failing every dispatch retry.
static bool work_is_buildable(const miner_job_t *job)
{
    return job->extranonce2_len <= MAX_EXTRANONCE2_LEN &&
           job->extranonce1_len <= sizeof(job->extranonce1) &&
           job->merkle_path_count <= MAX_MERKLE_BRANCHES;
}

static void take_work(GlobalState *GLOBAL_STATE, job_scheduler_t *scheduler, uint32_t slot)
{
    miner_job_lock();
    miner_job_copy(&scheduler->incoming, miner_job_get_slot(slot));
    GLOBAL_STATE->active_job_slot_idx = (uint8_t)(slot % MINER_JOB_POOL_SIZE);
    miner_job_unlock();

    if (scheduler->incoming.pool_generation != ASIC_result_task_get_pool_generation()) {
        ESP_LOGD(TAG, "Discarding invalidated work %s", scheduler->incoming.job_id);
        return;
    }
    if (!work_is_buildable(&scheduler->incoming)) {
        ESP_LOGE(TAG, "Rejecting job %s: %u-byte extranonce2 or %u merkle branches exceed limits",
                 scheduler->incoming.job_id, (unsigned int)scheduler->incoming.extranonce2_len,
                 (unsigned int)scheduler->incoming.merkle_path_count);
        return;
    }

    bool generation_changed = !scheduler->has_work ||
                              scheduler->incoming.pool_generation != scheduler->work_pool_generation;

    // Swap so the previous template's buffers become the next copy-out target.
    miner_job_t previous = scheduler->work;
    scheduler->work = scheduler->incoming;
    scheduler->incoming = previous;

    discard_prepared_job(scheduler);
    scheduler->has_work = true;
    scheduler->work_seq++;
    scheduler->work_pool_generation = scheduler->work.pool_generation;
    scheduler->coinbase_decoded = false;
    scheduler->extranonce_2 = 0;
    scheduler->extranonce_exhausted = false;
    scheduler->version_cursor = scheduler->work.version;
    scheduler->ntime_offset = 0;
    if (generation_changed) {
        reset_dispatch_state(scheduler);
    }

    ESP_LOGI(TAG, "New Work Activated (slot %lu) %s (type %d)%s",
             (unsigned long)slot, scheduler->work.job_id, scheduler->work.type,
             scheduler->work.clean_jobs ? ", clean" : "");

    // A first job, a clean job, or work the ASIC cannot refresh must not wait
    // for the residency deadline. Other work replaces the template sent at the
    // existing deadline; prepare its first header while the old one hashes.
    if (!scheduler->has_dispatched || scheduler->work.clean_jobs ||
        !work_is_refreshable(GLOBAL_STATE, scheduler)) {
        scheduler->dispatch_pending = true;
        scheduler->retry_not_before_us = 0;
    } else {
        prepare_next_job(GLOBAL_STATE, scheduler);
    }
}

// Move to the next distinct header after a successful dispatch.
static void advance_work(GlobalState *GLOBAL_STATE, job_scheduler_t *scheduler)
{
    miner_job_t *work = &scheduler->work;

    if (miner_job_is_rollable(work)) {
        if (!extranonce_2_increment(&scheduler->extranonce_2, work->extranonce2_len)) {
            scheduler->extranonce_exhausted = true;
            ESP_LOGW(TAG, "Extranonce2 space exhausted for job %s; waiting for fresh work",
                     work->job_id);
        }
        return;
    }
    if (hardware_rolls_versions(GLOBAL_STATE)) {
        return;
    }

    uint32_t version_mask = work->version_mask;
    if (version_mask == 0) {
        // With no rollable version bits, ntime is the only way to avoid
        // handing BM1397 the same nonce space again.
        scheduler->ntime_offset++;
        return;
    }
    uint8_t transmitted = software_midstates(GLOBAL_STATE, version_mask);
    for (uint8_t i = 0; i < transmitted; i++) {
        scheduler->version_cursor = increment_version_cursor(
            scheduler->version_cursor, work->version, version_mask);
    }
    if (scheduler->version_cursor == work->version) {
        // The masked version space wrapped. Advance ntime before starting the
        // version cycle again.
        scheduler->ntime_offset++;
    }
}

void create_jobs_task(void *pvParameters)
{
    GlobalState *GLOBAL_STATE = (GlobalState *)pvParameters;
    job_scheduler_t *scheduler = &s_scheduler;

    reset_scheduler(scheduler);

    // active_jobs / valid_jobs are allocated by SYSTEM_init_system(), before
    // any task that touches them can run.
    if (!esp_psram_is_initialized() ||
        !miner_job_alloc_buffers(&scheduler->work) ||
        !miner_job_alloc_buffers(&scheduler->incoming)) {
        ESP_LOGE(TAG, "Unable to allocate the job scheduler's work buffers");
        scheduler_fail_closed(GLOBAL_STATE, "ASIC job scheduler allocation failed");
        vTaskDelete(NULL);
        return;
    }
    scheduler->applied_version_mask = UINT32_MAX;
    scheduler->work_pool_generation = ASIC_result_task_get_pool_generation();

    int64_t interval_us = get_job_interval_us(GLOBAL_STATE);
    ESP_LOGI(TAG, "ASIC Job Interval: %.3f ms",
             (double)interval_us / MICROSECONDS_PER_MILLISECOND);
    ESP_LOGI(TAG, "ASIC Ready!");

    while (1) {
        int64_t now_us = esp_timer_get_time();
        bool refreshable = scheduler->has_work && work_is_refreshable(GLOBAL_STATE, scheduler);
        if (!scheduler->dispatch_pending && scheduler->has_dispatched && refreshable &&
            now_us >= scheduler->residency_deadline_us) {
            scheduler->dispatch_pending = true;
            scheduler->retry_not_before_us = 0;
        }

        int wait_ms;
        if (scheduler->dispatch_pending) {
            wait_ms = deadline_wait_ms(scheduler->retry_not_before_us, now_us);
        } else if (!scheduler->has_dispatched || !refreshable) {
            wait_ms = JOB_SCHEDULER_IDLE_POLL_MS;
        } else {
            wait_ms = deadline_wait_ms(scheduler->residency_deadline_us, now_us);
            if (wait_ms > JOB_SCHEDULER_IDLE_POLL_MS) {
                wait_ms = JOB_SCHEDULER_IDLE_POLL_MS;
            }
        }

        // Round a sub-tick wait up so a short deadline blocks instead of
        // spinning in zero-tick polls at lower tick rates.
        TickType_t wait_ticks = pdMS_TO_TICKS(wait_ms);
        if (wait_ms > 0 && wait_ticks == 0) {
            wait_ticks = 1;
        }

        uint32_t slot = 0;
        if (xTaskNotifyWait(0, ULONG_MAX, &slot, wait_ticks) == pdTRUE) {
            take_work(GLOBAL_STATE, scheduler, slot);
        }

        if (!scheduler->has_work) {
            continue;
        }

        // Snapshot the ASIC-slot epoch first, then verify the pool epoch. If a
        // clean races these reads, either the pool check drops the template or
        // the locked ASIC send rejects the old slot epoch.
        uint32_t expected_job_generation = ASIC_result_task_get_job_generation();
        if (scheduler->work_pool_generation != ASIC_result_task_get_pool_generation()) {
            ESP_LOGD(TAG, "Discarding pool-invalidated work before ASIC dispatch");
            drop_work(scheduler);
            continue;
        }

        now_us = esp_timer_get_time();
        if (scheduler->has_dispatched &&
            scheduler->last_dispatched_job_generation != expected_job_generation) {
            // ASIC recovery invalidates device slots but not the pool template.
            // Re-feed it immediately under the new slot epoch.
            if (scheduler->prepared_job_generation != expected_job_generation) {
                if (!work_is_refreshable(GLOBAL_STATE, scheduler)) {
                    // A header that cannot be refreshed was already partly
                    // scanned; move ntime so the ASIC does not repeat it.
                    scheduler->ntime_offset++;
                }
                scheduler->prepared_job_generation = expected_job_generation;
            }
            scheduler->dispatch_pending = true;
            scheduler->retry_not_before_us = 0;
        }
        if (!scheduler->dispatch_pending && scheduler->has_dispatched &&
            work_is_refreshable(GLOBAL_STATE, scheduler) &&
            now_us >= scheduler->residency_deadline_us) {
            scheduler->dispatch_pending = true;
            scheduler->retry_not_before_us = 0;
        }

        if (!scheduler->dispatch_pending || now_us < scheduler->retry_not_before_us) {
            continue;
        }

        if (!asic_lifecycle_is_running(GLOBAL_STATE)) {
            // Recovery can leave the scheduler alive while the ASIC is
            // temporarily unavailable. Avoid repeated allocation, hashing,
            // and warning logs until it is ready again.
            scheduler->retry_not_before_us = now_us + JOB_ASIC_NOT_READY_RETRY_US;
            continue;
        }

        uint32_t version_mask = scheduler->work.version_mask;
        if (scheduler->applied_version_mask != version_mask) {
            if (now_us < scheduler->mask_retry_not_before_us) {
                scheduler->retry_not_before_us = scheduler->mask_retry_not_before_us;
                continue;
            }
            ESP_LOGI(TAG, "Set chip version rolls %i", (int)(version_mask >> 13));
            esp_err_t err = ASIC_set_version_mask_if_running(GLOBAL_STATE, version_mask);
            if (err != ESP_OK) {
                // Keep the pending mask. A transient UART failure must not make
                // software believe the chip accepted it.
                ESP_LOGW(TAG, "ASIC version-mask update failed (%s); retrying",
                         esp_err_to_name(err));
                scheduler->mask_retry_not_before_us = now_us + JOB_VERSION_MASK_RETRY_US;
                scheduler->retry_not_before_us = scheduler->mask_retry_not_before_us;
                continue;
            }
            scheduler->applied_version_mask = version_mask;
            scheduler->mask_retry_not_before_us = 0;
        }

        if (scheduler->prepared_job != NULL && !prepared_job_matches(scheduler)) {
            discard_prepared_job(scheduler);
        }
        bm_job *job = scheduler->prepared_job;
        scheduler->prepared_job = NULL;
        if (job == NULL) {
            job = build_job(GLOBAL_STATE, &scheduler->work, scheduler->extranonce_2,
                            scheduler->version_cursor, scheduler->ntime_offset);
        }

        if (!send_job(GLOBAL_STATE, job, expected_job_generation)) {
            if (scheduler->work_pool_generation != ASIC_result_task_get_pool_generation()) {
                ESP_LOGD(TAG, "Discarding pool-invalidated work during ASIC dispatch");
                drop_work(scheduler);
                continue;
            }
            // Keep the old residency deadline. A transient allocation or UART
            // failure retries the newest work promptly instead of introducing
            // deadline drift or waiting a full interval.
            scheduler->retry_not_before_us = esp_timer_get_time() + JOB_DISPATCH_RETRY_US;
            continue;
        }

        // Anchor the residency interval to the completed UART dispatch;
        // preparing the next header below uses host idle time without
        // shortening the ASIC's nonce scan window.
        int64_t dispatch_completed_us = esp_timer_get_time();
        advance_work(GLOBAL_STATE, scheduler);

        scheduler->has_dispatched = true;
        scheduler->last_dispatched_job_generation = expected_job_generation;
        scheduler->prepared_job_generation = expected_job_generation;
        scheduler->dispatch_pending = false;
        scheduler->retry_not_before_us = 0;
        interval_us = get_job_interval_us(GLOBAL_STATE);
        scheduler->residency_deadline_us = dispatch_completed_us + interval_us;

        if (!scheduler->coinbase_decoded) {
            // Decode for the UI only after the first header is on the wire.
            SYSTEM_decode_and_apply_coinbase(GLOBAL_STATE, &scheduler->work);
            scheduler->coinbase_decoded = true;
        }

        prepare_next_job(GLOBAL_STATE, scheduler);
    }
}
