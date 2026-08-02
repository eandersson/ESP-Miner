#include <sys/time.h>
#include <limits.h>
#include <stdint.h>

#include "work_queue.h"
#include "global_state.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_psram.h"
#include "mining.h"
#include "string.h"
#include "esp_timer.h"

#include "asic.h"
#include "system.h"
#include "esp_heap_caps.h"
#include "freertos/task.h"
#include "sv2_protocol.h"
#include "stratum_api.h"
#include "stratum_v1_task.h"
#include "stratum_v2_task.h"
#include "asic_result_task.h"
#include "utils.h"

static const char *TAG = "create_jobs_task";

#define MAX_EXTRANONCE2_LEN 32
#define MAX_EXTRANONCE2_STR (MAX_EXTRANONCE2_LEN * 2 + 1)
#define JOB_SCHEDULER_IDLE_POLL_MS 100
#define JOB_DISPATCH_RETRY_US 10000
#define JOB_ASIC_NOT_READY_RETRY_US 100000
#define MICROSECONDS_PER_MILLISECOND 1000

static bool generate_work(GlobalState *GLOBAL_STATE, mining_notify *notification,
                          uint64_t extranonce_2, double difficulty,
                          uint32_t expected_generation);
static bool generate_work_sv2(GlobalState *GLOBAL_STATE, sv2_job_t *job,
                              double difficulty, uint32_t version_cursor,
                              uint32_t ntime_offset,
                              uint32_t version_mask,
                              uint32_t expected_generation);
static bool generate_work_sv2_ext(GlobalState *GLOBAL_STATE, sv2_ext_job_t *job,
                                  double difficulty,
                                  uint64_t extranonce_2_counter,
                                  uint32_t expected_generation);

static void free_work_item(void *work,
                           const work_queue_item_metadata *metadata)
{
    if (!work) return;
    if (metadata != NULL && metadata->free_fn != NULL) {
        metadata->free_fn(work);
    } else {
        free(work);
    }
}

static work_queue_item_kind get_active_work_kind(GlobalState *GLOBAL_STATE)
{
    if (GLOBAL_STATE->stratum_protocol != STRATUM_PROTOCOL_V2) {
        return WORK_QUEUE_ITEM_STRATUM_V1;
    }
    return stratum_v2_is_extended_channel(GLOBAL_STATE)
               ? WORK_QUEUE_ITEM_STRATUM_V2_EXTENDED
               : WORK_QUEUE_ITEM_STRATUM_V2_STANDARD;
}

static const char *work_kind_name(work_queue_item_kind kind)
{
    switch (kind) {
        case WORK_QUEUE_ITEM_STRATUM_V1:
            return "Stratum V1";
        case WORK_QUEUE_ITEM_STRATUM_V2_STANDARD:
            return "Stratum V2 standard";
        case WORK_QUEUE_ITEM_STRATUM_V2_EXTENDED:
            return "Stratum V2 extended";
        default:
            return "unknown";
    }
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
    // scheduler keeps this absolute deadline even though the condition wait
    // itself has millisecond resolution.
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

static bool is_sv2_standard_channel(GlobalState *GLOBAL_STATE,
                                    work_queue_item_kind kind)
{
    return kind == WORK_QUEUE_ITEM_STRATUM_V2_STANDARD &&
           GLOBAL_STATE->DEVICE_CONFIG.family.asic.id != BM1397;
}

static bool is_bm1397_sv2_standard_channel(
    GlobalState *GLOBAL_STATE, work_queue_item_kind kind)
{
    return kind == WORK_QUEUE_ITEM_STRATUM_V2_STANDARD &&
           GLOBAL_STATE->DEVICE_CONFIG.family.asic.id == BM1397;
}

static uint32_t increment_sv2_version_cursor(uint32_t cursor,
                                             uint32_t server_version,
                                             uint32_t version_mask)
{
    uint32_t rolled = increment_bitmask(cursor, version_mask);

    // Preserve the server-owned version bits so carry from a sparse or
    // exhausted mask wraps only within the negotiated rollable space.
    return (server_version & ~version_mask) |
           (rolled & version_mask);
}

static bool work_has_clean_jobs(void *work, work_queue_item_kind kind)
{
    if (kind == WORK_QUEUE_ITEM_STRATUM_V2_EXTENDED) {
        return ((sv2_ext_job_t *)work)->clean_jobs;
    }
    if (kind == WORK_QUEUE_ITEM_STRATUM_V2_STANDARD) {
        return ((sv2_job_t *)work)->clean_jobs;
    }
    return kind == WORK_QUEUE_ITEM_STRATUM_V1 &&
           ((mining_notify *)work)->clean_jobs;
}

static void log_dequeued_work(void *work, work_queue_item_kind kind)
{
    if (kind == WORK_QUEUE_ITEM_STRATUM_V2_EXTENDED) {
        ESP_LOGI(TAG, "New Work Dequeued SV2 ext job %lu",
                 ((sv2_ext_job_t *)work)->job_id);
    } else if (kind == WORK_QUEUE_ITEM_STRATUM_V2_STANDARD) {
        ESP_LOGI(TAG, "New Work Dequeued SV2 job %lu",
                 ((sv2_job_t *)work)->job_id);
    } else if (kind == WORK_QUEUE_ITEM_STRATUM_V1) {
        ESP_LOGI(TAG, "New Work Dequeued %s",
                 ((mining_notify *)work)->job_id);
    }
}

static bool apply_pending_control_updates(GlobalState *GLOBAL_STATE,
                                          double *difficulty)
{
    bool changed = false;

    if (GLOBAL_STATE->new_set_mining_difficulty_msg) {
        ESP_LOGI(TAG, "New pool difficulty %.2f",
                 GLOBAL_STATE->pool_difficulty);
        *difficulty = GLOBAL_STATE->pool_difficulty;
        GLOBAL_STATE->new_set_mining_difficulty_msg = false;
        changed = true;
    }

    if (GLOBAL_STATE->new_stratum_version_rolling_msg &&
        GLOBAL_STATE->ASIC_initalized) {
        ESP_LOGI(TAG, "Set chip version rolls %i",
                 (int)(GLOBAL_STATE->version_mask >> 13));
        ASIC_set_version_mask(GLOBAL_STATE, GLOBAL_STATE->version_mask);
        GLOBAL_STATE->new_stratum_version_rolling_msg = false;
        changed = true;
    }

    return changed;
}

static bool dispatch_work(GlobalState *GLOBAL_STATE, void *current_work,
                          work_queue_item_kind kind, double difficulty,
                          uint64_t extranonce_2,
                          uint32_t sv2_version_cursor,
                          uint32_t sv2_ntime_offset,
                          uint32_t version_mask,
                          uint32_t expected_generation)
{
    if (kind == WORK_QUEUE_ITEM_STRATUM_V2_EXTENDED) {
        return generate_work_sv2_ext(
            GLOBAL_STATE, (sv2_ext_job_t *)current_work, difficulty,
            extranonce_2, expected_generation);
    }
    if (kind == WORK_QUEUE_ITEM_STRATUM_V2_STANDARD) {
        return generate_work_sv2(
            GLOBAL_STATE, (sv2_job_t *)current_work, difficulty,
            sv2_version_cursor, sv2_ntime_offset, version_mask,
            expected_generation);
    }

    if (kind == WORK_QUEUE_ITEM_STRATUM_V1) {
        return generate_work(
            GLOBAL_STATE, (mining_notify *)current_work, extranonce_2,
            difficulty, expected_generation);
    }
    ESP_LOGE(TAG, "Cannot dispatch unknown work item kind");
    return false;
}

void create_jobs_task(void *pvParameters)
{
    GlobalState *GLOBAL_STATE = (GlobalState *)pvParameters;

    // Initialize ASIC task module (moved from ASIC_task)
    if (!esp_psram_is_initialized()) {
        ESP_LOGE(TAG, "PSRAM unavailable; cannot allocate ASIC job tracking");
        GLOBAL_STATE->ASIC_initalized = false;
        vTaskDelete(NULL);
        return;
    }

    GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs = heap_caps_calloc(
        128, sizeof(bm_job *), MALLOC_CAP_SPIRAM);
    GLOBAL_STATE->valid_jobs = heap_caps_calloc(
        128, sizeof(uint8_t), MALLOC_CAP_SPIRAM);
    if (GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs == NULL ||
        GLOBAL_STATE->valid_jobs == NULL) {
        ESP_LOGE(TAG, "Unable to allocate ASIC job tracking");
        heap_caps_free(GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs);
        heap_caps_free(GLOBAL_STATE->valid_jobs);
        GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs = NULL;
        GLOBAL_STATE->valid_jobs = NULL;
        GLOBAL_STATE->ASIC_initalized = false;
        vTaskDelete(NULL);
        return;
    }

    double difficulty = GLOBAL_STATE->pool_difficulty;
    void *current_work = NULL;
    work_queue_item_metadata current_work_metadata = {0};
    work_queue_item_kind observed_active_kind =
        get_active_work_kind(GLOBAL_STATE);
    uint64_t extranonce_2 = 0;
    int64_t interval_us = get_job_interval_us(GLOBAL_STATE);
    int64_t residency_deadline_us = 0;
    int64_t retry_not_before_us = 0;
    bool has_dispatched_work = false;
    bool dispatch_pending = false;
    bool control_refresh_pending = false;
    uint32_t current_work_pool_generation =
        ASIC_result_task_get_pool_generation();
    uint32_t last_dispatched_job_generation = 0;
    uint32_t prepared_job_generation = 0;
    uint32_t sv2_version_cursor = 0;
    uint32_t sv2_ntime_offset = 0;

    ESP_LOGI(TAG, "ASIC Job Interval: %.3f ms",
             (double)interval_us / MICROSECONDS_PER_MILLISECOND);
    ESP_LOGI(TAG, "ASIC Ready!");

    while (1) {
        // Protocol and SV2 channel subtype can change independently during
        // failover. Keep the item's immutable kind/destructor, and discard it
        // when it no longer matches the active connection.
        work_queue_item_kind active_kind =
            get_active_work_kind(GLOBAL_STATE);
        if (active_kind != observed_active_kind) {
            if (current_work != NULL) {
                ESP_LOGI(TAG, "Protocol switched from %s to %s, discarding current work",
                         work_kind_name(current_work_metadata.kind),
                         work_kind_name(active_kind));
                free_work_item(current_work,
                               &current_work_metadata);
                current_work = NULL;
                current_work_metadata =
                    (work_queue_item_metadata){0};
            }
            observed_active_kind = active_kind;
            extranonce_2 = 0;
            residency_deadline_us = 0;
            retry_not_before_us = 0;
            has_dispatched_work = false;
            dispatch_pending = false;
            current_work_pool_generation =
                ASIC_result_task_get_pool_generation();
            last_dispatched_job_generation = 0;
            prepared_job_generation = 0;
            sv2_version_cursor = 0;
            sv2_ntime_offset = 0;
        }

        if (apply_pending_control_updates(GLOBAL_STATE, &difficulty)) {
            control_refresh_pending = true;
            if (current_work != NULL &&
                is_bm1397_sv2_standard_channel(
                    GLOBAL_STATE, current_work_metadata.kind)) {
                sv2_version_cursor =
                    ((sv2_job_t *)current_work)->version;
                if (has_dispatched_work) {
                    // A changed mask or difficulty must not repeat the exact
                    // header batch that was already scanned.
                    sv2_ntime_offset++;
                } else {
                    sv2_ntime_offset = 0;
                }
            } else if (current_work != NULL &&
                       is_sv2_standard_channel(
                           GLOBAL_STATE, current_work_metadata.kind)) {
                // A control-only refresh still needs a distinct header so a
                // self-rolling ASIC does not restart an already-scanned job.
                if (has_dispatched_work) {
                    sv2_ntime_offset++;
                } else {
                    sv2_ntime_offset = 0;
                }
            }
            if (current_work != NULL) {
                dispatch_pending = true;
                retry_not_before_us = 0;
            }
        }

        int64_t now_us = esp_timer_get_time();
        if (!dispatch_pending && current_work != NULL &&
            has_dispatched_work &&
            !is_sv2_standard_channel(GLOBAL_STATE,
                                     current_work_metadata.kind) &&
            now_us >= residency_deadline_us) {
            dispatch_pending = true;
            retry_not_before_us = 0;
        }

        int wait_ms;
        if (dispatch_pending) {
            wait_ms = deadline_wait_ms(retry_not_before_us, now_us);
        } else if (current_work == NULL ||
                   is_sv2_standard_channel(GLOBAL_STATE,
                                           current_work_metadata.kind)) {
            wait_ms = JOB_SCHEDULER_IDLE_POLL_MS;
        } else {
            wait_ms = deadline_wait_ms(residency_deadline_us, now_us);
            if (wait_ms > JOB_SCHEDULER_IDLE_POLL_MS) {
                wait_ms = JOB_SCHEDULER_IDLE_POLL_MS;
            }
        }

        work_queue_item_metadata dequeued_metadata = {0};
        void *new_work = queue_dequeue_timeout(
            &GLOBAL_STATE->stratum_queue, wait_ms,
            &dequeued_metadata);

        if (new_work != NULL) {
            active_kind = get_active_work_kind(GLOBAL_STATE);
            if (active_kind != observed_active_kind) {
                free_work_item(current_work, &current_work_metadata);
                current_work = NULL;
                current_work_metadata =
                    (work_queue_item_metadata){0};
                observed_active_kind = active_kind;
                extranonce_2 = 0;
                residency_deadline_us = 0;
                retry_not_before_us = 0;
                has_dispatched_work = false;
                dispatch_pending = false;
                current_work_pool_generation =
                    ASIC_result_task_get_pool_generation();
                last_dispatched_job_generation = 0;
                prepared_job_generation = 0;
                sv2_version_cursor = 0;
                sv2_ntime_offset = 0;
            }

            bool saw_clean_jobs = false;
            bool accepted_new_work = false;
            // Drain immediately available notifications so a burst of
            // non-clean work produces one ASIC update containing the newest
            // template rather than each stale intermediate template.
            do {
                active_kind = get_active_work_kind(GLOBAL_STATE);
                if (dequeued_metadata.generation !=
                        ASIC_result_task_get_pool_generation() ||
                    dequeued_metadata.kind != active_kind) {
                    ESP_LOGD(TAG,
                             "Discarding stale/inactive queued %s work",
                             work_kind_name(dequeued_metadata.kind));
                    free_work_item(new_work, &dequeued_metadata);
                } else {
                    bool generation_changed =
                        dequeued_metadata.generation !=
                        current_work_pool_generation;
                    free_work_item(current_work,
                                   &current_work_metadata);
                    current_work = new_work;
                    current_work_metadata = dequeued_metadata;
                    current_work_pool_generation =
                        dequeued_metadata.generation;
                    if (is_bm1397_sv2_standard_channel(
                            GLOBAL_STATE,
                            current_work_metadata.kind)) {
                        sv2_version_cursor =
                            ((sv2_job_t *)current_work)->version;
                    } else {
                        sv2_version_cursor = 0;
                    }
                    sv2_ntime_offset = 0;
                    log_dequeued_work(current_work,
                                      current_work_metadata.kind);
                    saw_clean_jobs |= work_has_clean_jobs(
                        current_work, current_work_metadata.kind);
                    extranonce_2 = 0;
                    accepted_new_work = true;
                    if (generation_changed) {
                        has_dispatched_work = false;
                        last_dispatched_job_generation = 0;
                        prepared_job_generation = 0;
                        dispatch_pending = false;
                        residency_deadline_us = 0;
                        retry_not_before_us = 0;
                    }
                }
                new_work = queue_dequeue_timeout(
                    &GLOBAL_STATE->stratum_queue, 0,
                    &dequeued_metadata);
            } while (new_work != NULL);

            if (apply_pending_control_updates(GLOBAL_STATE, &difficulty)) {
                control_refresh_pending = true;
                if (!accepted_new_work && current_work != NULL &&
                    (is_bm1397_sv2_standard_channel(
                         GLOBAL_STATE, current_work_metadata.kind) ||
                     is_sv2_standard_channel(
                         GLOBAL_STATE, current_work_metadata.kind))) {
                    if (is_bm1397_sv2_standard_channel(
                            GLOBAL_STATE,
                            current_work_metadata.kind)) {
                        sv2_version_cursor =
                            ((sv2_job_t *)current_work)->version;
                    }
                    if (has_dispatched_work) {
                        sv2_ntime_offset++;
                    } else {
                        sv2_ntime_offset = 0;
                    }
                }
            }

            // A first job, clean job, or changed control value must not wait
            // for the ordinary residency deadline.
            if (current_work != NULL &&
                (!has_dispatched_work || saw_clean_jobs ||
                 control_refresh_pending ||
                 is_sv2_standard_channel(GLOBAL_STATE,
                                         current_work_metadata.kind))) {
                dispatch_pending = true;
                retry_not_before_us = 0;
            }
        } else if (apply_pending_control_updates(GLOBAL_STATE,
                                                 &difficulty)) {
            control_refresh_pending = true;
            if (current_work != NULL &&
                is_bm1397_sv2_standard_channel(
                    GLOBAL_STATE, current_work_metadata.kind)) {
                sv2_version_cursor =
                    ((sv2_job_t *)current_work)->version;
                if (has_dispatched_work) {
                    sv2_ntime_offset++;
                } else {
                    sv2_ntime_offset = 0;
                }
            } else if (current_work != NULL &&
                       is_sv2_standard_channel(
                           GLOBAL_STATE, current_work_metadata.kind)) {
                if (has_dispatched_work) {
                    sv2_ntime_offset++;
                } else {
                    sv2_ntime_offset = 0;
                }
            }
            if (current_work != NULL) {
                dispatch_pending = true;
                retry_not_before_us = 0;
            }
        }

        // Final subtype check before generating work. The active connection
        // may have switched while the queue wait or drain was in progress.
        active_kind = get_active_work_kind(GLOBAL_STATE);
        if (active_kind != observed_active_kind ||
            (current_work != NULL &&
             current_work_metadata.kind != active_kind)) {
            free_work_item(current_work, &current_work_metadata);
            current_work = NULL;
            current_work_metadata =
                (work_queue_item_metadata){0};
            observed_active_kind = active_kind;
            extranonce_2 = 0;
            residency_deadline_us = 0;
            retry_not_before_us = 0;
            has_dispatched_work = false;
            dispatch_pending = false;
            current_work_pool_generation =
                ASIC_result_task_get_pool_generation();
            last_dispatched_job_generation = 0;
            prepared_job_generation = 0;
            sv2_version_cursor = 0;
            sv2_ntime_offset = 0;
            continue;
        }

        if (current_work == NULL) {
            continue;
        }

        // Snapshot the ASIC-slot epoch first, then verify the pool epoch. If
        // clean work races these reads, either the pool check drops the
        // template or the locked ASIC send rejects the old slot epoch.
        uint32_t expected_job_generation =
            ASIC_result_task_get_job_generation();
        uint32_t live_pool_generation =
            ASIC_result_task_get_pool_generation();
        if (current_work_pool_generation != live_pool_generation) {
            ESP_LOGD(TAG,
                     "Discarding pool-invalidated work before ASIC dispatch");
            free_work_item(current_work, &current_work_metadata);
            current_work = NULL;
            current_work_metadata =
                (work_queue_item_metadata){0};
            current_work_pool_generation = live_pool_generation;
            last_dispatched_job_generation = 0;
            prepared_job_generation = 0;
            sv2_version_cursor = 0;
            sv2_ntime_offset = 0;
            residency_deadline_us = 0;
            retry_not_before_us = 0;
            has_dispatched_work = false;
            dispatch_pending = false;
            continue;
        }

        now_us = esp_timer_get_time();
        if (has_dispatched_work &&
            last_dispatched_job_generation != expected_job_generation) {
            // ASIC recovery invalidates device slots but not the pool
            // template. Re-feed that template immediately using the new slot
            // epoch instead of waiting for another pool notification.
            if (prepared_job_generation != expected_job_generation) {
                if (is_sv2_standard_channel(
                        GLOBAL_STATE, current_work_metadata.kind)) {
                    sv2_ntime_offset++;
                }
                prepared_job_generation = expected_job_generation;
            }
            dispatch_pending = true;
            retry_not_before_us = 0;
        }
        if (!dispatch_pending && has_dispatched_work &&
            !is_sv2_standard_channel(GLOBAL_STATE,
                                     current_work_metadata.kind) &&
            now_us >= residency_deadline_us) {
            dispatch_pending = true;
            retry_not_before_us = 0;
        }

        if (!dispatch_pending || now_us < retry_not_before_us) {
            continue;
        }

        if (!GLOBAL_STATE->ASIC_initalized) {
            // Recovery can leave the scheduler alive while the ASIC is
            // temporarily unavailable. Avoid repeated allocation, hashing,
            // and warning logs until it is ready again.
            retry_not_before_us =
                now_us + JOB_ASIC_NOT_READY_RETRY_US;
            continue;
        }

        uint32_t dispatch_version_mask =
            GLOBAL_STATE->version_mask;
        bool dispatched = dispatch_work(
            GLOBAL_STATE, current_work, current_work_metadata.kind,
            difficulty,
            extranonce_2, sv2_version_cursor, sv2_ntime_offset,
            dispatch_version_mask, expected_job_generation);
        if (!dispatched) {
            live_pool_generation =
                ASIC_result_task_get_pool_generation();
            if (live_pool_generation !=
                current_work_pool_generation) {
                ESP_LOGD(TAG,
                         "Discarding pool-invalidated work during ASIC dispatch");
                free_work_item(current_work, &current_work_metadata);
                current_work = NULL;
                current_work_metadata =
                    (work_queue_item_metadata){0};
                current_work_pool_generation =
                    live_pool_generation;
                last_dispatched_job_generation = 0;
                prepared_job_generation = 0;
                sv2_version_cursor = 0;
                sv2_ntime_offset = 0;
                residency_deadline_us = 0;
                retry_not_before_us = 0;
                has_dispatched_work = false;
                dispatch_pending = false;
                continue;
            }

            // Keep the old residency deadline. A transient allocation or UART
            // failure retries the newest coalesced work promptly instead of
            // introducing deadline drift or waiting a full interval.
            retry_not_before_us =
                esp_timer_get_time() + JOB_DISPATCH_RETRY_US;
            continue;
        }

        if (current_work_metadata.kind ==
                WORK_QUEUE_ITEM_STRATUM_V1 ||
            current_work_metadata.kind ==
                WORK_QUEUE_ITEM_STRATUM_V2_EXTENDED) {
            extranonce_2++;
        } else if (is_bm1397_sv2_standard_channel(
                       GLOBAL_STATE, current_work_metadata.kind)) {
            if (dispatch_version_mask == 0) {
                // With no rollable version bits, ntime is the only way to
                // avoid handing BM1397 the same nonce space again.
                sv2_ntime_offset++;
            } else {
                size_t transmitted_midstates =
                    version_mask_midstate_count(
                        dispatch_version_mask);
                for (size_t i = 0; i < transmitted_midstates; i++) {
                    sv2_version_cursor = increment_sv2_version_cursor(
                        sv2_version_cursor,
                        ((sv2_job_t *)current_work)->version,
                        dispatch_version_mask);
                }
                if (sv2_version_cursor ==
                    ((sv2_job_t *)current_work)->version) {
                    // The masked version space wrapped. Advance ntime before
                    // starting the version cycle again.
                    sv2_ntime_offset++;
                }
            }
        }

        has_dispatched_work = true;
        last_dispatched_job_generation = expected_job_generation;
        prepared_job_generation = expected_job_generation;
        dispatch_pending = false;
        control_refresh_pending = false;
        retry_not_before_us = 0;
        interval_us = get_job_interval_us(GLOBAL_STATE);
        residency_deadline_us = esp_timer_get_time() + interval_us;
    }
}

static bool generate_work(GlobalState *GLOBAL_STATE,
                          mining_notify *notification,
                          uint64_t extranonce_2, double difficulty,
                          uint32_t expected_generation)
{
    char *extranonce_1 = NULL;
    uint32_t extranonce_2_len = 0;
    if (!stratum_v1_snapshot_extranonce(
            GLOBAL_STATE, &extranonce_1, &extranonce_2_len)) {
        ESP_LOGE(TAG, "Unable to snapshot Stratum V1 extranonce");
        return false;
    }
    if (extranonce_2_len > MAX_EXTRANONCE2_LEN) {
        ESP_LOGE(TAG, "extranonce_2_len %lu exceeds maximum %d, skipping job",
                 (unsigned long)extranonce_2_len, MAX_EXTRANONCE2_LEN);
        free(extranonce_1);
        return false;
    }
    char extranonce_2_str[MAX_EXTRANONCE2_STR];
    extranonce_2_generate(extranonce_2, extranonce_2_len, extranonce_2_str);

    uint8_t coinbase_tx_hash[32];
    calculate_coinbase_tx_hash(notification->coinbase_1,
                               notification->coinbase_2, extranonce_1,
                               extranonce_2_str, coinbase_tx_hash);
    free(extranonce_1);

    uint8_t merkle_root[32];
    calculate_merkle_root_hash(coinbase_tx_hash, (uint8_t(*)[32])notification->merkle_branches, notification->n_merkle_branches, merkle_root);

    bm_job *next_job = allocate_bm_job(notification->job_id,
                                       extranonce_2_str);

    if (next_job == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for new job");
        return false;
    }

    construct_bm_job(notification, merkle_root, GLOBAL_STATE->version_mask, difficulty, next_job);
    next_job->version_mask = GLOBAL_STATE->version_mask;

    // Check if ASIC is initialized before trying to send work
    if (!GLOBAL_STATE->ASIC_initalized) {
        // Clean up the job since we're not sending it
        // Note: This job was never stored in active_jobs, so it's safe to free
        ESP_LOGW(TAG, "ASIC not initialized, skipping job send");
        release_bm_job(next_job);
        return false;
    }

    if (!ASIC_send_work(GLOBAL_STATE, next_job, expected_generation)) {
        release_bm_job(next_job);
        return false;
    }
    return true;
}

// Construct bm_job directly from SV2 fields (no coinbase/merkle computation
// needed). BM1366/68/70 roll versions internally; BM1397 receives successive
// four-midstate batches generated from the host-side version cursor.
static bool generate_work_sv2(GlobalState *GLOBAL_STATE, sv2_job_t *sv2_job,
                              double difficulty, uint32_t version_cursor,
                              uint32_t ntime_offset,
                              uint32_t version_mask,
                              uint32_t expected_generation)
{
    char jobid_str[16];
    snprintf(jobid_str, sizeof(jobid_str), "%" PRIu32, sv2_job->job_id);
    bm_job *next_job = allocate_bm_job(jobid_str, "");
    if (next_job == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for new SV2 job");
        return false;
    }

    uint32_t base_version = sv2_job->version;
    size_t midstate_count = version_mask != 0 ? 4 : 1;
    if (GLOBAL_STATE->DEVICE_CONFIG.family.asic.id == BM1397) {
        base_version = version_cursor;
        midstate_count =
            version_mask_midstate_count(version_mask);
    }

    next_job->version = base_version;
    next_job->target = sv2_job->nbits;
    next_job->ntime = sv2_job->ntime + ntime_offset;
    next_job->starting_nonce = 0;
    next_job->pool_diff = difficulty;

    // SV2 provides merkle_root and prev_hash in internal byte order (SHA-256 output order).
    // For bm_job storage: apply reverse_32bit_words (same as construct_bm_job does)
    reverse_32bit_words(sv2_job->merkle_root, next_job->merkle_root);
    reverse_32bit_words(sv2_job->prev_hash, next_job->prev_block_hash);

    // Compute midstate(s) using the same logic as construct_bm_job.
    // Midstate covers bytes 0-63 of block header: version(4B) + prev_hash(32B) + merkle_root[0:28](28B).
    uint8_t midstate_data[64];
    memcpy(midstate_data, &base_version, 4);
    memcpy(midstate_data + 4, sv2_job->prev_hash, 32);
    memcpy(midstate_data + 36, sv2_job->merkle_root, 28);

    uint8_t midstate[32];
    midstate_sha256_bin(midstate_data, 64, midstate);
    reverse_32bit_words(midstate, next_job->midstate);

    if (midstate_count == 4) {
        uint32_t rolled_version = increment_bitmask(base_version, version_mask);
        memcpy(midstate_data, &rolled_version, 4);
        midstate_sha256_bin(midstate_data, 64, midstate);
        reverse_32bit_words(midstate, next_job->midstate1);

        rolled_version = increment_bitmask(rolled_version, version_mask);
        memcpy(midstate_data, &rolled_version, 4);
        midstate_sha256_bin(midstate_data, 64, midstate);
        reverse_32bit_words(midstate, next_job->midstate2);

        rolled_version = increment_bitmask(rolled_version, version_mask);
        memcpy(midstate_data, &rolled_version, 4);
        midstate_sha256_bin(midstate_data, 64, midstate);
        reverse_32bit_words(midstate, next_job->midstate3);
        next_job->num_midstates = 4;
    } else {
        next_job->num_midstates = 1;
    }

    next_job->version_mask = version_mask;

    if (!GLOBAL_STATE->ASIC_initalized) {
        ESP_LOGW(TAG, "ASIC not initialized, skipping SV2 job send");
        release_bm_job(next_job);
        return false;
    }

    if (!ASIC_send_work(GLOBAL_STATE, next_job, expected_generation)) {
        release_bm_job(next_job);
        return false;
    }
    return true;
}

// Extended channel work generation: compute coinbase hash from prefix+extranonce+suffix,
// then merkle root from merkle path, then midstates. extranonce_2 provides unique work.
static bool generate_work_sv2_ext(GlobalState *GLOBAL_STATE, sv2_ext_job_t *ext_job,
                                  double difficulty,
                                  uint64_t extranonce_2_counter,
                                  uint32_t expected_generation)
{
    stratum_v2_extended_work_snapshot_t connection = {0};
    if (!stratum_v2_snapshot_extended_work(
            GLOBAL_STATE, expected_generation, &connection)) {
        return false;
    }
    if (connection.extranonce_size > 32) {
        ESP_LOGE(TAG, "SV2 extranonce size %u exceeds 32-byte work buffer",
                 connection.extranonce_size);
        return false;
    }

    uint32_t version_mask = GLOBAL_STATE->version_mask;

    // Derive extranonce_2 from counter
    // SV2 spec: extranonce_size is the miner's rollable portion (not total)
    uint8_t extranonce_2_len = connection.extranonce_size;
    uint8_t extranonce_2[32];
    memset(extranonce_2, 0, sizeof(extranonce_2));
    // Encode counter as big-endian bytes
    for (int i = extranonce_2_len - 1; i >= 0 && extranonce_2_counter > 0; i--) {
        extranonce_2[i] = (uint8_t)(extranonce_2_counter & 0xFF);
        extranonce_2_counter >>= 8;
    }

    char jobid_str[16];
    snprintf(jobid_str, sizeof(jobid_str), "%" PRIu32, ext_job->job_id);
    char en2_hex[65];
    bin2hex(extranonce_2, extranonce_2_len, en2_hex, sizeof(en2_hex));
    bm_job *next_job = allocate_bm_job(jobid_str, en2_hex);
    if (!next_job) {
        ESP_LOGE(TAG, "Failed to allocate memory for SV2 ext job");
        return false;
    }

    // Compute coinbase tx hash: prefix + extranonce_prefix + extranonce_2 + suffix
    uint8_t coinbase_tx_hash[32];
    calculate_coinbase_tx_hash_bin(
        ext_job->coinbase_prefix, ext_job->coinbase_prefix_len,
        connection.extranonce_prefix, connection.extranonce_prefix_len,
        extranonce_2, extranonce_2_len,
        ext_job->coinbase_suffix, ext_job->coinbase_suffix_len,
        coinbase_tx_hash);

    // Compute merkle root
    uint8_t merkle_root[32];
    calculate_merkle_root_hash(coinbase_tx_hash,
                               (const uint8_t (*)[32])ext_job->merkle_path,
                               ext_job->merkle_path_count, merkle_root);

    // Fill bm_job fields
    next_job->version = ext_job->version;
    next_job->target = ext_job->nbits;
    next_job->ntime = ext_job->ntime;  // no offset — extranonce provides uniqueness
    next_job->starting_nonce = 0;
    next_job->pool_diff = difficulty;

    // Same byte-order handling as generate_work_sv2
    reverse_32bit_words(merkle_root, next_job->merkle_root);
    reverse_32bit_words(ext_job->prev_hash, next_job->prev_block_hash);

    // Compute midstate(s)
    uint8_t midstate_data[64];
    uint32_t base_version = ext_job->version;
    memcpy(midstate_data, &base_version, 4);
    memcpy(midstate_data + 4, ext_job->prev_hash, 32);
    memcpy(midstate_data + 36, merkle_root, 28);

    uint8_t midstate[32];
    midstate_sha256_bin(midstate_data, 64, midstate);
    reverse_32bit_words(midstate, next_job->midstate);

    size_t midstate_count = version_mask != 0 ? 4 : 1;
    if (GLOBAL_STATE->DEVICE_CONFIG.family.asic.id == BM1397) {
        midstate_count =
            version_mask_midstate_count(version_mask);
    }

    if (midstate_count == 4) {
        uint32_t rolled_version = increment_bitmask(base_version, version_mask);
        memcpy(midstate_data, &rolled_version, 4);
        midstate_sha256_bin(midstate_data, 64, midstate);
        reverse_32bit_words(midstate, next_job->midstate1);

        rolled_version = increment_bitmask(rolled_version, version_mask);
        memcpy(midstate_data, &rolled_version, 4);
        midstate_sha256_bin(midstate_data, 64, midstate);
        reverse_32bit_words(midstate, next_job->midstate2);

        rolled_version = increment_bitmask(rolled_version, version_mask);
        memcpy(midstate_data, &rolled_version, 4);
        midstate_sha256_bin(midstate_data, 64, midstate);
        reverse_32bit_words(midstate, next_job->midstate3);
        next_job->num_midstates = 4;
    } else {
        next_job->num_midstates = 1;
    }

    next_job->version_mask = version_mask;

    if (!GLOBAL_STATE->ASIC_initalized) {
        ESP_LOGW(TAG, "ASIC not initialized, skipping SV2 ext job send");
        release_bm_job(next_job);
        return false;
    }

    if (!ASIC_send_work(GLOBAL_STATE, next_job, expected_generation)) {
        release_bm_job(next_job);
        return false;
    }
    return true;
}
