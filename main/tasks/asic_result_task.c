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
#include "sv2_protocol.h"
#include "hashrate_monitor_task.h"
#include "asic.h"
#include "freertos/task.h"
#include "scoreboard.h"
#include "self_test.h"
#include "freertos/queue.h"

static const char *TAG = "asic_result";

#define ASIC_RESULT_QUEUE_LENGTH 64
#define ASIC_RESULT_DEDUP_CACHE_SIZE 64

typedef struct
{
    task_result result;
    bm_job job;
    bool has_job;
    uint32_t generation;
    stratum_protocol_t protocol;
} queued_asic_result_t;

typedef struct
{
    uint64_t job_fingerprint;
    uint32_t generation;
    uint32_t nonce;
    uint32_t rolled_version;
    uint8_t job_id;
    bool valid;
} recent_asic_result_t;

static QueueHandle_t asic_result_queue;
static atomic_uint_fast32_t job_generation;
static atomic_uint_fast32_t pool_generation;
static atomic_uint_fast32_t enqueued_result_count;
static atomic_uint_fast32_t processed_result_count;
static atomic_uint_fast32_t dropped_result_count;
static atomic_uint_fast32_t invalid_job_count;
static atomic_uint_fast32_t stale_result_count;
static atomic_uint_fast32_t duplicate_result_count;
static atomic_uint_fast32_t metadata_failure_count;
static atomic_uint_fast32_t queue_high_watermark;
static atomic_uint_fast32_t max_queue_latency_ms;
static recent_asic_result_t recent_results[ASIC_RESULT_DEDUP_CACHE_SIZE];
static size_t recent_result_index;

static void free_queued_result(queued_asic_result_t *queued_result)
{
    if (queued_result == NULL || !queued_result->has_job) {
        return;
    }

    free(queued_result->job.jobid);
    free(queued_result->job.extranonce2);
    queued_result->job.jobid = NULL;
    queued_result->job.extranonce2 = NULL;
    queued_result->has_job = false;
}

static void drain_result_queue(void)
{
    if (asic_result_queue == NULL) {
        return;
    }

    queued_asic_result_t queued_result;
    while (xQueueReceive(asic_result_queue, &queued_result, 0) == pdTRUE) {
        free_queued_result(&queued_result);
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
    uint64_t job_fingerprint = fingerprint_job(&queued_result->job);

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

static void log_result_metrics(void)
{
    static int64_t last_log_us;
    int64_t now_us = esp_timer_get_time();
    if (last_log_us != 0 && now_us - last_log_us < 60000000) {
        return;
    }
    last_log_us = now_us;

    asic_rx_stats_t rx_stats;
    get_work_rx_stats(&rx_stats);
    ESP_LOGI(TAG,
             "RX metrics: frames=%lu crc=%lu discarded=%lu timeouts=%lu uart_err=%lu "
             "queued=%lu processed=%lu dropped=%lu stale=%lu duplicate=%lu "
             "invalid=%lu metadata_err=%lu queue_peak=%lu latency_max=%lums",
             (unsigned long)rx_stats.frames_received,
             (unsigned long)rx_stats.crc_errors,
             (unsigned long)rx_stats.discarded_bytes,
             (unsigned long)rx_stats.timeouts,
             (unsigned long)rx_stats.uart_errors,
             (unsigned long)atomic_load(&enqueued_result_count),
             (unsigned long)atomic_load(&processed_result_count),
             (unsigned long)atomic_load(&dropped_result_count),
             (unsigned long)atomic_load(&stale_result_count),
             (unsigned long)atomic_load(&duplicate_result_count),
             (unsigned long)atomic_load(&invalid_job_count),
             (unsigned long)atomic_load(&metadata_failure_count),
             (unsigned long)atomic_load(&queue_high_watermark),
             (unsigned long)atomic_load(&max_queue_latency_ms));
}

esp_err_t ASIC_result_task_init(void)
{
    if (asic_result_queue != NULL) {
        return ESP_OK;
    }

    asic_result_queue = xQueueCreate(ASIC_RESULT_QUEUE_LENGTH, sizeof(queued_asic_result_t));
    if (asic_result_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create ASIC result queue");
        return ESP_ERR_NO_MEM;
    }

    atomic_init(&job_generation, 1);
    atomic_init(&pool_generation, 1);
    atomic_init(&enqueued_result_count, 0);
    atomic_init(&processed_result_count, 0);
    atomic_init(&dropped_result_count, 0);
    atomic_init(&invalid_job_count, 0);
    atomic_init(&stale_result_count, 0);
    atomic_init(&duplicate_result_count, 0);
    atomic_init(&metadata_failure_count, 0);
    atomic_init(&queue_high_watermark, 0);
    atomic_init(&max_queue_latency_ms, 0);
    memset(recent_results, 0, sizeof(recent_results));
    recent_result_index = 0;
    return ESP_OK;
}

void ASIC_result_task_invalidate_jobs(void)
{
    atomic_fetch_add(&job_generation, 1);
    drain_result_queue();
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
        if (!GLOBAL_STATE->ASIC_initalized) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            log_result_metrics();
            continue;
        }

        task_result *asic_result = ASIC_process_work(GLOBAL_STATE);

        if (asic_result == NULL) {
            log_result_metrics();
            continue;
        }

        queued_asic_result_t queued_result = {
            .result = *asic_result,
            .has_job = false,
        };

        if (asic_result->register_type == REGISTER_INVALID) {
            uint8_t job_id = asic_result->job_id;

            // Snapshot the job before enqueueing. A queued result can outlive the
            // ASIC's job-ID cycle, so resolving the slot later could pair the
            // nonce with a newer job that reused the same ID.
            pthread_mutex_lock(&GLOBAL_STATE->valid_jobs_lock);
            bool valid = (GLOBAL_STATE->valid_jobs[job_id] != 0) &&
                         (GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs[job_id] != NULL);
            if (valid) {
                const bm_job *active_job =
                    GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs[job_id];
                queued_result.job = *active_job;
                queued_result.job.jobid = queued_result.job.jobid ? strdup(queued_result.job.jobid) : NULL;
                queued_result.job.extranonce2 = queued_result.job.extranonce2 ? strdup(queued_result.job.extranonce2) : NULL;
                queued_result.has_job = true;
                queued_result.generation =
                    (uint32_t)atomic_load(&job_generation);
                queued_result.protocol = GLOBAL_STATE->stratum_protocol;

                // Resolve the rolled version from the same locked job snapshot
                // used for nonce validation. This avoids pairing version data
                // from an old slot with metadata from a newly reused slot.
                if (GLOBAL_STATE->DEVICE_CONFIG.family.asic.id == BM1397) {
                    queued_result.result.rolled_version =
                        queued_result.job.version;
                    for (uint8_t i = 0;
                         i < queued_result.result.version_rolling_index; i++) {
                        queued_result.result.rolled_version = increment_bitmask(
                            queued_result.result.rolled_version,
                            queued_result.job.version_mask);
                    }
                } else {
                    queued_result.result.rolled_version =
                        queued_result.job.version |
                        queued_result.result.version_bits;
                }
            }
            pthread_mutex_unlock(&GLOBAL_STATE->valid_jobs_lock);

            if (!valid) {
                atomic_fetch_add(&invalid_job_count, 1);
                ESP_LOGD(TAG, "Invalid job nonce found, 0x%02X", job_id);
                continue;
            }
            if (queued_result.job.jobid == NULL || queued_result.job.extranonce2 == NULL) {
                atomic_fetch_add(&metadata_failure_count, 1);
                ESP_LOGE(TAG, "Failed to snapshot metadata for job 0x%02X", job_id);
                free_queued_result(&queued_result);
                continue;
            }
        } else {
            queued_result.generation =
                (uint32_t)atomic_load(&job_generation);
            queued_result.protocol = GLOBAL_STATE->stratum_protocol;
        }

        if (xQueueSend(asic_result_queue, &queued_result, 0) != pdTRUE) {
            free_queued_result(&queued_result);
            uint32_t dropped = (uint32_t)atomic_fetch_add(&dropped_result_count, 1) + 1;
            // Log at powers of two so a stalled consumer cannot create a log storm.
            if ((dropped & (dropped - 1)) == 0) {
                ESP_LOGW(TAG, "ASIC result queue full; dropped %lu result(s)",
                         (unsigned long)dropped);
            }
        } else {
            atomic_fetch_add(&enqueued_result_count, 1);
            update_atomic_max(&queue_high_watermark,
                              (uint32_t)uxQueueMessagesWaiting(asic_result_queue));
        }
        log_result_metrics();
    }
}

void ASIC_result_task(void *pvParameters)
{
    GlobalState *GLOBAL_STATE = (GlobalState *)pvParameters;
    queued_asic_result_t queued_result;

    while (1)
    {
        if (xQueueReceive(asic_result_queue, &queued_result, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        task_result *asic_result = &queued_result.result;
        atomic_fetch_add(&processed_result_count, 1);

        uint64_t latency_us = (uint64_t)esp_timer_get_time() - asic_result->timestamp_us;
        uint32_t latency_ms = latency_us / 1000 > UINT32_MAX
                                  ? UINT32_MAX
                                  : (uint32_t)(latency_us / 1000);
        update_atomic_max(&max_queue_latency_ms, latency_ms);

        if (asic_result->register_type != REGISTER_INVALID) {
            hashrate_monitor_register_read(GLOBAL_STATE, asic_result->register_type, asic_result->asic_nr, asic_result->value, asic_result->timestamp_us);
            continue;
        }

        if (queued_result.generation != (uint32_t)atomic_load(&job_generation)) {
            atomic_fetch_add(&stale_result_count, 1);
            free_queued_result(&queued_result);
            continue;
        }

        if (is_duplicate_result(&queued_result)) {
            atomic_fetch_add(&duplicate_result_count, 1);
            free_queued_result(&queued_result);
            continue;
        }

        bm_job *active_job = &queued_result.job;
        // check the nonce difficulty
        double nonce_diff = test_nonce_value(active_job, asic_result->nonce, asic_result->rolled_version);

        if (GLOBAL_STATE->SELF_TEST_MODULE.is_active) {
            self_test_record_nonce(GLOBAL_STATE, nonce_diff);
            free_queued_result(&queued_result);
            continue;
        }

        uint32_t version_bits = asic_result->rolled_version ^ active_job->version;
        if (queued_result.generation != (uint32_t)atomic_load(&job_generation)) {
            atomic_fetch_add(&stale_result_count, 1);
            free_queued_result(&queued_result);
            continue;
        }

        if (nonce_diff >= active_job->pool_diff)
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

                if (ret < 0) {
                    ESP_LOGW(TAG, "Failed to submit SV2 share (ret=%d, errno=%d: %s)",
                             ret, errno, strerror(errno));
                }
            } else {
                // V1: submit with JSON-RPC
                uint16_t active_idx = GLOBAL_STATE->SYSTEM_MODULE.is_using_fallback ? GLOBAL_STATE->SYSTEM_MODULE.secondary_pool_index : GLOBAL_STATE->SYSTEM_MODULE.primary_pool_index;
                char * user = GLOBAL_STATE->SYSTEM_MODULE.pools[active_idx].user;

                taskENTER_CRITICAL(&GLOBAL_STATE->stratum_mux);
                int uid = GLOBAL_STATE->send_uid++;
                taskEXIT_CRITICAL(&GLOBAL_STATE->stratum_mux);

                uint64_t sent_time_us = 0;
                int ret = stratum_v1_submit_share_safe(
                    GLOBAL_STATE, queued_result.generation, uid, user,
                    active_job->jobid, active_job->extranonce2,
                    active_job->ntime, asic_result->nonce, version_bits,
                    &sent_time_us);

                if (ret < 0) {
                    ESP_LOGW(TAG, "Unable to write share to socket (ret: %d, errno %d: %s)", ret, errno, strerror(errno));
                    // stratum_task recv loop will detect a broken connection on its next read and handle reconnection
                } else {
                    float process_time = (sent_time_us - asic_result->timestamp_us) / 1000.0f;
                    GLOBAL_STATE->SYSTEM_MODULE.process_time = process_time;
                    ESP_LOGD(TAG, "Processing time: %0.1f ms", process_time);
                }
            }
        }

        //log the ASIC response
        ESP_LOGD(TAG, "ID: %s, ASIC nr: %d, Core: %d/%d, ver: %08" PRIX32 " Nonce %08" PRIX32 " diff %.1f of %g.", active_job->jobid, asic_result->asic_nr, asic_result->core_id, asic_result->small_core_id, asic_result->rolled_version, asic_result->nonce, nonce_diff, active_job->pool_diff);

        SYSTEM_notify_found_nonce(GLOBAL_STATE, nonce_diff, active_job->target);

        scoreboard_add(&GLOBAL_STATE->SYSTEM_MODULE.scoreboard, nonce_diff, active_job->jobid, active_job->extranonce2, active_job->ntime, asic_result->nonce, version_bits);

        free_queued_result(&queued_result);
    }
}
