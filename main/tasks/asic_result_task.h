#ifndef ASIC_result_TASK_H_
#define ASIC_result_TASK_H_

#include <stdint.h>
#include "esp_err.h"

typedef struct {
    uint32_t registers_processed;
    uint32_t nonces_enqueued;
    uint32_t nonces_processed;
    uint32_t nonces_dropped;
    uint32_t nonce_queue_high_watermark;
    uint32_t nonce_max_latency_ms;
    uint32_t invalid_jobs;
    uint32_t ambiguous_jobs;
    uint32_t stale_results;
    uint32_t duplicate_results;
    uint32_t shares_enqueued;
    uint32_t shares_submitted;
    uint32_t shares_dropped;
    uint32_t stale_shares;
    uint32_t share_queue_high_watermark;
} asic_result_stats_t;

esp_err_t ASIC_result_task_init(void);
void ASIC_result_task_reset(void);
void ASIC_result_task_invalidate_jobs(void);
void ASIC_result_task_invalidate_pool_jobs(void);
uint32_t ASIC_result_task_get_job_generation(void);
uint32_t ASIC_result_task_get_pool_generation(void);
void ASIC_result_task_get_stats(asic_result_stats_t *stats);
void ASIC_result_rx_task(void *pvParameters);
void ASIC_result_task(void *pvParameters);
void ASIC_v1_share_submit_task(void *pvParameters);

#endif
