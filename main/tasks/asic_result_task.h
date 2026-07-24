#ifndef ASIC_result_TASK_H_
#define ASIC_result_TASK_H_

#include <stdint.h>
#include "esp_err.h"

esp_err_t ASIC_result_task_init(void);
void ASIC_result_task_reset(void);
void ASIC_result_task_invalidate_jobs(void);
void ASIC_result_task_invalidate_pool_jobs(void);
uint32_t ASIC_result_task_get_job_generation(void);
uint32_t ASIC_result_task_get_pool_generation(void);
void ASIC_result_rx_task(void *pvParameters);
void ASIC_result_task(void *pvParameters);

#endif
