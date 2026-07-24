#ifndef ASIC_result_TASK_H_
#define ASIC_result_TASK_H_

#include "esp_err.h"

esp_err_t ASIC_result_task_init(void);
void ASIC_result_task_reset(void);
void ASIC_result_task_invalidate_jobs(void);
void ASIC_result_rx_task(void *pvParameters);
void ASIC_result_task(void *pvParameters);

#endif
