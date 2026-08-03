#ifndef POWER_MANAGEMENT_TASK_H_
#define POWER_MANAGEMENT_TASK_H_

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct GlobalState GlobalState;

typedef struct
{
    float fan_perc;
    uint16_t fan_rpm;
    uint16_t fan2_rpm;
    float chip_temp_avg;
    float chip_temp2_avg;
    bool chip_temp_valid;
    bool chip_temp2_valid;
    uint32_t chip_temp_age_ms;
    uint32_t chip_temp2_age_ms;
    float vr_temp;
    float voltage;
    uint16_t requested_voltage_mv;
    float frequency_value;
    float actual_frequency;
    float requested_frequency;
    float thermal_frequency_cap;
    bool thermal_throttled;
    float expected_hashrate;
    float power;
    float current;
    float core_voltage;
    uint32_t asic_response_age_ms;
    uint32_t asic_progress_age_ms;
    bool startup_preflight_complete;
    bool startup_preflight_succeeded;
} PowerManagementModule;

void POWER_MANAGEMENT_init_frequency(GlobalState * GLOBAL_STATE);

esp_err_t POWER_MANAGEMENT_request_frequency(GlobalState *GLOBAL_STATE,
                                             float frequency_mhz);
esp_err_t POWER_MANAGEMENT_request_voltage(GlobalState *GLOBAL_STATE,
                                           uint16_t voltage_mv);
bool POWER_MANAGEMENT_wait_for_preflight(GlobalState *GLOBAL_STATE,
                                         uint32_t timeout_ms);

void POWER_MANAGEMENT_task(void * pvParameters);

#endif
