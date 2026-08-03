#include "thermal.h"
#include "device_config.h"
#include "global_state.h"

#include <math.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"

#include "EMC2101.h"
#include "EMC2103.h"
#include "EMC2302.h"
#include "TMP1075.h"
#include "asic_init.h"

static const char * TAG = "thermal";

typedef struct {
    float last_value;
    int64_t last_valid_us;
    bool has_value;
} sensor_history_t;

static sensor_history_t chip_sensor_history[2];

esp_err_t Thermal_init(DeviceConfig * DEVICE_CONFIG)
{
    chip_sensor_history[0] = (sensor_history_t){0};
    chip_sensor_history[1] = (sensor_history_t){0};
    if (DEVICE_CONFIG->EMC2101) {
        ESP_RETURN_ON_ERROR(EMC2101_init(DEVICE_CONFIG->temp_offset), TAG, "Failed to initialise EMC2101");
        // TODO: Improve this check.
        if (DEVICE_CONFIG->emc_ideality_factor != 0x00) {
            ESP_LOGI(TAG, "EMC2101 configuration: Ideality Factor: %02x, Beta Compensation: %02x", DEVICE_CONFIG->emc_ideality_factor, DEVICE_CONFIG->emc_beta_compensation);
            EMC2101_set_ideality_factor(DEVICE_CONFIG->emc_ideality_factor);
            EMC2101_set_beta_compensation(DEVICE_CONFIG->emc_beta_compensation);
        }
    }
    if (DEVICE_CONFIG->EMC2103) {
        ESP_RETURN_ON_ERROR(EMC2103_init(DEVICE_CONFIG->temp_offset, DEVICE_CONFIG->temp_flip), TAG, "Failed to initialise EMC2103");
    }
    if (DEVICE_CONFIG->EMC2302) {
        ESP_RETURN_ON_ERROR(EMC2302_init(), TAG, "Failed to initialise EMC2302");
    }
    if (DEVICE_CONFIG->TMP1075) {
        ESP_RETURN_ON_ERROR(TMP1075_init(DEVICE_CONFIG->temp_offset), TAG, "Failed to initialise TMP1075");
    }

    return ESP_OK;
}

//percent is a float between 0.0 and 1.0
esp_err_t Thermal_set_fan_percent(DeviceConfig * DEVICE_CONFIG, float percent)
{
    if (DEVICE_CONFIG->EMC2101) {
        return EMC2101_set_fan_speed(percent);
    }
    if (DEVICE_CONFIG->EMC2103) {
        return EMC2103_set_fan_speed(percent);
    }
    if (DEVICE_CONFIG->EMC2302) {
        return EMC2302_set_fan_speed(percent);
    }
    return ESP_OK;
}

uint16_t Thermal_get_fan_speed(DeviceConfig * DEVICE_CONFIG) 
{
    if (DEVICE_CONFIG->EMC2101) {
        return EMC2101_get_fan_speed();
    }
    if (DEVICE_CONFIG->EMC2103) {
        return EMC2103_get_fan_speed();
    }
    if (DEVICE_CONFIG->EMC2302) {
        return EMC2302_get_fan_speed();
    }
    return 0;
}

uint16_t Thermal_get_fan2_speed(DeviceConfig * DEVICE_CONFIG) 
{
    if (DEVICE_CONFIG->EMC2302) {
        return EMC2302_get_fan2_speed();
    }
    return 0;
}

bool Thermal_has_second_chip_sensor(const DeviceConfig *DEVICE_CONFIG)
{
    return DEVICE_CONFIG->TMP1075 || DEVICE_CONFIG->EMC2103;
}

bool Thermal_chip_sensors_available_when_stopped(const DeviceConfig *DEVICE_CONFIG)
{
    return DEVICE_CONFIG->TMP1075 ||
           (DEVICE_CONFIG->EMC2101 && DEVICE_CONFIG->emc_internal_temp);
}

static float read_chip_sensor(GlobalState *GLOBAL_STATE, unsigned int sensor_index,
                              bool *available)
{
    const DeviceConfig *config = &GLOBAL_STATE->DEVICE_CONFIG;
    *available = false;

    if (sensor_index > 1 ||
        (sensor_index == 1 && !Thermal_has_second_chip_sensor(config))) {
        return NAN;
    }

    if (config->TMP1075) {
        *available = true;
        return TMP1075_read_temperature((int)sensor_index);
    }

    if (config->EMC2101) {
        if (sensor_index != 0) {
            return NAN;
        }
        if (config->emc_internal_temp) {
            *available = true;
            return EMC2101_get_internal_temp();
        }
        if (asic_lifecycle_is_running(GLOBAL_STATE)) {
            *available = true;
            return EMC2101_get_external_temp();
        }
        return NAN;
    }

    if (config->EMC2103 && asic_lifecycle_is_running(GLOBAL_STATE)) {
        *available = true;
        return sensor_index == 0 ? EMC2103_get_external_temp()
                                 : EMC2103_get_external_temp2();
    }

    return NAN;
}

thermal_reading_t Thermal_get_chip_temp_reading(GlobalState *GLOBAL_STATE,
                                                 unsigned int sensor_index)
{
    thermal_reading_t result = {
        .value = NAN,
        .valid = false,
        .available = false,
        .age_ms = UINT32_MAX,
    };
    if (sensor_index > 1) {
        return result;
    }

    bool available = false;
    float raw = read_chip_sensor(GLOBAL_STATE, sensor_index, &available);
    int64_t now_us = esp_timer_get_time();
    sensor_history_t *history = &chip_sensor_history[sensor_index];

    result.available = available;
    // Temperatures outside the physical range of these devices are fault
    // encodings or corrupt transfers, never useful control inputs.
    if (available && isfinite(raw) && raw >= -40.0f && raw <= 150.0f) {
        history->last_value = raw;
        history->last_valid_us = now_us;
        history->has_value = true;
        result.value = raw;
        result.valid = true;
        result.age_ms = 0;
        return result;
    }

    if (history->has_value) {
        result.value = history->last_value;
        uint64_t age_ms = (uint64_t)(now_us - history->last_valid_us) / 1000ULL;
        result.age_ms = age_ms > UINT32_MAX ? UINT32_MAX : (uint32_t)age_ms;
    }
    return result;
}

float Thermal_get_chip_temp(GlobalState * GLOBAL_STATE)
{
    thermal_reading_t reading = Thermal_get_chip_temp_reading(GLOBAL_STATE, 0);
    return reading.valid ? reading.value : -1.0f;
}

float Thermal_get_chip_temp2(GlobalState * GLOBAL_STATE)
{
    thermal_reading_t reading = Thermal_get_chip_temp_reading(GLOBAL_STATE, 1);
    return reading.valid ? reading.value : -1.0f;
}
