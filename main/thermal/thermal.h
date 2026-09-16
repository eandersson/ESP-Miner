#ifndef THERMAL_H
#define THERMAL_H

#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct GlobalState GlobalState;
typedef struct DeviceConfig DeviceConfig;

esp_err_t Thermal_init(DeviceConfig * DEVICE_CONFIG);
esp_err_t Thermal_set_fan_percent(DeviceConfig * DEVICE_CONFIG, float percent);
uint16_t Thermal_get_fan_speed(DeviceConfig * DEVICE_CONFIG);
uint16_t Thermal_get_fan2_speed(DeviceConfig * DEVICE_CONFIG);

typedef struct {
    float value;
    bool valid;
    bool available;
    uint32_t age_ms;
} thermal_reading_t;

float Thermal_get_chip_temp(GlobalState * GLOBAL_STATE);
float Thermal_get_chip_temp2(GlobalState * GLOBAL_STATE);
thermal_reading_t Thermal_get_chip_temp_reading(GlobalState *GLOBAL_STATE, unsigned int sensor_index);
bool Thermal_has_second_chip_sensor(const DeviceConfig *DEVICE_CONFIG);
bool Thermal_chip_sensors_available_when_stopped(const DeviceConfig *DEVICE_CONFIG);

#endif // THERMAL_H
