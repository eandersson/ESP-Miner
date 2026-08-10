#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#include "global_state.h"
#include "nvs_config.h"
#include "vcore.h"
#include "thermal.h"
#include "power.h"
#include "asic.h"
#include "utils.h"
#include "asic_init.h"
#include "asic_reset.h"
#include "serial.h"
#include "hashrate_monitor_task.h"

#define POLL_RATE_MS 100

#define ASIC_MIN_FREQUENCY_MHZ 50.0f
#define ASIC_MAX_FREQUENCY_MHZ 800.0f
#define THROTTLE_TEMP_C 75.0f
#define THROTTLE_RELEASE_TEMP_C 70.0f
#define HARD_MAX_TEMP_C 90.0f
#define SAFE_TEMP_C 45.0f
#define TPS546_THROTTLE_TEMP_C 105.0f
#define TPS546_RELEASE_TEMP_C 95.0f
#define TPS546_MAX_TEMP_C 145.0f

#define THROTTLE_STEP_MHZ 25.0f
#define THROTTLE_INTERVAL_MS 5000U
#define THROTTLE_RELEASE_INTERVAL_MS 30000U
#define THROTTLE_RELEASE_FAST_INTERVAL_MS 10000U
#define THROTTLE_RELEASE_FAST_TEMP_C 68.0f
#define TPS546_RELEASE_FAST_TEMP_C 90.0f
#define THROTTLE_WARM_RESET_SAMPLES 20U
#define HARD_THERMAL_REDUCTION_MHZ 100.0f

#define TEMP_FAILURE_TIMEOUT_MS 3000U
#define COOLING_SAMPLE_MS 5000U
#define MIN_COOLING_CYCLES 6U

#define VCORE_TOLERANCE_MIN_MV 200
#define VCORE_TOLERANCE_PERCENT 12
#define VCORE_VERIFY_ATTEMPTS 3
#define VCORE_VERIFY_DELAY_MS 75
#define ASIC_TX_DRAIN_TIMEOUT_MS 100U

#define LIVENESS_START_GRACE_MS 30000U
#define LIVENESS_RESPONSE_TIMEOUT_MS 15000U
#define LIVENESS_PROGRESS_TIMEOUT_MS 60000U

#define RECOVERY_BASE_DELAY_MS 2000U
#define RECOVERY_MAX_DELAY_MS 60000U
#define RECOVERY_STABLE_RESET_MS 300000U
#define MAX_ASIC_OPTIONS 64U

static const char *TAG = "power_management";
static float last_rejected_frequency = NAN;
static bool have_last_rejected_frequency;
static int32_t last_rejected_voltage = -1;
static pthread_mutex_t request_mutex = PTHREAD_MUTEX_INITIALIZER;

typedef enum {
    RECOVERY_NONE = 0,
    RECOVERY_START_FAILURE,
    RECOVERY_POWER_FAULT,
    RECOVERY_SENSOR_FAULT,
    RECOVERY_ASIC_NO_RESPONSE,
    RECOVERY_ASIC_NO_PROGRESS,
    RECOVERY_HARD_OVERHEAT,
} recovery_reason_t;

typedef struct {
    uint16_t applied_voltage_mv;
    float applied_frequency_mhz;
    bool stopped_for_request;
    bool cold_boot_complete;
    bool recovery_pending;
    bool thermal_cooldown;
    recovery_reason_t recovery_reason;
    uint8_t recovery_attempts;
    TickType_t recovery_not_before;
    TickType_t stable_since;
    TickType_t invalid_temp_since;
    TickType_t throttle_last_step;
    TickType_t throttle_last_cool_sample;
    uint32_t throttle_cool_accumulated_ms;
    uint32_t throttle_fast_cool_accumulated_ms;
    unsigned int throttle_warm_samples;
    bool throttle_last_cool_was_fast;
    TickType_t next_cooling_sample;
    unsigned int cooling_cycles;
    unsigned int vcore_read_failures;
} power_control_t;

static bool ticks_elapsed(TickType_t now, TickType_t since, uint32_t interval_ms)
{
    return since != 0 && pdTICKS_TO_MS(now - since) >= interval_ms;
}

static uint32_t recovery_delay_ms(uint8_t attempt)
{
    uint8_t shift = attempt > 5 ? 5 : attempt;
    uint32_t delay = RECOVERY_BASE_DELAY_MS << shift;
    return delay > RECOVERY_MAX_DELAY_MS ? RECOVERY_MAX_DELAY_MS : delay;
}

static const char *recovery_reason_name(recovery_reason_t reason)
{
    switch (reason) {
        case RECOVERY_START_FAILURE: return "ASIC start failure";
        case RECOVERY_POWER_FAULT: return "regulator fault";
        case RECOVERY_SENSOR_FAULT: return "temperature sensor fault";
        case RECOVERY_ASIC_NO_RESPONSE: return "ASIC response timeout";
        case RECOVERY_ASIC_NO_PROGRESS: return "ASIC counter timeout";
        case RECOVERY_HARD_OVERHEAT: return "hard thermal limit";
        default: return "unspecified";
    }
}

static float expected_hashrate(GlobalState *GLOBAL_STATE)
{
    return GLOBAL_STATE->POWER_MANAGEMENT_MODULE.frequency_value *
           GLOBAL_STATE->DEVICE_CONFIG.family.asic.small_core_count *
           GLOBAL_STATE->DEVICE_CONFIG.family.asic_count / 1000.0f;
}

static bool option_bounds(const uint16_t *options, uint16_t *minimum,
                          uint16_t *maximum)
{
    if (options == NULL || options[0] == 0) {
        return false;
    }
    *minimum = options[0];
    *maximum = options[0];
    for (size_t i = 1; i < MAX_ASIC_OPTIONS && options[i] != 0; i++) {
        if (options[i] < *minimum) *minimum = options[i];
        if (options[i] > *maximum) *maximum = options[i];
    }
    return true;
}

static bool exact_option(const uint16_t *options, float value)
{
    if (options == NULL) {
        return false;
    }
    for (size_t i = 0; i < MAX_ASIC_OPTIONS && options[i] != 0; i++) {
        if (fabsf(value - (float)options[i]) < 0.001f) {
            return true;
        }
    }
    return false;
}

static bool frequency_request_valid(GlobalState *GLOBAL_STATE, float value)
{
    const AsicConfig *asic = &GLOBAL_STATE->DEVICE_CONFIG.family.asic;
    uint16_t minimum = 0;
    uint16_t maximum = 0;
    bool valid = isfinite(value) && value >= ASIC_MIN_FREQUENCY_MHZ &&
                 value <= ASIC_MAX_FREQUENCY_MHZ &&
                 option_bounds(asic->frequency_options, &minimum, &maximum) &&
                 value >= minimum && value <= maximum;
    if (valid && !nvs_config_get_bool(NVS_CONFIG_OVERCLOCK_ENABLED)) {
        valid = exact_option(asic->frequency_options, value);
    }
    return valid;
}

static bool voltage_request_valid(GlobalState *GLOBAL_STATE, uint16_t value)
{
    const AsicConfig *asic = &GLOBAL_STATE->DEVICE_CONFIG.family.asic;
    uint16_t minimum = 0;
    uint16_t maximum = 0;
    bool valid = option_bounds(asic->voltage_options, &minimum, &maximum) &&
                 value >= minimum && value <= maximum;
    if (valid && !nvs_config_get_bool(NVS_CONFIG_OVERCLOCK_ENABLED)) {
        valid = exact_option(asic->voltage_options, (float)value);
    }
    return valid;
}

esp_err_t POWER_MANAGEMENT_request_frequency(GlobalState *GLOBAL_STATE,
                                             float frequency_mhz)
{
    if (GLOBAL_STATE == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (pthread_mutex_lock(&request_mutex) != 0) {
        return ESP_FAIL;
    }

    esp_err_t err = ESP_ERR_INVALID_ARG;
    if (frequency_request_valid(GLOBAL_STATE, frequency_mhz)) {
        nvs_config_set_float(NVS_CONFIG_ASIC_FREQUENCY, frequency_mhz);
        err = ESP_OK;
    }

    pthread_mutex_unlock(&request_mutex);
    return err;
}

esp_err_t POWER_MANAGEMENT_request_voltage(GlobalState *GLOBAL_STATE,
                                           uint16_t voltage_mv)
{
    if (GLOBAL_STATE == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (pthread_mutex_lock(&request_mutex) != 0) {
        return ESP_FAIL;
    }

    esp_err_t err = ESP_ERR_INVALID_ARG;
    if (voltage_request_valid(GLOBAL_STATE, voltage_mv)) {
        nvs_config_set_u16(NVS_CONFIG_ASIC_VOLTAGE, voltage_mv);
        err = ESP_OK;
    }

    pthread_mutex_unlock(&request_mutex);
    return err;
}

static float requested_frequency(GlobalState *GLOBAL_STATE)
{
    const AsicConfig *asic = &GLOBAL_STATE->DEVICE_CONFIG.family.asic;
    if (GLOBAL_STATE->SELF_TEST_MODULE.is_active) {
        return asic->default_frequency_mhz;
    }

    float value = nvs_config_get_float(NVS_CONFIG_ASIC_FREQUENCY);
    bool valid = frequency_request_valid(GLOBAL_STATE, value);
    if (!valid) {
        bool same_rejection = have_last_rejected_frequency &&
            ((isnan(last_rejected_frequency) && isnan(value)) ||
             (isfinite(last_rejected_frequency) && isfinite(value) &&
              fabsf(last_rejected_frequency - value) < 0.01f));
        if (!same_rejection) {
            ESP_LOGE(TAG, "Rejected requested ASIC frequency %.2f MHz for %s; using family default %u MHz",
                     value, asic->name, asic->default_frequency_mhz);
            last_rejected_frequency = value;
            have_last_rejected_frequency = true;
        }
        return asic->default_frequency_mhz;
    }
    have_last_rejected_frequency = false;
    return value;
}

static uint16_t requested_voltage(GlobalState *GLOBAL_STATE)
{
    const AsicConfig *asic = &GLOBAL_STATE->DEVICE_CONFIG.family.asic;
    if (GLOBAL_STATE->SELF_TEST_MODULE.is_active) {
        return asic->default_voltage_mv;
    }

    uint16_t value = nvs_config_get_u16(NVS_CONFIG_ASIC_VOLTAGE);
    bool valid = voltage_request_valid(GLOBAL_STATE, value);
    if (!valid) {
        if (last_rejected_voltage != value) {
            ESP_LOGE(TAG, "Rejected requested ASIC voltage %umV for %s; using family default %umV",
                     value, asic->name, asic->default_voltage_mv);
            last_rejected_voltage = value;
        }
        return asic->default_voltage_mv;
    }
    last_rejected_voltage = -1;
    return value;
}

void POWER_MANAGEMENT_init_frequency(GlobalState *GLOBAL_STATE)
{
    PowerManagementModule *power = &GLOBAL_STATE->POWER_MANAGEMENT_MODULE;

    power->requested_frequency = requested_frequency(GLOBAL_STATE);
    power->requested_voltage_mv = requested_voltage(GLOBAL_STATE);
    if (!isfinite(power->thermal_frequency_cap) ||
        power->thermal_frequency_cap < ASIC_MIN_FREQUENCY_MHZ) {
        power->thermal_frequency_cap = power->requested_frequency;
    }
    power->frequency_value = fminf(power->requested_frequency,
                                  power->thermal_frequency_cap);
    power->actual_frequency = ASIC_MIN_FREQUENCY_MHZ;
    power->expected_hashrate = expected_hashrate(GLOBAL_STATE);

    char expected_hashrate_str[16] = {0};
    suffixString(power->expected_hashrate * 1e6f, expected_hashrate_str,
                 sizeof(expected_hashrate_str), 0);
    ESP_LOGI(TAG, "ASIC requested frequency: %g MHz, operating cap: %g MHz, expected hashrate: %sH/s",
             power->requested_frequency, power->thermal_frequency_cap,
             expected_hashrate_str);
}

static esp_err_t set_vcore_verified(GlobalState *GLOBAL_STATE,
                                    uint16_t voltage_mv)
{
    esp_err_t err = VCORE_set_voltage(GLOBAL_STATE, (float)voltage_mv / 1000.0f);
    if (err != ESP_OK || voltage_mv == 0) {
        return err;
    }

    int32_t tolerance_mv = (int32_t)voltage_mv * VCORE_TOLERANCE_PERCENT / 100;
    if (tolerance_mv < VCORE_TOLERANCE_MIN_MV) {
        tolerance_mv = VCORE_TOLERANCE_MIN_MV;
    }

    int16_t measured_mv = 0;
    for (int attempt = 0; attempt < VCORE_VERIFY_ATTEMPTS; attempt++) {
        vTaskDelay(pdMS_TO_TICKS(VCORE_VERIFY_DELAY_MS));
        measured_mv = VCORE_get_voltage_mv(GLOBAL_STATE);
        if (measured_mv > 0 &&
            abs((int32_t)measured_mv - (int32_t)voltage_mv) <= tolerance_mv) {
            return ESP_OK;
        }
    }

    ESP_LOGE(TAG, "VCORE verification failed: requested=%umV measured=%dmV tolerance=%ldmV",
             voltage_mv, measured_mv, (long)tolerance_mv);
    return ESP_FAIL;
}

static esp_err_t set_frequency_target(GlobalState *GLOBAL_STATE, float target_mhz)
{
    PowerManagementModule *power = &GLOBAL_STATE->POWER_MANAGEMENT_MODULE;
    target_mhz = fmaxf(target_mhz, ASIC_MIN_FREQUENCY_MHZ);
    if (fabsf(power->frequency_value - target_mhz) < 0.01f) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Changing ASIC operating frequency: %.2f -> %.2f MHz",
             power->frequency_value, target_mhz);
    float previous_target = power->frequency_value;
    power->frequency_value = target_mhz;
    power->expected_hashrate = expected_hashrate(GLOBAL_STATE);
    esp_err_t err = ASIC_set_frequency(GLOBAL_STATE);
    if (err == ESP_OK) {
        err = ASIC_set_nonce_space(GLOBAL_STATE);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ASIC frequency transition to %.2f MHz failed: %s",
                 target_mhz, esp_err_to_name(err));
        power->frequency_value = previous_target;
        power->expected_hashrate = expected_hashrate(GLOBAL_STATE);
    }
    return err;
}

static bool apply_operating_point(GlobalState *GLOBAL_STATE,
                                  power_control_t *control,
                                  uint16_t target_voltage_mv,
                                  float target_frequency_mhz)
{
    bool voltage_changed = target_voltage_mv != control->applied_voltage_mv;
    bool lower_voltage = target_voltage_mv < control->applied_voltage_mv;

    // Downclock before lowering voltage. This keeps the old high clock from
    // running undervolted throughout a multi-step PLL transition.
    if (target_frequency_mhz < control->applied_frequency_mhz - 0.01f) {
        if (set_frequency_target(GLOBAL_STATE, target_frequency_mhz) != ESP_OK) {
            return false;
        }
        control->applied_frequency_mhz = target_frequency_mhz;
    } else if (lower_voltage &&
               control->applied_frequency_mhz >
                   ASIC_MIN_FREQUENCY_MHZ + 0.01f) {
        // A voltage-only reduction still needs a safe transition point. Ramp
        // back to the requested clock only after the lower rail is verified.
        if (set_frequency_target(GLOBAL_STATE,
                                 ASIC_MIN_FREQUENCY_MHZ) != ESP_OK) {
            return false;
        }
        control->applied_frequency_mhz = ASIC_MIN_FREQUENCY_MHZ;
    }

    if (voltage_changed) {
        ESP_LOGI(TAG, "Changing VCORE: %umV -> %umV",
                 control->applied_voltage_mv, target_voltage_mv);
        if (set_vcore_verified(GLOBAL_STATE, target_voltage_mv) != ESP_OK) {
            return false;
        }
        control->applied_voltage_mv = target_voltage_mv;
    }

    // Raise voltage and verify it before increasing the clock.
    if (target_frequency_mhz > control->applied_frequency_mhz + 0.01f) {
        if (set_frequency_target(GLOBAL_STATE, target_frequency_mhz) != ESP_OK) {
            return false;
        }
        control->applied_frequency_mhz = target_frequency_mhz;
    }
    return true;
}

static void mining_stop(GlobalState *GLOBAL_STATE, power_control_t *control)
{
    if (!asic_lifecycle_is_running(GLOBAL_STATE)) {
        return;
    }

    ESP_LOGI(TAG, "Stopping mining");
    asic_lifecycle_set(GLOBAL_STATE, ASIC_LIFECYCLE_STOPPING);

    // Reset and power-off follow immediately below, so walking the PLL ladder
    // down first only delays the stop by ~100 ms per 6.25 MHz step (the
    // emergency path already cuts at operating frequency). Publish the 50 MHz
    // trackers directly so the next start ramps from the bottom, exactly as
    // the ladder used to leave them.
    GLOBAL_STATE->POWER_MANAGEMENT_MODULE.frequency_value = ASIC_MIN_FREQUENCY_MHZ;
    GLOBAL_STATE->POWER_MANAGEMENT_MODULE.actual_frequency = ASIC_MIN_FREQUENCY_MHZ;
    GLOBAL_STATE->POWER_MANAGEMENT_MODULE.expected_hashrate = 0.0f;
    control->applied_frequency_mhz = ASIC_MIN_FREQUENCY_MHZ;

    // A producer can pass its first lifecycle check immediately before
    // STOPPING is published. The command gate covers jobs, register reads,
    // mask changes, and PLL writes; valid_jobs_lock is nested inside it so no
    // job metadata can be published after the drain/reset boundary.
    pthread_mutex_lock(&GLOBAL_STATE->asic_command_lock);
    pthread_mutex_lock(&GLOBAL_STATE->valid_jobs_lock);
    esp_err_t drain_err = SERIAL_pause_tx(ASIC_TX_DRAIN_TIMEOUT_MS);
    if (drain_err != ESP_OK) {
        ESP_LOGW(TAG, "ASIC UART did not drain before shutdown: %s",
                 esp_err_to_name(drain_err));
    }
    if (asic_hold_reset_low() != ESP_OK) {
        ESP_LOGE(TAG, "Unable to hold ASIC reset low");
    }
    pthread_mutex_unlock(&GLOBAL_STATE->valid_jobs_lock);
    pthread_mutex_unlock(&GLOBAL_STATE->asic_command_lock);

    // With the ASIC held in reset it is safe to remove the core rail. Cutting
    // VCORE first would brown out a device still clocked at its operating PLL.
    if (VCORE_set_voltage(GLOBAL_STATE, 0.0f) != ESP_OK) {
        ESP_LOGE(TAG, "Unable to disable VCORE cleanly");
    }
    control->applied_voltage_mv = 0;

    vTaskDelay(pdMS_TO_TICKS(100));
    uart_flush(UART_NUM_1);
    asic_lifecycle_set(GLOBAL_STATE, ASIC_LIFECYCLE_STOPPED);
    ESP_LOGI(TAG, "Mining stopped");
}

static void mining_emergency_stop(GlobalState *GLOBAL_STATE,
                                  power_control_t *control,
                                  bool force_safety_fan)
{
    if (force_safety_fan) {
        // The fan task consumes the live latch, so publish it before any I/O.
        // Persistence is queued only after reset/power-off cannot be delayed.
        GLOBAL_STATE->SYSTEM_MODULE.overheat_mode = true;
    }

    ESP_LOGE(TAG, "Emergency ASIC stop");
    asic_lifecycle_set(GLOBAL_STATE, ASIC_LIFECYCLE_STOPPING);
    GLOBAL_STATE->POWER_MANAGEMENT_MODULE.expected_hashrate = 0.0f;

    // Close the TX gate without waiting for bytes already on the wire. This
    // prevents a producer that passed its RUNNING check on the other core from
    // enqueueing a command after reset/power removal. Safety shutdown still
    // skips both the PLL ladder and a graceful UART drain.
    pthread_mutex_lock(&GLOBAL_STATE->asic_command_lock);
    (void)SERIAL_pause_tx(0);

    // Safety faults must not spend seconds traversing the PLL ladder. Assert
    // reset and remove power immediately; graceful ramp-down is reserved for
    // user pause and pool-unavailable transitions.
    if (asic_hold_reset_low() != ESP_OK) {
        ESP_LOGE(TAG, "Unable to assert ASIC reset during emergency stop");
    }
    pthread_mutex_unlock(&GLOBAL_STATE->asic_command_lock);
    if (VCORE_set_voltage(GLOBAL_STATE, 0.0f) != ESP_OK) {
        ESP_LOGE(TAG, "Unable to disable VCORE during emergency stop");
    }
    control->applied_voltage_mv = 0;
    control->applied_frequency_mhz = ASIC_MIN_FREQUENCY_MHZ;
    GLOBAL_STATE->POWER_MANAGEMENT_MODULE.frequency_value =
        ASIC_MIN_FREQUENCY_MHZ;
    GLOBAL_STATE->POWER_MANAGEMENT_MODULE.actual_frequency =
        ASIC_MIN_FREQUENCY_MHZ;
    uart_flush(UART_NUM_1);
    asic_lifecycle_set(GLOBAL_STATE, ASIC_LIFECYCLE_STOPPED);

    if (force_safety_fan) {
        nvs_config_set_bool(NVS_CONFIG_OVERHEAT_MODE, true);
    }
}

static uint8_t mining_start(GlobalState *GLOBAL_STATE, power_control_t *control)
{
    PowerManagementModule *power = &GLOBAL_STATE->POWER_MANAGEMENT_MODULE;
    SystemModule *system = &GLOBAL_STATE->SYSTEM_MODULE;

    // Reserve STARTING and raise VCORE under the same gate used by shutdown.
    // A concurrent self-test/fault stop may publish STOPPING while waiting;
    // the post-I2C recheck then fails closed before this gate is released.
    pthread_mutex_lock(&GLOBAL_STATE->asic_command_lock);
    bool start_allowed =
        asic_lifecycle_get(GLOBAL_STATE) == ASIC_LIFECYCLE_STOPPED &&
        !system->hardware_fault && !system->mining_paused &&
        !system->pools_unavailable &&
        !GLOBAL_STATE->SELF_TEST_MODULE.is_finished;
    if (!start_allowed) {
        pthread_mutex_unlock(&GLOBAL_STATE->asic_command_lock);
        return 0;
    }

    ESP_LOGI(TAG, "Starting mining");
    asic_lifecycle_set(GLOBAL_STATE, ASIC_LIFECYCLE_STARTING);
    bool power_ready = VCORE_clear_faults(GLOBAL_STATE) == ESP_OK &&
        set_vcore_verified(GLOBAL_STATE, power->requested_voltage_mv) ==
            ESP_OK;
    bool start_still_valid = power_ready &&
        asic_lifecycle_get(GLOBAL_STATE) == ASIC_LIFECYCLE_STARTING &&
        !system->hardware_fault && !system->mining_paused &&
        !system->pools_unavailable &&
        !GLOBAL_STATE->SELF_TEST_MODULE.is_finished;
    if (!start_still_valid) {
        if (power_ready) {
            ESP_LOGW(TAG, "Mining start cancelled during VCORE ramp");
        } else {
            ESP_LOGE(TAG, "Mining start aborted: VCORE did not reach its requested setpoint");
        }
        asic_lifecycle_set(GLOBAL_STATE, ASIC_LIFECYCLE_STOPPING);
        (void)SERIAL_pause_tx(0);
        if (asic_hold_reset_low() != ESP_OK) {
            ESP_LOGE(TAG, "Unable to hold ASIC reset low after start failure");
        }
        pthread_mutex_unlock(&GLOBAL_STATE->asic_command_lock);
        if (VCORE_set_voltage(GLOBAL_STATE, 0.0f) != ESP_OK) {
            ESP_LOGE(TAG, "Unable to disable VCORE after start failure");
        }
        asic_lifecycle_set(GLOBAL_STATE, ASIC_LIFECYCLE_STOPPED);
        return 0;
    }
    pthread_mutex_unlock(&GLOBAL_STATE->asic_command_lock);
    control->applied_voltage_mv = power->requested_voltage_mv;

    vTaskDelay(pdMS_TO_TICKS(350));
    if (uart_flush(UART_NUM_1) != ESP_OK) {
        ESP_LOGW(TAG, "Unable to flush ASIC UART before recovery init");
    }

    power->frequency_value = fminf(power->requested_frequency,
                                  power->thermal_frequency_cap);
    power->actual_frequency = ASIC_MIN_FREQUENCY_MHZ;
    power->expected_hashrate = expected_hashrate(GLOBAL_STATE);
    // Consumers are gated on the lifecycle state for this whole window, so
    // the pre-RUNNING settle is pure downtime; cold boot already runs with 0.
    uint8_t chip_count = asic_initialize(GLOBAL_STATE, ASIC_INIT_RECOVERY, 500);

    if (chip_count == 0) {
        ESP_LOGE(TAG, "Mining start failed - ASIC not detected");
        // asic_initialize() already failed closed. Keep these trackers in
        // sync without repeating power/reset operations in reverse order.
        asic_lifecycle_set(GLOBAL_STATE, ASIC_LIFECYCLE_STOPPED);
        control->applied_voltage_mv = 0;
        control->applied_frequency_mhz = ASIC_MIN_FREQUENCY_MHZ;
        power->frequency_value = ASIC_MIN_FREQUENCY_MHZ;
        power->actual_frequency = ASIC_MIN_FREQUENCY_MHZ;
        power->expected_hashrate = 0.0f;
        return 0;
    }

    control->applied_frequency_mhz = power->frequency_value;
    ESP_LOGI(TAG, "Mining started successfully (%u chip(s))", chip_count);
    return chip_count;
}

static void refresh_temperature_readings(GlobalState *GLOBAL_STATE,
                                         thermal_reading_t *chip1,
                                         thermal_reading_t *chip2)
{
    PowerManagementModule *power = &GLOBAL_STATE->POWER_MANAGEMENT_MODULE;
    *chip1 = Thermal_get_chip_temp_reading(GLOBAL_STATE, 0);
    *chip2 = Thermal_get_chip_temp_reading(GLOBAL_STATE, 1);

    power->chip_temp_valid = chip1->valid;
    power->chip_temp2_valid = chip2->valid;
    power->chip_temp_age_ms = chip1->age_ms;
    power->chip_temp2_age_ms = chip2->age_ms;
    power->chip_temp_avg = isfinite(chip1->value) ? chip1->value : -1.0f;
    power->chip_temp2_avg = isfinite(chip2->value) ? chip2->value : -1.0f;
}

static void schedule_recovery(GlobalState *GLOBAL_STATE,
                              power_control_t *control,
                              recovery_reason_t reason,
                              bool force_safety_fan)
{
    mining_emergency_stop(GLOBAL_STATE, control, force_safety_fan);
    TickType_t now = xTaskGetTickCount();
    control->recovery_pending = true;
    control->recovery_reason = reason;
    uint32_t delay_ms = recovery_delay_ms(control->recovery_attempts);
    control->recovery_not_before = now + pdMS_TO_TICKS(delay_ms);
    if (control->recovery_attempts < UINT8_MAX) {
        control->recovery_attempts++;
    }

    ESP_LOGW(TAG, "Scheduled ASIC recovery after %s in %lums (attempt %u)",
             recovery_reason_name(reason), (unsigned long)delay_ms,
             control->recovery_attempts);
}

static void enter_thermal_cooldown(GlobalState *GLOBAL_STATE,
                                   power_control_t *control)
{
    PowerManagementModule *power = &GLOBAL_STATE->POWER_MANAGEMENT_MODULE;
    float current_cap = fminf(power->requested_frequency,
                              power->thermal_frequency_cap);
    power->thermal_frequency_cap =
        fmaxf(ASIC_MIN_FREQUENCY_MHZ,
              current_cap - HARD_THERMAL_REDUCTION_MHZ);
    power->thermal_throttled =
        power->thermal_frequency_cap < power->requested_frequency;

    // Latch fail-safe cooling before the immediate reset/power cut.
    GLOBAL_STATE->SYSTEM_MODULE.overheat_mode = true;
    mining_emergency_stop(GLOBAL_STATE, control, true);
    control->thermal_cooldown = true;
    control->recovery_reason = RECOVERY_HARD_OVERHEAT;
    control->cooling_cycles = 0;
    control->next_cooling_sample =
        xTaskGetTickCount() + pdMS_TO_TICKS(COOLING_SAMPLE_MS);
    ESP_LOGE(TAG, "Hard thermal limit reached; ASIC stopped with temporary %.2f MHz cap",
             power->thermal_frequency_cap);
}

static void refresh_requested_settings(GlobalState *GLOBAL_STATE)
{
    PowerManagementModule *power = &GLOBAL_STATE->POWER_MANAGEMENT_MODULE;
    float new_frequency = requested_frequency(GLOBAL_STATE);
    uint16_t new_voltage = requested_voltage(GLOBAL_STATE);

    if (fabsf(new_frequency - power->requested_frequency) >= 0.01f) {
        ESP_LOGI(TAG, "New ASIC frequency requested: %.2f MHz", new_frequency);
        power->requested_frequency = new_frequency;
        if (!power->thermal_throttled ||
            new_frequency <= power->thermal_frequency_cap) {
            power->thermal_frequency_cap = new_frequency;
            power->thermal_throttled = false;
        }
    }
    power->requested_voltage_mv = new_voltage;
}

static void update_soft_thermal_governor(GlobalState *GLOBAL_STATE,
                                         power_control_t *control,
                                         float hottest_temp,
                                         bool vr_required,
                                         bool vr_valid,
                                         TickType_t now)
{
    PowerManagementModule *power = &GLOBAL_STATE->POWER_MANAGEMENT_MODULE;
    bool hot = hottest_temp >= THROTTLE_TEMP_C ||
               (vr_required && vr_valid &&
                power->vr_temp >= TPS546_THROTTLE_TEMP_C);

    if (hot) {
        control->throttle_last_cool_sample = 0;
        control->throttle_cool_accumulated_ms = 0;
        control->throttle_fast_cool_accumulated_ms = 0;
        control->throttle_warm_samples = 0;
        control->throttle_last_cool_was_fast = false;
        if (control->throttle_last_step == 0 ||
            ticks_elapsed(now, control->throttle_last_step,
                          THROTTLE_INTERVAL_MS)) {
            float operating = fminf(power->requested_frequency,
                                    power->thermal_frequency_cap);
            float new_cap = fmaxf(ASIC_MIN_FREQUENCY_MHZ,
                                  operating - THROTTLE_STEP_MHZ);
            if (new_cap < power->thermal_frequency_cap) {
                power->thermal_frequency_cap = new_cap;
                power->thermal_throttled =
                    new_cap < power->requested_frequency;
                ESP_LOGW(TAG, "Soft thermal throttle: temporary frequency cap %.2f MHz",
                         new_cap);
            }
            control->throttle_last_step = now;
        }
        return;
    }

    if (!power->thermal_throttled) {
        control->throttle_last_cool_sample = 0;
        control->throttle_cool_accumulated_ms = 0;
        control->throttle_fast_cool_accumulated_ms = 0;
        control->throttle_warm_samples = 0;
        control->throttle_last_cool_was_fast = false;
        return;
    }

    if (vr_required && !vr_valid) {
        // Chip-temperature attack remains active when TPS546 telemetry is
        // unavailable, but no frequency may be restored without proving that
        // the regulator also has thermal headroom. Do not carry release credit
        // across this blind interval.
        control->throttle_last_cool_sample = 0;
        control->throttle_cool_accumulated_ms = 0;
        control->throttle_fast_cool_accumulated_ms = 0;
        control->throttle_warm_samples = 0;
        control->throttle_last_cool_was_fast = false;
        return;
    }

    bool cool = hottest_temp <= THROTTLE_RELEASE_TEMP_C &&
                (!vr_required || power->vr_temp <= TPS546_RELEASE_TEMP_C);
    if (!cool) {
        // Only intervals between consecutive qualified-cool samples count.
        // One noisy sample pauses rather than erases cooling credit, but warm
        // samples are not forgiven by intervening cool samples: repeated
        // alternating warm/cool input therefore still resets the window.
        control->throttle_last_cool_sample = 0;
        control->throttle_fast_cool_accumulated_ms = 0;
        control->throttle_last_cool_was_fast = false;
        if (control->throttle_warm_samples < THROTTLE_WARM_RESET_SAMPLES) {
            control->throttle_warm_samples++;
        }
        if (control->throttle_warm_samples >= THROTTLE_WARM_RESET_SAMPLES) {
            control->throttle_cool_accumulated_ms = 0;
            control->throttle_warm_samples = 0;
        }
        return;
    }

    bool fast_release_band =
        hottest_temp <= THROTTLE_RELEASE_FAST_TEMP_C &&
        (!vr_required ||
         power->vr_temp <= TPS546_RELEASE_FAST_TEMP_C);

    if (control->throttle_last_cool_sample != 0) {
        uint32_t elapsed_ms = pdTICKS_TO_MS(
            now - control->throttle_last_cool_sample);
        // A long scheduling or invalid-sensor gap is not evidence of cooling.
        if (elapsed_ms <= POLL_RATE_MS * 2U) {
            if (UINT32_MAX - control->throttle_cool_accumulated_ms <
                elapsed_ms) {
                control->throttle_cool_accumulated_ms = UINT32_MAX;
            } else {
                control->throttle_cool_accumulated_ms += elapsed_ms;
            }
            if (fast_release_band &&
                control->throttle_last_cool_was_fast) {
                if (UINT32_MAX -
                        control->throttle_fast_cool_accumulated_ms <
                    elapsed_ms) {
                    control->throttle_fast_cool_accumulated_ms = UINT32_MAX;
                } else {
                    control->throttle_fast_cool_accumulated_ms += elapsed_ms;
                }
            }
        }
    }
    if (!fast_release_band) {
        control->throttle_fast_cool_accumulated_ms = 0;
    }
    control->throttle_last_cool_sample = now;
    control->throttle_last_cool_was_fast = fast_release_band;

    // General cool credit survives harmless movement across the fast-band
    // boundary and releases after 30 s. The separate fast counter requires a
    // continuous 10 s with the additional temperature headroom.
    bool release_ready =
        control->throttle_cool_accumulated_ms >=
            THROTTLE_RELEASE_INTERVAL_MS ||
        control->throttle_fast_cool_accumulated_ms >=
            THROTTLE_RELEASE_FAST_INTERVAL_MS;
    if (release_ready) {
        power->thermal_frequency_cap =
            fminf(power->requested_frequency,
                  power->thermal_frequency_cap + THROTTLE_STEP_MHZ);
        power->thermal_throttled =
            power->thermal_frequency_cap < power->requested_frequency;
        control->throttle_cool_accumulated_ms = 0;
        control->throttle_fast_cool_accumulated_ms = 0;
        control->throttle_warm_samples = 0;
        ESP_LOGI(TAG, "Thermal headroom restored: temporary frequency cap %.2f MHz",
                 power->thermal_frequency_cap);
    }
}

void POWER_MANAGEMENT_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting");

    GlobalState *GLOBAL_STATE = (GlobalState *)pvParameters;
    PowerManagementModule *power = &GLOBAL_STATE->POWER_MANAGEMENT_MODULE;
    SystemModule *system = &GLOBAL_STATE->SYSTEM_MODULE;
    power_control_t control = {0};

    __atomic_store_n(&power->startup_preflight_succeeded, false,
                     __ATOMIC_RELAXED);
    __atomic_store_n(&power->startup_preflight_complete, false,
                     __ATOMIC_RELEASE);

    POWER_MANAGEMENT_init_frequency(GLOBAL_STATE);
    control.applied_frequency_mhz = power->frequency_value;

    // Cold-boot ASIC initialization is owned by main.c and happens after the
    // network is ready. Prepare and verify its VCORE here, as the legacy loop
    // did, without falsely publishing the ASIC as RUNNING.
    vTaskDelay(pdMS_TO_TICKS(500));

    pthread_mutex_lock(&GLOBAL_STATE->asic_command_lock);
    bool preflight_allowed =
        asic_lifecycle_get(GLOBAL_STATE) == ASIC_LIFECYCLE_STOPPED &&
        !system->hardware_fault &&
        !GLOBAL_STATE->SELF_TEST_MODULE.is_finished;
    esp_err_t preflight_err = ESP_ERR_INVALID_STATE;
    if (preflight_allowed) {
        preflight_err = VCORE_clear_faults(GLOBAL_STATE);
        if (preflight_err == ESP_OK) {
            preflight_err = set_vcore_verified(
                GLOBAL_STATE, power->requested_voltage_mv);
        }
    }
    bool preflight_still_valid =
        preflight_err == ESP_OK &&
        asic_lifecycle_get(GLOBAL_STATE) == ASIC_LIFECYCLE_STOPPED &&
        !system->hardware_fault &&
        !GLOBAL_STATE->SELF_TEST_MODULE.is_finished;
    pthread_mutex_unlock(&GLOBAL_STATE->asic_command_lock);

    if (!preflight_still_valid) {
        if (preflight_allowed && preflight_err != ESP_OK) {
            system->hardware_fault = true;
            snprintf(system->hardware_fault_msg,
                     sizeof(system->hardware_fault_msg),
                     "VCORE startup verification failed");
            ESP_LOGE(TAG, "%s", system->hardware_fault_msg);
        } else {
            ESP_LOGW(TAG, "VCORE startup preflight cancelled");
        }
        asic_lifecycle_set(GLOBAL_STATE, ASIC_LIFECYCLE_STOPPING);
        pthread_mutex_lock(&GLOBAL_STATE->asic_command_lock);
        (void)SERIAL_pause_tx(0);
        asic_hold_reset_low();
        pthread_mutex_unlock(&GLOBAL_STATE->asic_command_lock);
        VCORE_set_voltage(GLOBAL_STATE, 0.0f);
        control.applied_voltage_mv = 0;
        asic_lifecycle_set(GLOBAL_STATE, ASIC_LIFECYCLE_STOPPED);
        __atomic_store_n(&power->startup_preflight_succeeded, false,
                         __ATOMIC_RELAXED);
    } else {
        control.applied_voltage_mv = power->requested_voltage_mv;
        __atomic_store_n(&power->startup_preflight_succeeded, true,
                         __ATOMIC_RELAXED);
    }
    __atomic_store_n(&power->startup_preflight_complete, true,
                     __ATOMIC_RELEASE);

    while (1) {
        if (GLOBAL_STATE->SELF_TEST_MODULE.is_finished) {
            ESP_LOGI(TAG, "Stopped");
            vTaskDelete(NULL);
            return;
        }

        TickType_t now = xTaskGetTickCount();
        power->voltage = Power_get_input_voltage(GLOBAL_STATE);
        Power_get_output(GLOBAL_STATE, &power->power, &power->current);
        power->core_voltage = VCORE_get_voltage_mv(GLOBAL_STATE);
        power->vr_temp = Power_get_vreg_temp(GLOBAL_STATE);

        thermal_reading_t chip1;
        thermal_reading_t chip2;
        refresh_temperature_readings(GLOBAL_STATE, &chip1, &chip2);
        refresh_requested_settings(GLOBAL_STATE);

        bool second_sensor_required =
            Thermal_has_second_chip_sensor(&GLOBAL_STATE->DEVICE_CONFIG);
        bool chip_temps_valid = chip1.valid &&
            (!second_sensor_required || chip2.valid);
        bool vr_valid = !GLOBAL_STATE->DEVICE_CONFIG.TPS546 ||
            (isfinite(power->vr_temp) && power->vr_temp >= -40.0f &&
             power->vr_temp <= 200.0f);

        bool safety_stop = system->hardware_fault;
        bool requested_stop = system->mining_paused ||
                              system->pools_unavailable;
        bool wants_stop = safety_stop || requested_stop;
        asic_lifecycle_state_t lifecycle = asic_lifecycle_get(GLOBAL_STATE);
        if (lifecycle == ASIC_LIFECYCLE_RUNNING) {
            control.cold_boot_complete = true;
            if (!wants_stop) {
                control.stopped_for_request = false;
            }
        }

        if (wants_stop) {
            if (lifecycle == ASIC_LIFECYCLE_RUNNING) {
                if (safety_stop) {
                    mining_emergency_stop(GLOBAL_STATE, &control, false);
                } else {
                    mining_stop(GLOBAL_STATE, &control);
                }
            }
            // Cooldown/recovery already owns the restart. Do not overwrite it
            // with the ordinary pause/pool-stop state.
            if (!control.thermal_cooldown && !control.recovery_pending) {
                control.stopped_for_request = true;
            }
        }

        if (control.thermal_cooldown) {
            if ((int32_t)(now - control.next_cooling_sample) >= 0) {
                bool safe = vr_valid &&
                    (!GLOBAL_STATE->DEVICE_CONFIG.TPS546 ||
                     power->vr_temp <= TPS546_RELEASE_TEMP_C);

                if (Thermal_chip_sensors_available_when_stopped(
                        &GLOBAL_STATE->DEVICE_CONFIG)) {
                    safe = safe && chip_temps_valid &&
                           chip1.value <= SAFE_TEMP_C &&
                           (!second_sensor_required || chip2.value <= SAFE_TEMP_C);
                }

                control.cooling_cycles = safe ? control.cooling_cycles + 1 : 0;
                ESP_LOGW(TAG, "Thermal cooldown: safe cycle %u/%u, VR %.1fC, ASIC %.1fC/%.1fC, sensors %s",
                         control.cooling_cycles, MIN_COOLING_CYCLES,
                         power->vr_temp, power->chip_temp_avg,
                         power->chip_temp2_avg,
                         chip_temps_valid ? "valid" : "invalid");
                control.next_cooling_sample =
                    now + pdMS_TO_TICKS(COOLING_SAMPLE_MS);

                if (control.cooling_cycles >= MIN_COOLING_CYCLES) {
                    control.thermal_cooldown = false;
                    control.recovery_pending = true;
                    control.recovery_not_before =
                        now + pdMS_TO_TICKS(recovery_delay_ms(control.recovery_attempts));
                }
            }
            vTaskDelay(pdMS_TO_TICKS(POLL_RATE_MS));
            continue;
        }

        if (control.recovery_pending) {
            if (!wants_stop && lifecycle == ASIC_LIFECYCLE_STOPPED &&
                (int32_t)(now - control.recovery_not_before) >= 0) {
                if (mining_start(GLOBAL_STATE, &control) > 0) {
                    ESP_LOGI(TAG, "ASIC recovery after %s succeeded",
                             recovery_reason_name(control.recovery_reason));
                    control.recovery_pending = false;
                    control.recovery_reason = RECOVERY_NONE;
                    control.stable_since = now;
                    control.invalid_temp_since = 0;
                    control.vcore_read_failures = 0;
                    nvs_config_set_bool(NVS_CONFIG_OVERHEAT_MODE, false);
                    system->overheat_mode = false;
                } else {
                    TickType_t failed_at = xTaskGetTickCount();
                    uint32_t delay_ms = recovery_delay_ms(control.recovery_attempts);
                    control.recovery_not_before = failed_at + pdMS_TO_TICKS(delay_ms);
                    if (control.recovery_attempts < UINT8_MAX) {
                        control.recovery_attempts++;
                    }
                    ESP_LOGW(TAG, "ASIC recovery failed; retrying in %lums",
                             (unsigned long)delay_ms);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(POLL_RATE_MS));
            continue;
        }

        if (wants_stop) {
            vTaskDelay(pdMS_TO_TICKS(POLL_RATE_MS));
            continue;
        }

        if (control.stopped_for_request) {
            if (control.cold_boot_complete &&
                lifecycle == ASIC_LIFECYCLE_STOPPED &&
                (control.recovery_not_before == 0 ||
                 (int32_t)(now - control.recovery_not_before) >= 0)) {
                if (mining_start(GLOBAL_STATE, &control) > 0) {
                    control.stopped_for_request = false;
                    control.recovery_attempts = 0;
                    control.recovery_reason = RECOVERY_NONE;
                    control.recovery_not_before = 0;
                    control.stable_since = now;
                } else {
                    TickType_t failed_at = xTaskGetTickCount();
                    control.recovery_reason = RECOVERY_START_FAILURE;
                    control.recovery_not_before =
                        failed_at + pdMS_TO_TICKS(
                            recovery_delay_ms(control.recovery_attempts));
                    if (control.recovery_attempts < UINT8_MAX) {
                        control.recovery_attempts++;
                    }
                }
            }
            vTaskDelay(pdMS_TO_TICKS(POLL_RATE_MS));
            continue;
        }

        lifecycle = asic_lifecycle_get(GLOBAL_STATE);
        if (lifecycle != ASIC_LIFECYCLE_RUNNING) {
            vTaskDelay(pdMS_TO_TICKS(POLL_RATE_MS));
            continue;
        }

        if (control.stable_since == 0) {
            control.stable_since = now;
        } else if (ticks_elapsed(now, control.stable_since,
                                 RECOVERY_STABLE_RESET_MS)) {
            control.recovery_attempts = 0;
        }

        if (!chip_temps_valid) {
            // Do not bridge a release interval across a period with no valid
            // ASIC temperature sample.
            control.throttle_last_cool_sample = 0;
            control.throttle_cool_accumulated_ms = 0;
            control.throttle_fast_cool_accumulated_ms = 0;
            control.throttle_warm_samples = 0;
            control.throttle_last_cool_was_fast = false;
            if (control.invalid_temp_since == 0) {
                control.invalid_temp_since = now;
                ESP_LOGW(TAG, "ASIC temperature sensor unavailable; fan forced to fail-safe speed");
            } else if (ticks_elapsed(now, control.invalid_temp_since,
                                     TEMP_FAILURE_TIMEOUT_MS)) {
                schedule_recovery(GLOBAL_STATE, &control,
                                  RECOVERY_SENSOR_FAULT, true);
                vTaskDelay(pdMS_TO_TICKS(POLL_RATE_MS));
                continue;
            }
            // Do not raise clocks or alter VCORE while thermal protection is
            // blind, even during the short debounce window.
            vTaskDelay(pdMS_TO_TICKS(POLL_RATE_MS));
            continue;
        } else {
            control.invalid_temp_since = 0;
        }

        float hottest_temp = chip1.valid ? chip1.value : -INFINITY;
        if (chip2.valid) {
            hottest_temp = fmaxf(hottest_temp, chip2.value);
        }

        bool hard_overheat = chip_temps_valid && hottest_temp >= HARD_MAX_TEMP_C;
        hard_overheat = hard_overheat ||
            (vr_valid && GLOBAL_STATE->DEVICE_CONFIG.TPS546 &&
             power->vr_temp >= TPS546_MAX_TEMP_C);
        if (hard_overheat) {
            enter_thermal_cooldown(GLOBAL_STATE, &control);
            vTaskDelay(pdMS_TO_TICKS(POLL_RATE_MS));
            continue;
        }

        esp_err_t fault_err = VCORE_check_fault(GLOBAL_STATE);
        if (fault_err != ESP_OK) {
            control.vcore_read_failures++;
        } else {
            control.vcore_read_failures = 0;
        }
        if (system->power_fault != 0 || control.vcore_read_failures >= 3) {
            schedule_recovery(GLOBAL_STATE, &control,
                              RECOVERY_POWER_FAULT, false);
            vTaskDelay(pdMS_TO_TICKS(POLL_RATE_MS));
            continue;
        }

        hashrate_liveness_t liveness;
        hashrate_monitor_get_liveness(GLOBAL_STATE, &liveness);
        power->asic_response_age_ms = liveness.response_seen
                                          ? liveness.response_age_ms
                                          : UINT32_MAX;
        power->asic_progress_age_ms = liveness.progress_seen
                                          ? liveness.progress_age_ms
                                          : UINT32_MAX;
        if (!GLOBAL_STATE->SELF_TEST_MODULE.is_active && liveness.initialized &&
            liveness.monitor_age_ms >= LIVENESS_START_GRACE_MS) {
            if (!liveness.response_seen ||
                liveness.response_age_ms >= LIVENESS_RESPONSE_TIMEOUT_MS) {
                schedule_recovery(GLOBAL_STATE, &control,
                                  RECOVERY_ASIC_NO_RESPONSE, false);
                vTaskDelay(pdMS_TO_TICKS(POLL_RATE_MS));
                continue;
            }
            if (system->work_received > 0 &&
                (!liveness.progress_seen ||
                 liveness.progress_age_ms >= LIVENESS_PROGRESS_TIMEOUT_MS)) {
                schedule_recovery(GLOBAL_STATE, &control,
                                  RECOVERY_ASIC_NO_PROGRESS, false);
                vTaskDelay(pdMS_TO_TICKS(POLL_RATE_MS));
                continue;
            }
        }

        if (chip_temps_valid) {
            update_soft_thermal_governor(GLOBAL_STATE, &control, hottest_temp,
                                         GLOBAL_STATE->DEVICE_CONFIG.TPS546,
                                         vr_valid,
                                         now);
        }

        float target_frequency = fminf(power->requested_frequency,
                                       power->thermal_frequency_cap);
        if (!apply_operating_point(GLOBAL_STATE, &control,
                                   power->requested_voltage_mv,
                                   target_frequency)) {
            if (GLOBAL_STATE->SELF_TEST_MODULE.is_finished) {
                // tests_done() owns the ordered reset/power-off sequence.
            } else if (system->hardware_fault) {
                mining_emergency_stop(GLOBAL_STATE, &control, false);
                control.stopped_for_request = true;
            } else if (system->mining_paused || system->pools_unavailable) {
                mining_stop(GLOBAL_STATE, &control);
                control.stopped_for_request = true;
            } else {
                schedule_recovery(GLOBAL_STATE, &control,
                                  RECOVERY_POWER_FAULT, false);
            }
            vTaskDelay(pdMS_TO_TICKS(POLL_RATE_MS));
            continue;
        }

        bool overheat_mode = nvs_config_get_bool(NVS_CONFIG_OVERHEAT_MODE);
        bool thermally_safe = hottest_temp <= THROTTLE_RELEASE_TEMP_C &&
            (!GLOBAL_STATE->DEVICE_CONFIG.TPS546 ||
             (vr_valid && power->vr_temp <= TPS546_RELEASE_TEMP_C));
        if (overheat_mode && thermally_safe &&
            !power->thermal_throttled && !control.thermal_cooldown) {
            // Clear a stale persisted latch after a reboot or a completed
            // recovery only once all live sensors prove adequate headroom.
            nvs_config_set_bool(NVS_CONFIG_OVERHEAT_MODE, false);
            overheat_mode = false;
        }
        if (overheat_mode != system->overheat_mode) {
            system->overheat_mode = overheat_mode;
            ESP_LOGI(TAG, "Overheat mode updated to: %d", overheat_mode);
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_RATE_MS));
    }
}

bool POWER_MANAGEMENT_wait_for_preflight(GlobalState *GLOBAL_STATE,
                                         uint32_t timeout_ms)
{
    if (GLOBAL_STATE == NULL) {
        return false;
    }

    PowerManagementModule *power = &GLOBAL_STATE->POWER_MANAGEMENT_MODULE;
    TickType_t started = xTaskGetTickCount();
    while (!__atomic_load_n(&power->startup_preflight_complete,
                            __ATOMIC_ACQUIRE)) {
        if (pdTICKS_TO_MS(xTaskGetTickCount() - started) >= timeout_ms) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return __atomic_load_n(&power->startup_preflight_succeeded,
                           __ATOMIC_ACQUIRE);
}
