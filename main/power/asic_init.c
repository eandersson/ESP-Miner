#include "asic_init.h"
#include "global_state.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "asic.h"
#include "asic_common.h"
#include "serial.h"
#include "asic_reset.h"
#include "asic_result_task.h"
#include "mining.h"
#include "vcore.h"

static const char *TAG = "asic_init";

static uint8_t asic_fail_closed(GlobalState *GLOBAL_STATE,
                                const char *status)
{
    GLOBAL_STATE->SYSTEM_MODULE.asic_status = status;
    asic_lifecycle_set(GLOBAL_STATE, ASIC_LIFECYCLE_STOPPING);
    if (VCORE_set_voltage(GLOBAL_STATE, 0.0f) != ESP_OK) {
        ESP_LOGE(TAG, "Unable to disable VCORE after ASIC initialization failure");
    }
    if (asic_hold_reset_low() != ESP_OK) {
        ESP_LOGE(TAG, "Unable to hold ASIC reset low after initialization failure");
    }
    asic_lifecycle_set(GLOBAL_STATE, ASIC_LIFECYCLE_STOPPED);
    return 0;
}

void asic_lifecycle_set(GlobalState *GLOBAL_STATE, asic_lifecycle_state_t state)
{
    // Keep the misspelled legacy flag synchronized until all consumers have
    // migrated.  Publish it after RUNNING and clear it before any other state
    // so legacy job producers fail closed during transitions.
    if (state != ASIC_LIFECYCLE_RUNNING) {
        __atomic_store_n(&GLOBAL_STATE->ASIC_initalized, false, __ATOMIC_RELEASE);
    }
    __atomic_store_n(&GLOBAL_STATE->asic_lifecycle, state, __ATOMIC_RELEASE);
    if (state == ASIC_LIFECYCLE_RUNNING) {
        __atomic_store_n(&GLOBAL_STATE->ASIC_initalized, true, __ATOMIC_RELEASE);
    }
}

asic_lifecycle_state_t asic_lifecycle_get(const GlobalState *GLOBAL_STATE)
{
    return __atomic_load_n(&GLOBAL_STATE->asic_lifecycle, __ATOMIC_ACQUIRE);
}

bool asic_lifecycle_is_running(const GlobalState *GLOBAL_STATE)
{
    return asic_lifecycle_get(GLOBAL_STATE) == ASIC_LIFECYCLE_RUNNING;
}

uint8_t asic_initialize(GlobalState *GLOBAL_STATE, asic_init_mode_t mode, uint32_t stabilization_delay_ms)
{
    const char *mode_str = (mode == ASIC_INIT_COLD_BOOT) ? "cold boot" : "recovery";
    ESP_LOGI(TAG, "Starting ASIC initialization (%s mode)", mode_str);
    asic_lifecycle_set(GLOBAL_STATE, ASIC_LIFECYCLE_STARTING);

    if (asic_reset() != ESP_OK) {
        ESP_LOGE(TAG, "ASIC reset failed!");
        return asic_fail_closed(GLOBAL_STATE, "ASIC reset failed");
    }

    // Check actual UART state for safety
    bool uart_initialized = SERIAL_is_initialized();
    
    // Verify mode matches actual state
    if (mode == ASIC_INIT_COLD_BOOT && uart_initialized) {
        ESP_LOGW(TAG, "Cold boot mode but UART already initialized - will reset baud only");
    } else if (mode == ASIC_INIT_RECOVERY && !uart_initialized) {
        ESP_LOGW(TAG, "Recovery mode but UART not initialized - will do full init");
    }
    
    // Use actual state for decision, not just mode
    if (!uart_initialized) {
        // Fresh boot - full UART initialization
        ESP_LOGI(TAG, "Performing full UART initialization");
        if (SERIAL_init() != ESP_OK) {
            return asic_fail_closed(
                GLOBAL_STATE, "ASIC UART initialization failed");
        }
    } else {
        // Live recovery - ASIC was reset, UART needs baud reset to 115200
        // This preserves the running system and avoids reboot
        ESP_LOGI(TAG, "UART already initialized, resetting baud to %d", UART_FREQ);
        if (SERIAL_set_baud(UART_FREQ) != ESP_OK) {
            return asic_fail_closed(
                GLOBAL_STATE, "ASIC UART baud reset failed");
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "Detecting ASIC chips...");
    clear_asic_chain_error();
    uint8_t chip_count = ASIC_init(GLOBAL_STATE);
    
    if (chip_count == 0) {
        const char *chain_error = get_asic_chain_error();
        ESP_LOGE(TAG, "ASIC initialization failed - chip chain detection failed");
        return asic_fail_closed(
            GLOBAL_STATE, chain_error != NULL
                              ? chain_error
                              : "ASIC chain detection failed");
    }

    // ASIC_init restores the chip's power-on default. Reapply the last
    // negotiated effective mask so cadence and hardware search space remain
    // aligned across live recovery.
    esp_err_t err = ASIC_restore_version_mask(GLOBAL_STATE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Unable to restore ASIC version mask: %s",
                 esp_err_to_name(err));
        return asic_fail_closed(
            GLOBAL_STATE, "ASIC version-mask restore failed");
    }

    ESP_LOGI(TAG, "Setting max baud rate and clearing buffers");
    int max_baud = 0;
    err = ASIC_set_max_baud(GLOBAL_STATE, &max_baud);
    if (err == ESP_OK) {
        err = SERIAL_set_baud(max_baud);
    }
    if (err == ESP_OK) {
        err = SERIAL_clear_buffer();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Unable to configure ASIC UART at operating baud");
        return asic_fail_closed(
            GLOBAL_STATE, "ASIC UART operating baud failed");
    }

    // Hardware reset invalidates every device-side job slot. Publish the new
    // slot epoch and clear host metadata atomically, while leaving the pool
    // generation unchanged so create_jobs can immediately re-feed its valid
    // current template after recovery.
    pthread_mutex_lock(&GLOBAL_STATE->valid_jobs_lock);
    ASIC_result_task_reset();
    for (int i = 0; i < 128; i++) {
        bm_job *active_job = NULL;
        bm_job *retired_job = NULL;
        if (GLOBAL_STATE->valid_jobs != NULL) {
            GLOBAL_STATE->valid_jobs[i] = 0;
        }
        if (GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs != NULL) {
            active_job = GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs[i];
            GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs[i] = NULL;
        }
        if (GLOBAL_STATE->ASIC_TASK_MODULE.retired_jobs != NULL) {
            retired_job = GLOBAL_STATE->ASIC_TASK_MODULE.retired_jobs[i];
            GLOBAL_STATE->ASIC_TASK_MODULE.retired_jobs[i] = NULL;
        }
        if (GLOBAL_STATE->ASIC_TASK_MODULE.active_job_dispatch_us != NULL) {
            GLOBAL_STATE->ASIC_TASK_MODULE.active_job_dispatch_us[i] = 0;
        }
        if (GLOBAL_STATE->ASIC_TASK_MODULE.retired_job_dispatch_us != NULL) {
            GLOBAL_STATE->ASIC_TASK_MODULE.retired_job_dispatch_us[i] = 0;
        }
        if (active_job != NULL) {
            release_bm_job(active_job);
        }
        if (retired_job != NULL && retired_job != active_job) {
            release_bm_job(retired_job);
        }
    }
    pthread_mutex_unlock(&GLOBAL_STATE->valid_jobs_lock);

    if (stabilization_delay_ms > 0) {
        ESP_LOGI(TAG, "Waiting %u ms for tasks to stabilize...", stabilization_delay_ms);
        vTaskDelay(stabilization_delay_ms / portTICK_PERIOD_MS);
    }

    // Reset liveness ages before publishing RUNNING. Consumers are held off by
    // the lifecycle state for the entire stabilization interval.
    hashrate_monitor_reset_measurements(GLOBAL_STATE);
    asic_lifecycle_set(GLOBAL_STATE, ASIC_LIFECYCLE_RUNNING);

    ESP_LOGI(TAG, "ASIC initialized successfully with %d chip(s) (%s mode)", chip_count, mode_str);
    return chip_count;
}
