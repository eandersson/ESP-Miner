#include <stdlib.h>
#include <stdio.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_heap_caps.h"
#include "cJSON.h"

#include "asic_result_task.h"
#include "create_jobs_task.h"
#include "hashrate_monitor_task.h"
#include "fan_controller_task.h"
#include "statistics_task.h"
#include "global_state.h"
#include "system.h"
#include "http_server.h"
#include "protocol_coordinator.h"
#include "i2c_bitaxe.h"
#include "adc.h"
#include "nvs_config.h"
#include "self_test.h"
#include "asic.h"
#include "bap/bap.h"
#include "device_config.h"
#include "connect.h"
#include "asic_reset.h"
#include "asic_init.h"
#include "vcore.h"
#include "task_monitor.h"
#include "filesystem.h"
#include "log_buffer.h"
#include "setup_ble.h"
#include "esp_ota_ops.h"

static GlobalState GLOBAL_STATE;

static const char * TAG = "bitaxe";

#define ASIC_TASK_CORE 1
#define POWER_PREFLIGHT_TIMEOUT_MS 5000U

static void fail_asic_closed(const char *status)
{
    GLOBAL_STATE.SYSTEM_MODULE.hardware_fault = true;
    GLOBAL_STATE.SYSTEM_MODULE.asic_status = status;
    snprintf(GLOBAL_STATE.SYSTEM_MODULE.hardware_fault_msg,
             sizeof(GLOBAL_STATE.SYSTEM_MODULE.hardware_fault_msg), "%s",
             status);
    asic_lifecycle_set(&GLOBAL_STATE, ASIC_LIFECYCLE_STOPPING);
    if (asic_hold_reset_low() != ESP_OK) {
        ESP_LOGE(TAG, "Unable to assert ASIC reset while failing closed");
    }
    if (VCORE_set_voltage(&GLOBAL_STATE, 0.0f) != ESP_OK) {
        ESP_LOGE(TAG, "Unable to disable VCORE while failing closed");
    }
    GLOBAL_STATE.POWER_MANAGEMENT_MODULE.expected_hashrate = 0.0f;
    asic_lifecycle_set(&GLOBAL_STATE, ASIC_LIFECYCLE_STOPPED);
}

static void heap_alloc_failed_hook(size_t requested_size, uint32_t caps, const char *function_name)
{
    if (caps & MALLOC_CAP_SPIRAM) {
        ESP_EARLY_LOGE(TAG, "%s failed to allocate %zu bytes from PSRAM", function_name, requested_size);
        abort();
    }
}

static void *cjson_malloc_psram(size_t size)
{
    if (esp_psram_is_initialized()) {
        return heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    }
    return malloc(size);
}

static void cjson_free_psram(void *ptr)
{
    free(ptr);
}

void app_main(void)
{
    ESP_ERROR_CHECK(heap_caps_register_failed_alloc_callback(heap_alloc_failed_hook));

    cJSON_Hooks hooks = {
        .malloc_fn = cjson_malloc_psram,
        .free_fn = cjson_free_psram
    };
    cJSON_InitHooks(&hooks);
    if (esp_psram_is_initialized()) {
        GLOBAL_STATE.psram_is_available = true;
        log_buffer_init();
    } else {
        ESP_LOGE(TAG, "No PSRAM available on ESP32 device!");
    }

    ESP_LOGI(TAG, "Welcome to the bitaxe - FOSS || GTFO!");

    if (xTaskCreateWithCaps(cpu_monitor_task, "cpu_monitor", 4096, (void *)&GLOBAL_STATE, 1, NULL, MALLOC_CAP_SPIRAM) != pdPASS) {
        ESP_LOGE(TAG, "Error creating cpu monitor task");
    }
#ifdef CONFIG_ENABLE_TASK_MONITOR
    if (xTaskCreateWithCaps(task_monitor_task, "task_monitor", 8192, NULL, 1, NULL, MALLOC_CAP_SPIRAM) != pdPASS) {
        ESP_LOGE(TAG, "Error creating task monitor task");
    }
#endif
  
    // Init I2C
    ESP_ERROR_CHECK(i2c_bitaxe_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Initialize RST pin to low early to minimize ASIC power consumption
    ESP_ERROR_CHECK(asic_hold_reset_low());
    ESP_LOGI(TAG, "RST pin initialized to low");

    // wait for I2C to init
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Init ADC
    ADC_init();

    // initialize the ESP32 NVS
    if (nvs_config_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init NVS");
        return;
    }

    // Confirm app validity for OTA rollback
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            ESP_LOGI(TAG, "First boot after OTA update, confirming app validity");
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }

    // Ensure SSID is initialized before any screen/self-test uses it.
    GLOBAL_STATE.SYSTEM_MODULE.ssid = nvs_config_get_string(NVS_CONFIG_WIFI_SSID);
    if (GLOBAL_STATE.SYSTEM_MODULE.ssid == NULL) {
        ESP_LOGW(TAG, "No SSID configured in NVS, using empty string");
        GLOBAL_STATE.SYSTEM_MODULE.ssid = strdup("");
        if (GLOBAL_STATE.SYSTEM_MODULE.ssid == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for SSID");
            return;
        }
    }

    if (device_config_init(&GLOBAL_STATE) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init device config");
        return;
    }

    if (self_test_init(&GLOBAL_STATE) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init self test");
        return;
    }

    SYSTEM_init_system(&GLOBAL_STATE);
    if (scoreboard_init(&GLOBAL_STATE.SYSTEM_MODULE.scoreboard) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init scoreboard");
    }

    if (!GLOBAL_STATE.SELF_TEST_MODULE.is_active) {
        wifi_init(&GLOBAL_STATE);
    }

    esp_err_t system_init_ret = SYSTEM_init_peripherals(&GLOBAL_STATE);
    
    if (system_init_ret == ESP_OK) {
        if (xTaskCreate(POWER_MANAGEMENT_task, "power management", 8192, (void *) &GLOBAL_STATE, 10, NULL) != pdPASS) {
            ESP_LOGE(TAG, "Error creating power management task");
            fail_asic_closed("Power management task creation failed");
            __atomic_store_n(
                &GLOBAL_STATE.POWER_MANAGEMENT_MODULE.startup_preflight_succeeded,
                false, __ATOMIC_RELAXED);
            __atomic_store_n(
                &GLOBAL_STATE.POWER_MANAGEMENT_MODULE.startup_preflight_complete,
                true, __ATOMIC_RELEASE);
        }
        if (!GLOBAL_STATE.SELF_TEST_MODULE.is_active) {
            if (xTaskCreate(FAN_CONTROLLER_task, "fan_controller", 8192, (void *) &GLOBAL_STATE, 5, NULL) != pdPASS) {
                ESP_LOGE(TAG, "Error creating fan controller task");
            }
        }
    } else {
        ESP_LOGE(TAG, "Critical peripheral initialization failure (%s). Entering degraded mode.", esp_err_to_name(GLOBAL_STATE.SELF_TEST_MODULE.system_init_ret));
    }
    
    if (!GLOBAL_STATE.SELF_TEST_MODULE.is_active) {
        // start the API for AxeOS
        start_rest_server(&GLOBAL_STATE);
    }

    // After mounting SPIFFS
    SYSTEM_init_versions(&GLOBAL_STATE);

    // Pre-cache partition descriptions and space usage percentage
    SYSTEM_init_partitions(&GLOBAL_STATE);

    // Initialize BAP interface
    esp_err_t bap_ret = BAP_init(&GLOBAL_STATE);
    if (bap_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BAP interface: %d", bap_ret);
        // Continue anyway, as BAP is not critical for core functionality
    }

    // While the device is still in setup mode (config AP up but no WiFi
    // connection), expose the BLE provisioning service so the miner can be
    // configured over Bluetooth. A short grace period avoids spinning up BLE on
    // a normal boot that connects within a few seconds. setup_ble_start() is
    // idempotent and only takes effect once the AP is actually enabled.
    int setup_ble_grace_ms = 0;
    while (!GLOBAL_STATE.SYSTEM_MODULE.is_connected) {
        if (GLOBAL_STATE.SYSTEM_MODULE.ap_enabled && setup_ble_grace_ms >= 5000) {
            setup_ble_start(&GLOBAL_STATE);
        }
        setup_ble_grace_ms += 100;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    // Connected to WiFi: tear down the setup BLE service to free the radio.
    setup_ble_stop();

    queue_init(&GLOBAL_STATE.stratum_queue);
    if (ASIC_result_task_init() != ESP_OK) {
        ESP_LOGE(TAG, "Unable to initialize ASIC result processing");
        return;
    }

    if (system_init_ret == ESP_OK) {
        bool power_ready = POWER_MANAGEMENT_wait_for_preflight(
            &GLOBAL_STATE, POWER_PREFLIGHT_TIMEOUT_MS);
        if (!power_ready || GLOBAL_STATE.SYSTEM_MODULE.hardware_fault) {
            ESP_LOGE(TAG,
                     "ASIC cold start blocked: power preflight did not complete successfully");
            fail_asic_closed("VCORE startup preflight failed");
            if (!GLOBAL_STATE.SELF_TEST_MODULE.is_active) {
                return;
            }
            self_test_show_message(
                &GLOBAL_STATE, GLOBAL_STATE.SYSTEM_MODULE.asic_status);
            system_init_ret = ESP_FAIL;
        } else if (asic_initialize(&GLOBAL_STATE, ASIC_INIT_COLD_BOOT, 0) == 0) {
            if (!GLOBAL_STATE.SELF_TEST_MODULE.is_active) {
                return;
            }

            self_test_show_message(&GLOBAL_STATE, GLOBAL_STATE.SYSTEM_MODULE.asic_status);
            system_init_ret = ESP_FAIL;
        } else {
            bool critical_task_failure = false;
            if (xTaskCreatePinnedToCore(create_jobs_task, "stratum miner",
                                        8192, (void *)&GLOBAL_STATE, 20,
                                        NULL, ASIC_TASK_CORE) != pdPASS) {
                ESP_LOGE(TAG, "Error creating stratum miner task");
                critical_task_failure = true;
            }
            // Keep the latency-sensitive UART path on core 1. ESP-IDF's Wi-Fi
            // task is pinned to core 0, so this prevents radio work and ASIC RX
            // from preempting each other during bursts.
            if (xTaskCreatePinnedToCore(ASIC_result_rx_task, "asic result rx", 4096, (void *) &GLOBAL_STATE, 21, NULL, ASIC_TASK_CORE) != pdPASS) {
                ESP_LOGE(TAG, "Error creating asic result rx task");
                critical_task_failure = true;
            }
            if (xTaskCreatePinnedToCore(ASIC_result_task, "asic result", 8192, (void *) &GLOBAL_STATE, 15, NULL, ASIC_TASK_CORE) != pdPASS) {
                ESP_LOGE(TAG, "Error creating asic result task");
                critical_task_failure = true;
            }
            if (xTaskCreateWithCaps(ASIC_v1_share_submit_task, "v1 share tx", 6144, (void *) &GLOBAL_STATE, 8, NULL, MALLOC_CAP_SPIRAM) != pdPASS) {
                ESP_LOGE(TAG, "Error creating Stratum V1 share submit task");
                critical_task_failure = true;
            }

            if (xTaskCreateWithCaps(hashrate_monitor_task, "hashrate monitor", 8192, (void *) &GLOBAL_STATE, 5, NULL, MALLOC_CAP_SPIRAM) != pdPASS) {
                ESP_LOGE(TAG, "Error creating hashrate monitor task");
                critical_task_failure = true;
            }
            if (xTaskCreateWithCaps(statistics_task, "statistics", 8192, (void *) &GLOBAL_STATE, 3, NULL, MALLOC_CAP_SPIRAM) != pdPASS) {
                ESP_LOGE(TAG, "Error creating statistics task");
            }

            if (critical_task_failure) {
                fail_asic_closed("Critical ASIC task creation failed");
                if (!GLOBAL_STATE.SELF_TEST_MODULE.is_active) {
                    return;
                }
                self_test_show_message(
                    &GLOBAL_STATE, GLOBAL_STATE.SYSTEM_MODULE.asic_status);
                system_init_ret = ESP_FAIL;
            }
        }
    }

    protocol_coordinator_init(&GLOBAL_STATE);
    if (xTaskCreateWithCaps(protocol_coordinator_task, "protocol coord", 3072, (void *) &GLOBAL_STATE, 5, NULL, MALLOC_CAP_SPIRAM) != pdPASS) {
        ESP_LOGE(TAG, "Error creating protocol coordinator task");
    }

    if (GLOBAL_STATE.SELF_TEST_MODULE.is_active) {
        GLOBAL_STATE.SELF_TEST_MODULE.system_init_ret = system_init_ret;
        if (xTaskCreateWithCaps(self_test_task, "self_test", 8192, (void *) &GLOBAL_STATE, 10, NULL, MALLOC_CAP_SPIRAM) != pdPASS) {
            ESP_LOGE(TAG, "Error creating self test task");
        }
    }
}
