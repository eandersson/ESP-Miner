#include "frequency_transition_bmXX.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include "global_state.h"

#define EPSILON 0.0001f
#define STEP_SIZE 6.25 // MHz step size

static const char * TAG = "frequency_transition";

static esp_err_t apply_frequency(GlobalState *global_state,
                                 set_hash_frequency_fn set_frequency_fn,
                                 float requested_frequency)
{
    float applied_frequency = 0.0f;
    esp_err_t err = set_frequency_fn(requested_frequency, &applied_frequency);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to apply %g MHz: %s", requested_frequency,
                 esp_err_to_name(err));
        return err;
    }
    if (!isfinite(applied_frequency) || applied_frequency <= 0.0f) {
        ESP_LOGE(TAG, "Frequency callback returned invalid applied value %g",
                 applied_frequency);
        return ESP_ERR_INVALID_RESPONSE;
    }

    global_state->POWER_MANAGEMENT_MODULE.actual_frequency = applied_frequency;
    return ESP_OK;
}

esp_err_t do_frequency_transition(GlobalState *GLOBAL_STATE,
                                  set_hash_frequency_fn set_frequency_fn)
{
    if (GLOBAL_STATE == NULL || set_frequency_fn == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    float target_frequency = GLOBAL_STATE->POWER_MANAGEMENT_MODULE.frequency_value;
    float current_frequency = GLOBAL_STATE->POWER_MANAGEMENT_MODULE.actual_frequency;

    if (!isfinite(target_frequency) || target_frequency <= 0.0f ||
        !isfinite(current_frequency) || current_frequency <= 0.0f) {
        ESP_LOGE(TAG, "Invalid frequency transition %g -> %g MHz",
                 current_frequency, target_frequency);
        return ESP_ERR_INVALID_ARG;
    }

    if (fabs(current_frequency - target_frequency) < EPSILON) {
        return ESP_OK;
    }

    if (fabs(target_frequency - current_frequency) < STEP_SIZE) {
        return apply_frequency(GLOBAL_STATE, set_frequency_fn, target_frequency);
    }

    ESP_LOGI(TAG, "Transitioning frequency from %g MHz to %g MHz",
             current_frequency, target_frequency);

    int current_step = (target_frequency > current_frequency) ? (int)floor(current_frequency / STEP_SIZE) : (int)ceil(current_frequency / STEP_SIZE);
    int target_step = (target_frequency > current_frequency) ? (int)floor(target_frequency / STEP_SIZE) : (int)ceil(target_frequency / STEP_SIZE);

    if (current_step != target_step) {
        int signum = (target_frequency > current_frequency) ? 1 : -1;
        
        while ((signum > 0 && current_step < target_step) ||
               (signum < 0 && current_step > target_step)) {
            current_step += signum;

            current_frequency = current_step * STEP_SIZE;
            esp_err_t err = apply_frequency(GLOBAL_STATE, set_frequency_fn,
                                            current_frequency);
            if (err != ESP_OK) {
                return err;
            }
            
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
    
    if (fabs(current_frequency - target_frequency) > EPSILON) {
        current_frequency = target_frequency;
        esp_err_t err = apply_frequency(GLOBAL_STATE, set_frequency_fn,
                                        current_frequency);
        if (err != ESP_OK) {
            return err;
        }
    }
    
    ESP_LOGI(TAG, "Successfully transitioned to %g MHz", target_frequency);
    return ESP_OK;
}
