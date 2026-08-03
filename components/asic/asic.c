#include <string.h>

#include <esp_log.h>

#include "bm1397.h"
#include "bm1366.h"
#include "bm1368.h"
#include "bm1370.h"

#include "asic.h"
#include "global_state.h"
#include "mining.h"
#include "device_config.h"
#include "nvs_config.h"
#include "frequency_transition_bmXX.h"
#include "utils.h"

static const char *TAG = "asic";
static uint32_t effective_version_mask = STRATUM_DEFAULT_VERSION_MASK;

uint8_t ASIC_init(GlobalState * GLOBAL_STATE)
{
    ESP_LOGI(TAG, "Initializing %dx %s", GLOBAL_STATE->DEVICE_CONFIG.family.asic_count, GLOBAL_STATE->DEVICE_CONFIG.family.asic.name);
    switch (GLOBAL_STATE->DEVICE_CONFIG.family.asic.id) {
        case BM1397:
            return BM1397_init(GLOBAL_STATE);
        case BM1366:
            return BM1366_init(GLOBAL_STATE);
        case BM1368:
            return BM1368_init(GLOBAL_STATE);
        case BM1370:
            return BM1370_init(GLOBAL_STATE);
    }
    ESP_LOGE(TAG, "Unknown ASIC id %d", GLOBAL_STATE->DEVICE_CONFIG.family.asic.id);
    return 0;
}

task_result * ASIC_process_work(GlobalState * GLOBAL_STATE)
{
    switch (GLOBAL_STATE->DEVICE_CONFIG.family.asic.id) {
        case BM1397:
            return BM1397_process_work(GLOBAL_STATE);
        case BM1366:
            return BM1366_process_work(GLOBAL_STATE);
        case BM1368:
            return BM1368_process_work(GLOBAL_STATE);
        case BM1370:
            return BM1370_process_work(GLOBAL_STATE);
    }
    ESP_LOGE(TAG, "Unknown ASIC id %d — cannot process work", GLOBAL_STATE->DEVICE_CONFIG.family.asic.id);
    return NULL;
}

esp_err_t ASIC_set_max_baud(GlobalState *GLOBAL_STATE, int *baud)
{
    if (GLOBAL_STATE == NULL || baud == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    switch (GLOBAL_STATE->DEVICE_CONFIG.family.asic.id) {
        case BM1397:
            return BM1397_set_max_baud(baud);
        case BM1366:
            return BM1366_set_max_baud(baud);
        case BM1368:
            return BM1368_set_max_baud(baud);
        case BM1370:
            return BM1370_set_max_baud(baud);
    }
    ESP_LOGE(TAG, "Unknown ASIC id %d — cannot set max baud", GLOBAL_STATE->DEVICE_CONFIG.family.asic.id);
    return ESP_ERR_NOT_SUPPORTED;
}

bool ASIC_send_work(GlobalState * GLOBAL_STATE, bm_job * next_job,
                    uint32_t expected_generation)
{
    switch (GLOBAL_STATE->DEVICE_CONFIG.family.asic.id) {
        case BM1397:
            return BM1397_send_work(GLOBAL_STATE, next_job,
                                    expected_generation);
        case BM1366:
            return BM1366_send_work(GLOBAL_STATE, next_job,
                                    expected_generation);
        case BM1368:
            return BM1368_send_work(GLOBAL_STATE, next_job,
                                    expected_generation);
        case BM1370:
            return BM1370_send_work(GLOBAL_STATE, next_job,
                                    expected_generation);
        default:
            ESP_LOGE(TAG, "Unknown ASIC id %d — cannot send work", GLOBAL_STATE->DEVICE_CONFIG.family.asic.id);
            return false;
    }
}

esp_err_t ASIC_set_version_mask(GlobalState *GLOBAL_STATE, uint32_t mask)
{
    if (GLOBAL_STATE == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err;
    switch (GLOBAL_STATE->DEVICE_CONFIG.family.asic.id) {
        case BM1397:
            err = BM1397_set_version_mask(mask);
            break;
        case BM1366:
            err = BM1366_set_version_mask(mask);
            break;
        case BM1368:
            err = BM1368_set_version_mask(mask);
            break;
        case BM1370:
            err = BM1370_set_version_mask(mask);
            break;
        default:
            ESP_LOGE(TAG, "Unknown ASIC id %d — cannot set version mask", GLOBAL_STATE->DEVICE_CONFIG.family.asic.id);
            return ESP_ERR_NOT_SUPPORTED;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set ASIC version mask 0x%08lx: %s",
                 (unsigned long)mask, esp_err_to_name(err));
        return err;
    }
    effective_version_mask = mask;
    return ESP_OK;
}

esp_err_t ASIC_restore_version_mask(GlobalState *GLOBAL_STATE)
{
    return ASIC_set_version_mask(GLOBAL_STATE, effective_version_mask);
}

esp_err_t ASIC_set_frequency(GlobalState *GLOBAL_STATE)
{
    if (GLOBAL_STATE == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    switch (GLOBAL_STATE->DEVICE_CONFIG.family.asic.id) {
        case BM1397:
            return do_frequency_transition(GLOBAL_STATE, BM1397_send_hash_frequency);
        case BM1366:
            return do_frequency_transition(GLOBAL_STATE, BM1366_send_hash_frequency);
        case BM1368:
            return do_frequency_transition(GLOBAL_STATE, BM1368_send_hash_frequency);
        case BM1370:
            return do_frequency_transition(GLOBAL_STATE, BM1370_send_hash_frequency);
    }
    ESP_LOGE(TAG, "Unknown ASIC id %d — cannot set frequency", GLOBAL_STATE->DEVICE_CONFIG.family.asic.id);
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ASIC_set_nonce_space(GlobalState *GLOBAL_STATE)
{
    if (GLOBAL_STATE == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    float nonce_percent = 1.0;
    int cores = GLOBAL_STATE->DEVICE_CONFIG.family.asic.core_count;
    int asic_count = GLOBAL_STATE->DEVICE_CONFIG.family.asic_count;
    float frequency = GLOBAL_STATE->POWER_MANAGEMENT_MODULE.actual_frequency;

    switch (GLOBAL_STATE->DEVICE_CONFIG.family.asic.id) {
        case BM1397:
            return ESP_OK;
        case BM1366:
            return BM1366_set_nonce_space(nonce_percent, frequency, asic_count, cores);
        case BM1368:
            return BM1368_set_nonce_space(nonce_percent, frequency, asic_count, cores);
        case BM1370:
            return BM1370_set_nonce_space(nonce_percent, frequency, asic_count, cores);
    }
    ESP_LOGE(TAG, "Unknown ASIC id %d — cannot set nonce space", GLOBAL_STATE->DEVICE_CONFIG.family.asic.id);
    return ESP_ERR_NOT_SUPPORTED;
}

double ASIC_get_asic_job_frequency_ms(GlobalState * GLOBAL_STATE)
{
    float freq = GLOBAL_STATE->POWER_MANAGEMENT_MODULE.actual_frequency;
    if (freq <= 0.0f) {
        freq = GLOBAL_STATE->POWER_MANAGEMENT_MODULE.frequency_value;
    }
    int cores = GLOBAL_STATE->DEVICE_CONFIG.family.asic.core_count;
    int small_cores = GLOBAL_STATE->DEVICE_CONFIG.family.asic.small_core_count;
    int asic_count = GLOBAL_STATE->DEVICE_CONFIG.family.asic_count;
    switch (GLOBAL_STATE->DEVICE_CONFIG.family.asic.id) {
        case BM1397:
            // no version-rolling so same Nonce Space is splitted between Big Cores
            return calculate_bm_timeout_ms(freq, asic_count, small_cores, cores,
                                           version_mask_midstate_count(
                                               GLOBAL_STATE->version_mask),
                                           1.0,
                                           GLOBAL_STATE->DEVICE_CONFIG.family.asic.default_asic_timeout);
        case BM1366:
        case BM1368:
        case BM1370: {
            // The calculated scan already accounts for chain topology. Apply the
            // pool-facing cap once so multi-chip chains are not restarted early.
            double refresh_cap_ms =
                nvs_config_get_u16(NVS_CONFIG_ASIC_JOB_INTERVAL);
            if (refresh_cap_ms < 1000.0 || refresh_cap_ms > 60000.0) {
                refresh_cap_ms =
                    GLOBAL_STATE->DEVICE_CONFIG.family.asic.default_asic_timeout;
            }
            // BM1366/68/70 expose the 16 BIP320 rolling bits at positions
            // 13..28. Ignore any echoed pool bits outside that hardware field.
            uint32_t effective_asic_mask =
                (effective_version_mask >> 13) & UINT32_C(0xffff);
            size_t version_space =
                version_mask_value_count(effective_asic_mask);
            return calculate_bm_job_interval_ms(freq, asic_count, small_cores,
                                                cores, version_space,
                                                refresh_cap_ms);
        }
    }
    ESP_LOGE(TAG, "Unknown ASIC id %d — cannot compute job frequency", GLOBAL_STATE->DEVICE_CONFIG.family.asic.id);
    return 500;
}

void ASIC_read_registers(GlobalState * GLOBAL_STATE)
{
    switch (GLOBAL_STATE->DEVICE_CONFIG.family.asic.id) {
        case BM1397:
            BM1397_read_registers();
            break;
        case BM1366:
            BM1366_read_registers();
            break;
        case BM1368:
            BM1368_read_registers();
            break;
        case BM1370:
            BM1370_read_registers();
            break;
        default:
            ESP_LOGE(TAG, "Unknown ASIC id %d — cannot read registers", GLOBAL_STATE->DEVICE_CONFIG.family.asic.id);
            break;
    }
}
