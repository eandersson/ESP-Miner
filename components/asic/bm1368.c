#include "bm1368.h"

#include "asic_result_task.h"
#include "crc.h"
#include "global_state.h"
#include "mining.h"
#include "serial.h"
#include "utils.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "frequency_transition_bmXX.h"
#include "pll.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>

#define BM1368_CHIP_ID 0x1368
#define BM1368_CHIP_ID_RESPONSE_LENGTH 11

#define TYPE_JOB 0x20
#define TYPE_CMD 0x40

#define GROUP_SINGLE 0x00
#define GROUP_ALL 0x10

#define CMD_SETADDRESS 0x00
#define CMD_WRITE 0x01
#define CMD_READ 0x02
#define CMD_INACTIVE 0x03

#define MISC_CONTROL 0x18
#define FAST_UART_CONFIGURATION 0x28
#define BM1368_MIN_FREQUENCY_MHZ 50.0f
#define BM1368_MAX_FREQUENCY_MHZ 1200.0f

static const register_type_t REGISTER_MAP[] = {
    [0x4C] = REGISTER_ERROR_COUNT,
    [0x88] = REGISTER_DOMAIN_0_COUNT,
    [0x89] = REGISTER_DOMAIN_1_COUNT,
    [0x8A] = REGISTER_DOMAIN_2_COUNT,
    [0x8B] = REGISTER_DOMAIN_3_COUNT,
    [0x8C] = REGISTER_TOTAL_COUNT
};

typedef struct __attribute__((__packed__))
{
    uint32_t nonce;                   // 2-5
    uint8_t midstate_num;             // 6
    uint8_t id;                       // 7
    uint16_t version;                 // 8-9
} bm1368_asic_result_job_t;

typedef struct __attribute__((__packed__))
{
    uint32_t value;                   // 2-5
    uint8_t asic_address;             // 6
    uint8_t register_address;         // 7
    uint16_t                  : 16;   // 8-9
} bm1368_asic_result_cmd_t;

typedef struct __attribute__((__packed__))
{
    uint16_t preamble;                // 0-1
    union {
        bm1368_asic_result_job_t job; // 2-9
        bm1368_asic_result_cmd_t cmd; // 2-9
    };
    uint8_t crc             : 5;      // 10:0-5
    uint8_t                 : 2;      // 10:6-7
    uint8_t is_job_response : 1;      // 10:8
} bm1368_asic_result_t;

static const char * TAG = "bm1368";

static task_result result;

static int address_interval;

#define BM1368_INIT_REQUIRE_SENT(expression, step)                         \
    do {                                                                  \
        if (!(expression)) {                                              \
            ESP_LOGE(TAG, "ASIC initialization TX failed at %s", step);  \
            return 0;                                                     \
        }                                                                 \
    } while (0)

#define BM1368_INIT_REQUIRE_OK(expression, step)                           \
    do {                                                                  \
        esp_err_t init_err = (expression);                                \
        if (init_err != ESP_OK) {                                         \
            ESP_LOGE(TAG, "ASIC initialization failed at %s: %s", step,  \
                     esp_err_to_name(init_err));                           \
            return 0;                                                     \
        }                                                                 \
    } while (0)

static bool _send_BM1368(uint8_t header, const uint8_t * data, uint8_t data_len, bool debug)
{
    packet_type_t packet_type = (header & TYPE_JOB) ? JOB_PACKET : CMD_PACKET;
    uint8_t total_length = (packet_type == JOB_PACKET) ? (data_len + 6) : (data_len + 5);

    uint8_t buf[total_length];

    buf[0] = 0x55;
    buf[1] = 0xAA;
    buf[2] = header;
    buf[3] = (packet_type == JOB_PACKET) ? (data_len + 4) : (data_len + 3);
    memcpy(buf + 4, data, data_len);

    if (packet_type == JOB_PACKET) {
        uint16_t crc16_total = crc16_false(buf + 2, data_len + 2);
        buf[4 + data_len] = (crc16_total >> 8) & 0xFF;
        buf[5 + data_len] = crc16_total & 0xFF;
    } else {
        buf[4 + data_len] = crc5(buf + 2, data_len + 2);
    }

    return SERIAL_send(buf, total_length, debug);
}


static bool _send_chain_inactive(void)
{
    unsigned char read_address[2] = {0x00, 0x00};
    return _send_BM1368((TYPE_CMD | GROUP_ALL | CMD_INACTIVE), read_address,
                        2, BM1368_SERIALTX_DEBUG);
}

static bool _set_chip_address(uint8_t chipAddr)
{
    unsigned char read_address[2] = {chipAddr, 0x00};
    return _send_BM1368((TYPE_CMD | GROUP_SINGLE | CMD_SETADDRESS),
                        read_address, 2, BM1368_SERIALTX_DEBUG);
}

esp_err_t BM1368_set_version_mask(uint32_t version_mask)
{
    int versions_to_roll = version_mask >> 13;
    uint8_t version_byte0 = (versions_to_roll >> 8);
    uint8_t version_byte1 = (versions_to_roll & 0xFF); 
    uint8_t version_cmd[] = {0x00, 0xA4, 0x90, 0x00, version_byte0, version_byte1};
    return _send_BM1368(TYPE_CMD | GROUP_ALL | CMD_WRITE, version_cmd, 6,
                        BM1368_SERIALTX_DEBUG) ? ESP_OK : ESP_FAIL;
}

static esp_err_t BM1368_set_hash_counting_number(uint32_t hcn)
{
    if (hcn == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t set_10_hash_counting[6] = {0x00, 0x10, 0x00, 0x00, 0x00, 0x00};
    set_10_hash_counting[2] = (hcn >> 24) & 0xFF;
    set_10_hash_counting[3] = (hcn >> 16) & 0xFF;
    set_10_hash_counting[4] = (hcn >> 8) & 0xFF;
    set_10_hash_counting[5] = hcn & 0xFF;
    return _send_BM1368((TYPE_CMD | GROUP_ALL | CMD_WRITE),
                        set_10_hash_counting, 6,
                        BM1368_SERIALTX_DEBUG) ? ESP_OK : ESP_FAIL;
}

esp_err_t BM1368_set_nonce_space(double nonce_percent, float frequency,
                                 uint16_t asic_count, uint16_t cores)
{
    if (!isfinite(nonce_percent) || nonce_percent <= 0.0 ||
        nonce_percent > 1.0 || !isfinite(frequency) || frequency <= 0.0f ||
        asic_count == 0 || cores == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t hcn = calculate_bm_hcn(frequency, asic_count, cores,
                                    nonce_percent, FREQ_MULT, 0.0);
    if (hcn == 0) {
        ESP_LOGE(TAG, "Invalid nonce-space parameters: frequency=%g count=%u cores=%u",
                 frequency, asic_count, cores);
        return ESP_ERR_INVALID_ARG;
    }
    return BM1368_set_hash_counting_number(hcn);
}

esp_err_t BM1368_send_hash_frequency(float target_freq,
                                     float *applied_frequency)
{
    if (applied_frequency == NULL || !isfinite(target_freq) ||
        target_freq < BM1368_MIN_FREQUENCY_MHZ ||
        target_freq > BM1368_MAX_FREQUENCY_MHZ) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t fb_divider, refdiv, postdiv1, postdiv2;
    float new_freq;

    esp_err_t err = pll_get_parameters(target_freq, 144, 235, &fb_divider,
                                       &refdiv, &postdiv1, &postdiv2,
                                       &new_freq);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t vdo_scale = (fb_divider * FREQ_MULT / refdiv >= 2400) ? 0x50 : 0x40;
    uint8_t postdiv = (((postdiv1 - 1) & 0xf) << 4) | ((postdiv2 - 1) & 0xf);
    uint8_t freqbuf[6] = {0x00, 0x08, vdo_scale, fb_divider, refdiv, postdiv};

    if (!_send_BM1368(TYPE_CMD | GROUP_ALL | CMD_WRITE, freqbuf,
                      sizeof(freqbuf), BM1368_SERIALTX_DEBUG)) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Setting Frequency to %g MHz (%g)", target_freq, new_freq);

    *applied_frequency = new_freq;
    return ESP_OK;
}

uint8_t BM1368_init(GlobalState * GLOBAL_STATE)
{
    // set version mask
    for (int i = 0; i < 4; i++) {
        BM1368_INIT_REQUIRE_OK(
            BM1368_set_version_mask(STRATUM_DEFAULT_VERSION_MASK),
            "initial version mask");
    }

    BM1368_INIT_REQUIRE_SENT(
        _send_BM1368(TYPE_CMD | GROUP_ALL | CMD_READ,
                     (uint8_t[]){0x00, 0x00}, 2, false),
        "chip ID request");

    uint16_t asic_count = GLOBAL_STATE->DEVICE_CONFIG.family.asic_count;
    int chip_counter = count_asic_chips(asic_count, BM1368_CHIP_ID, BM1368_CHIP_ID_RESPONSE_LENGTH);

    if (chip_counter == 0) {
        return 0;
    }

    BM1368_INIT_REQUIRE_SENT(_send_chain_inactive(), "chain inactive");
    
    uint8_t init_cmds[][6] = {
        {0x00, 0xA8, 0x00, 0x07, 0x00, 0x00},
        {0x00, 0x18, 0xFF, 0x0F, 0xC1, 0x00},
        {0x00, 0x3C, 0x80, 0x00, 0x8b, 0x00},
        {0x00, 0x3C, 0x80, 0x00, 0x80, 0x18},
        {0x00, 0x14, 0x00, 0x00, 0x00, 0xFF},
        {0x00, 0x54, 0x00, 0x00, 0x00, 0x03}, //Analog Mux
        {0x00, 0x58, 0x02, 0x11, 0x11, 0x11}
    };

    for (int i = 0; i < sizeof(init_cmds) / sizeof(init_cmds[0]); i++) {
        BM1368_INIT_REQUIRE_SENT(
            _send_BM1368(TYPE_CMD | GROUP_ALL | CMD_WRITE, init_cmds[i],
                         6, false),
            "global register setup");
    }

    address_interval = 256 / chip_counter;
    for (int i = 0; i < chip_counter; i++) {
        BM1368_INIT_REQUIRE_SENT(_set_chip_address(i * address_interval),
                                 "chip address assignment");
    }

    for (int i = 0; i < chip_counter; i++) {
        uint8_t chip_init_cmds[][6] = {
            {i * address_interval, 0xA8, 0x00, 0x07, 0x01, 0xF0},
            {i * address_interval, 0x18, 0xF0, 0x00, 0xC1, 0x00},
            {i * address_interval, 0x3C, 0x80, 0x00, 0x8b, 0x00},
            {i * address_interval, 0x3C, 0x80, 0x00, 0x80, 0x18},
            {i * address_interval, 0x3C, 0x80, 0x00, 0x82, 0xAA}
        };

        for (int j = 0; j < sizeof(chip_init_cmds) / sizeof(chip_init_cmds[0]); j++) {
            BM1368_INIT_REQUIRE_SENT(
                _send_BM1368(TYPE_CMD | GROUP_SINGLE | CMD_WRITE,
                             chip_init_cmds[j], 6, false),
                "per-chip register setup");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    uint16_t difficulty = GLOBAL_STATE->DEVICE_CONFIG.family.asic.difficulty;

    uint8_t difficulty_mask[6];
    get_difficulty_mask(difficulty, difficulty_mask);
    BM1368_INIT_REQUIRE_SENT(
        _send_BM1368((TYPE_CMD | GROUP_ALL | CMD_WRITE), difficulty_mask,
                     6, BM1368_SERIALTX_DEBUG),
        "difficulty mask");

    BM1368_INIT_REQUIRE_OK(
        do_frequency_transition(GLOBAL_STATE, BM1368_send_hash_frequency),
        "frequency transition");

    float frequency = GLOBAL_STATE->POWER_MANAGEMENT_MODULE.frequency_value;
    int cores = GLOBAL_STATE->DEVICE_CONFIG.family.asic.core_count;

    BM1368_INIT_REQUIRE_OK(
        BM1368_set_nonce_space(1.0, frequency, asic_count, cores),
        "nonce space");
    BM1368_INIT_REQUIRE_OK(
        BM1368_set_version_mask(STRATUM_DEFAULT_VERSION_MASK),
        "final version mask");

    return chip_counter;
}

esp_err_t BM1368_set_default_baud(int *baud)
{
    if (baud == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    unsigned char baudrate[9] = {0x00, MISC_CONTROL, 0x00, 0x00, 0b01111010, 0b00110001};
    if (!_send_BM1368((TYPE_CMD | GROUP_ALL | CMD_WRITE), baudrate, 6,
                      BM1368_SERIALTX_DEBUG)) {
        return ESP_FAIL;
    }
    *baud = 115749;
    return ESP_OK;
}

esp_err_t BM1368_set_max_baud(int *baud)
{
    if (baud == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGI(TAG, "Setting max baud of 1000000");

    unsigned char fast_uart[] = {0x00, FAST_UART_CONFIGURATION, 0x11, 0x30, 0x02, 0x00};
    if (!_send_BM1368((TYPE_CMD | GROUP_ALL | CMD_WRITE), fast_uart, 6,
                      BM1368_SERIALTX_DEBUG)) {
        return ESP_FAIL;
    }

    *baud = 1000000;
    return ESP_OK;
}

static uint8_t id = 0;

bool BM1368_send_work(GlobalState * GLOBAL_STATE, bm_job * next_bm_job,
                      uint32_t expected_generation)
{
    if (GLOBAL_STATE == NULL || next_bm_job == NULL ||
        GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs == NULL ||
        GLOBAL_STATE->ASIC_TASK_MODULE.retired_jobs == NULL ||
        GLOBAL_STATE->ASIC_TASK_MODULE.active_job_dispatch_us == NULL ||
        GLOBAL_STATE->ASIC_TASK_MODULE.retired_job_dispatch_us == NULL ||
        GLOBAL_STATE->valid_jobs == NULL) {
        ESP_LOGE(TAG, "Cannot send job before job tracking is initialized");
        return false;
    }

    BM1368_job job = {0};
    job.num_midstates = 0x01;
    memcpy(&job.starting_nonce, &next_bm_job->starting_nonce, 4);
    memcpy(&job.nbits, &next_bm_job->target, 4);
    memcpy(&job.ntime, &next_bm_job->ntime, 4);
    memcpy(job.merkle_root, next_bm_job->merkle_root, 32);
    memcpy(job.prev_block_hash, next_bm_job->prev_block_hash, 32);
    memcpy(&job.version, &next_bm_job->version, 4);

    // Invalidate the reused slot before TX, then publish metadata only after the
    // UART accepted the complete packet. Holding the lock across this short
    // enqueue makes send/publication atomic with clean-job invalidation.
    pthread_mutex_lock(&GLOBAL_STATE->valid_jobs_lock);
    if (__atomic_load_n(&GLOBAL_STATE->asic_lifecycle, __ATOMIC_ACQUIRE) !=
        ASIC_LIFECYCLE_RUNNING) {
        pthread_mutex_unlock(&GLOBAL_STATE->valid_jobs_lock);
        ESP_LOGD(TAG, "Discarding job while ASIC is stopping");
        return false;
    }
    if (ASIC_result_task_get_job_generation() != expected_generation) {
        pthread_mutex_unlock(&GLOBAL_STATE->valid_jobs_lock);
        ESP_LOGW(TAG, "Discarding job from stale generation %lu",
                 (unsigned long)expected_generation);
        return false;
    }
    const uint8_t next_id = (id + 24) % 128;
    job.job_id = next_id;
    bm_job *replaced_job =
        GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs[job.job_id];
    bm_job *prior_retired_job =
        GLOBAL_STATE->ASIC_TASK_MODULE.retired_jobs[job.job_id];
    int64_t replaced_dispatch_us =
        GLOBAL_STATE->ASIC_TASK_MODULE.active_job_dispatch_us[job.job_id];
    GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs[job.job_id] = NULL;
    GLOBAL_STATE->ASIC_TASK_MODULE.active_job_dispatch_us[job.job_id] = 0;
    GLOBAL_STATE->valid_jobs[job.job_id] = 0;
    if (replaced_job != NULL) {
        GLOBAL_STATE->ASIC_TASK_MODULE.retired_jobs[job.job_id] = replaced_job;
        GLOBAL_STATE->ASIC_TASK_MODULE.retired_job_dispatch_us[job.job_id] =
            replaced_dispatch_us;
    }

    bool sent = _send_BM1368((TYPE_JOB | GROUP_SINGLE | CMD_WRITE),
                             (const uint8_t *)&job, sizeof(job),
                             BM1368_DEBUG_WORK);
    if (sent) {
        GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs[job.job_id] = next_bm_job;
        GLOBAL_STATE->ASIC_TASK_MODULE.active_job_dispatch_us[job.job_id] =
            esp_timer_get_time();
        GLOBAL_STATE->valid_jobs[job.job_id] = 1;
        id = next_id;
    }
    pthread_mutex_unlock(&GLOBAL_STATE->valid_jobs_lock);

    if (replaced_job != NULL && prior_retired_job != NULL &&
        prior_retired_job != replaced_job &&
        prior_retired_job != next_bm_job) {
        release_bm_job(prior_retired_job);
    }

    if (!sent) {
        ESP_LOGE(TAG, "Failed to send job 0x%02X; slot remains invalid",
                 job.job_id);
        return false;
    }

    #if BM1368_DEBUG_JOBS
    ESP_LOGI(TAG, "⁠​‌‌​​​‌​​‌‌​‌​​‌​‌‌‌​‌​​​‌‌​​​​‌​‌‌‌‌​​​​‌‌​​‌​‌⁠Send Job: %02X", job.job_id);
    #endif

    return true;
}

task_result * BM1368_process_work(GlobalState * GLOBAL_STATE)
{
    bm1368_asic_result_t asic_result = {0};
    (void)GLOBAL_STATE;

    memset(&result, 0, sizeof(task_result));

    if (receive_work((uint8_t *)&asic_result, sizeof(asic_result), &result.timestamp_us) == ESP_FAIL) {
        return NULL;
    }

    if (!asic_result.is_job_response) {
        result.register_type = asic_register_map_lookup(
            REGISTER_MAP, sizeof(REGISTER_MAP) / sizeof(REGISTER_MAP[0]),
            asic_result.cmd.register_address);
        if (result.register_type == REGISTER_INVALID) {
            ESP_LOGW(TAG, "Unknown register read: %02x", asic_result.cmd.register_address);
            return NULL;
        }
        result.asic_nr = asic_result.cmd.asic_address / address_interval;
        result.value = ntohl(asic_result.cmd.value);
        
        return &result;
    }

    uint8_t job_id = (asic_result.job.id & 0xf0) >> 1;
    uint32_t nonce_h = ntohl(asic_result.job.nonce);
    uint8_t asic_nr = (uint8_t)((nonce_h >> 17) & 0xff) / address_interval;
    uint8_t core_id = (uint8_t)((nonce_h >> 25) & 0x7f);
    uint8_t small_core_id = asic_result.job.id & 0x0f;
    uint32_t version_bits = (ntohs(asic_result.job.version) << 13);

    result.job_id = job_id;
    result.nonce = asic_result.job.nonce;
    result.version_bits = version_bits;
    result.asic_nr = asic_nr;
    result.core_id = core_id;
    result.small_core_id = small_core_id;

    return &result;
}

void BM1368_read_registers(void)
{
    int size = sizeof(REGISTER_MAP) / sizeof(REGISTER_MAP[0]);
    for (int reg = 0; reg < size; reg++) {
        if (REGISTER_MAP[reg] != REGISTER_INVALID) {
            _send_BM1368((TYPE_CMD | GROUP_ALL | CMD_READ), (uint8_t[]){0x00, reg}, 2, BM1368_SERIALTX_DEBUG);
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
    }
}
