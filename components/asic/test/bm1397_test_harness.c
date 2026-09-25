#include "bm1397_test_bindings.h"
#include "bm1397_test_harness.h"

#include <pthread.h>
#include <stdbool.h>
#include <string.h>
#include "asic_init.h"
#include "bm1397.h"
#include "frequency_transition_bmXX.h"
#include "global_state.h"
#include "mining.h"
#include "serial.h"
#include "unity.h"

const bm1397_harness_driver_t bm1397_harness_driver = {
    BM1397_init, BM1397_send_work, BM1397_set_ticket_difficulty,
    BM1397_process_work,
};

enum { MAX_PACKETS = 32, JOB_SLOTS = 128 };
static GlobalState fixture_state;
static bm_job *active_jobs[JOB_SLOTS];
static bm_job *retired_jobs[JOB_SLOTS];
static int64_t active_dispatch_us[JOB_SLOTS];
static int64_t retired_dispatch_us[JOB_SLOTS];
static uint8_t valid_jobs[JOB_SLOTS];
static bm1397_harness_packet_t packets[MAX_PACKETS];
static size_t packet_count;
static uint8_t queued_response[BM1397_HARNESS_RESPONSE_SIZE];
static bool response_ready;

GlobalState *bm1397_harness_begin(void)
{
    memset(active_jobs, 0, sizeof(active_jobs));
    memset(retired_jobs, 0, sizeof(retired_jobs));
    memset(active_dispatch_us, 0, sizeof(active_dispatch_us));
    memset(retired_dispatch_us, 0, sizeof(retired_dispatch_us));
    memset(valid_jobs, 0, sizeof(valid_jobs));
    fixture_state = (GlobalState) {
        .DEVICE_CONFIG.family.asic_count = 2,
        .DEVICE_CONFIG.family.asic.id = BM1397,
        .DEVICE_CONFIG.family.asic.difficulty = 256,
        .DEVICE_CONFIG.family.asic.core_count = 672,
        .POWER_MANAGEMENT_MODULE.frequency_value = 200.0f,
        .ASIC_TASK_MODULE.active_jobs = active_jobs,
        .ASIC_TASK_MODULE.retired_jobs = retired_jobs,
        .ASIC_TASK_MODULE.active_job_dispatch_us = active_dispatch_us,
        .ASIC_TASK_MODULE.retired_job_dispatch_us = retired_dispatch_us,
        .ASIC_TASK_MODULE.valid_jobs = valid_jobs,
        .asic_lifecycle = ASIC_LIFECYCLE_RUNNING,
    };
    TEST_ASSERT_EQUAL_INT(0, pthread_mutex_init(
        &fixture_state.ASIC_TASK_MODULE.valid_jobs_lock, NULL));
    packet_count = 0;
    response_ready = false;
    return &fixture_state;
}

void bm1397_harness_end(void)
{
    for (size_t index = 0; index < JOB_SLOTS; ++index) {
        if (retired_jobs[index] != NULL && retired_jobs[index] != active_jobs[index]) {
            release_bm_job(retired_jobs[index]);
        }
        if (active_jobs[index] != NULL) {
            release_bm_job(active_jobs[index]);
        }
        active_jobs[index] = NULL;
        retired_jobs[index] = NULL;
    }
    TEST_ASSERT_EQUAL_INT(0, pthread_mutex_trylock(
        &fixture_state.ASIC_TASK_MODULE.valid_jobs_lock));
    TEST_ASSERT_EQUAL_INT(0, pthread_mutex_unlock(
        &fixture_state.ASIC_TASK_MODULE.valid_jobs_lock));
    TEST_ASSERT_EQUAL_INT(0, pthread_mutex_destroy(
        &fixture_state.ASIC_TASK_MODULE.valid_jobs_lock));
}

void bm1397_harness_clear_packets(void)
{
    packet_count = 0;
}

size_t bm1397_harness_packet_count(void)
{
    return packet_count;
}

const bm1397_harness_packet_t *bm1397_harness_packet(size_t index)
{
    TEST_ASSERT_TRUE(index < packet_count);
    return &packets[index];
}

bm_job *bm1397_harness_active_job(uint8_t job_id)
{
    TEST_ASSERT_TRUE(job_id < JOB_SLOTS);
    return active_jobs[job_id];
}

bm_job *bm1397_harness_retired_job(uint8_t job_id)
{
    TEST_ASSERT_TRUE(job_id < JOB_SLOTS);
    return retired_jobs[job_id];
}

void bm1397_harness_install_job(uint8_t job_id, bm_job *job)
{
    TEST_ASSERT_TRUE(job_id < JOB_SLOTS);
    TEST_ASSERT_NULL(active_jobs[job_id]);
    active_jobs[job_id] = job;
    active_dispatch_us[job_id] = 1;
    valid_jobs[job_id] = 1;
}

uint32_t bm1397_harness_generation(void)
{
    return 1;
}

uint32_t bm1397_stub_job_generation(void)
{
    return 1;
}

void bm1397_harness_queue_response(
    const uint8_t response[BM1397_HARNESS_RESPONSE_SIZE])
{
    response_ready = response != NULL;
    if (response_ready) {
        memcpy(queued_response, response, sizeof(queued_response));
    }
}

bool bm1397_spy_serial_send(const uint8_t *bytes, size_t length, bool debug)
{
    (void)debug;
    TEST_ASSERT_TRUE(packet_count < MAX_PACKETS);
    TEST_ASSERT_TRUE(length > 0 && length <= sizeof(packets[0].bytes));
    bm1397_harness_packet_t *packet = &packets[packet_count++];
    packet->length = length;
    memcpy(packet->bytes, bytes, packet->length);
    return true;
}

esp_err_t bm1397_stub_serial_set_baud(int baud)
{
    (void)baud;
    return ESP_OK;
}

esp_err_t bm1397_stub_serial_clear_buffer(void)
{
    return ESP_OK;
}

esp_err_t bm1397_fake_receive_work(uint8_t *buffer, int size,
                                  uint64_t *timestamp_us)
{
    TEST_ASSERT_EQUAL_INT(sizeof(queued_response), size);
    if (!response_ready) {
        return ESP_FAIL;
    }
    memcpy(buffer, queued_response, sizeof(queued_response));
    response_ready = false;
    *timestamp_us = UINT64_C(123456789);
    return ESP_OK;
}

int bm1397_stub_count_chips(uint16_t count, uint16_t chip_id,
                           int response_length)
{
    TEST_ASSERT_EQUAL_HEX16(0x1397, chip_id);
    TEST_ASSERT_EQUAL_INT(BM1397_HARNESS_RESPONSE_SIZE, response_length);
    return count;
}

esp_err_t bm1397_stub_frequency_transition(
    GlobalState *state, set_hash_frequency_fn set_frequency)
{
    (void)state;
    (void)set_frequency;
    return ESP_OK;
}

void bm1397_stub_delay(TickType_t ticks)
{
    (void)ticks;
}
