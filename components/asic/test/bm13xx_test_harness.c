#include "bm13xx_test_bindings.h"
#include "bm13xx_test_harness.h"

#include <pthread.h>
#include <string.h>
#include "asic_init.h"
#include "bm1366.h"
#include "bm1368.h"
#include "bm1370.h"
#include "bm1373.h"
#include "frequency_transition_bmXX.h"
#include "global_state.h"
#include "mining.h"
#include "serial.h"
#include "unity.h"

const bm13xx_harness_driver_t
    bm13xx_harness_drivers[BM13XX_HARNESS_DRIVER_COUNT] = {
        {"BM1366", BM1366_init, BM1366_send_work, BM1366_set_version_mask, BM1366_set_ticket_difficulty, BM1366_process_work, 0x13, 4},
        {"BM1368", BM1368_init, BM1368_send_work, BM1368_set_version_mask, BM1368_set_ticket_difficulty, BM1368_process_work, 0x23, 5},
        {"BM1370", BM1370_init, BM1370_send_work, BM1370_set_version_mask, BM1370_set_ticket_difficulty, BM1370_process_work, 0x23, 4},
        {"BM1373", BM1373_init, BM1373_send_work, BM1373_set_version_mask, BM1373_set_ticket_difficulty, BM1373_process_work, 0x23, 1},
    };

enum { MAX_PACKETS = 128, JOB_SLOTS = 128 };
static GlobalState fixture_state;
static bm_job *active_jobs[JOB_SLOTS];
static bm_job *retired_jobs[JOB_SLOTS];
static int64_t active_dispatch_us[JOB_SLOTS];
static int64_t retired_dispatch_us[JOB_SLOTS];
static uint8_t valid_jobs[JOB_SLOTS];
static bm13xx_harness_packet_t packets[MAX_PACKETS];
static size_t packet_count;
static uint8_t queued_response[BM13XX_HARNESS_RESPONSE_SIZE];
static bool response_ready;
static unsigned failed_writes;
static unsigned delay_count;
static uint32_t job_generation;

GlobalState *bm13xx_harness_begin(void)
{
    memset(active_jobs, 0, sizeof(active_jobs));
    memset(retired_jobs, 0, sizeof(retired_jobs));
    memset(active_dispatch_us, 0, sizeof(active_dispatch_us));
    memset(retired_dispatch_us, 0, sizeof(retired_dispatch_us));
    memset(valid_jobs, 0, sizeof(valid_jobs));
    fixture_state = (GlobalState) {
        .DEVICE_CONFIG.family.asic_count = 2,
        .DEVICE_CONFIG.family.voltage_domains = 1,
        .DEVICE_CONFIG.family.asic.difficulty = 256,
        .DEVICE_CONFIG.family.asic.core_count = 112,
        .POWER_MANAGEMENT_MODULE.frequency_value = 200.0f,
        .POWER_MANAGEMENT_MODULE.actual_frequency = 200.0f,
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
    failed_writes = 0;
    delay_count = 0;
    job_generation = 1;
    return &fixture_state;
}

void bm13xx_harness_end(void)
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
    /* Also catches a driver returning without releasing the job lock. */
    TEST_ASSERT_EQUAL_INT(0, pthread_mutex_trylock(
        &fixture_state.ASIC_TASK_MODULE.valid_jobs_lock));
    TEST_ASSERT_EQUAL_INT(0, pthread_mutex_unlock(
        &fixture_state.ASIC_TASK_MODULE.valid_jobs_lock));
    TEST_ASSERT_EQUAL_INT(0, pthread_mutex_destroy(
        &fixture_state.ASIC_TASK_MODULE.valid_jobs_lock));
}

void bm13xx_harness_clear_packets(void)
{
    packet_count = 0;
    delay_count = 0;
}

size_t bm13xx_harness_packet_count(void)
{
    return packet_count;
}

const bm13xx_harness_packet_t *bm13xx_harness_packet(size_t index)
{
    TEST_ASSERT_TRUE(index < packet_count);
    return &packets[index];
}

bm_job *bm13xx_harness_active_job(uint8_t job_id)
{
    TEST_ASSERT_TRUE(job_id < JOB_SLOTS);
    return active_jobs[job_id];
}

bm_job *bm13xx_harness_retired_job(uint8_t job_id)
{
    TEST_ASSERT_TRUE(job_id < JOB_SLOTS);
    return retired_jobs[job_id];
}

bool bm13xx_harness_job_valid(uint8_t job_id)
{
    TEST_ASSERT_TRUE(job_id < JOB_SLOTS);
    return valid_jobs[job_id] != 0;
}

void bm13xx_harness_install_job(uint8_t job_id, bm_job *job)
{
    TEST_ASSERT_TRUE(job_id < JOB_SLOTS);
    TEST_ASSERT_NULL(active_jobs[job_id]);
    active_jobs[job_id] = job;
    active_dispatch_us[job_id] = 1;
    valid_jobs[job_id] = 1;
}

void bm13xx_harness_fail_writes(unsigned count)
{
    failed_writes = count;
}

unsigned bm13xx_harness_delay_count(void)
{
    return delay_count;
}

uint32_t bm13xx_harness_generation(void)
{
    return job_generation;
}

void bm13xx_harness_set_generation(uint32_t generation)
{
    job_generation = generation;
}

void bm13xx_harness_set_running(bool running)
{
    fixture_state.asic_lifecycle =
        running ? ASIC_LIFECYCLE_RUNNING : ASIC_LIFECYCLE_STOPPING;
}

void bm13xx_harness_queue_response(
    const uint8_t response[BM13XX_HARNESS_RESPONSE_SIZE])
{
    response_ready = response != NULL;
    if (response_ready) {
        memcpy(queued_response, response, sizeof(queued_response));
    }
}

uint32_t bm13xx_stub_job_generation(void)
{
    return job_generation;
}

bool bm13xx_spy_serial_send(const uint8_t *bytes, size_t length, bool debug)
{
    (void)debug;
    TEST_ASSERT_TRUE(packet_count < MAX_PACKETS);
    TEST_ASSERT_TRUE(length > 0 && length <= sizeof(packets[0].bytes));
    bm13xx_harness_packet_t *packet = &packets[packet_count++];
    packet->length = length;
    memcpy(packet->bytes, bytes, packet->length);
    if (failed_writes > 0) {
        failed_writes--;
        return false;
    }
    return true;
}

esp_err_t bm13xx_stub_serial_set_baud(int baud)
{
    TEST_ASSERT_EQUAL_INT(3000000, baud);
    return ESP_OK;
}

esp_err_t bm13xx_stub_serial_clear_buffer(void)
{
    return ESP_OK;
}

esp_err_t bm13xx_fake_receive_work(uint8_t *buffer, int size,
                                  uint64_t *timestamp_us)
{
    TEST_ASSERT_EQUAL_INT(sizeof(queued_response), size);
    if (!response_ready) {
        return ESP_FAIL;
    }
    memcpy(buffer, queued_response, sizeof(queued_response));
    response_ready = false;
    *timestamp_us = 123456789;
    return ESP_OK;
}

int bm13xx_stub_count_chips(uint16_t count, uint16_t chip_id,
                           int response_length)
{
    (void)chip_id;
    TEST_ASSERT_EQUAL_INT(11, response_length);
    return count;
}

int bm13xx_stub_count_chips_with_alias(
    uint16_t count, uint16_t chip_id, uint16_t alias, int response_length)
{
    TEST_ASSERT_EQUAL_HEX16(0x1372, chip_id);
    TEST_ASSERT_EQUAL_HEX16(0x1373, alias);
    TEST_ASSERT_EQUAL_INT(9, response_length);
    return count;
}

esp_err_t bm13xx_stub_frequency_transition(
    GlobalState *state, set_hash_frequency_fn set_frequency)
{
    /* Clock ramps are outside the version-rolling contract. */
    (void)state;
    (void)set_frequency;
    return ESP_OK;
}

void bm13xx_spy_delay(TickType_t ticks)
{
    (void)ticks;
    delay_count++;
}
