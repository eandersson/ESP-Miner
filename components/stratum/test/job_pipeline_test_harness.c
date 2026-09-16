#include "mining_test_bindings.h"
#include "job_pipeline_test_harness.h"

#include <setjmp.h>
#include <string.h>

#include "esp_err.h"
#include "esp_timer.h"
#include "esp_psram.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "asic.h"
#include "asic_init.h"
#include "asic_reset.h"
#include "asic_result_task.h"
#include "global_state.h"
#include "serial.h"
#include "system.h"
#include "vcore.h"

#include "../../../main/tasks/create_jobs_task.h"

static jmp_buf harness_exit;
static const job_pipeline_harness_event_t *harness_events;
static size_t harness_event_count;
static size_t harness_event_index;
static job_pipeline_harness_result_t *harness_result;
static int harness_job_frequency_ms;
static bool harness_asic_running;
static int64_t harness_now_us;
static GlobalState harness_state;

/* The scheduler waits on task notifications with absolute deadlines. A
 * scripted timeout consumes the whole requested wait on a virtual clock, so
 * residency and retry deadlines expire exactly when the script says. */
static BaseType_t fake_task_notify_wait(
    uint32_t bits_to_clear_on_entry, uint32_t bits_to_clear_on_exit,
    uint32_t *notification_value, TickType_t ticks_to_wait)
{
    (void)bits_to_clear_on_entry;
    (void)bits_to_clear_on_exit;

    if (harness_event_index >= harness_event_count) {
        longjmp(harness_exit, 1);
    }

    const job_pipeline_harness_event_t *event =
        &harness_events[harness_event_index++];
    if (event->type == JOB_PIPELINE_HARNESS_NOTIFY) {
        *notification_value = event->slot;
        return pdTRUE;
    }
    harness_now_us += (int64_t)ticks_to_wait * portTICK_PERIOD_MS * 1000;
    return pdFALSE;
}

static int64_t fake_timer_get_time(void)
{
    return harness_now_us;
}

/* Counts any delay the scheduler adds instead of waiting on notifications. */
static __attribute__((unused)) void spy_task_delay(TickType_t ticks)
{
    harness_now_us += (int64_t)ticks * portTICK_PERIOD_MS * 1000;
    harness_result->delay_count++;
}

static void fake_task_delete(TaskHandle_t task)
{
    (void)task;
    longjmp(harness_exit, 3);
}

static bool spy_asic_send_work(GlobalState *state, bm_job *job,
                               uint32_t expected_generation)
{
    (void)state;
    (void)expected_generation;
    if (harness_result->job_count >= JOB_PIPELINE_HARNESS_MAX_JOBS) {
        longjmp(harness_exit, 2);
    }
    harness_result->jobs[harness_result->job_count++] = job;
    return true;
}

static esp_err_t spy_asic_set_version_mask(GlobalState *state, uint32_t mask)
{
    (void)state;
    if (harness_result->version_mask_count >= JOB_PIPELINE_HARNESS_MAX_JOBS) {
        longjmp(harness_exit, 2);
    }
    harness_result->version_masks[harness_result->version_mask_count++] = mask;
    return ESP_OK;
}

static double stub_asic_get_job_frequency(GlobalState *state)
{
    (void)state;
    return harness_job_frequency_ms;
}

static void spy_decode_coinbase(GlobalState *state, const miner_job_t *job)
{
    (void)state;
    (void)job;
    harness_result->coinbase_decode_count++;
}

static bool stub_lifecycle_is_running(const GlobalState *state)
{
    (void)state;
    return harness_asic_running;
}

static void stub_lifecycle_set(GlobalState *state, asic_lifecycle_state_t lifecycle)
{
    (void)state;
    (void)lifecycle;
}

static uint32_t stub_pool_generation(void)
{
    return 0;
}

static uint32_t stub_job_generation(void)
{
    return 1;
}

static bool stub_psram_is_initialized(void)
{
    return true;
}

static esp_err_t stub_serial_pause_tx(uint32_t drain_timeout_ms)
{
    (void)drain_timeout_ms;
    return ESP_OK;
}

static esp_err_t stub_hold_reset_low(void)
{
    return ESP_OK;
}

static esp_err_t stub_vcore_set_voltage(GlobalState *state, float voltage)
{
    (void)state;
    (void)voltage;
    return ESP_OK;
}

/* Test components cannot attach compile definitions to one source, so compile
 * the task into this test-only translation unit and interpose its boundaries. */
#ifdef xTaskNotifyWait
#undef xTaskNotifyWait
#endif
#ifdef vTaskDelay
#undef vTaskDelay
#endif
#define xTaskNotifyWait fake_task_notify_wait
#define esp_timer_get_time fake_timer_get_time
#define vTaskDelay spy_task_delay
#define vTaskDelete fake_task_delete
#define ASIC_send_work spy_asic_send_work
#define ASIC_set_version_mask_if_running spy_asic_set_version_mask
#define ASIC_get_asic_job_frequency_ms stub_asic_get_job_frequency
#define SYSTEM_decode_and_apply_coinbase spy_decode_coinbase
#define asic_lifecycle_is_running stub_lifecycle_is_running
#define asic_lifecycle_set stub_lifecycle_set
#define ASIC_result_task_get_pool_generation stub_pool_generation
#define ASIC_result_task_get_job_generation stub_job_generation
#define esp_psram_is_initialized stub_psram_is_initialized
#define SERIAL_pause_tx stub_serial_pause_tx
#define asic_hold_reset_low stub_hold_reset_low
#define VCORE_set_voltage stub_vcore_set_voltage
#include "../../../main/tasks/create_jobs_task.c"
#undef VCORE_set_voltage
#undef asic_hold_reset_low
#undef SERIAL_pause_tx
#undef esp_psram_is_initialized
#undef ASIC_result_task_get_job_generation
#undef ASIC_result_task_get_pool_generation
#undef asic_lifecycle_set
#undef asic_lifecycle_is_running
#undef SYSTEM_decode_and_apply_coinbase
#undef ASIC_get_asic_job_frequency_ms
#undef ASIC_set_version_mask_if_running
#undef ASIC_send_work
#undef vTaskDelete
#undef vTaskDelay
#undef esp_timer_get_time
#undef xTaskNotifyWait

void job_pipeline_harness_run(
    job_pipeline_harness_config_t config,
    const job_pipeline_harness_event_t *events, size_t event_count,
    job_pipeline_harness_result_t *result)
{
    if (result == NULL || event_count > JOB_PIPELINE_HARNESS_MAX_EVENTS ||
        (event_count > 0 && events == NULL)) {
        return;
    }

    memset(result, 0, sizeof(*result));
    harness_state = (GlobalState) {
        .DEVICE_CONFIG.family.asic.hardware_version_rolling =
            config.hardware_version_rolling,
        .DEVICE_CONFIG.family.asic.software_midstates =
            config.software_midstates,
    };
    pthread_mutex_init(&harness_state.asic_command_lock, NULL);

    harness_events = events;
    harness_event_count = event_count;
    harness_event_index = 0;
    harness_result = result;
    harness_job_frequency_ms = config.job_frequency_ms;
    harness_asic_running = config.asic_initialized;
    harness_now_us = 1000000;
    mining_allocator_fault_injector_reset(config.allocation_failure_at);

    int exit_reason = setjmp(harness_exit);
    if (exit_reason == 0) {
        create_jobs_task(&harness_state);
    }

    result->active_job_slot = harness_state.active_job_slot_idx;
    result->allocation_count = mining_allocator_fault_injector_calls();
    harness_events = NULL;
    harness_event_count = 0;
    harness_event_index = 0;
    harness_result = NULL;
    mining_allocator_fault_injector_reset(0);
    pthread_mutex_destroy(&harness_state.asic_command_lock);
}

void job_pipeline_harness_result_free(job_pipeline_harness_result_t *result)
{
    if (result == NULL) return;
    for (size_t index = 0; index < result->job_count; ++index) {
        release_bm_job(result->jobs[index]);
        result->jobs[index] = NULL;
    }
    result->job_count = 0;
}
