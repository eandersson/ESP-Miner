#ifndef TEST_STUB_GLOBAL_STATE_H
#define TEST_STUB_GLOBAL_STATE_H

#include <stdbool.h>
#include <stdint.h>
#include <pthread.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "asic_init.h"
#include "scoreboard.h"

/* Mirrors main/device_config.h, which is not on the test include path. */
typedef enum
{
    BM1397,
    BM1366,
    BM1368,
    BM1370,
    BM1373,
} Asic;

/*
 * Shared test view for the real job task, result task and isolated driver
 * copies. Only the fields read by those production files belong here.
 */
typedef struct GlobalState {
    void *create_jobs_task_handle;
    volatile uint8_t active_job_slot_idx;
    struct {
        struct {
            uint16_t asic_count;
            uint8_t voltage_domains;
            struct {
                Asic id;
                bool hardware_version_rolling;
                uint8_t software_midstates;
                uint16_t difficulty;
                uint16_t core_count;
            } asic;
        } family;
    } DEVICE_CONFIG;
    struct {
        struct bm_job **active_jobs;
        struct bm_job **retired_jobs;
        int64_t *active_job_dispatch_us;
        int64_t *retired_job_dispatch_us;
        uint8_t *valid_jobs;
        pthread_mutex_t valid_jobs_lock;
    } ASIC_TASK_MODULE;
    struct {
        float frequency_value;
        float actual_frequency;
        float expected_hashrate;
    } POWER_MANAGEMENT_MODULE;
    volatile asic_lifecycle_state_t asic_lifecycle;
    volatile bool ASIC_initalized;
    pthread_mutex_t asic_command_lock;
    struct {
        float process_time;
        Scoreboard scoreboard;
        bool hardware_fault;
        char hardware_fault_msg[64];
    } SYSTEM_MODULE;
    struct {
        bool is_active;
    } SELF_TEST_MODULE;
} GlobalState;

#endif /* TEST_STUB_GLOBAL_STATE_H */
