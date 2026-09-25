#include "result_task_test_bindings.h"

#include "asic.h"
#include "asic_common.h"
#include "asic_init.h"
#include "global_state.h"
#include "hashrate_monitor_task.h"
#include "mining.h"
#include "scoreboard.h"
#include "self_test.h"
#include "stratum_task.h"
#include "stratum_v1_client.h"
#include "stratum_v2_client.h"
#include "system.h"
#include "unity.h"

#include <float.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* The fake receive below needs the real FreeRTOS function. */
#undef xQueueReceive

enum { JOB_SLOTS = 128, JOB_ID = 8, MAX_EVENTS = 4 };

// nbits for a target just below 2^256: almost every hash solves the block.
#define EASIEST_NBITS 0x2100ffffu

static jmp_buf loop_exit;
static GlobalState fixture_state;
static bm_job *fixture_active[JOB_SLOTS];
static bm_job *fixture_retired[JOB_SLOTS];
static int64_t fixture_active_us[JOB_SLOTS];
static int64_t fixture_retired_us[JOB_SLOTS];
static uint8_t fixture_valid[JOB_SLOTS];
static task_result fixture_events[MAX_EVENTS];
static size_t fixture_event_count;
static size_t fixture_event_index;
static bool fixture_running;
static double fixture_announced_difficulty;
static int fixture_submit_result;
static uint64_t fixture_sent_time;
static bool fixture_replace_during_submit;

static unsigned fixture_delays;
static unsigned fixture_registers;
static unsigned fixture_v1_submissions;
static unsigned fixture_sv2_submissions;
static unsigned fixture_scores;
static unsigned fixture_notifications;
static unsigned fixture_blocks;
static unsigned fixture_self_tests;
static const bm_job *fixture_submitted_job;
static uint32_t fixture_submitted_version;
static uint32_t fixture_submitted_generation;
static char fixture_submitted_id[32];

void result_task_spy_delay(TickType_t ticks)
{
    TEST_ASSERT_EQUAL_UINT32(pdMS_TO_TICKS(100), ticks);
    fixture_delays++;
    fixture_running = true;
}

BaseType_t result_task_fake_queue_receive(QueueHandle_t queue, void *item,
                                          TickType_t ticks_to_wait)
{
    BaseType_t received = xQueueReceive(queue, item, 0);
    if (received != pdTRUE && ticks_to_wait == portMAX_DELAY) {
        longjmp(loop_exit, 2);
    }
    return received;
}

task_result *result_task_fake_process_work(GlobalState *state)
{
    TEST_ASSERT_EQUAL_PTR(&fixture_state, state);
    if (fixture_event_index >= fixture_event_count) {
        longjmp(loop_exit, 1);
    }
    return &fixture_events[fixture_event_index++];
}

double result_task_stub_job_frequency(GlobalState *state)
{
    TEST_ASSERT_EQUAL_PTR(&fixture_state, state);
    return 1000.0;
}

bool result_task_fake_lifecycle_running(const GlobalState *state)
{
    TEST_ASSERT_EQUAL_PTR(&fixture_state, state);
    return fixture_running;
}

double result_task_stub_current_difficulty(GlobalState *state)
{
    TEST_ASSERT_EQUAL_PTR(&fixture_state, state);
    return fixture_announced_difficulty;
}

static void record_submission(const bm_job *job, uint32_t version)
{
    TEST_ASSERT_NOT_NULL(job);
    TEST_ASSERT_EQUAL_UINT32(123, job->ntime);
    TEST_ASSERT_NOT_NULL(job->jobid);
    TEST_ASSERT_NOT_NULL(job->extranonce2);
    fixture_submitted_job = job;
    fixture_submitted_version = version;
    snprintf(fixture_submitted_id, sizeof(fixture_submitted_id), "%s", job->jobid);

    if (fixture_replace_during_submit) {
        // The slot's reference goes away while the share is on the wire; the
        // queued reference must keep the job alive.
        pthread_mutex_lock(&fixture_state.ASIC_TASK_MODULE.valid_jobs_lock);
        release_bm_job(fixture_active[JOB_ID]);
        fixture_active[JOB_ID] = NULL;
        fixture_valid[JOB_ID] = 0;
        pthread_mutex_unlock(&fixture_state.ASIC_TASK_MODULE.valid_jobs_lock);
        TEST_ASSERT_EQUAL_STRING("42", job->jobid);
    }
}

int result_task_fake_submit_v1(GlobalState *state, const bm_job *job,
                               uint32_t expected_generation, uint32_t nonce,
                               uint32_t version_bits, double share_difficulty,
                               uint64_t *sent_time_us)
{
    TEST_ASSERT_EQUAL_PTR(&fixture_state, state);
    TEST_ASSERT_EQUAL_HEX32(7, nonce);
    TEST_ASSERT_TRUE(share_difficulty > 0);
    fixture_submitted_generation = expected_generation;
    record_submission(job, version_bits);
    fixture_v1_submissions++;
    *sent_time_us = fixture_sent_time;
    return fixture_submit_result;
}

int result_task_fake_submit_share(GlobalState *state, const bm_job *job,
                                  uint32_t nonce, uint32_t rolled_version,
                                  uint64_t *sent_time)
{
    TEST_ASSERT_EQUAL_PTR(&fixture_state, state);
    TEST_ASSERT_EQUAL_HEX32(7, nonce);
    record_submission(job, rolled_version);
    fixture_sv2_submissions++;
    *sent_time = fixture_sent_time;
    return fixture_submit_result;
}

void result_task_spy_record_nonce(GlobalState *state, double difficulty)
{
    TEST_ASSERT_EQUAL_PTR(&fixture_state, state);
    TEST_ASSERT_TRUE(difficulty > 0);
    fixture_self_tests++;
}

void result_task_spy_notify_found_nonce(GlobalState *state, double difficulty)
{
    TEST_ASSERT_EQUAL_PTR(&fixture_state, state);
    TEST_ASSERT_TRUE(difficulty > 0);
    fixture_notifications++;
}

void result_task_spy_block_submitted(GlobalState *state, double difficulty,
                                     uint32_t target)
{
    TEST_ASSERT_EQUAL_PTR(&fixture_state, state);
    TEST_ASSERT_TRUE(difficulty > 0);
    TEST_ASSERT_EQUAL_HEX32(EASIEST_NBITS, target);
    fixture_blocks++;
}

esp_err_t result_task_spy_scoreboard_add(
    Scoreboard *scoreboard, double difficulty, const char *job_id,
    const char *extranonce, uint32_t ntime, uint32_t nonce,
    uint32_t version_bits)
{
    TEST_ASSERT_EQUAL_PTR(&fixture_state.SYSTEM_MODULE.scoreboard, scoreboard);
    TEST_ASSERT_TRUE(difficulty > 0);
    TEST_ASSERT_EQUAL_UINT32(123, ntime);
    TEST_ASSERT_EQUAL_UINT32(7, nonce);
    TEST_ASSERT_EQUAL_HEX32(0x00002000, version_bits);
    TEST_ASSERT_EQUAL_STRING("aabb", extranonce);
    TEST_ASSERT_NOT_NULL(job_id);
    fixture_scores++;
    return ESP_OK;
}

void result_task_spy_register_read(void *state, register_type_t type,
                                   uint8_t asic_nr, uint32_t value,
                                   uint64_t timestamp)
{
    TEST_ASSERT_EQUAL_PTR(&fixture_state, state);
    TEST_ASSERT_EQUAL(REGISTER_TOTAL_COUNT, type);
    TEST_ASSERT_EQUAL_UINT8(2, asic_nr);
    TEST_ASSERT_EQUAL_UINT32(55, value);
    TEST_ASSERT_TRUE(timestamp == UINT64_C(1000));
    fixture_registers++;
}

static bm_job *make_job(miner_job_type_t type, double pool_diff)
{
    bm_job *job = allocate_bm_job("42", "aabb");
    TEST_ASSERT_NOT_NULL(job);
    job->version = 0x20000004;
    job->version_mask = 0x1fffe000;
    job->version_rolling_enabled = true;
    job->ntime = 123;
    job->target = 0x1705dd01;
    job->pool_diff = pool_diff;
    job->job_type = type;
    job->session_id = 5;
    return job;
}

static void install_job(bm_job *job, int64_t dispatch_us)
{
    fixture_active[JOB_ID] = job;
    fixture_active_us[JOB_ID] = dispatch_us;
    fixture_valid[JOB_ID] = 1;
}

static void retire_job(bm_job *job, int64_t dispatch_us)
{
    fixture_retired[JOB_ID] = job;
    fixture_retired_us[JOB_ID] = dispatch_us;
}

static void queue_register_event(void)
{
    fixture_events[fixture_event_count++] = (task_result) {
        .register_type = REGISTER_TOTAL_COUNT,
        .asic_nr = 2,
        .value = 55,
        .timestamp_us = 1000,
    };
}

static void queue_nonce_event(void)
{
    fixture_events[fixture_event_count++] = (task_result) {
        .job_id = JOB_ID,
        .nonce = 7,
        .version_bits = 0x2000,
        .timestamp_us = 1000,
    };
}

static void fixture_begin(void)
{
    memset(&fixture_state, 0, sizeof(fixture_state));
    memset(fixture_active, 0, sizeof(fixture_active));
    memset(fixture_retired, 0, sizeof(fixture_retired));
    memset(fixture_active_us, 0, sizeof(fixture_active_us));
    memset(fixture_retired_us, 0, sizeof(fixture_retired_us));
    memset(fixture_valid, 0, sizeof(fixture_valid));
    memset(fixture_events, 0, sizeof(fixture_events));

    fixture_state.DEVICE_CONFIG.family.asic.id = BM1370;
    // A zero ticket lets any nonce reproduce, so only ownership and timing
    // decide how a result resolves.
    fixture_state.DEVICE_CONFIG.family.asic.difficulty = 0;
    fixture_state.ASIC_TASK_MODULE.active_jobs = fixture_active;
    fixture_state.ASIC_TASK_MODULE.retired_jobs = fixture_retired;
    fixture_state.ASIC_TASK_MODULE.active_job_dispatch_us = fixture_active_us;
    fixture_state.ASIC_TASK_MODULE.retired_job_dispatch_us = fixture_retired_us;
    fixture_state.ASIC_TASK_MODULE.valid_jobs = fixture_valid;
    TEST_ASSERT_EQUAL_INT(0, pthread_mutex_init(
        &fixture_state.ASIC_TASK_MODULE.valid_jobs_lock, NULL));

    TEST_ASSERT_EQUAL_INT(ESP_OK, ASIC_result_task_init());
    // A new job generation starts every case with empty queues.
    ASIC_result_task_reset();

    fixture_event_count = 0;
    fixture_event_index = 0;
    fixture_running = true;
    fixture_announced_difficulty = 1e-30;
    fixture_submit_result = 1;
    fixture_sent_time = 2000;
    fixture_replace_during_submit = false;
    fixture_delays = 0;
    fixture_registers = 0;
    fixture_v1_submissions = 0;
    fixture_sv2_submissions = 0;
    fixture_scores = 0;
    fixture_notifications = 0;
    fixture_blocks = 0;
    fixture_self_tests = 0;
    fixture_submitted_job = NULL;
    fixture_submitted_version = 0;
    fixture_submitted_generation = 0;
    fixture_submitted_id[0] = '\0';
}

static void fixture_end(void)
{
    for (size_t index = 0; index < JOB_SLOTS; ++index) {
        if (fixture_retired[index] != NULL && fixture_retired[index] != fixture_active[index]) {
            release_bm_job(fixture_retired[index]);
        }
        if (fixture_active[index] != NULL) {
            release_bm_job(fixture_active[index]);
        }
        fixture_active[index] = NULL;
        fixture_retired[index] = NULL;
    }
    TEST_ASSERT_EQUAL_INT(0, pthread_mutex_destroy(
        &fixture_state.ASIC_TASK_MODULE.valid_jobs_lock));
}

static void run_rx(void)
{
    if (setjmp(loop_exit) == 0) {
        ASIC_result_rx_task(&fixture_state);
    }
}

static void run_validation(void)
{
    if (setjmp(loop_exit) == 0) {
        ASIC_result_task(&fixture_state);
    }
}

static void run_submitter(void)
{
    if (setjmp(loop_exit) == 0) {
        ASIC_v1_share_submit_task(&fixture_state);
    }
}

static void run_pipeline(void)
{
    run_rx();
    run_validation();
    run_submitter();
}

TEST_CASE("result pipeline routes registers to the monitor and waits for RUNNING",
          "[asic][result]")
{
    fixture_begin();
    fixture_running = false;
    queue_register_event();
    run_pipeline();

    TEST_ASSERT_EQUAL_UINT32(1, fixture_delays);
    TEST_ASSERT_EQUAL_UINT32(1, fixture_registers);
    TEST_ASSERT_EQUAL_UINT32(
        0, fixture_v1_submissions + fixture_sv2_submissions + fixture_scores +
               fixture_notifications + fixture_self_tests);
    fixture_end();
}

TEST_CASE("result pipeline submits V1 shares from an owned job reference",
          "[asic][result][ownership]")
{
    fixture_begin();
    install_job(make_job(JOB_TYPE_V1, 1e-30), 500);
    fixture_replace_during_submit = true;
    queue_register_event();
    queue_nonce_event();
    run_pipeline();

    TEST_ASSERT_EQUAL_UINT32(1, fixture_registers);
    TEST_ASSERT_EQUAL_UINT32(1, fixture_v1_submissions);
    TEST_ASSERT_EQUAL_UINT32(0, fixture_sv2_submissions);
    TEST_ASSERT_EQUAL_UINT32(1, fixture_scores);
    TEST_ASSERT_EQUAL_UINT32(1, fixture_notifications);
    // An ordinary share is far from the mainnet target.
    TEST_ASSERT_EQUAL_UINT32(0, fixture_blocks);
    TEST_ASSERT_EQUAL_STRING("42", fixture_submitted_id);
    // BIP310 V1 submits carry the masked field of the rolled version.
    TEST_ASSERT_EQUAL_HEX32(0x00002000, fixture_submitted_version);
    TEST_ASSERT_EQUAL_UINT32(ASIC_result_task_get_job_generation(),
                             fixture_submitted_generation);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, fixture_state.SYSTEM_MODULE.process_time);
    fixture_end();
}

TEST_CASE("result pipeline submits SV2 shares directly with the rolled version",
          "[asic][result]")
{
    for (int type = JOB_TYPE_SV2_STANDARD; type <= JOB_TYPE_SV2_EXTENDED; ++type) {
        fixture_begin();
        install_job(make_job((miner_job_type_t)type, 1e-30), 500);
        queue_nonce_event();
        run_pipeline();

        TEST_ASSERT_EQUAL_UINT32(0, fixture_v1_submissions);
        TEST_ASSERT_EQUAL_UINT32(1, fixture_sv2_submissions);
        TEST_ASSERT_EQUAL_HEX32(0x20002004, fixture_submitted_version);
        TEST_ASSERT_EQUAL_UINT32(1, fixture_scores);
        TEST_ASSERT_EQUAL_FLOAT(1.0f, fixture_state.SYSTEM_MODULE.process_time);
        fixture_end();
    }
}

TEST_CASE("result pipeline drops results for invalid or missing job slots",
          "[asic][result][job-store]")
{
    for (int missing = 0; missing <= 1; ++missing) {
        fixture_begin();
        if (!missing) {
            install_job(make_job(JOB_TYPE_V1, 1e-30), 500);
            fixture_valid[JOB_ID] = 0;
        }
        asic_result_stats_t before;
        ASIC_result_task_get_stats(&before);
        queue_nonce_event();
        run_pipeline();

        asic_result_stats_t after;
        ASIC_result_task_get_stats(&after);
        TEST_ASSERT_EQUAL_UINT32(1, after.invalid_jobs - before.invalid_jobs);
        TEST_ASSERT_EQUAL_UINT32(
            0, fixture_v1_submissions + fixture_sv2_submissions + fixture_scores +
                   fixture_notifications + fixture_self_tests);
        fixture_end();
    }
}

TEST_CASE("result pipeline records self-test nonces without submitting",
          "[asic][result]")
{
    fixture_begin();
    fixture_state.SELF_TEST_MODULE.is_active = true;
    install_job(make_job(JOB_TYPE_V1, 1e-30), 500);
    queue_nonce_event();
    run_pipeline();

    TEST_ASSERT_EQUAL_UINT32(1, fixture_self_tests);
    TEST_ASSERT_EQUAL_UINT32(
        0, fixture_v1_submissions + fixture_sv2_submissions + fixture_scores +
               fixture_notifications);
    fixture_end();
}

TEST_CASE("result pipeline enforces share thresholds and drops repeated results",
          "[asic][result]")
{
    // Zero is not a usable difficulty and fails closed; DBL_MAX is unreachable.
    const double difficulties[] = {0, DBL_MAX};
    for (size_t index = 0; index < sizeof(difficulties) / sizeof(difficulties[0]); ++index) {
        fixture_begin();
        install_job(make_job(JOB_TYPE_V1, difficulties[index]), 500);
        queue_nonce_event();
        run_pipeline();
        TEST_ASSERT_EQUAL_UINT32(0, fixture_v1_submissions);
        TEST_ASSERT_EQUAL_UINT32(1, fixture_scores);
        TEST_ASSERT_EQUAL_UINT32(1, fixture_notifications);
        fixture_end();
    }

    // A higher difficulty announced after the job was built applies at once.
    fixture_begin();
    fixture_announced_difficulty = DBL_MAX;
    install_job(make_job(JOB_TYPE_V1, 1e-30), 500);
    queue_nonce_event();
    run_pipeline();
    TEST_ASSERT_EQUAL_UINT32(0, fixture_v1_submissions);
    fixture_end();

    fixture_begin();
    install_job(make_job(JOB_TYPE_V1, 1e-30), 500);
    queue_nonce_event();
    queue_nonce_event();
    asic_result_stats_t before;
    ASIC_result_task_get_stats(&before);
    run_pipeline();
    asic_result_stats_t after;
    ASIC_result_task_get_stats(&after);
    TEST_ASSERT_EQUAL_UINT32(1, fixture_v1_submissions);
    TEST_ASSERT_EQUAL_UINT32(1, fixture_scores);
    TEST_ASSERT_EQUAL_UINT32(1, after.duplicate_results - before.duplicate_results);
    fixture_end();
}

TEST_CASE("result pipeline submits block solutions below the share difficulty",
          "[asic][result][block]")
{
    // The pool asks for an unreachable share difficulty, but the nonce meets
    // the network target, as with a testnet minimum-difficulty block. Every
    // job type submits it, and the block counts only once it is on the wire.
    for (int type = JOB_TYPE_V1; type <= JOB_TYPE_SV2_EXTENDED; ++type) {
        for (int fails = 0; fails <= 1; ++fails) {
            fixture_begin();
            fixture_announced_difficulty = DBL_MAX;
            fixture_submit_result = fails ? -1 : 1;
            bm_job *job = make_job((miner_job_type_t)type, DBL_MAX);
            job->target = EASIEST_NBITS;
            TEST_ASSERT_TRUE(mining_share_solves_block(
                test_nonce_value(job, 7, 0x20002004), job->target));
            install_job(job, 500);
            queue_nonce_event();
            run_pipeline();

            TEST_ASSERT_EQUAL_UINT32(
                1, fixture_v1_submissions + fixture_sv2_submissions);
            TEST_ASSERT_EQUAL_UINT32(fails ? 0 : 1, fixture_blocks);
            fixture_end();
        }
    }
}

TEST_CASE("result pipeline resolves late nonces to the retired slot owner",
          "[asic][result][job-store]")
{
    // The replacement was dispatched after the nonce was read, so only the
    // retired owner can have produced it.
    fixture_begin();
    bm_job *retired = make_job(JOB_TYPE_V1, 1e-30);
    retire_job(retired, 400);
    install_job(make_job(JOB_TYPE_V1, 1e-30), 5000);
    queue_nonce_event();
    run_pipeline();
    TEST_ASSERT_EQUAL_UINT32(1, fixture_v1_submissions);
    TEST_ASSERT_EQUAL_PTR(retired, fixture_submitted_job);
    fixture_end();

    // Both owners were on the chip when the nonce was read and both reproduce
    // it: do not guess.
    fixture_begin();
    retire_job(make_job(JOB_TYPE_V1, 1e-30), 400);
    install_job(make_job(JOB_TYPE_V1, 1e-30), 500);
    asic_result_stats_t before;
    ASIC_result_task_get_stats(&before);
    queue_nonce_event();
    run_pipeline();
    asic_result_stats_t after;
    ASIC_result_task_get_stats(&after);
    TEST_ASSERT_EQUAL_UINT32(0, fixture_v1_submissions);
    TEST_ASSERT_EQUAL_UINT32(1, after.ambiguous_jobs - before.ambiguous_jobs);
    fixture_end();
}

TEST_CASE("result pipeline discards shares queued before an invalidation",
          "[asic][result]")
{
    fixture_begin();
    install_job(make_job(JOB_TYPE_V1, 1e-30), 500);
    queue_nonce_event();
    run_rx();
    run_validation();

    asic_result_stats_t before;
    ASIC_result_task_get_stats(&before);
    ASIC_result_task_invalidate_jobs();
    run_submitter();
    asic_result_stats_t after;
    ASIC_result_task_get_stats(&after);

    TEST_ASSERT_EQUAL_UINT32(0, fixture_v1_submissions);
    TEST_ASSERT_EQUAL_UINT32(1, after.stale_shares - before.stale_shares);
    fixture_end();
}

TEST_CASE("result pipeline counts submit worker outcomes",
          "[asic][result]")
{
    static const struct {
        int submit_result;
        bool stale;
    } cases[] = {
        {STRATUM_V1_SUBMIT_STALE, true},
        {STRATUM_V1_SUBMIT_FILTERED, false},
        {-1, false},
    };
    for (size_t index = 0; index < sizeof(cases) / sizeof(cases[0]); ++index) {
        fixture_begin();
        fixture_submit_result = cases[index].submit_result;
        install_job(make_job(JOB_TYPE_V1, 1e-30), 500);
        queue_nonce_event();
        asic_result_stats_t before;
        ASIC_result_task_get_stats(&before);
        run_pipeline();
        asic_result_stats_t after;
        ASIC_result_task_get_stats(&after);

        TEST_ASSERT_EQUAL_UINT32(1, fixture_v1_submissions);
        TEST_ASSERT_EQUAL_UINT32(0, after.shares_submitted - before.shares_submitted);
        TEST_ASSERT_EQUAL_UINT32(cases[index].stale ? 1 : 0,
                                 after.stale_shares - before.stale_shares);
        TEST_ASSERT_EQUAL_UINT32(cases[index].stale ? 0 : 1,
                                 after.shares_dropped - before.shares_dropped);
        TEST_ASSERT_EQUAL_FLOAT(0, fixture_state.SYSTEM_MODULE.process_time);
        fixture_end();
    }
}
