#ifndef RESULT_TASK_TEST_BINDINGS_H_
#define RESULT_TASK_TEST_BINDINGS_H_

#include <inttypes.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/* The RX, validation and V1 submit loops run unchanged. Their hardware,
 * network and scoring boundaries are test doubles, and a blocking queue
 * receive on an empty queue leaves the loop instead of waiting forever. */
#ifdef vTaskDelay
#undef vTaskDelay
#endif
#define vTaskDelay result_task_spy_delay
#define xQueueReceive result_task_fake_queue_receive
#define ASIC_process_work result_task_fake_process_work
#define ASIC_get_asic_job_frequency_ms result_task_stub_job_frequency
#define asic_lifecycle_is_running result_task_fake_lifecycle_running
#define stratum_submit_share result_task_fake_submit_share
#define stratum_v1_submit_share_checked result_task_fake_submit_v1
#define stratum_v1_get_current_difficulty result_task_stub_current_difficulty
#define self_test_record_nonce result_task_spy_record_nonce
#define SYSTEM_notify_found_nonce result_task_spy_notify_found_nonce
#define SYSTEM_notify_block_submitted result_task_spy_block_submitted
#define scoreboard_add result_task_spy_scoreboard_add
#define hashrate_monitor_register_read result_task_spy_register_read

void result_task_spy_delay(TickType_t ticks);
BaseType_t result_task_fake_queue_receive(QueueHandle_t queue, void *item,
                                          TickType_t ticks_to_wait);

#include "../../../main/tasks/asic_result_task.h"

#endif /* RESULT_TASK_TEST_BINDINGS_H_ */
