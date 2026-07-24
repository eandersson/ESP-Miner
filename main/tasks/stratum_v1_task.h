#ifndef STRATUM_V1_TASK_H_
#define STRATUM_V1_TASK_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct GlobalState GlobalState;

void stratum_v1_task(void *pvParameters);
void stratum_v1_close_connection(GlobalState *GLOBAL_STATE);
void stratum_v1_interrupt_connection(GlobalState *GLOBAL_STATE);
bool stratum_v1_snapshot_extranonce(GlobalState *GLOBAL_STATE,
                                    char **extranonce,
                                    uint32_t *extranonce_2_len);
int stratum_v1_submit_share_safe(
    GlobalState *GLOBAL_STATE, uint32_t expected_generation, int uid,
    const char *user, const char *job_id, const char *extranonce_2,
    uint32_t ntime, uint32_t nonce, uint32_t version_bits,
    uint64_t *sent_time_us);

#endif // STRATUM_V1_TASK_H_
