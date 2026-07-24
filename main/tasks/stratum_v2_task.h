#ifndef STRATUM_V2_TASK_H
#define STRATUM_V2_TASK_H

#include <stdbool.h>
#include <stdint.h>

typedef struct GlobalState GlobalState;

typedef struct
{
    uint8_t extranonce_prefix[32];
    uint8_t extranonce_prefix_len;
    uint8_t extranonce_size;
} stratum_v2_extended_work_snapshot_t;

void stratum_v2_task(void *pvParameters);
void stratum_v2_close_connection(GlobalState *GLOBAL_STATE);
void stratum_v2_interrupt_connection(GlobalState *GLOBAL_STATE);
int stratum_v2_submit_share(GlobalState *GLOBAL_STATE, uint32_t job_id, uint32_t nonce,
                             uint32_t ntime, uint32_t version,
                             uint32_t expected_generation);
int stratum_v2_submit_share_extended(GlobalState *GLOBAL_STATE, uint32_t job_id,
                                     uint32_t nonce, uint32_t ntime, uint32_t version,
                                     const uint8_t *extranonce, uint8_t extranonce_len,
                                     uint32_t expected_generation);
bool stratum_v2_is_extended_channel(GlobalState *GLOBAL_STATE);
bool stratum_v2_snapshot_extended_work(
    GlobalState *GLOBAL_STATE, uint32_t expected_generation,
    stratum_v2_extended_work_snapshot_t *snapshot);

#endif // STRATUM_V2_TASK_H
