#ifndef STRATUM_V1_TASK_H_
#define STRATUM_V1_TASK_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct GlobalState GlobalState;
typedef struct mining_notify mining_notify;

typedef struct
{
    mining_notify *notification;
    char *extranonce_1;
    uint32_t extranonce_2_len;
    double difficulty;
    uint32_t version_mask;
    bool version_rolling_enabled;
} stratum_v1_work;

// Negative and distinct from the stratum_socket write results (-1/-2) and
// STRATUM_V1_SUBMIT_FORMAT_ERROR (-3), so the submit worker can distinguish
// policy outcomes from local formatting and transport failures. STALE covers
// a share whose generation moved or whose connection is already closing.
enum {
    STRATUM_V1_SUBMIT_FILTERED = -4,
    STRATUM_V1_SUBMIT_STALE = -5,
};

void stratum_v1_task(void *pvParameters);
void stratum_v1_close_connection(GlobalState *GLOBAL_STATE);
void stratum_v1_interrupt_connection(GlobalState *GLOBAL_STATE);
double stratum_v1_get_current_difficulty(GlobalState *GLOBAL_STATE);
int stratum_v1_submit_share_safe(
    GlobalState *GLOBAL_STATE, uint32_t expected_generation, int uid,
    const char *user, const char *job_id, const char *extranonce_2,
    uint32_t ntime, uint32_t nonce, bool version_rolling_enabled,
    uint32_t version_bits, double share_difficulty, double job_difficulty,
    uint64_t *sent_time_us);

#endif // STRATUM_V1_TASK_H_
