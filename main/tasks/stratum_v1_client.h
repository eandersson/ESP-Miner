#ifndef STRATUM_V1_CLIENT_H_
#define STRATUM_V1_CLIENT_H_

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include "mining.h"

typedef struct GlobalState GlobalState;

// Negative and distinct from the stratum_socket write results (-1/-2) and
// STRATUM_V1_SUBMIT_FORMAT_ERROR (-3), so the submit worker can tell policy
// outcomes from local formatting and transport failures. STALE covers a share
// whose job generation moved or whose connection is gone or closing.
enum {
    STRATUM_V1_SUBMIT_FILTERED = -4,
    STRATUM_V1_SUBMIT_STALE = -5,
};

// Run the Stratum V1 client loop for pool_idx until disconnect, ASIC pause, or reconnect requested.
esp_err_t stratum_v1_run(GlobalState *GLOBAL_STATE, uint16_t pool_idx);

void stratum_v1_close_connection(GlobalState *GLOBAL_STATE);

// Latest difficulty announced on the live connection (0 when disconnected).
double stratum_v1_get_current_difficulty(GlobalState *GLOBAL_STATE);

// Submit a share on the connection that issued the job. Rejects the share
// without writing when its generation or session is stale, or when it is below
// max(job difficulty, currently announced difficulty). A write failure closes
// the exact socket used, so the client reconnects.
int stratum_v1_submit_share_checked(GlobalState *GLOBAL_STATE, const bm_job *job,
                                    uint32_t expected_generation, uint32_t nonce,
                                    uint32_t version_bits, double share_difficulty,
                                    uint64_t *sent_time_us);

// Submit a solved ASIC share to the active Stratum V1 pool connection
int stratum_v1_submit_share(GlobalState *GLOBAL_STATE, const bm_job *active_job, uint32_t nonce, uint32_t rolled_version, uint64_t *sent_time_us);

// Probe a Stratum V1 pool to check reachability and credentials
bool stratum_v1_probe_pool(GlobalState *GLOBAL_STATE, uint16_t pool_idx);

#endif // STRATUM_V1_CLIENT_H_
