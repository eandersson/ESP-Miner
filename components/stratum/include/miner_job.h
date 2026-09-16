#ifndef MINER_JOB_H_
#define MINER_JOB_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef enum {
    JOB_TYPE_V1 = 0,
    JOB_TYPE_SV2_STANDARD,
    JOB_TYPE_SV2_EXTENDED,
} miner_job_type_t;

#define MAX_COINBASE_PREFIX_LEN 1024
#define MAX_COINBASE_SUFFIX_LEN 64512
#define MAX_COINBASE_BIN_LEN    MAX_COINBASE_SUFFIX_LEN
#define MAX_MERKLE_BRANCHES 32
#define MAX_JOB_ID_LEN 32
#define MINER_JOB_POOL_SIZE 8

typedef struct {
    miner_job_type_t type;
    char             job_id[MAX_JOB_ID_LEN];
    uint32_t         version;
    uint8_t          prev_hash[32]; // binary 32 bytes
    uint32_t         ntime;
    uint32_t         nbits;
    bool             clean_jobs;

    // Multi-pool difficulty and version rolling configuration
    double           pool_diff;
    uint32_t         version_mask;

    // Extranonce configuration for this job / channel / pool
    uint8_t          extranonce1[32];
    uint8_t          extranonce1_len;
    uint8_t          extranonce2_len;
    uint8_t          pool_id;
    // Stratum connection that produced this job (see bm_job.session_id).
    uint32_t         session_id;
    // Job-invalidation epoch at publication (ASIC_result_task pool
    // generation). The scheduler drops work published before a clean job or
    // disconnect instead of mining it for a dead session.
    uint32_t         pool_generation;

    // Merkle tree branches (32-byte binary hashes)
    uint8_t          merkle_path[MAX_MERKLE_BRANCHES][32];
    uint8_t          merkle_path_count;

    // Coinbase binary payload (pointing to PSRAM-allocated buffers)
    uint8_t         *coinbase_prefix;
    uint16_t         coinbase_prefix_len;
    uint8_t         *coinbase_suffix;
    uint16_t         coinbase_suffix_len;

    // Pre-computed Merkle root (for SV2 Standard)
    uint8_t          merkle_root[32];
} miner_job_t;

// Pre-allocated ring buffer pool helpers
void miner_job_pool_init(void);
miner_job_t *miner_job_get_slot(size_t index);

// Serializes ring-slot writers (stratum clients, self-test) with the job
// scheduler's copy-out. Hold it only while writing a slot or copying one out;
// never across network or UART I/O.
void miner_job_lock(void);
void miner_job_unlock(void);

// Allocate full-size coinbase buffers for a job kept outside the ring (the
// scheduler's working copy, a client's parse buffer). Returns false on OOM.
bool miner_job_alloc_buffers(miner_job_t *job);
void miner_job_free_buffers(miner_job_t *job);

// Copy every field and the used coinbase bytes of src into dst while keeping
// dst's own buffers. Both jobs must own full-size coinbase buffers.
void miner_job_copy(miner_job_t *dst, const miner_job_t *src);

static inline bool miner_job_is_rollable(const miner_job_t *job)
{
    return (job != NULL) && (job->extranonce2_len > 0) && (job->coinbase_prefix_len > 0);
}

#endif /* MINER_JOB_H_ */
