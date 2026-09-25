#include <string.h>
#include <stdio.h>
#include <limits.h>
#include "esp_log.h"
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdatomic.h>
#include <stdlib.h>
#include "mining.h"
#include "stratum_api.h"
#include "utils.h"
#include "psa/crypto.h"

typedef struct
{
    atomic_uint ref_count;
    bm_job job;
    char metadata[];
} allocated_bm_job_t;

static allocated_bm_job_t *get_job_allocation(bm_job *job)
{
    return (allocated_bm_job_t *)((char *)job -
                                  offsetof(allocated_bm_job_t, job));
}

bm_job *allocate_bm_job(const char *jobid, const char *extranonce2)
{
    if (jobid == NULL || extranonce2 == NULL) {
        return NULL;
    }

    size_t jobid_size = strlen(jobid) + 1;
    size_t extranonce2_size = strlen(extranonce2) + 1;
    if (jobid_size > SIZE_MAX - extranonce2_size ||
        sizeof(allocated_bm_job_t) >
            SIZE_MAX - jobid_size - extranonce2_size) {
        return NULL;
    }

    allocated_bm_job_t *allocation = calloc(
        1, sizeof(*allocation) + jobid_size + extranonce2_size);
    if (allocation == NULL) {
        return NULL;
    }

    bm_job *job = &allocation->job;
    char *metadata = allocation->metadata;
    job->jobid = metadata;
    memcpy(job->jobid, jobid, jobid_size);
    job->extranonce2 = metadata + jobid_size;
    memcpy(job->extranonce2, extranonce2, extranonce2_size);
    atomic_init(&allocation->ref_count, 1);
    return job;
}

void retain_bm_job(bm_job *job)
{
    if (job != NULL) {
        allocated_bm_job_t *allocation = get_job_allocation(job);
        atomic_fetch_add_explicit(&allocation->ref_count, 1,
                                  memory_order_relaxed);
    }
}

void release_bm_job(bm_job *job)
{
    if (job == NULL) {
        return;
    }

    // The caller owns the initial reference and transfers it to active_jobs on
    // a successful send. Result queue entries retain their own references so a
    // reused slot cannot free metadata that a pending result still needs.
    allocated_bm_job_t *allocation = get_job_allocation(job);
    if (atomic_fetch_sub_explicit(&allocation->ref_count, 1,
                                  memory_order_acq_rel) != 1) {
        return;
    }

    free(allocation);
}

void free_bm_job(bm_job *job)
{
    if (job == NULL) {
        return;
    }
    free(job->jobid);
    free(job->extranonce2);
    free(job);
}


void calculate_coinbase_tx_hash_bin(const uint8_t *prefix, size_t prefix_len,
                                    const uint8_t *extranonce_prefix, size_t ep_len,
                                    const uint8_t *extranonce_2, size_t e2_len,
                                    const uint8_t *suffix, size_t suffix_len,
                                    uint8_t dest[32])
{
    // Hash the four coinbase segments directly. Extended-channel work is
    // regenerated frequently, so avoiding a temporary allocation here reduces
    // heap churn and ensures allocation pressure cannot turn an interval into
    // invalid work with an uninitialized coinbase hash.
    uint8_t first_hash[32];
    size_t first_hash_len = 0;
    psa_hash_operation_t operation = PSA_HASH_OPERATION_INIT;
    psa_status_t status = psa_hash_setup(&operation, PSA_ALG_SHA_256);
    if (status == PSA_SUCCESS) status = psa_hash_update(&operation, prefix, prefix_len);
    if (status == PSA_SUCCESS) status = psa_hash_update(&operation, extranonce_prefix, ep_len);
    if (status == PSA_SUCCESS) status = psa_hash_update(&operation, extranonce_2, e2_len);
    if (status == PSA_SUCCESS) status = psa_hash_update(&operation, suffix, suffix_len);
    if (status == PSA_SUCCESS) {
        status = psa_hash_finish(&operation, first_hash, sizeof(first_hash), &first_hash_len);
    }
    psa_hash_abort(&operation);

    if (status != PSA_SUCCESS || first_hash_len != sizeof(first_hash)) {
        memset(dest, 0, 32);
        return;
    }
    sha256_bin(first_hash, sizeof(first_hash), dest);
}

void construct_bm_job_from_miner_job(const miner_job_t *job, const uint32_t version, const uint8_t merkle_root[32], const uint32_t version_mask, const double difficulty, const uint8_t software_midstates, bm_job *new_job)
{
    new_job->version = (version != 0) ? version : job->version;
    new_job->target = job->nbits;
    new_job->ntime = job->ntime;
    new_job->starting_nonce = 0;
    new_job->pool_diff = (job->pool_diff > 0) ? job->pool_diff : difficulty;
    new_job->pool_id = job->pool_id;
    new_job->job_type = job->type;
    new_job->session_id = job->session_id;
    uint32_t effective_mask = (job->version_mask != 0) ? job->version_mask : version_mask;
    new_job->version_mask = effective_mask;
    // BIP310: without a negotiated mask the V1 submit carries no version
    // field, and the base version is used unchanged.
    new_job->version_rolling_enabled = effective_mask != 0;
    new_job->num_midstates = 0;
    reverse_32bit_words(merkle_root, new_job->merkle_root);
    reverse_32bit_words(job->prev_hash, new_job->prev_block_hash);

    if (software_midstates == 0)
    {
        return;
    }

    // make the midstate hash
    uint8_t midstate_data[64];
    memcpy(midstate_data + 4, job->prev_hash, 32);
    memcpy(midstate_data + 36, merkle_root, 28);

    uint32_t current_ver = new_job->version;
    uint8_t midstate[32];

    for (int i = 0; i < software_midstates && i < BM_JOB_MAX_MIDSTATES; i++)
    {
        if (i > 0)
        {
            if (effective_mask == 0)
            {
                break;
            }
            current_ver = increment_bitmask(current_ver, effective_mask);
        }
        memcpy(midstate_data, &current_ver, 4);
        midstate_sha256_bin(midstate_data, 64, midstate);
        reverse_32bit_words(midstate, new_job->midstates[i]);
        new_job->num_midstates++;
    }
}

void calculate_merkle_root_hash(const uint8_t coinbase_tx_hash[32], const uint8_t merkle_branches[][32], const int num_merkle_branches, uint8_t dest[32])
{
    uint8_t both_merkles[64];
    memcpy(both_merkles, coinbase_tx_hash, 32);
    for (int i = 0; i < num_merkle_branches; i++) {
        memcpy(both_merkles + 32, merkle_branches[i], 32);
        double_sha256_bin(both_merkles, 64, both_merkles);
    }

    memcpy(dest, both_merkles, 32);
}


bool extranonce_2_generate(uint64_t extranonce_2, uint32_t length,
                           char *dest, size_t dest_size)
{
    if (dest == NULL || length > MAX_EXTRANONCE_2_LEN ||
        dest_size < (size_t)length * 2 + 1) {
        return false;
    }

    // A fixed-size buffer also handles a valid zero-byte extranonce without a
    // zero-length VLA. The scheduler prevents the uint64_t counter from
    // wrapping, so each generated value is unique for the current template.
    uint8_t extranonce_2_bytes[MAX_EXTRANONCE_2_LEN] = {0};
    
    // Copy the extranonce_2 value into the buffer, handling endianness
    // Copy up to the size of uint64_t or the requested length, whichever is smaller
    size_t copy_len = (length < sizeof(uint64_t)) ? length : sizeof(uint64_t);
    memcpy(extranonce_2_bytes, &extranonce_2, copy_len);
    
    // Convert the bytes to hex string
    bin2hex(extranonce_2_bytes, length, dest, dest_size);
    return true;
}

bool extranonce_2_increment(uint64_t *extranonce_2, uint32_t length)
{
    if (extranonce_2 == NULL || length > MAX_EXTRANONCE_2_LEN ||
        length == 0) {
        return false;
    }

    uint64_t maximum = UINT64_MAX;
    if (length < sizeof(uint64_t)) {
        maximum = (UINT64_C(1) << (length * CHAR_BIT)) - 1;
    }
    if (*extranonce_2 >= maximum) {
        return false;
    }

    (*extranonce_2)++;
    return true;
}

double mining_v1_effective_share_difficulty(double job_difficulty,
                                            double announced_difficulty)
{
    // Fail closed if either side is not a finite positive value. Normal jobs
    // are validated before reaching this helper; DBL_MAX prevents corrupted
    // connection state from putting a below-target share on the wire.
    if (!isfinite(job_difficulty) || !(job_difficulty > 0.0) ||
        !isfinite(announced_difficulty) ||
        !(announced_difficulty > 0.0)) {
        return DBL_MAX;
    }

    return job_difficulty > announced_difficulty
               ? job_difficulty
               : announced_difficulty;
}

bool mining_share_solves_block(double share_difficulty, uint32_t nbits)
{
    // Both sides are truediffone divided by a 256-bit value, so this is
    // hash <= target. Without a target (nbits 0) nothing is a block.
    double network_difficulty = networkDifficulty(nbits);
    return isfinite(network_difficulty) && network_difficulty > 0.0 &&
           share_difficulty >= network_difficulty;
}

uint16_t mining_ticket_difficulty(uint16_t configured_ticket, uint32_t nbits)
{
    if (configured_ticket == 0) {
        return 0;
    }
    uint16_t ticket = 1;
    while (ticket <= configured_ticket / 2) {
        ticket <<= 1;
    }
    double network_difficulty = networkDifficulty(nbits);
    if (isfinite(network_difficulty) && network_difficulty > 0.0) {
        while (ticket > 1 && (double)ticket > network_difficulty) {
            ticket >>= 1;
        }
    }
    return ticket;
}

double hash_to_pdiff(const uint8_t hash[32])
{
    if (!hash) return (double)UINT32_MAX;
    double s64 = le256todouble(hash);
    if (s64 <= 0.0 || isnan(s64) || isinf(s64)) return (double)UINT32_MAX;
    double diff = truediffone / s64;
    if (isnan(diff) || isinf(diff) || diff <= 0.0) return (double)UINT32_MAX;
    return diff;
}

///////cgminer nonce testing
/* testing a nonce and return the diff - 0 means invalid */
double test_nonce_value(const bm_job *job, const uint32_t nonce, const uint32_t rolled_version)
{
    uint8_t header[80];

    // // TODO: use the midstate hash instead of hashing the whole header
    // uint32_t rolled_version = job->version;
    // for (int i = 0; i < midstate_index; i++) {
    //     rolled_version = increment_bitmask(rolled_version, job->version_mask);
    // }

    // copy data from job to header
    memcpy(header, &rolled_version, 4);
    reverse_32bit_words(job->prev_block_hash, header + 4);
    reverse_32bit_words(job->merkle_root, header + 36);
    memcpy(header + 68, &job->ntime, 4);
    memcpy(header + 72, &job->target, 4);
    memcpy(header + 76, &nonce, 4);

    uint8_t hash_result[32];
    double_sha256_bin(header, 80, hash_result);

    return hash_to_pdiff(hash_result);
}

uint32_t increment_bitmask(const uint32_t value, const uint32_t mask)
{
    if (mask == 0) {
        return value;
    }

    uint32_t new_value = value & ~mask;
    bool carry = true;

    // Treat the selected bit positions as a packed little-endian counter.
    // This carries across gaps in sparse masks, preserves every unmasked bit,
    // and naturally wraps to zero within the mask.
    for (uint32_t bit = 1; bit != 0; bit <<= 1) {
        if ((mask & bit) == 0) {
            continue;
        }

        bool bit_is_set = (value & bit) != 0;
        if (carry) {
            bit_is_set = !bit_is_set;
            carry = !bit_is_set;
        }
        if (bit_is_set) {
            new_value |= bit;
        }
    }

    return new_value;
}

size_t version_mask_midstate_count(uint32_t version_mask)
{
    // Work packet formats have established one- and four-midstate encodings.
    // A zero- or one-bit mask cannot supply four distinct versions.
    return version_mask != 0 &&
                   (version_mask & (version_mask - 1)) != 0
               ? 4
               : 1;
}

size_t version_mask_value_count(uint32_t version_mask)
{
    unsigned int bit_count = 0;
    for (uint32_t mask = version_mask; mask != 0; mask >>= 1) {
        bit_count += mask & 1U;
    }
    if (bit_count >= sizeof(size_t) * CHAR_BIT) {
        return SIZE_MAX;
    }
    return (size_t)1U << bit_count;
}
