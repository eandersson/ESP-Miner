#include <string.h>
#include <stdio.h>
#include <limits.h>
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

static bm_job *allocate_bm_job_internal(const char *jobid,
                                        const char *extranonce2,
                                        const char *session_user)
{
    if (jobid == NULL || extranonce2 == NULL) {
        return NULL;
    }

    size_t jobid_size = strlen(jobid) + 1;
    size_t extranonce2_size = strlen(extranonce2) + 1;
    size_t session_user_size =
        session_user != NULL ? strlen(session_user) + 1 : 0;
    if (jobid_size > SIZE_MAX - extranonce2_size ||
        jobid_size + extranonce2_size > SIZE_MAX - session_user_size ||
        sizeof(allocated_bm_job_t) > SIZE_MAX - jobid_size -
                                                extranonce2_size -
                                                session_user_size) {
        return NULL;
    }

    allocated_bm_job_t *allocation = calloc(
        1, sizeof(*allocation) + jobid_size + extranonce2_size +
               session_user_size);
    if (allocation == NULL) {
        return NULL;
    }

    bm_job *job = &allocation->job;
    char *metadata = allocation->metadata;
    job->jobid = metadata;
    memcpy(job->jobid, jobid, jobid_size);
    job->extranonce2 = metadata + jobid_size;
    memcpy(job->extranonce2, extranonce2, extranonce2_size);
    if (session_user_size != 0) {
        job->session_user = metadata + jobid_size + extranonce2_size;
        memcpy(job->session_user, session_user, session_user_size);
    }
    atomic_init(&allocation->ref_count, 1);
    return job;
}

bm_job *allocate_bm_job(const char *jobid, const char *extranonce2)
{
    return allocate_bm_job_internal(jobid, extranonce2, NULL);
}

bm_job *allocate_bm_job_for_user(const char *jobid, const char *extranonce2,
                                 const char *session_user)
{
    if (session_user == NULL) {
        return NULL;
    }
    return allocate_bm_job_internal(jobid, extranonce2, session_user);
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
    free(job->session_user);
    free(job);
}

void calculate_coinbase_tx_hash(const char *coinbase_1, const char *coinbase_2, const char *extranonce, const char *extranonce_2, uint8_t dest[32])
{
    size_t len1 = strlen(coinbase_1);
    size_t len2 = strlen(extranonce);
    size_t len3 = strlen(extranonce_2);
    size_t len4 = strlen(coinbase_2);

    size_t coinbase_tx_bin_len = (len1 + len2 + len3 + len4) / 2;

    uint8_t coinbase_tx_bin[coinbase_tx_bin_len];

    size_t bin_offset = 0;
    bin_offset += hex2bin(coinbase_1, coinbase_tx_bin + bin_offset, coinbase_tx_bin_len - bin_offset);
    bin_offset += hex2bin(extranonce, coinbase_tx_bin + bin_offset, coinbase_tx_bin_len - bin_offset);
    bin_offset += hex2bin(extranonce_2, coinbase_tx_bin + bin_offset, coinbase_tx_bin_len - bin_offset);
    bin_offset += hex2bin(coinbase_2, coinbase_tx_bin + bin_offset, coinbase_tx_bin_len - bin_offset);

    double_sha256_bin(coinbase_tx_bin, coinbase_tx_bin_len, dest);
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

// take a mining_notify struct with ascii hex strings and convert it to a bm_job struct
void construct_bm_job(mining_notify *params, const uint8_t merkle_root[32], const uint32_t version_mask, const double difficulty, bm_job *new_job)
{
    new_job->version = params->version;
    new_job->target = params->target;
    new_job->ntime = params->ntime;
    new_job->starting_nonce = 0;
    new_job->pool_diff = difficulty;
    reverse_32bit_words(merkle_root, new_job->merkle_root);

    uint8_t prev_block_hash[32];
    hex2bin(params->prev_block_hash, prev_block_hash, 32);
    reverse_endianness_per_word(prev_block_hash);
    reverse_32bit_words(prev_block_hash, new_job->prev_block_hash);

    // make the midstate hash
    uint8_t midstate_data[64];

    // copy 64 bytes header data into midstate (and deal with endianess)
    memcpy(midstate_data, &new_job->version, 4);      // copy version
    memcpy(midstate_data + 4, prev_block_hash, 32);   // copy prev_block_hash
    memcpy(midstate_data + 36, merkle_root, 28);      // copy merkle_root

    uint8_t midstate[32];
    midstate_sha256_bin(midstate_data, 64, midstate); // make the midstate hash
    reverse_32bit_words(midstate, new_job->midstate); // reverse the midstate words for the BM job packet

    if (version_mask_midstate_count(version_mask) == 4)
    {
        uint32_t rolled_version = increment_bitmask(new_job->version, version_mask);
        memcpy(midstate_data, &rolled_version, 4);
        midstate_sha256_bin(midstate_data, 64, midstate);
        reverse_32bit_words(midstate, new_job->midstate1);

        rolled_version = increment_bitmask(rolled_version, version_mask);
        memcpy(midstate_data, &rolled_version, 4);
        midstate_sha256_bin(midstate_data, 64, midstate);
        reverse_32bit_words(midstate, new_job->midstate2);

        rolled_version = increment_bitmask(rolled_version, version_mask);
        memcpy(midstate_data, &rolled_version, 4);
        midstate_sha256_bin(midstate_data, 64, midstate);
        reverse_32bit_words(midstate, new_job->midstate3);
        new_job->num_midstates = 4;
    }
    else
    {
        new_job->num_midstates = 1;
    }
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

double hash_to_pdiff(const uint8_t hash[32])
{
    double s64 = le256todouble(hash);
    if (s64 == 0.0) return (double)UINT32_MAX;
    return truediffone / s64;
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
