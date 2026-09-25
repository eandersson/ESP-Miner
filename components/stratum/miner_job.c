#include "miner_job.h"
#include "stratum_api.h"
#include "utils.h"
#include <string.h>
#include <pthread.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_heap_caps.h"

static miner_job_t s_job_pool[MINER_JOB_POOL_SIZE];
static size_t s_job_pool_suffix_capacity[MINER_JOB_POOL_SIZE];
static pthread_mutex_t s_job_pool_lock = PTHREAD_MUTEX_INITIALIZER;

void miner_job_lock(void)
{
    pthread_mutex_lock(&s_job_pool_lock);
}

void miner_job_unlock(void)
{
    pthread_mutex_unlock(&s_job_pool_lock);
}

bool miner_job_alloc_buffers(miner_job_t *job)
{
    if (job == NULL) {
        return false;
    }
    // Same placement as the ring: PSRAM, with internal RAM only where the
    // build has no PSRAM (unit tests).
    if (job->coinbase_prefix == NULL) {
        job->coinbase_prefix = heap_caps_calloc(1, MAX_COINBASE_PREFIX_LEN, MALLOC_CAP_SPIRAM);
        if (job->coinbase_prefix == NULL) {
            job->coinbase_prefix = calloc(1, MAX_COINBASE_PREFIX_LEN);
        }
    }
    if (job->coinbase_suffix == NULL) {
        job->coinbase_suffix = heap_caps_calloc(1, MAX_COINBASE_SUFFIX_LEN, MALLOC_CAP_SPIRAM);
        if (job->coinbase_suffix == NULL) {
            job->coinbase_suffix = calloc(1, MAX_COINBASE_SUFFIX_LEN);
        }
    }
    if (job->coinbase_prefix == NULL || job->coinbase_suffix == NULL) {
        miner_job_free_buffers(job);
        return false;
    }
    return true;
}

void miner_job_free_buffers(miner_job_t *job)
{
    if (job == NULL) {
        return;
    }
    free(job->coinbase_prefix);
    free(job->coinbase_suffix);
    job->coinbase_prefix = NULL;
    job->coinbase_suffix = NULL;
    job->coinbase_prefix_len = 0;
    job->coinbase_suffix_len = 0;
}

bool miner_job_ensure_suffix_capacity(miner_job_t *job, size_t needed)
{
    if (job == NULL || needed > MAX_COINBASE_SUFFIX_LEN) {
        return false;
    }
    for (size_t i = 0; i < MINER_JOB_POOL_SIZE; i++) {
        if (job != &s_job_pool[i]) {
            continue;
        }
        if (needed == 0 || (job->coinbase_suffix != NULL &&
                            needed <= s_job_pool_suffix_capacity[i])) {
            return true;
        }
        // Only the no-PSRAM fallback is small. Its calloc buffer can grow
        // on demand without reserving 8 * 63 KiB of internal RAM at boot.
        uint8_t *larger = realloc(job->coinbase_suffix, needed);
        if (larger == NULL) {
            return false;
        }
        job->coinbase_suffix = larger;
        s_job_pool_suffix_capacity[i] = needed;
        return true;
    }
    return needed == 0 || job->coinbase_suffix != NULL;
}

bool miner_job_copy(miner_job_t *dst, const miner_job_t *src)
{
    if (dst == NULL || src == NULL || dst == src) {
        return false;
    }

    if (src->coinbase_prefix_len > MAX_COINBASE_PREFIX_LEN ||
        (src->coinbase_prefix_len > 0 &&
         (src->coinbase_prefix == NULL || dst->coinbase_prefix == NULL)) ||
        (src->coinbase_suffix_len > 0 && src->coinbase_suffix == NULL) ||
        !miner_job_ensure_suffix_capacity(dst, src->coinbase_suffix_len)) {
        return false;
    }

    uint8_t *prefix = dst->coinbase_prefix;
    uint8_t *suffix = dst->coinbase_suffix;
    *dst = *src;
    dst->coinbase_prefix = prefix;
    dst->coinbase_suffix = suffix;

    uint16_t prefix_len = src->coinbase_prefix_len;
    uint16_t suffix_len = src->coinbase_suffix_len;
    if (prefix_len > MAX_COINBASE_PREFIX_LEN || prefix == NULL ||
        (prefix_len > 0 && src->coinbase_prefix == NULL)) {
        prefix_len = 0;
    }
    if (suffix_len > MAX_COINBASE_SUFFIX_LEN || suffix == NULL ||
        (suffix_len > 0 && src->coinbase_suffix == NULL)) {
        suffix_len = 0;
    }
    if (prefix_len > 0) {
        memcpy(prefix, src->coinbase_prefix, prefix_len);
    }
    if (suffix_len > 0) {
        memcpy(suffix, src->coinbase_suffix, suffix_len);
    }
    dst->coinbase_prefix_len = prefix_len;
    dst->coinbase_suffix_len = suffix_len;
    return true;
}

void miner_job_pool_init(void)
{
    for (size_t i = 0; i < MINER_JOB_POOL_SIZE; i++) {
        if (!s_job_pool[i].coinbase_prefix) {
            s_job_pool[i].coinbase_prefix = heap_caps_calloc(1, MAX_COINBASE_PREFIX_LEN, MALLOC_CAP_SPIRAM);
            if (!s_job_pool[i].coinbase_prefix) {
                s_job_pool[i].coinbase_prefix = calloc(1, MAX_COINBASE_PREFIX_LEN);
            }
        }
        if (!s_job_pool[i].coinbase_suffix) {
            s_job_pool[i].coinbase_suffix = heap_caps_calloc(1, MAX_COINBASE_SUFFIX_LEN, MALLOC_CAP_SPIRAM);
            if (!s_job_pool[i].coinbase_suffix) {
                s_job_pool[i].coinbase_suffix = calloc(1, 2048);
                if (s_job_pool[i].coinbase_suffix) {
                    s_job_pool_suffix_capacity[i] = 2048;
                }
            } else {
                s_job_pool_suffix_capacity[i] = MAX_COINBASE_SUFFIX_LEN;
            }
        }
        uint8_t *p_buf = s_job_pool[i].coinbase_prefix;
        uint8_t *s_buf = s_job_pool[i].coinbase_suffix;
        memset(&s_job_pool[i], 0, sizeof(miner_job_t));
        s_job_pool[i].coinbase_prefix = p_buf;
        s_job_pool[i].coinbase_suffix = s_buf;
    }
}

miner_job_t *miner_job_get_slot(size_t index)
{
    size_t slot_idx = index % MINER_JOB_POOL_SIZE;
    if (!s_job_pool[slot_idx].coinbase_prefix || !s_job_pool[slot_idx].coinbase_suffix) {
        miner_job_pool_init();
    }
    return &s_job_pool[slot_idx];
}
