#include "esp_log.h"
#include "esp_transport.h"
#include "esp_transport_tcp.h"
#include <lwip/sockets.h>
#include "esp_timer.h"
#include "system.h"
#include "global_state.h"
#include "stratum_v2_task.h"
#include "stratum_socket.h"
#include "protocol_coordinator.h"
#include "connect.h"
#include "sv2_protocol.h"
#include "sv2_noise.h"
#include "mining.h"
#include "stratum_api.h"
#include "work_queue.h"
#include "asic_result_task.h"
#include "utils.h"
#include "libbase58.h"
#include "device_config.h"
#include "coinbase_decoder.h"
#include "esp_heap_caps.h"

#include <string.h>
#include <stdlib.h>
#include <pthread.h>

#define MAX_RETRY_ATTEMPTS 3
#define TRANSPORT_TIMEOUT_MS 5000
#define SV2_MAX_FRAME_SIZE 8192
#define SV2_MAX_EXTRANONCE_SIZE 32
#define SHUTDOWN_POLL_INTERVAL_MS 100

static const char *TAG = "stratum_v2_task";
static pthread_mutex_t sv2_lifecycle_lock = PTHREAD_MUTEX_INITIALIZER;

static void stratum_v2_shutdown_aware_delay(uint32_t delay_ms)
{
    while (delay_ms > 0) {
        if (protocol_coordinator_v2_should_shutdown()) {
            return;
        }

        uint32_t interval_ms = delay_ms < SHUTDOWN_POLL_INTERVAL_MS
                                   ? delay_ms
                                   : SHUTDOWN_POLL_INTERVAL_MS;
        vTaskDelay(pdMS_TO_TICKS(interval_ms));
        delay_ms -= interval_ms;
    }
}

// A transport is not visible to stratum_v2_interrupt_connection() until it is
// published in GLOBAL_STATE. The task that created an unpublished handle must
// therefore close and destroy it itself when a stop races connect/setup.
static void stratum_v2_destroy_unpublished_transport(
    esp_transport_handle_t transport)
{
    if (transport != NULL) {
        esp_transport_close(transport);
        esp_transport_destroy(transport);
    }
}

static void free_sv2_extended_queued_work(void *work)
{
    sv2_ext_job_free((sv2_ext_job_t *)work);
}

static void stratum_v2_free_pending_jobs(sv2_conn_t *conn)
{
    if (conn == NULL) {
        return;
    }

    for (int i = 0; i < SV2_PENDING_JOBS_SIZE; i++) {
        conn->pending_jobs[i].valid = false;
        if (conn->ext_pending_jobs[i] != NULL) {
            sv2_ext_job_free(conn->ext_pending_jobs[i]);
            conn->ext_pending_jobs[i] = NULL;
        }
    }
}

// Load authority pubkey from NVS (base58-encoded) into 32-byte buffer.
// SV2 format: base58check(0x0001_LE + 32_byte_xonly_pubkey)
// Decoded: 2-byte version + 32-byte pubkey + 4-byte checksum = 38 bytes
// Returns true if a valid base58 pubkey was decoded.
static bool stratum_v2_load_authority_pubkey(const PoolConfig *pool,
                                             uint8_t out[32])
{
    const char *b58_key = pool->sv2_authority_pubkey;
    if (!b58_key || strlen(b58_key) == 0) {
        return false;
    }

    uint8_t decoded[64];
    size_t decoded_len = sizeof(decoded);

    if (!b58tobin(decoded, &decoded_len, b58_key, 0)) {
        ESP_LOGE(TAG, "Failed to decode base58 authority pubkey");
        return false;
    }

    // base58check = 2-byte version + 32-byte pubkey + 4-byte checksum = 38 bytes
    if (decoded_len != 38) {
        ESP_LOGE(TAG, "Invalid decoded length: %zu (expected 38)", decoded_len);
        return false;
    }

    // b58tobin right-aligns data in the buffer
    uint8_t *data = decoded + (sizeof(decoded) - decoded_len);

    // Verify version (0x0001 in little-endian)
    if (data[0] != 0x01 || data[1] != 0x00) {
        ESP_LOGE(TAG, "Invalid key version: 0x%02x%02x (expected 0x0100)", data[1], data[0]);
        return false;
    }

    memcpy(out, data + 2, 32);

    ESP_LOGI(TAG, "Successfully decoded base58 authority pubkey");
    return true;
}

static sv2_channel_type_t sv2_select_channel_type(
    GlobalState *GLOBAL_STATE, const PoolConfig *pool)
{
    sv2_channel_type_t type = SV2_CHANNEL_EXTENDED;  // default, and forced for BM1397
    sv2_channel_type_t parsed = pool->sv2_channel_type;
    if (parsed == SV2_CHANNEL_STANDARD) {
        if (GLOBAL_STATE->DEVICE_CONFIG.family.asic.id != BM1397) {
            type = SV2_CHANNEL_STANDARD;
        }
    } else if (parsed == SV2_CHANNEL_EXTENDED) {
        type = SV2_CHANNEL_EXTENDED;
    }
    return type;
}

void stratum_v2_close_connection(GlobalState *GLOBAL_STATE)
{
    ESP_LOGE(TAG, "Shutting down SV2 connection and restarting...");

    sv2_noise_ctx_t *noise_ctx;
    esp_transport_handle_t transport;

    // Serialize teardown with share submission. Invalidation happens while the
    // lifecycle lock is held, so a result that passed an earlier stale check
    // must re-check its generation before it can use a replacement connection.
    pthread_mutex_lock(&sv2_lifecycle_lock);
    SYSTEM_clean_jobs_queue(GLOBAL_STATE);
    stratum_v2_free_pending_jobs(GLOBAL_STATE->sv2_conn);
    if (GLOBAL_STATE->sv2_conn != NULL) {
        GLOBAL_STATE->sv2_conn->channel_opened = false;
    }
    noise_ctx = GLOBAL_STATE->sv2_noise_ctx;
    transport = GLOBAL_STATE->transport;
    GLOBAL_STATE->sv2_noise_ctx = NULL;
    GLOBAL_STATE->transport = NULL;
    GLOBAL_STATE->sv2_conn = NULL;
    pthread_mutex_unlock(&sv2_lifecycle_lock);

    if (noise_ctx != NULL) {
        sv2_noise_destroy(noise_ctx);
    }
    if (transport != NULL) {
        esp_transport_close(transport);
        esp_transport_destroy(transport);
    }
    stratum_v2_shutdown_aware_delay(1000);
}

void stratum_v2_interrupt_connection(GlobalState *GLOBAL_STATE)
{
    pthread_mutex_lock(&sv2_lifecycle_lock);
    if (GLOBAL_STATE->transport != NULL) {
        esp_transport_close(GLOBAL_STATE->transport);
    }
    pthread_mutex_unlock(&sv2_lifecycle_lock);
}

static void stratum_v2_clean_jobs(GlobalState *GLOBAL_STATE)
{
    // Keep the generation change ordered with share submission. Once this
    // returns, no submitter can have passed the old-generation check and still
    // be waiting to write that stale share.
    pthread_mutex_lock(&sv2_lifecycle_lock);
    SYSTEM_clean_jobs_queue(GLOBAL_STATE);
    pthread_mutex_unlock(&sv2_lifecycle_lock);
}

// Track per-share submit timestamps for response time measurement.
// SubmitShares.Success can batch-acknowledge multiple shares via its
// last_sequence_number field, so we key the submit time by sequence number
// (ring buffer) and measure against the specific share being acknowledged
// rather than just the most recent submit.
#define SV2_SUBMIT_TIMING_SLOTS 32
static int64_t stratum_v2_submit_time_us[SV2_SUBMIT_TIMING_SLOTS] = {0};

// Shares submitted but not yet resolved by the pool (rises while it batches acks).
static void stratum_v2_update_pending_shares(GlobalState *GLOBAL_STATE)
{
    sv2_conn_t *conn = GLOBAL_STATE->sv2_conn;
    if (!conn) {
        GLOBAL_STATE->SYSTEM_MODULE.shares_pending = 0;
        return;
    }
    uint32_t pending = (conn->sequence_number > conn->resolved_shares)
                           ? (conn->sequence_number - conn->resolved_shares)
                           : 0;
    GLOBAL_STATE->SYSTEM_MODULE.shares_pending = (uint16_t)(pending > UINT16_MAX ? UINT16_MAX : pending);
}

// Timestamp a submitted share (for response-time measurement) and refresh pending.
static void stratum_v2_track_submit(GlobalState *GLOBAL_STATE, uint32_t sequence_number)
{
    stratum_v2_submit_time_us[sequence_number % SV2_SUBMIT_TIMING_SLOTS] = esp_timer_get_time();
    stratum_v2_update_pending_shares(GLOBAL_STATE);
}

int stratum_v2_submit_share(GlobalState *GLOBAL_STATE, uint32_t job_id, uint32_t nonce,
                            uint32_t ntime, uint32_t version,
                            uint32_t expected_generation)
{
    pthread_mutex_lock(&sv2_lifecycle_lock);
    if (expected_generation != ASIC_result_task_get_job_generation() ||
        !GLOBAL_STATE->transport || !GLOBAL_STATE->sv2_conn ||
        !GLOBAL_STATE->sv2_noise_ctx ||
        !GLOBAL_STATE->sv2_conn->channel_opened ||
        GLOBAL_STATE->sv2_conn->channel_type != SV2_CHANNEL_STANDARD) {
        pthread_mutex_unlock(&sv2_lifecycle_lock);
        return -1;
    }

    sv2_conn_t *conn = GLOBAL_STATE->sv2_conn;
    uint8_t buf[SV2_FRAME_HEADER_SIZE + 24];

    uint32_t sequence_number = conn->sequence_number++;
    int len = sv2_build_submit_shares_standard(buf, sizeof(buf),
                                                conn->channel_id,
                                                sequence_number,
                                                job_id, nonce, ntime, version);
    if (len < 0) {
        pthread_mutex_unlock(&sv2_lifecycle_lock);
        return -1;
    }

    stratum_v2_track_submit(GLOBAL_STATE, sequence_number);
    int result = sv2_noise_send(
        GLOBAL_STATE->sv2_noise_ctx, GLOBAL_STATE->transport, buf, len);
    pthread_mutex_unlock(&sv2_lifecycle_lock);
    return result;
}

int stratum_v2_submit_share_extended(GlobalState *GLOBAL_STATE, uint32_t job_id,
                                     uint32_t nonce, uint32_t ntime, uint32_t version,
                                     const uint8_t *extranonce, uint8_t extranonce_len,
                                     uint32_t expected_generation)
{
    pthread_mutex_lock(&sv2_lifecycle_lock);
    if (expected_generation != ASIC_result_task_get_job_generation() ||
        !GLOBAL_STATE->transport || !GLOBAL_STATE->sv2_conn ||
        !GLOBAL_STATE->sv2_noise_ctx ||
        !GLOBAL_STATE->sv2_conn->channel_opened ||
        GLOBAL_STATE->sv2_conn->channel_type != SV2_CHANNEL_EXTENDED ||
        extranonce == NULL ||
        extranonce_len != GLOBAL_STATE->sv2_conn->extranonce_size) {
        pthread_mutex_unlock(&sv2_lifecycle_lock);
        return -1;
    }

    sv2_conn_t *conn = GLOBAL_STATE->sv2_conn;
    uint8_t buf[SV2_FRAME_HEADER_SIZE + 24 + 1 + 32];

    uint32_t sequence_number = conn->sequence_number++;
    int len = sv2_build_submit_shares_extended(buf, sizeof(buf),
                                                conn->channel_id,
                                                sequence_number,
                                                job_id, nonce, ntime, version,
                                                extranonce, extranonce_len);
    if (len < 0) {
        pthread_mutex_unlock(&sv2_lifecycle_lock);
        return -1;
    }

    stratum_v2_track_submit(GLOBAL_STATE, sequence_number);
    int result = sv2_noise_send(
        GLOBAL_STATE->sv2_noise_ctx, GLOBAL_STATE->transport, buf, len);
    pthread_mutex_unlock(&sv2_lifecycle_lock);
    return result;
}

bool stratum_v2_is_extended_channel(GlobalState *GLOBAL_STATE)
{
    pthread_mutex_lock(&sv2_lifecycle_lock);
    bool is_extended =
        GLOBAL_STATE->sv2_conn &&
        GLOBAL_STATE->sv2_conn->channel_type == SV2_CHANNEL_EXTENDED;
    pthread_mutex_unlock(&sv2_lifecycle_lock);
    return is_extended;
}

bool stratum_v2_snapshot_extended_work(
    GlobalState *GLOBAL_STATE, uint32_t expected_generation,
    stratum_v2_extended_work_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }

    pthread_mutex_lock(&sv2_lifecycle_lock);
    sv2_conn_t *conn = GLOBAL_STATE->sv2_conn;
    bool valid =
        expected_generation == ASIC_result_task_get_job_generation() &&
        conn != NULL && conn->channel_opened &&
        conn->channel_type == SV2_CHANNEL_EXTENDED &&
        conn->extranonce_prefix_len <= sizeof(snapshot->extranonce_prefix) &&
        conn->extranonce_size <= 32;
    if (valid) {
        snapshot->extranonce_prefix_len = conn->extranonce_prefix_len;
        snapshot->extranonce_size = conn->extranonce_size;
        memcpy(snapshot->extranonce_prefix, conn->extranonce_prefix,
               conn->extranonce_prefix_len);
    }
    pthread_mutex_unlock(&sv2_lifecycle_lock);
    return valid;
}

// Enqueue an sv2_job_t onto the stratum queue. start_immediately controls
// scheduling only; invalidating work from the previous prevhash is handled
// explicitly by SetNewPrevHash.
static void stratum_v2_enqueue_job(GlobalState *GLOBAL_STATE,
                                   uint32_t job_id, uint32_t version,
                                   const uint8_t merkle_root[32], const uint8_t prev_hash[32],
                                   uint32_t ntime, uint32_t nbits,
                                   bool start_immediately)
{
    sv2_job_t *job = malloc(sizeof(sv2_job_t));
    if (!job) {
        ESP_LOGE(TAG, "Failed to allocate sv2_job_t");
        return;
    }

    job->job_id = job_id;
    job->version = version;
    memcpy(job->merkle_root, merkle_root, 32);
    memcpy(job->prev_hash, prev_hash, 32);
    job->ntime = ntime;
    job->nbits = nbits;
    // create_jobs_task uses this field to bypass its periodic refresh delay.
    // In SV2, an active job (min_ntime present) must start immediately, but it
    // does not by itself invalidate shares from the current prevhash.
    job->clean_jobs = start_immediately;

    GLOBAL_STATE->SYSTEM_MODULE.work_received++;

    SYSTEM_notify_new_ntime(GLOBAL_STATE, ntime);

    if (start_immediately) {
        // Keep only the newest active-now job. This is a queue-only operation:
        // already-running jobs remain valid until SetNewPrevHash says otherwise.
        queue_clear(&GLOBAL_STATE->stratum_queue);
    }

    queue_enqueue(
        &GLOBAL_STATE->stratum_queue, job,
        (work_queue_item_metadata) {
            .generation = ASIC_result_task_get_pool_generation(),
            .kind = WORK_QUEUE_ITEM_STRATUM_V2_STANDARD,
            .free_fn = free,
        });
}

// Enqueue an sv2_ext_job_t onto the stratum queue (extended channels).
static void stratum_v2_enqueue_ext_job(GlobalState *GLOBAL_STATE,
                                        sv2_ext_job_t *job)
{
    GLOBAL_STATE->SYSTEM_MODULE.work_received++;

    SYSTEM_notify_new_ntime(GLOBAL_STATE, job->ntime);

    if (job->clean_jobs) {
        // min_ntime makes this job active immediately. Replace queued work,
        // without invalidating results from jobs on the same prevhash.
        queue_clear(&GLOBAL_STATE->stratum_queue);
    }

    queue_enqueue(
        &GLOBAL_STATE->stratum_queue, job,
        (work_queue_item_metadata) {
            .generation = ASIC_result_task_get_pool_generation(),
            .kind = WORK_QUEUE_ITEM_STRATUM_V2_EXTENDED,
            .free_fn = free_sv2_extended_queued_work,
        });
}

// Decode coinbase from extended job prefix/suffix by converting to hex and reusing V1 decoder
static void stratum_v2_decode_coinbase(GlobalState *GLOBAL_STATE,
                                       sv2_conn_t *conn,
                                       const PoolConfig *pool,
                                       const sv2_ext_job_t *job)
{
    bool decode_coinbase = pool->decode_coinbase_tx;

    // Check for BIP141 SegWit marker/flag in prefix (bytes[4]==0x00, bytes[5]!=0x00).
    // Some SV2 pools send the coinbase in witness format; the V1 decoder expects
    // non-witness format, so strip the 2-byte marker+flag if present.
    const uint8_t *prefix = job->coinbase_prefix;
    uint16_t prefix_len = job->coinbase_prefix_len;
    uint8_t *stripped_prefix = NULL;

    if (prefix_len > 5 && prefix[4] == 0x00 && prefix[5] != 0x00) {
        ESP_LOGD(TAG, "Stripping BIP141 marker/flag from coinbase prefix");
        prefix_len -= 2;
        stripped_prefix = malloc(prefix_len);
        if (!stripped_prefix) {
            ESP_LOGE(TAG, "Failed to allocate stripped prefix");
            return;
        }
        memcpy(stripped_prefix, prefix, 4);              // version
        memcpy(stripped_prefix + 4, prefix + 6, prefix_len - 4); // skip marker+flag
        prefix = stripped_prefix;
    }

    // Convert binary prefix/suffix/extranonce to hex strings for the V1 decoder
    char *coinbase_1_hex = malloc(prefix_len * 2 + 1);
    char *coinbase_2_hex = malloc(job->coinbase_suffix_len * 2 + 1);
    char *extranonce1_hex = malloc(conn->extranonce_prefix_len * 2 + 1);
    if (!coinbase_1_hex || !coinbase_2_hex || !extranonce1_hex) {
        ESP_LOGE(TAG, "Failed to allocate hex buffers for coinbase decode");
        free(coinbase_1_hex);
        free(coinbase_2_hex);
        free(extranonce1_hex);
        free(stripped_prefix);
        return;
    }

    bin2hex(prefix, prefix_len,
            coinbase_1_hex, prefix_len * 2 + 1);
    bin2hex(job->coinbase_suffix, job->coinbase_suffix_len,
            coinbase_2_hex, job->coinbase_suffix_len * 2 + 1);
    bin2hex(conn->extranonce_prefix, conn->extranonce_prefix_len,
            extranonce1_hex, conn->extranonce_prefix_len * 2 + 1);
    free(stripped_prefix);

    // SV2 spec: extranonce_size is the miner's rollable portion (not total)
    int extranonce2_len = conn->extranonce_size;

    // Build a temporary mining_notify for the existing V1 decoder
    mining_notify notify = {
        .coinbase_1 = coinbase_1_hex,
        .coinbase_2 = coinbase_2_hex,
        .target = conn->prev_hash_nbits,
    };

    mining_notification_result_t *result = heap_caps_malloc(sizeof(mining_notification_result_t),
                                                            MALLOC_CAP_SPIRAM);
    if (!result) {
        ESP_LOGE(TAG, "Failed to allocate coinbase decode result");
        free(coinbase_1_hex);
        free(coinbase_2_hex);
        free(extranonce1_hex);
        return;
    }
    memset(result, 0, sizeof(mining_notification_result_t));

    const char *user = pool->user;

    esp_err_t err = coinbase_process_notification(&notify, extranonce1_hex, extranonce2_len,
                                                   user, decode_coinbase, result);
    free(coinbase_1_hex);
    free(coinbase_2_hex);
    free(extranonce1_hex);

    if (err != ESP_OK) {
        // Log first bytes of prefix for debugging format issues
        char prefix_hex[13] = {0}; // 6 bytes = 12 hex chars + null
        bin2hex(job->coinbase_prefix,
                job->coinbase_prefix_len < 6 ? job->coinbase_prefix_len : 6,
                prefix_hex, sizeof(prefix_hex));
        ESP_LOGE(TAG, "Failed to decode extended job coinbase (err=0x%x, prefix_len=%u, "
                 "suffix_len=%u, en_prefix_len=%u, en_size=%u, prefix_start=%s)",
                 err, job->coinbase_prefix_len, job->coinbase_suffix_len,
                 conn->extranonce_prefix_len, conn->extranonce_size, prefix_hex);
        free(result);
        return;
    }

    if (result->block_height != 0 && (uint32_t)GLOBAL_STATE->block_height != result->block_height) {
        ESP_LOGI(TAG, "Block height %d", result->block_height);
        GLOBAL_STATE->block_height = result->block_height;
    }

    if (result->scriptsig) {
        if (strcmp(result->scriptsig, GLOBAL_STATE->scriptsig) != 0) {
            ESP_LOGI(TAG, "Scriptsig: %s", result->scriptsig);
            strncpy(GLOBAL_STATE->scriptsig, result->scriptsig, sizeof(GLOBAL_STATE->scriptsig) - 1);
            GLOBAL_STATE->scriptsig[sizeof(GLOBAL_STATE->scriptsig) - 1] = '\0';
        }
        free(result->scriptsig);
    }

    if (result->output_count > MAX_COINBASE_TX_OUTPUTS) {
        result->output_count = MAX_COINBASE_TX_OUTPUTS;
    }

    GLOBAL_STATE->coinbase_value_total_satoshis = result->total_value_satoshis;
    ESP_LOGI(TAG, "Coinbase outputs: %d, total value: %llu%s",
             result->output_count, result->total_value_satoshis,
             result->decode_coinbase_tx ? " sats" : "");

    if (result->output_count != GLOBAL_STATE->coinbase_output_count ||
        memcmp(result->outputs, GLOBAL_STATE->coinbase_outputs,
               sizeof(coinbase_output_t) * result->output_count) != 0) {

        GLOBAL_STATE->coinbase_output_count = result->output_count;
        memcpy(GLOBAL_STATE->coinbase_outputs, result->outputs,
               sizeof(coinbase_output_t) * result->output_count);
        GLOBAL_STATE->coinbase_value_user_satoshis = result->user_value_satoshis;
        for (int i = 0; i < result->output_count; i++) {
            if (result->outputs[i].value_satoshis > 0) {
                if (result->outputs[i].is_user_output) {
                    ESP_LOGI(TAG, "  Output %d: %s (%llu sat) (Your payout address)",
                             i, result->outputs[i].address, result->outputs[i].value_satoshis);
                } else {
                    ESP_LOGI(TAG, "  Output %d: %s (%llu sat)",
                             i, result->outputs[i].address, result->outputs[i].value_satoshis);
                }
            } else {
                ESP_LOGI(TAG, "  Output %d: %s", i, result->outputs[i].address);
            }
        }
    }

    free(result);
}

// Handle NewExtendedMiningJob message
static void stratum_v2_handle_new_extended_mining_job(
    GlobalState *GLOBAL_STATE, sv2_conn_t *conn, const PoolConfig *pool,
    const uint8_t *payload, uint32_t len)
{
    uint32_t channel_id;
    sv2_ext_job_t *job = sv2_parse_new_extended_mining_job(payload, len, &channel_id);
    if (!job) {
        ESP_LOGE(TAG, "Failed to parse NewExtendedMiningJob");
        return;
    }

    ESP_LOGI(TAG, "New extended mining job: id=%lu, version=%08lx, merkle_branches=%d, "
             "coinbase_prefix=%u, coinbase_suffix=%u, future=%s",
             job->job_id, job->version, job->merkle_path_count,
             job->coinbase_prefix_len, job->coinbase_suffix_len,
             job->ntime > 0 ? "no" : "yes");

    // Decode coinbase transaction (block height, scriptsig, outputs)
    stratum_v2_decode_coinbase(GLOBAL_STATE, conn, pool, job);

    int slot = job->job_id % SV2_PENDING_JOBS_SIZE;

    if (job->ntime > 0) {
        // Has min_ntime — this is a current job
        if (conn->has_prev_hash) {
            memcpy(job->prev_hash, conn->prev_hash, 32);
            job->nbits = conn->prev_hash_nbits;
            job->clean_jobs = true;
            stratum_v2_enqueue_ext_job(GLOBAL_STATE, job);
        } else {
            // Store as pending until we get SetNewPrevHash
            if (conn->ext_pending_jobs[slot]) {
                sv2_ext_job_free(conn->ext_pending_jobs[slot]);
            }
            conn->ext_pending_jobs[slot] = job;
        }
    } else {
        // Future job — store in pending ring
        if (conn->ext_pending_jobs[slot]) {
            sv2_ext_job_free(conn->ext_pending_jobs[slot]);
        }
        conn->ext_pending_jobs[slot] = job;
    }
}

// Handle NewMiningJob message
static void stratum_v2_handle_new_mining_job(GlobalState *GLOBAL_STATE, sv2_conn_t *conn,
                                             const uint8_t *payload, uint32_t len)
{
    uint32_t channel_id, job_id, version, min_ntime;
    bool has_min_ntime;
    uint8_t merkle_root[32];

    if (sv2_parse_new_mining_job(payload, len, &channel_id, &job_id,
                                 &has_min_ntime, &min_ntime,
                                 &version, merkle_root) != 0) {
        ESP_LOGE(TAG, "Failed to parse NewMiningJob");
        return;
    }

    ESP_LOGI(TAG, "New mining job: id=%lu, version=%08lx, future=%s",
             job_id, version, has_min_ntime ? "no" : "yes");

    int slot = job_id % SV2_PENDING_JOBS_SIZE;

    if (has_min_ntime) {
        if (conn->has_prev_hash) {
            stratum_v2_enqueue_job(GLOBAL_STATE, job_id, version, merkle_root,
                                   conn->prev_hash, min_ntime,
                                   conn->prev_hash_nbits, true);
        } else {
            conn->pending_jobs[slot].job_id = job_id;
            conn->pending_jobs[slot].version = version;
            memcpy(conn->pending_jobs[slot].merkle_root, merkle_root, 32);
            conn->pending_jobs[slot].valid = true;
        }
    } else {
        conn->pending_jobs[slot].job_id = job_id;
        conn->pending_jobs[slot].version = version;
        memcpy(conn->pending_jobs[slot].merkle_root, merkle_root, 32);
        conn->pending_jobs[slot].valid = true;
    }
}

// Handle SetNewPrevHash message
static void stratum_v2_handle_set_new_prev_hash(GlobalState *GLOBAL_STATE, sv2_conn_t *conn,
                                                 const uint8_t *payload, uint32_t len)
{
    uint32_t channel_id, job_id, min_ntime, nbits;
    uint8_t prev_hash[32];

    if (sv2_parse_set_new_prev_hash(payload, len, &channel_id, &job_id,
                                     prev_hash, &min_ntime, &nbits) != 0) {
        ESP_LOGE(TAG, "Failed to parse SetNewPrevHash");
        return;
    }

    ESP_LOGI(TAG, "New prev_hash: job_id=%lu, ntime=%lu, nbits=%08lx", job_id, min_ntime, nbits);

    GLOBAL_STATE->network_nonce_diff = (uint64_t) networkDifficulty(nbits);
    suffixString(GLOBAL_STATE->network_nonce_diff, GLOBAL_STATE->network_diff_string, DIFF_STRING_SIZE, 0);

    memcpy(conn->prev_hash, prev_hash, 32);
    conn->prev_hash_ntime = min_ntime;
    conn->prev_hash_nbits = nbits;
    conn->has_prev_hash = true;

    int slot = job_id % SV2_PENDING_JOBS_SIZE;

    // SetNewPrevHash is the SV2 clean-work event. Only its referenced job is
    // valid, so invalidate queued/running ASIC work even if that job is missing
    // from our pending ring.
    stratum_v2_clean_jobs(GLOBAL_STATE);

    // Snapshot the referenced jobs before invalidating every pending entry.
    sv2_pending_job_t selected_job = {0};
    if (conn->pending_jobs[slot].valid && conn->pending_jobs[slot].job_id == job_id) {
        selected_job = conn->pending_jobs[slot];
    }

    sv2_ext_job_t *selected_ext_job = NULL;
    if (conn->ext_pending_jobs[slot] &&
        conn->ext_pending_jobs[slot]->job_id == job_id) {
        selected_ext_job = conn->ext_pending_jobs[slot];
    }

    for (int i = 0; i < SV2_PENDING_JOBS_SIZE; i++) {
        conn->pending_jobs[i].valid = false;

        sv2_ext_job_t *pending_ext_job = conn->ext_pending_jobs[i];
        conn->ext_pending_jobs[i] = NULL;
        if (pending_ext_job != NULL && pending_ext_job != selected_ext_job) {
            sv2_ext_job_free(pending_ext_job);
        }
    }

    if (selected_job.valid) {
        stratum_v2_enqueue_job(GLOBAL_STATE, job_id,
                               selected_job.version,
                               selected_job.merkle_root,
                               prev_hash, min_ntime, nbits, true);
    }

    if (selected_ext_job != NULL) {
        memcpy(selected_ext_job->prev_hash, prev_hash, 32);
        selected_ext_job->ntime = min_ntime;
        selected_ext_job->nbits = nbits;
        selected_ext_job->clean_jobs = true;
        stratum_v2_enqueue_ext_job(GLOBAL_STATE, selected_ext_job);
    }

    if (!selected_job.valid && selected_ext_job == NULL) {
        ESP_LOGW(TAG, "SetNewPrevHash references unknown job %lu", job_id);
    }
}

// Handle SetTarget message
static void stratum_v2_handle_set_target(GlobalState *GLOBAL_STATE, sv2_conn_t *conn,
                                         const uint8_t *payload, uint32_t len)
{
    uint32_t channel_id;
    uint8_t max_target[32];

    if (sv2_parse_set_target(payload, len, &channel_id, max_target) != 0) {
        ESP_LOGE(TAG, "Failed to parse SetTarget");
        return;
    }

    memcpy(conn->target, max_target, 32);
    double pdiff = hash_to_pdiff(max_target);
    ESP_LOGI(TAG, "Set pool difficulty: %g", pdiff);
    SYSTEM_set_pool_difficulty(GLOBAL_STATE, pdiff, true);
}

void stratum_v2_task(void *pvParameters)
{
    GlobalState *GLOBAL_STATE = (GlobalState *)pvParameters;

    // Set default version mask for version rolling
    GLOBAL_STATE->version_mask = STRATUM_DEFAULT_VERSION_MASK;
    GLOBAL_STATE->new_stratum_version_rolling_msg = true;

    // Heap-allocate sv2_conn to avoid dangling pointer after task exit
    sv2_conn_t *conn = calloc(1, sizeof(sv2_conn_t));
    if (!conn) {
        ESP_LOGE(TAG, "Failed to allocate sv2_conn");
        protocol_coordinator_notify_failure();
        vTaskDelete(NULL);
        return;
    }
    uint8_t *frame_buf = heap_caps_malloc(SV2_MAX_FRAME_SIZE, MALLOC_CAP_SPIRAM);
    uint8_t *recv_buf = heap_caps_malloc(SV2_MAX_FRAME_SIZE, MALLOC_CAP_SPIRAM);

    if (!frame_buf || !recv_buf) {
        ESP_LOGE(TAG, "Failed to allocate frame buffers");
        free(frame_buf);
        free(recv_buf);
        free(conn);
        protocol_coordinator_notify_failure();
        vTaskDelete(NULL);
        return;
    }

    int retry_attempts = 0;
    bool use_fallback = GLOBAL_STATE->SYSTEM_MODULE.is_using_fallback;
    uint16_t pool_idx = use_fallback ? GLOBAL_STATE->SYSTEM_MODULE.secondary_pool_index : GLOBAL_STATE->SYSTEM_MODULE.primary_pool_index;
    PoolConfig pool = {0};
    if (!SYSTEM_get_pool_config_snapshot(GLOBAL_STATE, pool_idx, &pool)) {
        ESP_LOGE(TAG, "Unable to snapshot SV2 pool configuration");
        free(frame_buf);
        free(recv_buf);
        free(conn);
        protocol_coordinator_notify_failure();
        vTaskDelete(NULL);
        return;
    }
    sv2_channel_type_t channel_type =
        sv2_select_channel_type(GLOBAL_STATE, &pool);
    const char *stratum_url = pool.url ? pool.url : "";
    uint16_t port = pool.port;

    ESP_LOGI(TAG, "Starting SV2 task (%s), connecting to %s:%d (free heap: %lu)",
             use_fallback ? "fallback" : "primary",
             stratum_url, port, (unsigned long)esp_get_free_heap_size());

    while (1) {
        // Check if coordinator wants us to shut down
        if (protocol_coordinator_v2_should_shutdown()) {
            ESP_LOGI(TAG, "Shutdown requested by coordinator");
            stratum_v2_close_connection(GLOBAL_STATE);
            free(frame_buf);
            free(recv_buf);
            free(conn);
            SYSTEM_release_pool_config_snapshot(&pool);
            protocol_coordinator_v2_exited();
            vTaskDelete(NULL);
            return;
        }

        if (!wifi_is_connected()) {
            ESP_LOGI(TAG, "WiFi disconnected, waiting...");
            for (int i = 0;
                 i < 100 && !protocol_coordinator_v2_should_shutdown() &&
                 !wifi_is_connected();
                 i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            continue;
        }

        if (retry_attempts >= MAX_RETRY_ATTEMPTS) {
            // Notify the coordinator of failure and let it handle fallback
            ESP_LOGW(TAG, "Max SV2 retry attempts reached (%d), notifying coordinator",
                     retry_attempts);
            stratum_v2_close_connection(GLOBAL_STATE);
            free(frame_buf);
            free(recv_buf);
            free(conn);
            SYSTEM_release_pool_config_snapshot(&pool);
            // Send only failure event — coordinator knows the task exited because it failed
            protocol_coordinator_notify_failure();
            vTaskDelete(NULL);
            return;
        }

        ESP_LOGI(TAG, "Connecting to stratum+sv2://%s:%d (attempt %d)", stratum_url, port, retry_attempts + 1);

        // Create plain TCP transport
        esp_transport_handle_t transport = esp_transport_tcp_init();
        if (!transport) {
            ESP_LOGE(TAG, "Failed to init TCP transport");
            snprintf(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info,
                     sizeof(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info), "SV2: Internal error");
            retry_attempts++;
            stratum_v2_shutdown_aware_delay(5000);
            continue;
        }

        if (protocol_coordinator_v2_should_shutdown()) {
            stratum_v2_destroy_unpublished_transport(transport);
            continue;
        }

        // Resolve up front and connect by IP so DNS stays non-blocking (a long
        // DNS timeout otherwise stalls the lwIP stack and starves the HTTP server).
        stratum_connection_info_t conn_info;
        esp_err_t resolve_ret =
            stratum_socket_resolve(stratum_url, port, &conn_info);
        if (protocol_coordinator_v2_should_shutdown()) {
            stratum_v2_destroy_unpublished_transport(transport);
            continue;
        }
        if (resolve_ret != ESP_OK) {
            ESP_LOGE(TAG, "Address resolution failed for %s", stratum_url);
            snprintf(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info,
                     sizeof(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info), "SV2: Pool unreachable");
            stratum_v2_destroy_unpublished_transport(transport);
            retry_attempts++;
            stratum_v2_shutdown_aware_delay(5000);
            continue;
        }

        int64_t connect_start_us = esp_timer_get_time();

        esp_err_t ret = esp_transport_connect(transport, conn_info.host_ip, port, TRANSPORT_TIMEOUT_MS);
        if (protocol_coordinator_v2_should_shutdown()) {
            stratum_v2_destroy_unpublished_transport(transport);
            continue;
        }
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "TCP connect failed to %s:%d (%s) (err %d)", stratum_url, port, conn_info.host_ip, ret);
            snprintf(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info,
                     sizeof(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info), "SV2: Pool unreachable");
            stratum_v2_destroy_unpublished_transport(transport);
            retry_attempts++;
            stratum_v2_shutdown_aware_delay(5000);
            continue;
        }

        ESP_LOGI(TAG, "TCP connected to %s:%d (%s)", stratum_url, port, conn_info.host_ip);

        // Configure the still-local handle before publication so a concurrent
        // stop cannot close it underneath setsockopt().
        stratum_socket_set_options(transport);

        // Publish atomically with the shutdown check. If stop has already set
        // the flag, its earlier interrupt could not see this local handle; if
        // stop arrives after publication, it takes this same lock and closes it.
        pthread_mutex_lock(&sv2_lifecycle_lock);
        bool publish_transport =
            !protocol_coordinator_v2_should_shutdown();
        if (publish_transport) {
            GLOBAL_STATE->transport = transport;
        }
        pthread_mutex_unlock(&sv2_lifecycle_lock);
        if (!publish_transport) {
            stratum_v2_destroy_unpublished_transport(transport);
            continue;
        }

        // Reset connection state
        memset(conn, 0, sizeof(*conn));
        conn->channel_type = channel_type;
        pthread_mutex_lock(&sv2_lifecycle_lock);
        GLOBAL_STATE->sv2_conn = conn;
        stratum_v2_update_pending_shares(GLOBAL_STATE);
        pthread_mutex_unlock(&sv2_lifecycle_lock);

        // --- Noise Handshake ---
        ESP_LOGI(TAG, "Starting Noise handshake (Noise_NX_Secp256k1+EllSwift_ChaChaPoly_SHA256)");

        sv2_noise_ctx_t *noise_ctx = sv2_noise_create();
        if (!noise_ctx) {
            ESP_LOGE(TAG, "Failed to create noise context");
            snprintf(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info,
                     sizeof(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info), "SV2: Internal error");
            stratum_v2_close_connection(GLOBAL_STATE);
            retry_attempts++;
            continue;
        }
        pthread_mutex_lock(&sv2_lifecycle_lock);
        bool publish_noise_ctx =
            !protocol_coordinator_v2_should_shutdown();
        if (publish_noise_ctx) {
            GLOBAL_STATE->sv2_noise_ctx = noise_ctx;
        }
        pthread_mutex_unlock(&sv2_lifecycle_lock);
        if (!publish_noise_ctx) {
            sv2_noise_destroy(noise_ctx);
            stratum_v2_close_connection(GLOBAL_STATE);
            continue;
        }

        // Load the optional authority pubkey and whether this pool requires it
        uint8_t auth_key[32];
        bool has_auth = stratum_v2_load_authority_pubkey(&pool, auth_key);
        bool require_auth = pool.sv2_require_auth;

        // When auth is required but no usable authority key is configured,
        // refuse to connect rather than mine against an unverifiable server
        if (require_auth && !has_auth) {
            ESP_LOGE(TAG, "SV2 authentication required but no authority pubkey configured, refusing to connect");
            snprintf(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info,
                     sizeof(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info), "SV2: Auth required - no key");
            stratum_v2_close_connection(GLOBAL_STATE);
            retry_attempts++;
            continue;
        }

        if (has_auth) {
            ESP_LOGI(TAG, "Authority pubkey configured, will verify server certificate");
        } else {
            ESP_LOGW(TAG, "No authority pubkey configured, server identity will not be verified");
        }

        int handshake_ret = sv2_noise_handshake(
            noise_ctx, transport, has_auth ? auth_key : NULL);
        if (protocol_coordinator_v2_should_shutdown()) {
            stratum_v2_close_connection(GLOBAL_STATE);
            continue;
        }
        if (handshake_ret != 0) {
            ESP_LOGE(TAG, "Noise handshake failed, reconnecting...");
            snprintf(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info,
                     sizeof(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info), "SV2: Auth failed - check key");
            stratum_v2_close_connection(GLOBAL_STATE);
            retry_attempts++;
            continue;
        }

        // Mirror SV1's IPv4/IPv6 indicator in the URL tooltip — the Mode field
        // already conveys the protocol (SV2 Standard/Extended Channel), and Noise
        // encryption is implied for SV2.
        const char *ip_protocol = "IPv4";
        struct sockaddr_storage peer_addr;
        socklen_t peer_len = sizeof(peer_addr);
        int sock = esp_transport_get_socket(transport);
        if (sock >= 0 && getpeername(sock, (struct sockaddr *)&peer_addr, &peer_len) == 0) {
            ip_protocol = (peer_addr.ss_family == AF_INET6) ? "IPv6" : "IPv4";
        }
        snprintf(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info,
                 sizeof(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info), "%s", ip_protocol);

        ESP_LOGI(TAG, "Encrypted channel established (ChaCha20-Poly1305) (free heap: %lu)",
                 (unsigned long)esp_get_free_heap_size());

        // --- SV2 Protocol Handshake (encrypted) ---

        uint8_t hdr_buf[6];
        sv2_frame_header_t hdr;
        int payload_len;

        // Select channel type and set connection state
        uint32_t setup_flags = (channel_type == SV2_CHANNEL_STANDARD) ? 0x01 : 0x00;

        // 1. Send SetupConnection
        {
            const char *device_model = GLOBAL_STATE->DEVICE_CONFIG.family.asic.name;
            ESP_LOGI(TAG, "Sending SetupConnection (vendor=bitaxe, hw=%s, channel=%s)",
                     device_model ? device_model : "",
                     channel_type == SV2_CHANNEL_EXTENDED ? SV2_CHANNEL_TYPE_EXTENDED : SV2_CHANNEL_TYPE_STANDARD);
            int frame_len = sv2_build_setup_connection(frame_buf, SV2_MAX_FRAME_SIZE,
                                                       stratum_url, port,
                                                       "bitaxe", device_model ? device_model : "",
                                                       "", "", setup_flags);
            if (frame_len < 0 || sv2_noise_send(noise_ctx, transport, frame_buf, frame_len) != 0) {
                ESP_LOGE(TAG, "Failed to send SetupConnection");
                snprintf(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info,
                         sizeof(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info), "SV2: Connection lost");
                stratum_v2_close_connection(GLOBAL_STATE);
                retry_attempts++;
                continue;
            }
        }

        // 2. Receive SetupConnectionSuccess
        {
            if (sv2_noise_recv(noise_ctx, transport, hdr_buf, recv_buf,
                               SV2_MAX_FRAME_SIZE, &payload_len) != 0) {
                ESP_LOGE(TAG, "Failed to receive SetupConnectionSuccess");
                snprintf(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info,
                         sizeof(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info), "SV2: Pool not responding");
                stratum_v2_close_connection(GLOBAL_STATE);
                retry_attempts++;
                continue;
            }
            sv2_parse_frame_header(hdr_buf, &hdr);

            if (hdr.msg_type != SV2_MSG_SETUP_CONNECTION_SUCCESS) {
                ESP_LOGE(TAG, "SetupConnection rejected by pool (msg_type=0x%02x)", hdr.msg_type);
                snprintf(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info,
                         sizeof(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info), "SV2: Pool rejected config");
                stratum_v2_close_connection(GLOBAL_STATE);
                retry_attempts++;
                continue;
            }

            uint16_t used_version;
            uint32_t flags;
            if (sv2_parse_setup_connection_success(recv_buf, payload_len, &used_version, &flags) != 0) {
                ESP_LOGE(TAG, "Failed to parse SetupConnectionSuccess");
                stratum_v2_close_connection(GLOBAL_STATE);
                retry_attempts++;
                continue;
            }
            ESP_LOGI(TAG, "Pool accepted connection: SV2 version=%d, flags=0x%08lx", used_version, flags);
        }

        // 3. Send OpenMiningChannel (extended or standard)
        {
            const char *user = pool.user;
            float hash_rate = 1e12;
            int frame_len;

            if (channel_type == SV2_CHANNEL_EXTENDED) {
                ESP_LOGI(TAG, "Opening extended mining channel (user=%s)", user ? user : "(empty)");
                frame_len = sv2_build_open_extended_mining_channel(frame_buf, SV2_MAX_FRAME_SIZE,
                                                                    1, user ? user : "", hash_rate, 2);
            } else {
                ESP_LOGI(TAG, "Opening standard mining channel (user=%s)", user ? user : "(empty)");
                frame_len = sv2_build_open_standard_mining_channel(frame_buf, SV2_MAX_FRAME_SIZE,
                                                                    1, user ? user : "", hash_rate);
            }

            if (frame_len < 0 || sv2_noise_send(noise_ctx, transport, frame_buf, frame_len) != 0) {
                ESP_LOGE(TAG, "Failed to send OpenMiningChannel");
                snprintf(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info,
                         sizeof(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info), "SV2: Connection lost");
                stratum_v2_close_connection(GLOBAL_STATE);
                retry_attempts++;
                continue;
            }
        }

        // 4. Receive OpenMiningChannelSuccess
        {
            if (sv2_noise_recv(noise_ctx, transport, hdr_buf, recv_buf,
                               SV2_MAX_FRAME_SIZE, &payload_len) != 0) {
                ESP_LOGE(TAG, "Failed to receive OpenChannelSuccess");
                snprintf(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info,
                         sizeof(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info), "SV2: Pool not responding");
                stratum_v2_close_connection(GLOBAL_STATE);
                retry_attempts++;
                continue;
            }
            sv2_parse_frame_header(hdr_buf, &hdr);

            uint8_t expected_msg = (channel_type == SV2_CHANNEL_EXTENDED)
                                   ? SV2_MSG_OPEN_EXTENDED_MINING_CHANNEL_SUCCESS
                                   : SV2_MSG_OPEN_STANDARD_MINING_CHANNEL_SUCCESS;

            if (hdr.msg_type != expected_msg) {
                ESP_LOGE(TAG, "OpenChannel rejected by pool (msg_type=0x%02x, expected=0x%02x)",
                         hdr.msg_type, expected_msg);
                snprintf(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info,
                         sizeof(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info), "SV2: Pool rejected miner");
                stratum_v2_close_connection(GLOBAL_STATE);
                retry_attempts++;
                continue;
            }

            uint32_t request_id, channel_id, group_channel_id;
            uint8_t target[32];

            if (channel_type == SV2_CHANNEL_EXTENDED) {
                uint16_t extranonce_size;
                uint8_t extranonce_prefix[32];
                uint8_t extranonce_prefix_len;

                if (sv2_parse_open_extended_channel_success(recv_buf, payload_len,
                                                            &request_id, &channel_id, target,
                                                            &extranonce_size,
                                                            extranonce_prefix, &extranonce_prefix_len,
                                                            &group_channel_id) != 0) {
                    ESP_LOGE(TAG, "Failed to parse OpenExtendedChannelSuccess");
                    stratum_v2_close_connection(GLOBAL_STATE);
                    retry_attempts++;
                    continue;
                }

                if (extranonce_size > SV2_MAX_EXTRANONCE_SIZE) {
                    ESP_LOGE(TAG,
                             "Pool extranonce size %u exceeds supported maximum %u",
                             extranonce_size, SV2_MAX_EXTRANONCE_SIZE);
                    stratum_v2_close_connection(GLOBAL_STATE);
                    retry_attempts++;
                    continue;
                }

                conn->extranonce_size = (uint8_t)extranonce_size;
                conn->extranonce_prefix_len = extranonce_prefix_len;
                memcpy(conn->extranonce_prefix, extranonce_prefix, extranonce_prefix_len);

                ESP_LOGI(TAG, "Extended channel: extranonce_size=%d, prefix_len=%d",
                         extranonce_size, extranonce_prefix_len);
            } else {
                uint8_t extranonce_prefix[32];
                uint8_t extranonce_prefix_len;

                if (sv2_parse_open_channel_success(recv_buf, payload_len,
                                                    &request_id, &channel_id, target,
                                                    extranonce_prefix, &extranonce_prefix_len,
                                                    &group_channel_id) != 0) {
                    ESP_LOGE(TAG, "Failed to parse OpenChannelSuccess");
                    stratum_v2_close_connection(GLOBAL_STATE);
                    retry_attempts++;
                    continue;
                }
            }

            conn->channel_id = channel_id;
            memcpy(conn->target, target, 32);
            // Publish readiness only after every field used by share submission
            // has been initialized. Submitters acquire the same mutex.
            pthread_mutex_lock(&sv2_lifecycle_lock);
            conn->channel_opened = true;
            pthread_mutex_unlock(&sv2_lifecycle_lock);

            double pdiff = hash_to_pdiff(target);
            SYSTEM_set_pool_difficulty(GLOBAL_STATE, pdiff, true);

            ESP_LOGI(TAG, "Mining channel opened: channel_id=%lu, group=%lu, type=%s",
                     channel_id, group_channel_id,
                     channel_type == SV2_CHANNEL_EXTENDED ? SV2_CHANNEL_TYPE_EXTENDED : SV2_CHANNEL_TYPE_STANDARD);
            ESP_LOGI(TAG, "Set pool difficulty: %g", pdiff);
        }

        // Connection successful, reset retry counter
        retry_attempts = 0;
        // Tell the coordinator so it clears its failure counter and pools_unavailable.
        protocol_coordinator_notify_success();

        {
            float elapsed_ms = (float)(esp_timer_get_time() - connect_start_us) / 1000.0f;
            ESP_LOGI(TAG, "SV2+Noise connection ready (%.0f ms). Waiting for jobs from %s:%d",
                     elapsed_ms, stratum_url, port);
        }

        // --- Main receive loop ---
        while (1) {
            if (sv2_noise_recv(noise_ctx, transport, hdr_buf, recv_buf,
                               SV2_MAX_FRAME_SIZE, &payload_len) != 0) {
                ESP_LOGE(TAG, "Failed to receive frame, reconnecting...");
                retry_attempts++;
                stratum_v2_close_connection(GLOBAL_STATE);
                break;
            }

            sv2_parse_frame_header(hdr_buf, &hdr);

            switch (hdr.msg_type) {
                case SV2_MSG_NEW_MINING_JOB:
                    stratum_v2_handle_new_mining_job(GLOBAL_STATE, conn, recv_buf, hdr.msg_length);
                    break;

                case SV2_MSG_NEW_EXTENDED_MINING_JOB:
                    stratum_v2_handle_new_extended_mining_job(
                        GLOBAL_STATE, conn, &pool, recv_buf, hdr.msg_length);
                    break;

                case SV2_MSG_SET_NEW_PREV_HASH:
                    stratum_v2_handle_set_new_prev_hash(GLOBAL_STATE, conn, recv_buf, hdr.msg_length);
                    break;

                case SV2_MSG_SET_TARGET:
                    stratum_v2_handle_set_target(GLOBAL_STATE, conn, recv_buf, hdr.msg_length);
                    break;

                case SV2_MSG_SUBMIT_SHARES_SUCCESS: {
                    uint32_t channel_id, last_sequence_number, accepted_count;
                    if (sv2_parse_submit_shares_success(recv_buf, hdr.msg_length, &channel_id, &last_sequence_number, &accepted_count) == 0) {
                        // Measure against the share acknowledged by last_sequence_number — the
                        // most recent share in the ack, giving the cleanest available round trip.
                        // accepted_count is surfaced separately so the UI can flag batch acks,
                        // where the elapsed time also includes the pool's batching window.
                        int slot = last_sequence_number % SV2_SUBMIT_TIMING_SLOTS;
                        int64_t submit_time_us = stratum_v2_submit_time_us[slot];
                        if (submit_time_us > 0) {
                            float response_time_ms = (float)(esp_timer_get_time() - submit_time_us) / 1000.0f;
                            ESP_LOGI(TAG, "Shares accepted: %lu (%.1f ms)", accepted_count, response_time_ms);
                            GLOBAL_STATE->SYSTEM_MODULE.response_time = response_time_ms;
                            GLOBAL_STATE->SYSTEM_MODULE.response_share_batch = (uint16_t)accepted_count;
                            stratum_v2_submit_time_us[slot] = 0;
                        } else {
                            ESP_LOGI(TAG, "Shares accepted: %lu", accepted_count);
                        }
                        for (uint32_t i = 0; i < accepted_count; i++) {
                            SYSTEM_notify_accepted_share(GLOBAL_STATE);
                        }
                        uint32_t resolved = last_sequence_number + 1;  // 0-based seq -> count
                        if (resolved > conn->resolved_shares) {
                            conn->resolved_shares = resolved;
                        }
                        stratum_v2_update_pending_shares(GLOBAL_STATE);
                    }
                    break;
                }

                case SV2_MSG_SUBMIT_SHARES_ERROR: {
                    uint32_t channel_id, seq_num;
                    char error_code[64];
                    if (sv2_parse_submit_shares_error(recv_buf, hdr.msg_length,
                                                      &channel_id, &seq_num,
                                                      error_code, sizeof(error_code)) == 0) {
                        ESP_LOGW(TAG, "Share rejected: %s", error_code);
                        SYSTEM_notify_rejected_share(GLOBAL_STATE, error_code);
                        uint32_t resolved = seq_num + 1;
                        if (resolved > conn->resolved_shares) {
                            conn->resolved_shares = resolved;
                        }
                        stratum_v2_update_pending_shares(GLOBAL_STATE);
                    }
                    break;
                }

                default:
                    ESP_LOGW(TAG, "Unknown SV2 message type: 0x%02x (len=%lu)", hdr.msg_type, hdr.msg_length);
                    break;
            }
        }
    }

    // Should not reach here, but clean up just in case
    stratum_v2_close_connection(GLOBAL_STATE);
    free(frame_buf);
    free(recv_buf);
    free(conn);
    vTaskDelete(NULL);
}
