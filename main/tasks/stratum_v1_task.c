#include "esp_log.h"
#include "esp_system.h"
#include "system.h"
#include "global_state.h"
#include <lwip/tcpip.h>
#include "stratum_v1_task.h"
#include "stratum_api.h"
#include "stratum_socket.h"
#include "protocol_coordinator.h"
#include "connect.h"
#include "work_queue.h"
#include "asic_result_task.h"
#include "asic_init.h"
#include <esp_sntp.h>
#include "esp_timer.h"
#include "esp_transport.h"
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include "utils.h"
#include "coinbase_decoder.h"
#include <esp_heap_caps.h>
#include "esp_transport_ssl.h"
#include "freertos/task.h"

#define MAX_RETRY_ATTEMPTS 3
#define MAX_CRITICAL_RETRY_ATTEMPTS 5
#define PORT CONFIG_STRATUM_PORT
#define STRATUM_URL CONFIG_STRATUM_URL
#define STRATUM_TLS CONFIG_STRATUM_TLS
#define STRATUM_CERT CONFIG_STRATUM_CERT

#define FALLBACK_PORT CONFIG_FALLBACK_STRATUM_PORT
#define FALLBACK_STRATUM_URL CONFIG_FALLBACK_STRATUM_URL
#define FALLBACK_STRATUM_TLS CONFIG_FALLBACK_STRATUM_TLS
#define FALLBACK_STRATUM_CERT CONFIG_FALLBACK_STRATUM_CERT

#define STRATUM_PW CONFIG_STRATUM_PW
#define FALLBACK_STRATUM_PW CONFIG_FALLBACK_STRATUM_PW
#define STRATUM_DIFFICULTY CONFIG_STRATUM_DIFFICULTY

#define TRANSPORT_TIMEOUT_MS 5000
#define CONFIGURE_RESPONSE_TIMEOUT_US 10000000LL

#define BUFFER_SIZE 1024

static const char *TAG = "stratum_v1_task";
static pthread_mutex_t v1_lifecycle_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t v1_writers_drained = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t v1_state_lock = PTHREAD_MUTEX_INITIALIZER;
static unsigned int v1_active_writers = 0;
static bool v1_connection_closing = true;

static bool version_mask_meets_minimum(uint32_t mask)
{
    unsigned int bit_count = 0;
    while (mask != 0) {
        bit_count += mask & 1U;
        mask >>= 1;
    }
    return bit_count >= STRATUM_VERSION_ROLLING_MIN_BIT_COUNT;
}

static void free_v1_queued_work(void *work)
{
    stratum_v1_work *v1_work = (stratum_v1_work *)work;
    if (v1_work == NULL) {
        return;
    }
    if (v1_work->notification != NULL) {
        STRATUM_V1_free_mining_notify(v1_work->notification);
    }
    free(v1_work->extranonce_1);
    free(v1_work);
}

static mining_notify *clone_mining_notify(const mining_notify *source)
{
    if (source == NULL || source->job_id == NULL ||
        source->prev_block_hash == NULL || source->coinbase_1 == NULL ||
        source->coinbase_2 == NULL ||
        (source->n_merkle_branches != 0 &&
         source->merkle_branches == NULL) ||
        source->n_merkle_branches > MAX_MERKLE_BRANCHES) {
        return NULL;
    }

    mining_notify *copy = calloc(1, sizeof(*copy));
    if (copy == NULL) {
        return NULL;
    }
    copy->job_id = strdup(source->job_id);
    copy->prev_block_hash = strdup(source->prev_block_hash);
    copy->coinbase_1 = strdup(source->coinbase_1);
    copy->coinbase_2 = strdup(source->coinbase_2);
    copy->n_merkle_branches = source->n_merkle_branches;
    copy->version = source->version;
    copy->target = source->target;
    copy->ntime = source->ntime;
    copy->clean_jobs = source->clean_jobs;

    size_t branch_bytes = source->n_merkle_branches * HASH_SIZE;
    if (branch_bytes != 0) {
        copy->merkle_branches = malloc(branch_bytes);
        if (copy->merkle_branches != NULL) {
            memcpy(copy->merkle_branches, source->merkle_branches,
                   branch_bytes);
        }
    }
    if (copy->job_id == NULL || copy->prev_block_hash == NULL ||
        copy->coinbase_1 == NULL || copy->coinbase_2 == NULL ||
        (branch_bytes != 0 && copy->merkle_branches == NULL)) {
        STRATUM_V1_free_mining_notify(copy);
        return NULL;
    }
    return copy;
}

static StratumApiV1Message stratum_api_v1_message = {};

static int stratum_get_next_uid(GlobalState * GLOBAL_STATE)
{
    taskENTER_CRITICAL(&GLOBAL_STATE->stratum_mux);
    int uid = GLOBAL_STATE->send_uid++;
    taskEXIT_CRITICAL(&GLOBAL_STATE->stratum_mux);
    return uid;
}


static void stratum_v1_reset_uid(GlobalState *GLOBAL_STATE)
{
    ESP_LOGI(TAG, "Resetting stratum uid");
    taskENTER_CRITICAL(&GLOBAL_STATE->stratum_mux);
    GLOBAL_STATE->send_uid = 1;
    taskEXIT_CRITICAL(&GLOBAL_STATE->stratum_mux);
}

void stratum_v1_close_connection(GlobalState *GLOBAL_STATE)
{
    ESP_LOGE(TAG, "Shutting down socket and restarting...");

    // Mark closing and close first so a stalled share writer wakes promptly.
    // Job invalidation advances the generation and drains queued shares
    // independently; it deliberately does not wait for the submit lock.
    pthread_mutex_lock(&v1_lifecycle_lock);
    v1_connection_closing = true;
    esp_transport_handle_t transport = GLOBAL_STATE->transport;
    if (transport != NULL) {
        esp_transport_close(transport);
    }
    pthread_mutex_unlock(&v1_lifecycle_lock);

    SYSTEM_clean_jobs_queue(GLOBAL_STATE);

    pthread_mutex_lock(&v1_lifecycle_lock);
    GLOBAL_STATE->transport = NULL;
    pthread_mutex_unlock(&v1_lifecycle_lock);

    if (transport != NULL) {
        // Keep the transport object alive until every writer that retained it
        // observes the close and exits.
        pthread_mutex_lock(&v1_lifecycle_lock);
        while (v1_active_writers != 0) {
            pthread_cond_wait(&v1_writers_drained, &v1_lifecycle_lock);
        }
        pthread_mutex_unlock(&v1_lifecycle_lock);
        esp_transport_destroy(transport);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void stratum_v1_interrupt_connection(GlobalState *GLOBAL_STATE)
{
    pthread_mutex_lock(&v1_lifecycle_lock);
    v1_connection_closing = true;
    if (GLOBAL_STATE->transport != NULL) {
        // Do not destroy here; the owning V1 task performs the writer-drain
        // wait in stratum_v1_close_connection(). Keep the lifecycle lock held
        // through close so the owner cannot detach and destroy this handle
        // between the pointer load and the close call.
        esp_transport_close(GLOBAL_STATE->transport);
    }
    pthread_mutex_unlock(&v1_lifecycle_lock);
}

static void stratum_v1_clean_jobs(GlobalState *GLOBAL_STATE)
{
    SYSTEM_clean_jobs_queue(GLOBAL_STATE);
}

static void stratum_v1_reset_connection_state(GlobalState *GLOBAL_STATE)
{
    SYSTEM_clean_jobs_queue(GLOBAL_STATE);
    pthread_mutex_lock(&v1_lifecycle_lock);
    pthread_mutex_lock(&v1_state_lock);
    char *old_extranonce = GLOBAL_STATE->extranonce_str;
    GLOBAL_STATE->extranonce_str = NULL;
    GLOBAL_STATE->extranonce_2_len = 0;
    GLOBAL_STATE->pool_difficulty = 1.0;
    GLOBAL_STATE->stratum_v1_version_rolling_enabled = false;
    GLOBAL_STATE->stratum_v1_version_mask = 0;
    GLOBAL_STATE->version_mask = 0;
    GLOBAL_STATE->new_stratum_version_rolling_msg = true;
    pthread_mutex_unlock(&v1_state_lock);
    pthread_mutex_unlock(&v1_lifecycle_lock);
    free(old_extranonce);
}

static void stratum_v1_set_difficulty(GlobalState *GLOBAL_STATE,
                                      double difficulty)
{
    // Do not take stratum_v1_submit_lock here. Submitters serialize socket
    // writes separately and re-read pool_difficulty under v1_state_lock before
    // each write.
    // An already-snapshotted write may finish, but blocking pool RX behind its
    // five-second socket timeout would delay subsequent difficulty/job data.
    pthread_mutex_lock(&v1_state_lock);
    GLOBAL_STATE->pool_difficulty = difficulty;
    pthread_mutex_unlock(&v1_state_lock);
}

double stratum_v1_get_current_difficulty(GlobalState *GLOBAL_STATE)
{
    if (GLOBAL_STATE == NULL) {
        return 0.0;
    }

    pthread_mutex_lock(&v1_state_lock);
    double difficulty = GLOBAL_STATE->pool_difficulty;
    pthread_mutex_unlock(&v1_state_lock);
    return difficulty;
}

static void stratum_v1_set_rolling_state(GlobalState *GLOBAL_STATE,
                                         bool enabled, uint32_t mask)
{
    if (!enabled) {
        mask = 0;
    }

    // A BIP310 state change is valid immediately. Publish it and invalidate
    // every old job as one lifecycle operation so a submission cannot observe
    // a new submit shape with an old job generation.
    SYSTEM_clean_jobs_queue(GLOBAL_STATE);
    pthread_mutex_lock(&v1_lifecycle_lock);
    pthread_mutex_lock(&v1_state_lock);
    GLOBAL_STATE->stratum_v1_version_rolling_enabled = enabled;
    GLOBAL_STATE->stratum_v1_version_mask = mask;
    GLOBAL_STATE->version_mask = mask;
    GLOBAL_STATE->new_stratum_version_rolling_msg = true;
    pthread_mutex_unlock(&v1_state_lock);
    pthread_mutex_unlock(&v1_lifecycle_lock);
}

static void stratum_v1_replace_extranonce(GlobalState *GLOBAL_STATE,
                                          char *extranonce,
                                          uint32_t extranonce_2_len)
{
    SYSTEM_clean_jobs_queue(GLOBAL_STATE);
    pthread_mutex_lock(&v1_lifecycle_lock);
    pthread_mutex_lock(&v1_state_lock);
    char *old_extranonce = GLOBAL_STATE->extranonce_str;
    GLOBAL_STATE->extranonce_str = extranonce;
    GLOBAL_STATE->extranonce_2_len = extranonce_2_len;
    pthread_mutex_unlock(&v1_state_lock);
    pthread_mutex_unlock(&v1_lifecycle_lock);
    free(old_extranonce);
}

static stratum_v1_work *stratum_v1_create_work(
    GlobalState *GLOBAL_STATE, mining_notify *notification)
{
    stratum_v1_work *work = calloc(1, sizeof(*work));
    if (work == NULL) {
        return NULL;
    }

    pthread_mutex_lock(&v1_state_lock);
    if (GLOBAL_STATE->extranonce_str != NULL) {
        work->extranonce_1 = strdup(GLOBAL_STATE->extranonce_str);
    }
    work->extranonce_2_len = GLOBAL_STATE->extranonce_2_len;
    work->difficulty = GLOBAL_STATE->pool_difficulty;
    work->version_rolling_enabled =
        GLOBAL_STATE->stratum_v1_version_rolling_enabled;
    work->version_mask = work->version_rolling_enabled
                             ? GLOBAL_STATE->stratum_v1_version_mask
                             : 0;
    pthread_mutex_unlock(&v1_state_lock);

    if (work->extranonce_1 == NULL ||
        work->extranonce_2_len > MAX_EXTRANONCE_2_LEN ||
        !(work->difficulty > 0.0)) {
        free(work->extranonce_1);
        free(work);
        return NULL;
    }
    work->notification = notification;
    return work;
}

int stratum_v1_submit_share_safe(
    GlobalState *GLOBAL_STATE, uint32_t expected_generation, int uid,
    const char *user, const char *job_id, const char *extranonce_2,
    uint32_t ntime, uint32_t nonce, bool version_rolling_enabled,
    uint32_t version_bits, double share_difficulty, double job_difficulty,
    uint64_t *sent_time_us)
{
    // Serializes submits so at most one wire write is in flight. The final
    // generation check under this lock rejects any share whose job was
    // invalidated before the write started; invalidation itself does not
    // wait on this lock (see SYSTEM_clean_jobs_queue).
    pthread_mutex_lock(&GLOBAL_STATE->stratum_v1_submit_lock);
    pthread_mutex_lock(&v1_lifecycle_lock);
    if (expected_generation != ASIC_result_task_get_job_generation() ||
        GLOBAL_STATE->transport == NULL || v1_connection_closing) {
        pthread_mutex_unlock(&v1_lifecycle_lock);
        pthread_mutex_unlock(&GLOBAL_STATE->stratum_v1_submit_lock);
        return STRATUM_V1_SUBMIT_STALE;
    }

    pthread_mutex_lock(&v1_state_lock);
    double announced_difficulty = GLOBAL_STATE->pool_difficulty;
    pthread_mutex_unlock(&v1_state_lock);
    double required_difficulty = mining_v1_effective_share_difficulty(
        job_difficulty, announced_difficulty);
    if (!(share_difficulty >= required_difficulty)) {
        pthread_mutex_unlock(&v1_lifecycle_lock);
        pthread_mutex_unlock(&GLOBAL_STATE->stratum_v1_submit_lock);
        return STRATUM_V1_SUBMIT_FILTERED;
    }

    esp_transport_handle_t transport = GLOBAL_STATE->transport;
    v1_active_writers++;
    pthread_mutex_unlock(&v1_lifecycle_lock);

    int result = STRATUM_V1_submit_share(
        transport, uid, user, job_id, extranonce_2, ntime,
        nonce, version_rolling_enabled, version_bits, sent_time_us);
    int submit_errno = errno;

    pthread_mutex_lock(&v1_lifecycle_lock);
    if ((result == STRATUM_SOCKET_WRITE_ERROR ||
         result == STRATUM_SOCKET_WRITE_TRUNCATED) &&
        GLOBAL_STATE->transport == transport && !v1_connection_closing) {
        // Bind the failure to the exact handle used above. Interrupting later
        // in the worker can race the owner through reconnect and close a newly
        // installed transport instead of this failed one.
        v1_connection_closing = true;
        esp_transport_close(transport);
    }
    v1_active_writers--;
    if (v1_active_writers == 0) {
        pthread_cond_broadcast(&v1_writers_drained);
    }
    pthread_mutex_unlock(&v1_lifecycle_lock);
    pthread_mutex_unlock(&GLOBAL_STATE->stratum_v1_submit_lock);
    errno = submit_errno;
    return result;
}

static void decode_mining_notification(GlobalState *GLOBAL_STATE,
                                       const stratum_v1_work *work)
{
    const mining_notify *mining_notification = work->notification;
    mining_notification_result_t *result = heap_caps_malloc(sizeof(mining_notification_result_t), MALLOC_CAP_SPIRAM);
    if (!result) {
        ESP_LOGE(TAG, "Failed to allocate result in PSRAM");
        return;
    }
    memset(result, 0, sizeof(mining_notification_result_t));

    uint16_t pool_idx = GLOBAL_STATE->SYSTEM_MODULE.is_using_fallback ? GLOBAL_STATE->SYSTEM_MODULE.secondary_pool_index : GLOBAL_STATE->SYSTEM_MODULE.primary_pool_index;
    const char *user = GLOBAL_STATE->SYSTEM_MODULE.pools[pool_idx].user;
    bool decode_coinbase_tx = GLOBAL_STATE->SYSTEM_MODULE.pools[pool_idx].decode_coinbase_tx;

    if (coinbase_process_notification(mining_notification,
                                     work->extranonce_1,
                                     work->extranonce_2_len,
                                     user,
                                     decode_coinbase_tx,
                                     result) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to process mining notification");
        free(result);
        return;
    }

    // Update network difficulty
    GLOBAL_STATE->network_nonce_diff = (uint64_t) result->network_difficulty;
    suffixString(result->network_difficulty, GLOBAL_STATE->network_diff_string, DIFF_STRING_SIZE, 0);

    // Update block height
    if (result->block_height != GLOBAL_STATE->block_height) {
        ESP_LOGI(TAG, "Block height %d", result->block_height);
        GLOBAL_STATE->block_height = result->block_height;
    }

    // Update block signals (BIP-110, BIP-54, etc.)
    GLOBAL_STATE->block_signals_count = 0;
    if (result->bip54_signaling) {
        strncpy(GLOBAL_STATE->block_signals[GLOBAL_STATE->block_signals_count], "BIP-54", MAX_BLOCK_SIGNAL_LEN - 1);
        GLOBAL_STATE->block_signals[GLOBAL_STATE->block_signals_count][MAX_BLOCK_SIGNAL_LEN - 1] = '\0';
        GLOBAL_STATE->block_signals_count++;
        ESP_LOGI(TAG, "BIP-54 signaling detected");
    }
    if (result->bip110_signaling) {
        strncpy(GLOBAL_STATE->block_signals[GLOBAL_STATE->block_signals_count], "BIP-110", MAX_BLOCK_SIGNAL_LEN - 1);
        GLOBAL_STATE->block_signals[GLOBAL_STATE->block_signals_count][MAX_BLOCK_SIGNAL_LEN - 1] = '\0';
        GLOBAL_STATE->block_signals_count++;
        ESP_LOGI(TAG, "BIP-110 signaling detected");
    }

    // Update scriptsig
    if (result->scriptsig) {
        if (strcmp(result->scriptsig, GLOBAL_STATE->scriptsig) != 0) {
            ESP_LOGI(TAG, "Scriptsig: %s", result->scriptsig);
            strncpy(GLOBAL_STATE->scriptsig, result->scriptsig, sizeof(GLOBAL_STATE->scriptsig) - 1);
            GLOBAL_STATE->scriptsig[sizeof(GLOBAL_STATE->scriptsig) - 1] = '\0';
        }
        free(result->scriptsig);
    }

    // Update coinbase outputs
    // Safety guard: ensure output_count doesn't exceed array capacity
    if (result->output_count > MAX_COINBASE_TX_OUTPUTS) {
        result->output_count = MAX_COINBASE_TX_OUTPUTS;
    }

    GLOBAL_STATE->coinbase_value_total_satoshis = result->total_value_satoshis;
    ESP_LOGI(TAG, "Coinbase outputs: %d, total value: %llu%s", result->output_count, result->total_value_satoshis, result->decode_coinbase_tx ? " sats" : "");

    if (result->output_count != GLOBAL_STATE->coinbase_output_count ||
        memcmp(result->outputs, GLOBAL_STATE->coinbase_outputs, sizeof(coinbase_output_t) * result->output_count) != 0) {

        GLOBAL_STATE->coinbase_output_count = result->output_count;
        memcpy(GLOBAL_STATE->coinbase_outputs, result->outputs, sizeof(coinbase_output_t) * result->output_count);
        GLOBAL_STATE->coinbase_value_user_satoshis = result->user_value_satoshis;
        for (int i = 0; i < result->output_count; i++) {
            if (result->outputs[i].value_satoshis > 0) {
                if (result->outputs[i].is_user_output) {
                    ESP_LOGI(TAG, "  Output %d: %s (%llu sat) (Your payout address)", i, result->outputs[i].address, result->outputs[i].value_satoshis);
                } else {
                    ESP_LOGI(TAG, "  Output %d: %s (%llu sat)", i, result->outputs[i].address, result->outputs[i].value_satoshis);
                }
            } else {
                ESP_LOGI(TAG, "  Output %d: %s", i, result->outputs[i].address);
            }
        }
    }

    free(result);
}

static bool stratum_v1_enqueue_work(GlobalState *GLOBAL_STATE,
                                    mining_notify *notification)
{
    stratum_v1_work *work =
        stratum_v1_create_work(GLOBAL_STATE, notification);
    if (work == NULL) {
        ESP_LOGW(TAG,
                 "Dropping V1 notification until valid extranonce and difficulty state is available");
        STRATUM_V1_free_mining_notify(notification);
        return false;
    }

    // Snapshot the generation before decoding. Decoding reads the immutable
    // work but can take long enough for a concurrent reconnect/clean to run.
    // If that happens, the stale-generation item is rejected by the scheduler
    // after it is published.
    uint32_t generation = ASIC_result_task_get_pool_generation();
    decode_mining_notification(GLOBAL_STATE, work);
    queue_enqueue(
        &GLOBAL_STATE->stratum_queue, work,
        (work_queue_item_metadata) {
            .generation = generation,
            .kind = WORK_QUEUE_ITEM_STRATUM_V1,
            .free_fn = free_v1_queued_work,
        });
    return true;
}

static void stratum_v1_refresh_latest_work(
    GlobalState *GLOBAL_STATE, const mining_notify *latest_notification)
{
    if (latest_notification == NULL) {
        return;
    }
    mining_notify *refresh = clone_mining_notify(latest_notification);
    if (refresh == NULL) {
        ESP_LOGE(TAG, "Unable to refresh V1 work after connection-state update");
        return;
    }
    stratum_v1_enqueue_work(GLOBAL_STATE, refresh);
}

void stratum_v1_task(void *pvParameters)
{
    GlobalState *GLOBAL_STATE = (GlobalState *)pvParameters;

    uint16_t pool_idx = GLOBAL_STATE->SYSTEM_MODULE.is_using_fallback ? GLOBAL_STATE->SYSTEM_MODULE.secondary_pool_index : GLOBAL_STATE->SYSTEM_MODULE.primary_pool_index;
    char *stratum_url = GLOBAL_STATE->SYSTEM_MODULE.pools[pool_idx].url;
    uint16_t port = GLOBAL_STATE->SYSTEM_MODULE.pools[pool_idx].port;

    STRATUM_V1_initialize_buffer();
    int retry_attempts = 0;
    int retry_critical_attempts = 0;
    // Explicit protocol rejection is sticky for this pool task. Transport
    // failures during negotiation only skip the next probe: this preserves a
    // compatibility reconnect without permanently losing version rolling due
    // to a transient socket drop.
    stratum_v1_bip310_state_t bip310_state =
        STRATUM_V1_BIP310_STATE_INITIALIZER;
    mining_notify *latest_notification = NULL;

    ESP_LOGI(TAG, "Opening connection to pool: %s:%d", stratum_url, port);
    while (1) {
        // Check if coordinator wants us to shut down
        if (protocol_coordinator_v1_should_shutdown()) {
            ESP_LOGI(TAG, "Coordinator requested shutdown, exiting");
            stratum_v1_close_connection(GLOBAL_STATE);
            if (latest_notification != NULL) {
                STRATUM_V1_free_mining_notify(latest_notification);
                latest_notification = NULL;
            }
            cleanup_stratum_buffer();
            protocol_coordinator_v1_exited();
            vTaskDelete(NULL);
            return;
        }

        if (!asic_lifecycle_is_running(GLOBAL_STATE)) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        if (!wifi_is_connected()) {
            ESP_LOGI(TAG, "WiFi disconnected, attempting to reconnect...");
            vTaskDelay(10000 / portTICK_PERIOD_MS);
            continue;
        }

        if (retry_attempts >= MAX_RETRY_ATTEMPTS)
        {
            // Notify the coordinator and exit. The coordinator owns the
            // "all pools unreachable" decision, pool swapping, and power-pause
            // recovery — see protocol_coordinator.c.
            ESP_LOGW(TAG, "Max V1 retry attempts reached (%d), notifying coordinator", retry_attempts);
            stratum_v1_close_connection(GLOBAL_STATE);
            if (latest_notification != NULL) {
                STRATUM_V1_free_mining_notify(latest_notification);
                latest_notification = NULL;
            }
            cleanup_stratum_buffer();
            protocol_coordinator_notify_failure();
            vTaskDelete(NULL);
            return;
        }

        pool_idx = GLOBAL_STATE->SYSTEM_MODULE.is_using_fallback ? GLOBAL_STATE->SYSTEM_MODULE.secondary_pool_index : GLOBAL_STATE->SYSTEM_MODULE.primary_pool_index;
        stratum_url = GLOBAL_STATE->SYSTEM_MODULE.pools[pool_idx].url;
        port = GLOBAL_STATE->SYSTEM_MODULE.pools[pool_idx].port;

        stratum_connection_info_t conn_info;
        if (stratum_socket_resolve(stratum_url, port, &conn_info) != ESP_OK) {
            ESP_LOGE(TAG, "Address resolution failed for %s", stratum_url);
            retry_attempts++;
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "Connecting to: stratum+tcp://%s:%d (%s)", stratum_url, port, conn_info.host_ip);

        tls_mode tls = GLOBAL_STATE->SYSTEM_MODULE.pools[pool_idx].tls;
        char * cert = GLOBAL_STATE->SYSTEM_MODULE.pools[pool_idx].cert;
        retry_critical_attempts = 0;

        esp_transport_handle_t transport =
            STRATUM_V1_transport_init(tls, cert);
        // Check if transport was initialized
        if (transport == NULL) {
            ESP_LOGE(TAG, "Transport initialization failed.");
            if (++retry_critical_attempts > MAX_CRITICAL_RETRY_ATTEMPTS) {
                ESP_LOGE(TAG, "Max retry attempts reached, restarting...");
                esp_restart();
            }
            retry_attempts++;
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }
        retry_critical_attempts = 0;

        // Use the already-resolved IP to avoid a second DNS lookup inside esp_transport_connect.
        // This prevents long DNS timeouts from blocking the lwIP stack and starving the HTTP server.
        if (tls != DISABLED) {
            esp_transport_ssl_set_common_name(transport, stratum_url);
        }
        ESP_LOGI(TAG, "Transport initialized, connecting to %s:%d (%s)", stratum_url, port, conn_info.host_ip);
        esp_err_t ret = esp_transport_connect(
            transport, conn_info.host_ip, port, TRANSPORT_TIMEOUT_MS);
        if (ret != ESP_OK) {
            retry_attempts++;
            ESP_LOGE(TAG, "Transport unable to connect to %s:%d (errno %d). Attempt: %d", stratum_url, port, ret, retry_attempts);
            // close the transport
            esp_transport_close(transport);
            esp_transport_destroy(transport);
            // instead of restarting, retry this every 5 seconds
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }

        stratum_socket_set_options(transport);
        pthread_mutex_lock(&v1_lifecycle_lock);
        GLOBAL_STATE->transport = transport;
        v1_connection_closing = false;
        pthread_mutex_unlock(&v1_lifecycle_lock);

        const char *protocol = (conn_info.addr_family == AF_INET6) ? "IPv6" : "IPv4";
        const char *tls_status;

        switch (tls) {
            case DISABLED:     tls_status = ""; break;
            case BUNDLED_CRT:  tls_status = " (TLS)"; break;
            case CUSTOM_CRT:   tls_status = " (TLS Cert)"; break;
            default:           tls_status = ""; break;
        }

        snprintf(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info,
                 sizeof(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info),
                 "%s%s", protocol, tls_status);

        stratum_v1_reset_uid(GLOBAL_STATE);
        stratum_v1_reset_connection_state(GLOBAL_STATE);
        // Discard any complete or partial JSON lines and request timings from
        // the prior socket. Buffered data is connection-scoped and must never
        // cross a normal client.reconnect.
        STRATUM_V1_initialize_buffer();
        if (latest_notification != NULL) {
            STRATUM_V1_free_mining_notify(latest_notification);
            latest_notification = NULL;
        }

        ///// Start Stratum Action
        int configure_message_id = -1;
        bool configure_pending = false;
        int64_t configure_sent_us = 0;
        bool use_legacy_v1 =
            !STRATUM_V1_bip310_should_probe(&bip310_state);
        if (!use_legacy_v1) {
            configure_message_id = stratum_get_next_uid(GLOBAL_STATE);
            int configure_ret = STRATUM_V1_configure_version_rolling(
                GLOBAL_STATE->transport, configure_message_id,
                STRATUM_DEFAULT_VERSION_MASK,
                STRATUM_VERSION_ROLLING_MIN_BIT_COUNT);
            if (configure_ret < 0) {
                // A partial/failed configure can leave the JSON stream
                // unusable. Reconnect once in legacy mode instead of sending
                // the same unsupported request forever.
                ESP_LOGW(TAG,
                         "Unable to send mining.configure; retrying in legacy V1 mode");
                STRATUM_V1_bip310_transient_failure(&bip310_state);
                retry_attempts++;
                stratum_v1_close_connection(GLOBAL_STATE);
                continue;
            }
            configure_pending = true;
            configure_sent_us = esp_timer_get_time();
        } else {
            ESP_LOGI(TAG,
                     "Using legacy Stratum V1 without BIP310 version rolling");
        }

        // mining.subscribe - ID: 2
        STRATUM_V1_subscribe(GLOBAL_STATE->transport, stratum_get_next_uid(GLOBAL_STATE), GLOBAL_STATE->DEVICE_CONFIG.family.asic.name);

        char *username = GLOBAL_STATE->SYSTEM_MODULE.pools[pool_idx].user;
        char *password = GLOBAL_STATE->SYSTEM_MODULE.pools[pool_idx].pass;

        int authorize_message_id = stratum_get_next_uid(GLOBAL_STATE);

        //mining.authorize - ID: 3
        STRATUM_V1_authorize(GLOBAL_STATE->transport, authorize_message_id, username, password);

        while (1) {
            // Check if coordinator wants us to shut down
            if (protocol_coordinator_v1_should_shutdown()) {
                ESP_LOGI(TAG, "Coordinator requested shutdown during recv loop, exiting");
                stratum_v1_close_connection(GLOBAL_STATE);
                if (latest_notification != NULL) {
                    STRATUM_V1_free_mining_notify(latest_notification);
                    latest_notification = NULL;
                }
                cleanup_stratum_buffer();
                protocol_coordinator_v1_exited();
                vTaskDelete(NULL);
                return;
            }

            char *line = STRATUM_V1_receive_jsonrpc_line(GLOBAL_STATE->transport);
            if (!line) {
                if (configure_pending) {
                    ESP_LOGW(TAG,
                             "Pool closed before mining.configure completed; falling back to legacy V1");
                    STRATUM_V1_bip310_transient_failure(&bip310_state);
                    configure_pending = false;
                }
                ESP_LOGE(TAG, "Failed to receive JSON-RPC line, reconnecting...");
                retry_attempts++;
                stratum_v1_close_connection(GLOBAL_STATE);
                break;
            }

            if (!asic_lifecycle_is_running(GLOBAL_STATE)) {
                free(line);
                ESP_LOGI(TAG, "Mining paused, disconnecting from pool");
                retry_attempts = 0;
                stratum_v1_close_connection(GLOBAL_STATE);
                break;
            }

            int64_t receive_time_us = esp_timer_get_time();

            bool reconnect_requested = false;
            if (!STRATUM_V1_parse(&stratum_api_v1_message, line)) {
                if (configure_pending &&
                    stratum_api_v1_message.message_id ==
                        configure_message_id) {
                    ESP_LOGW(TAG,
                             "Pool returned an invalid mining.configure response; using legacy V1");
                    configure_pending = false;
                } else if (configure_pending && configure_sent_us > 0 &&
                           receive_time_us - configure_sent_us >=
                               CONFIGURE_RESPONSE_TIMEOUT_US) {
                    ESP_LOGW(TAG,
                             "mining.configure response timed out; continuing this connection in legacy V1 mode");
                    configure_pending = false;
                }
                ESP_LOGE(TAG, "Failed to parse Stratum message, ignoring");
                STRATUM_V1_reset_message(&stratum_api_v1_message);
                free(line);
                continue;
            }
            free(line);

            switch (stratum_api_v1_message.method) {
                case METHOD_UNKNOWN:
                    // Should never happen
                    break;

                case MINING_NOTIFY:
                    GLOBAL_STATE->SYSTEM_MODULE.work_received++;
                    SYSTEM_notify_new_ntime(GLOBAL_STATE, stratum_api_v1_message.mining_notification->ntime);
                    mining_notify *latest_copy = clone_mining_notify(
                        stratum_api_v1_message.mining_notification);
                    if (latest_copy != NULL) {
                        if (latest_notification != NULL) {
                            STRATUM_V1_free_mining_notify(
                                latest_notification);
                        }
                        latest_notification = latest_copy;
                    } else {
                        ESP_LOGW(TAG,
                                 "Unable to retain latest V1 notification for immediate control updates");
                    }
                    if (stratum_api_v1_message.mining_notification->clean_jobs) {
                        // A clean notification invalidates work already running on
                        // the ASIC as well as work still waiting in the queue. Do
                        // this even when the queue is empty: create_jobs_task
                        // normally has the current item dequeued while it waits.
                        stratum_v1_clean_jobs(GLOBAL_STATE);
                    }
                    stratum_v1_enqueue_work(
                        GLOBAL_STATE,
                        stratum_api_v1_message.mining_notification);
                    stratum_api_v1_message.mining_notification = NULL;
                    break;

                case MINING_SET_DIFFICULTY:
                    ESP_LOGI(TAG, "Set pool difficulty: %.2f", stratum_api_v1_message.new_difficulty);
                    // Keep the per-job snapshot for spec-compliant pools. The
                    // submit path also applies increases immediately because
                    // some pools enforce the new threshold without a notify.
                    stratum_v1_set_difficulty(
                        GLOBAL_STATE,
                        stratum_api_v1_message.new_difficulty);
                    break;

                case MINING_SET_VERSION_MASK:
                    if (!GLOBAL_STATE->stratum_v1_version_rolling_enabled) {
                        ESP_LOGW(TAG,
                                 "Ignoring mining.set_version_mask before successful BIP310 negotiation");
                        break;
                    }
                    uint32_t updated_mask =
                        stratum_api_v1_message.version_mask &
                        STRATUM_DEFAULT_VERSION_MASK;
                    if (updated_mask !=
                        stratum_api_v1_message.version_mask) {
                        ESP_LOGW(TAG,
                                 "Pool version mask %08lx exceeds hardware mask; using %08lx",
                                 (unsigned long)stratum_api_v1_message.version_mask,
                                 (unsigned long)updated_mask);
                    }
                    ESP_LOGI(TAG, "Set version mask: %08lx",
                             (unsigned long)updated_mask);
                    if (!version_mask_meets_minimum(updated_mask)) {
                        ESP_LOGW(TAG,
                                 "Pool version mask does not satisfy the negotiated minimum bit count; disabling version rolling");
                        STRATUM_V1_bip310_mark_unsupported(&bip310_state);
                        stratum_v1_set_rolling_state(GLOBAL_STATE, false, 0);
                        stratum_v1_refresh_latest_work(
                            GLOBAL_STATE, latest_notification);
                        break;
                    }
                    pthread_mutex_lock(&v1_state_lock);
                    bool mask_unchanged =
                        GLOBAL_STATE->stratum_v1_version_rolling_enabled &&
                        GLOBAL_STATE->stratum_v1_version_mask == updated_mask;
                    pthread_mutex_unlock(&v1_state_lock);
                    if (mask_unchanged) {
                        // Some pools re-broadcast the active mask
                        // periodically. Nothing observable changes, so skip
                        // the full job invalidation and template refresh a
                        // real mask change requires.
                        ESP_LOGD(TAG,
                                 "Ignoring redundant identical version mask");
                        break;
                    }
                    stratum_v1_set_rolling_state(GLOBAL_STATE, true,
                                                 updated_mask);
                    stratum_v1_refresh_latest_work(GLOBAL_STATE,
                                                   latest_notification);
                    break;

                case STRATUM_RESULT_CONFIGURE:
                    if (!configure_pending ||
                        stratum_api_v1_message.message_id !=
                            configure_message_id) {
                        ESP_LOGW(TAG,
                                 "Ignoring unexpected mining.configure result id %d",
                                 stratum_api_v1_message.message_id);
                        break;
                    }
                    configure_pending = false;
                    if (stratum_api_v1_message.response_success) {
                        uint32_t negotiated_mask =
                            stratum_api_v1_message.version_mask &
                            STRATUM_DEFAULT_VERSION_MASK;
                        if (negotiated_mask !=
                            stratum_api_v1_message.version_mask) {
                            ESP_LOGW(TAG,
                                     "Pool returned unsupported version bits %08lx; using intersection %08lx",
                                     (unsigned long)stratum_api_v1_message.version_mask,
                                     (unsigned long)negotiated_mask);
                        }
                        if (!version_mask_meets_minimum(negotiated_mask)) {
                            ESP_LOGW(TAG,
                                     "Pool accepted BIP310 with fewer than %u usable version bits; continuing in legacy V1 mode",
                                     (unsigned int)STRATUM_VERSION_ROLLING_MIN_BIT_COUNT);
                            STRATUM_V1_bip310_mark_unsupported(&bip310_state);
                            stratum_v1_set_rolling_state(
                                GLOBAL_STATE, false, 0);
                            stratum_v1_refresh_latest_work(
                                GLOBAL_STATE, latest_notification);
                            break;
                        }
                        ESP_LOGI(TAG,
                                 "Configure result accepted, version mask: %08lx",
                                 (unsigned long)negotiated_mask);
                        STRATUM_V1_bip310_mark_supported(&bip310_state);
                        stratum_v1_set_rolling_state(
                            GLOBAL_STATE, true, negotiated_mask);
                        stratum_v1_refresh_latest_work(
                            GLOBAL_STATE, latest_notification);
                        protocol_coordinator_notify_success();
                    } else {
                        STRATUM_V1_bip310_mark_unsupported(&bip310_state);
                        ESP_LOGW(TAG,
                                 "Configure result rejected; continuing in legacy V1 mode: %s",
                                 stratum_api_v1_message.error_str != NULL
                                     ? stratum_api_v1_message.error_str
                                     : "unknown");
                    }
                    break;

                case MINING_SET_EXTRANONCE:
                case STRATUM_RESULT_SUBSCRIBE:
                    ESP_LOGI(TAG, "Set extranonce: %s, extranonce_2_len: %d", stratum_api_v1_message.extranonce_str, stratum_api_v1_message.extranonce_2_len);
                    stratum_v1_replace_extranonce(
                        GLOBAL_STATE,
                        stratum_api_v1_message.extranonce_str,
                        stratum_api_v1_message.extranonce_2_len);
                    stratum_api_v1_message.extranonce_str = NULL;
                    stratum_v1_refresh_latest_work(GLOBAL_STATE,
                                                   latest_notification);
                    break;

                case MINING_PING:
                    STRATUM_V1_pong(GLOBAL_STATE->transport, stratum_api_v1_message.message_id);
                    break;

                case CLIENT_RECONNECT:
                    ESP_LOGE(TAG, "Pool requested client reconnect...");
                    stratum_v1_close_connection(GLOBAL_STATE);
                    reconnect_requested = true;
                    break;

                case CLIENT_SHOW_MESSAGE:
                    break;

                case CLIENT_GET_VERSION:
                    STRATUM_V1_send_version(GLOBAL_STATE->transport, stratum_api_v1_message.message_id);
                    break;

                case STRATUM_RESULT:
                    {
                        if (configure_pending &&
                            stratum_api_v1_message.message_id ==
                                configure_message_id) {
                            configure_pending = false;
                            STRATUM_V1_bip310_mark_unsupported(&bip310_state);
                            ESP_LOGW(TAG,
                                     "mining.configure rejected; continuing in legacy V1 mode: %s",
                                     stratum_api_v1_message.error_str != NULL
                                         ? stratum_api_v1_message.error_str
                                         : "unsupported");
                            break;
                        }
                        float response_time_ms = STRATUM_V1_get_response_time_ms(stratum_api_v1_message.message_id, receive_time_us);
                        if (response_time_ms >= 0) {
                            if (stratum_api_v1_message.response_success) {
                                ESP_LOGI(TAG, "message result accepted");
                                ESP_LOGI(TAG, "Stratum response time: %.1f ms", response_time_ms);
                                GLOBAL_STATE->SYSTEM_MODULE.response_time = response_time_ms;
                                SYSTEM_notify_accepted_share(GLOBAL_STATE);
                            } else {
                                ESP_LOGW(TAG, "message result rejected: %s", stratum_api_v1_message.error_str);
                                SYSTEM_notify_rejected_share(GLOBAL_STATE, stratum_api_v1_message.error_str);
                            }
                        } else {
                            // Reset retry attempts after successfully receiving data.
                            retry_attempts = 0;
                            // Tell the coordinator setup succeeded so it clears its
                            // failure counter and pools_unavailable.
                            protocol_coordinator_notify_success();
                            if (stratum_api_v1_message.response_success) {
                                ESP_LOGI(TAG, "setup message accepted");
                                if (stratum_api_v1_message.message_id == authorize_message_id) {
                                    uint16_t difficulty = GLOBAL_STATE->SYSTEM_MODULE.pools[pool_idx].difficulty;
                                    if (difficulty > 0) {
                                        STRATUM_V1_suggest_difficulty(GLOBAL_STATE->transport, stratum_get_next_uid(GLOBAL_STATE), difficulty);
                                    }
                                    bool extranonce_subscribe = GLOBAL_STATE->SYSTEM_MODULE.pools[pool_idx].extranonce_subscribe;
                                    if (extranonce_subscribe) {
                                        STRATUM_V1_extranonce_subscribe(GLOBAL_STATE->transport, stratum_get_next_uid(GLOBAL_STATE));
                                    }
                                }
                            } else {
                                ESP_LOGE(TAG, "setup message rejected: %s", stratum_api_v1_message.error_str);
                            }
                        }
                    }
                    break;
            }
            if (configure_pending && configure_sent_us > 0 &&
                receive_time_us - configure_sent_us >=
                    CONFIGURE_RESPONSE_TIMEOUT_US) {
                ESP_LOGW(TAG,
                         "mining.configure response timed out; continuing this connection in legacy V1 mode");
                configure_pending = false;
            }
            STRATUM_V1_reset_message(&stratum_api_v1_message);
            if (reconnect_requested) {
                break;
            }
        }
    }
    vTaskDelete(NULL);
}
