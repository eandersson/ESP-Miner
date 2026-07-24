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
#include <esp_sntp.h>
#include "esp_timer.h"
#include "esp_transport.h"
#include <stdbool.h>
#include <string.h>
#include <pthread.h>
#include "utils.h"
#include "coinbase_decoder.h"
#include <esp_heap_caps.h>
#include "esp_transport_ssl.h"
#include "freertos/task.h"

#define MAX_RETRY_ATTEMPTS 3
#define MAX_CRITICAL_RETRY_ATTEMPTS 5
#define MAX_EXTRANONCE_2_LEN 32

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

#define BUFFER_SIZE 1024

static const char *TAG = "stratum_v1_task";
static pthread_mutex_t v1_lifecycle_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t v1_extranonce_lock = PTHREAD_MUTEX_INITIALIZER;

static void free_v1_queued_work(void *work)
{
    STRATUM_V1_free_mining_notify((mining_notify *)work);
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

    pthread_mutex_lock(&v1_lifecycle_lock);
    SYSTEM_clean_jobs_queue(GLOBAL_STATE);
    esp_transport_handle_t transport = GLOBAL_STATE->transport;
    GLOBAL_STATE->transport = NULL;
    pthread_mutex_unlock(&v1_lifecycle_lock);

    if (transport != NULL) {
        esp_transport_close(transport);
        esp_transport_destroy(transport);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void stratum_v1_interrupt_connection(GlobalState *GLOBAL_STATE)
{
    pthread_mutex_lock(&v1_lifecycle_lock);
    if (GLOBAL_STATE->transport != NULL) {
        esp_transport_close(GLOBAL_STATE->transport);
    }
    pthread_mutex_unlock(&v1_lifecycle_lock);
}

static void stratum_v1_clean_jobs(GlobalState *GLOBAL_STATE)
{
    pthread_mutex_lock(&v1_lifecycle_lock);
    SYSTEM_clean_jobs_queue(GLOBAL_STATE);
    pthread_mutex_unlock(&v1_lifecycle_lock);
}

bool stratum_v1_snapshot_extranonce(GlobalState *GLOBAL_STATE,
                                    char **extranonce,
                                    uint32_t *extranonce_2_len)
{
    if (extranonce == NULL || extranonce_2_len == NULL) {
        return false;
    }

    pthread_mutex_lock(&v1_extranonce_lock);
    char *copy = GLOBAL_STATE->extranonce_str != NULL
                     ? strdup(GLOBAL_STATE->extranonce_str)
                     : NULL;
    uint32_t length = GLOBAL_STATE->extranonce_2_len;
    pthread_mutex_unlock(&v1_extranonce_lock);

    if (copy == NULL) {
        return false;
    }
    *extranonce = copy;
    *extranonce_2_len = length;
    return true;
}

static void stratum_v1_replace_extranonce(GlobalState *GLOBAL_STATE,
                                          char *extranonce,
                                          uint32_t extranonce_2_len)
{
    pthread_mutex_lock(&v1_extranonce_lock);
    char *old_extranonce = GLOBAL_STATE->extranonce_str;
    GLOBAL_STATE->extranonce_str = extranonce;
    GLOBAL_STATE->extranonce_2_len = extranonce_2_len;
    pthread_mutex_unlock(&v1_extranonce_lock);
    free(old_extranonce);
}

int stratum_v1_submit_share_safe(
    GlobalState *GLOBAL_STATE, uint32_t expected_generation, int uid,
    const char *user, const char *job_id, const char *extranonce_2,
    uint32_t ntime, uint32_t nonce, uint32_t version_bits,
    uint64_t *sent_time_us)
{
    pthread_mutex_lock(&v1_lifecycle_lock);
    if (expected_generation != ASIC_result_task_get_job_generation() ||
        GLOBAL_STATE->transport == NULL) {
        pthread_mutex_unlock(&v1_lifecycle_lock);
        return -1;
    }

    int result = STRATUM_V1_submit_share(
        GLOBAL_STATE->transport, uid, user, job_id, extranonce_2, ntime,
        nonce, version_bits, sent_time_us);
    pthread_mutex_unlock(&v1_lifecycle_lock);
    return result;
}

static void decode_mining_notification(GlobalState * GLOBAL_STATE, const mining_notify *mining_notification)
{
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
                                     GLOBAL_STATE->extranonce_str,
                                     GLOBAL_STATE->extranonce_2_len,
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

void stratum_v1_task(void *pvParameters)
{
    GlobalState *GLOBAL_STATE = (GlobalState *)pvParameters;

    uint16_t pool_idx = GLOBAL_STATE->SYSTEM_MODULE.is_using_fallback ? GLOBAL_STATE->SYSTEM_MODULE.secondary_pool_index : GLOBAL_STATE->SYSTEM_MODULE.primary_pool_index;
    char *stratum_url = GLOBAL_STATE->SYSTEM_MODULE.pools[pool_idx].url;
    uint16_t port = GLOBAL_STATE->SYSTEM_MODULE.pools[pool_idx].port;

    STRATUM_V1_initialize_buffer();
    int retry_attempts = 0;
    int retry_critical_attempts = 0;

    ESP_LOGI(TAG, "Opening connection to pool: %s:%d", stratum_url, port);
    while (1) {
        // Check if coordinator wants us to shut down
        if (protocol_coordinator_v1_should_shutdown()) {
            ESP_LOGI(TAG, "Coordinator requested shutdown, exiting");
            stratum_v1_close_connection(GLOBAL_STATE);
            protocol_coordinator_v1_exited();
            vTaskDelete(NULL);
            return;
        }

        if (!GLOBAL_STATE->ASIC_initalized) {
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
        stratum_v1_clean_jobs(GLOBAL_STATE);

        ///// Start Stratum Action
        // mining.configure - ID: 1
        STRATUM_V1_configure_version_rolling(GLOBAL_STATE->transport, stratum_get_next_uid(GLOBAL_STATE), &GLOBAL_STATE->version_mask);

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
                protocol_coordinator_v1_exited();
                vTaskDelete(NULL);
            }

            char *line = STRATUM_V1_receive_jsonrpc_line(GLOBAL_STATE->transport);
            if (!line) {
                ESP_LOGE(TAG, "Failed to receive JSON-RPC line, reconnecting...");
                retry_attempts++;
                stratum_v1_close_connection(GLOBAL_STATE);
                break;
            }

            if (!GLOBAL_STATE->ASIC_initalized) {
                free(line);
                ESP_LOGI(TAG, "Mining paused, disconnecting from pool");
                retry_attempts = 0;
                stratum_v1_close_connection(GLOBAL_STATE);
                break;
            }

            int64_t receive_time_us = esp_timer_get_time();

            bool reconnect_requested = false;
            if (!STRATUM_V1_parse(&stratum_api_v1_message, line)) {
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
                    if (stratum_api_v1_message.mining_notification->clean_jobs) {
                        // A clean notification invalidates work already running on
                        // the ASIC as well as work still waiting in the queue. Do
                        // this even when the queue is empty: create_jobs_task
                        // normally has the current item dequeued while it waits.
                        stratum_v1_clean_jobs(GLOBAL_STATE);
                    }
                    // queue_enqueue performs the capacity check while holding the
                    // queue lock. Avoid racing create_jobs_task via queue.count.
                    queue_enqueue(
                        &GLOBAL_STATE->stratum_queue,
                        stratum_api_v1_message.mining_notification,
                        (work_queue_item_metadata) {
                            .generation =
                                ASIC_result_task_get_pool_generation(),
                            .kind = WORK_QUEUE_ITEM_STRATUM_V1,
                            .free_fn = free_v1_queued_work,
                        });
                    decode_mining_notification(GLOBAL_STATE, stratum_api_v1_message.mining_notification);
                    stratum_api_v1_message.mining_notification = NULL;
                    break;

                case MINING_SET_DIFFICULTY:
                    ESP_LOGI(TAG, "Set pool difficulty: %.2f", stratum_api_v1_message.new_difficulty);
                    GLOBAL_STATE->pool_difficulty = stratum_api_v1_message.new_difficulty;
                    GLOBAL_STATE->new_set_mining_difficulty_msg = true;
                    break;

                case MINING_SET_VERSION_MASK:
                    ESP_LOGI(TAG, "Set version mask: %08lx", stratum_api_v1_message.version_mask);
                    GLOBAL_STATE->version_mask = stratum_api_v1_message.version_mask;
                    GLOBAL_STATE->new_stratum_version_rolling_msg = true;
                    break;

                case STRATUM_RESULT_CONFIGURE:
                    if (stratum_api_v1_message.response_success) {
                        ESP_LOGI(TAG, "Configure result accepted, version mask: %08lx", stratum_api_v1_message.version_mask);
                        GLOBAL_STATE->version_mask = stratum_api_v1_message.version_mask;
                        GLOBAL_STATE->new_stratum_version_rolling_msg = true;
                        protocol_coordinator_notify_success();
                    } else {
                        ESP_LOGE(TAG, "Configure result rejected: %s", stratum_api_v1_message.error_str);
                    }
                    break;

                case MINING_SET_EXTRANONCE:
                case STRATUM_RESULT_SUBSCRIBE:
                    if (stratum_api_v1_message.method == MINING_SET_EXTRANONCE) {
                        // A new extranonce1 changes every coinbase. Invalidate
                        // both queued and device-resident jobs before publishing
                        // it so retained work cannot mix the old and new values.
                        stratum_v1_clean_jobs(GLOBAL_STATE);
                    }
                    // Validate extranonce_2_len to prevent buffer overflow
                    if (stratum_api_v1_message.extranonce_2_len > MAX_EXTRANONCE_2_LEN) {
                        ESP_LOGW(TAG, "Extranonce_2_len %d exceeds maximum %d, clamping to maximum",
                                 stratum_api_v1_message.extranonce_2_len, MAX_EXTRANONCE_2_LEN);
                        stratum_api_v1_message.extranonce_2_len = MAX_EXTRANONCE_2_LEN;
                    }
                    ESP_LOGI(TAG, "Set extranonce: %s, extranonce_2_len: %d", stratum_api_v1_message.extranonce_str, stratum_api_v1_message.extranonce_2_len);
                    stratum_v1_replace_extranonce(
                        GLOBAL_STATE,
                        stratum_api_v1_message.extranonce_str,
                        stratum_api_v1_message.extranonce_2_len);
                    stratum_api_v1_message.extranonce_str = NULL;
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
                    break;                case STRATUM_RESULT:
                    {
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
            STRATUM_V1_reset_message(&stratum_api_v1_message);
            if (reconnect_requested) {
                break;
            }
        }
    }
    vTaskDelete(NULL);
}
