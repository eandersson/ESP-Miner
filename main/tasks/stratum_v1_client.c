#include "esp_log.h"
#include "system.h"
#include "global_state.h"
#include <lwip/tcpip.h>
#include <lwip/sockets.h>
#include "stratum_v1_client.h"
#include "stratum_task.h"
#include "stratum_api.h"
#include "stratum_socket.h"
#include "asic_result_task.h"
#include "connect.h"
#include <esp_sntp.h>
#include "esp_timer.h"
#include "esp_transport.h"
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include "utils.h"
#include "miner_job.h"
#include <esp_heap_caps.h>
#include "esp_transport_ssl.h"
#include "freertos/task.h"

#define TRANSPORT_TIMEOUT_MS 5000
#define PROBE_RECV_BUFFER_SIZE 2048
#define CONFIGURE_RESPONSE_TIMEOUT_US 10000000LL
#define SETUP_RESPONSE_TIMEOUT_US 10000000LL

static const char *TAG = "stratum_v1";

static StratumApiV1Message *s_v1_msg = NULL;

// Connection state shared with the share submit worker. Guarded by
// GLOBAL_STATE->transport_mutex. The socket write itself runs unlocked; close
// waits for s_active_writers to drain before destroying the transport.
static sv1_conn_t *s_v1_conn = NULL;
static uint32_t s_session_id = 0;
static bool s_closing = true;
static unsigned int s_active_writers = 0;
static pthread_cond_t s_writers_drained = PTHREAD_COND_INITIALIZER;

// Owned by the stratum task.
static miner_job_t s_parse_job;   // scratch destination for mining.notify
static miner_job_t s_latest_job;  // newest accepted template
static bool s_latest_job_valid = false;
static bool s_version_rolling_enabled = false;

// BIP310 outcome for the pool last connected to. An explicit rejection is
// sticky for that pool; a transport failure during negotiation only skips the
// next probe, so one dropped socket cannot permanently disable version rolling.
static struct {
    stratum_v1_bip310_state_t state;
    char url[256];
    uint16_t port;
    bool valid;
} s_bip310;

typedef struct
{
    int subscribe_message_id;
    int authorize_message_id;
    int64_t deadline_us;
    bool extranonce_ready;
    bool authorized;
    bool unidentified_success_received;
    bool post_auth_actions_sent;
    bool complete;
} stratum_v1_setup_state_t;

typedef struct
{
    GlobalState *gs;
    esp_transport_handle_t transport;
    const PoolConfig *pool;
    uint16_t pool_idx;
    uint32_t session_id;
    stratum_v1_setup_state_t setup;
    int configure_message_id;
    bool configure_pending;
    int64_t configure_sent_us;
} v1_session_t;

typedef enum
{
    V1_LINE_CONTINUE,
    V1_LINE_FAIL,
    V1_LINE_POOL_RECONNECT,
} v1_line_result_t;

static bool add_active_job_id(char active_job_ids[][MAX_JOB_ID_LEN], int *count, const char *job_id)
{
    for (int i = 0; i < *count; i++) {
        if (strncmp(active_job_ids[i], job_id, MAX_JOB_ID_LEN) == 0) {
            return false;
        }
    }
    if (*count < SV1_MAX_ACTIVE_JOB_IDS) {
        strlcpy(active_job_ids[*count], job_id, MAX_JOB_ID_LEN);
        (*count)++;
    } else {
        for (int i = 1; i < SV1_MAX_ACTIVE_JOB_IDS; i++) {
            memcpy(active_job_ids[i - 1], active_job_ids[i], MAX_JOB_ID_LEN);
        }
        strlcpy(active_job_ids[SV1_MAX_ACTIVE_JOB_IDS - 1], job_id, MAX_JOB_ID_LEN);
    }
    return true;
}

static void clear_active_job_ids(char active_job_ids[][MAX_JOB_ID_LEN], int *count)
{
    *count = 0;
}

static int stratum_get_next_uid(GlobalState * GLOBAL_STATE)
{
    pthread_mutex_lock(&GLOBAL_STATE->transport_mutex);
    int uid = s_v1_conn ? s_v1_conn->send_uid++ : 1;
    pthread_mutex_unlock(&GLOBAL_STATE->transport_mutex);
    return uid;
}

static bool version_mask_meets_minimum(uint32_t mask)
{
    unsigned int bit_count = 0;
    while (mask != 0) {
        bit_count += mask & 1U;
        mask >>= 1;
    }
    return bit_count >= STRATUM_VERSION_ROLLING_MIN_BIT_COUNT;
}

static bool v1_setup_id_matches(const StratumApiV1Message *message, int pending_message_id)
{
    return message != NULL && message->has_message_id &&
           pending_message_id >= 0 &&
           message->message_id == pending_message_id;
}

static bool v1_setup_timed_out(const stratum_v1_setup_state_t *setup, int64_t now_us)
{
    return !setup->complete && setup->deadline_us > 0 && now_us >= setup->deadline_us;
}

static void v1_maybe_complete_setup(stratum_v1_setup_state_t *setup)
{
    if (setup->complete || !setup->extranonce_ready || !setup->authorized) {
        return;
    }
    setup->complete = true;
    ESP_LOGI(TAG, "Stratum V1 subscription and pool setup succeeded");
}

static void v1_check_configure_timeout(v1_session_t *session, int64_t now_us)
{
    if (session->configure_pending && session->configure_sent_us > 0 &&
        now_us - session->configure_sent_us >= CONFIGURE_RESPONSE_TIMEOUT_US) {
        ESP_LOGW(TAG, "mining.configure response timed out; continuing this connection in legacy V1 mode");
        session->configure_pending = false;
    }
}

// A failure while BIP310 is still being negotiated only skips the next probe.
static void v1_negotiation_interrupted(v1_session_t *session)
{
    if (session->configure_pending) {
        STRATUM_V1_bip310_transient_failure(&s_bip310.state);
    }
}

static bool v1_ensure_buffers(void)
{
    if (s_v1_msg == NULL) {
        s_v1_msg = heap_caps_calloc(1, sizeof(StratumApiV1Message), MALLOC_CAP_SPIRAM);
        if (s_v1_msg == NULL) {
            s_v1_msg = calloc(1, sizeof(StratumApiV1Message));
        }
        if (s_v1_msg == NULL) {
            return false;
        }
    }
    return miner_job_alloc_buffers(&s_parse_job) && miner_job_alloc_buffers(&s_latest_job);
}

// Stamp the newest template with this connection's state and hand it to the
// job scheduler. Returns false while the pool has not provided an extranonce.
static bool v1_publish_latest(v1_session_t *session)
{
    GlobalState *GLOBAL_STATE = session->gs;
    sv1_conn_t *conn = s_v1_conn;
    if (!s_latest_job_valid || !session->setup.extranonce_ready || conn == NULL) {
        return false;
    }

    miner_job_t *job = &s_latest_job;
    job->pool_id = (uint8_t)session->pool_idx;
    job->session_id = session->session_id;
    job->pool_diff = conn->pool_difficulty;
    job->version_mask = s_version_rolling_enabled ? conn->version_mask : 0;
    job->extranonce1_len = conn->extranonce1_len;
    if (conn->extranonce1_len > 0) {
        memcpy(job->extranonce1, conn->extranonce1, conn->extranonce1_len);
    }
    job->extranonce2_len = conn->extranonce2_len;
    job->pool_generation = ASIC_result_task_get_pool_generation();

    miner_job_lock();
    uint8_t slot = (uint8_t)((GLOBAL_STATE->active_job_slot_idx + 1) % 2);
    miner_job_copy(miner_job_get_slot(slot), job);
    miner_job_unlock();

    if (GLOBAL_STATE->create_jobs_task_handle) {
        xTaskNotify(GLOBAL_STATE->create_jobs_task_handle, slot, eSetValueWithOverwrite);
    }
    return true;
}

// A BIP310 change applies immediately. Invalidate every job built with the old
// submit shape before work with the new shape is published.
static void v1_set_rolling_state(v1_session_t *session, bool enabled, uint32_t mask)
{
    SYSTEM_clean_jobs_queue(session->gs);
    s_version_rolling_enabled = enabled;
    s_v1_conn->version_mask = enabled ? mask : 0;
}

static bool v1_apply_extranonce(v1_session_t *session, const StratumApiV1Message *message)
{
    sv1_conn_t *conn = s_v1_conn;
    if (message->extranonce_str == NULL || message->extranonce_2_len < 0 ||
        message->extranonce_2_len > MAX_EXTRANONCE_2_LEN) {
        return false;
    }
    size_t hex_len = strlen(message->extranonce_str);
    size_t extranonce1_len = hex_len / 2;
    if ((hex_len % 2) != 0 || extranonce1_len > sizeof(conn->extranonce1)) {
        return false;
    }
    uint8_t extranonce1[sizeof(conn->extranonce1)];
    if (extranonce1_len > 0 &&
        hex2bin(message->extranonce_str, extranonce1, extranonce1_len) != extranonce1_len) {
        return false;
    }

    // Shares built on the previous extranonce can no longer be accepted.
    SYSTEM_clean_jobs_queue(session->gs);
    memcpy(conn->extranonce1, extranonce1, extranonce1_len);
    conn->extranonce1_len = (uint8_t)extranonce1_len;
    conn->extranonce2_len = (uint8_t)message->extranonce_2_len;
    ESP_LOGI(TAG, "Set extranonce: %s, extranonce_2_len: %d",
             message->extranonce_str, message->extranonce_2_len);
    return true;
}

// Record authorization and send the post-authorization requests once. Returns
// false when one of those writes fails.
static bool v1_mark_authorized(v1_session_t *session, const char *evidence)
{
    if (!session->setup.authorized && evidence != NULL) {
        ESP_LOGI(TAG, "%s", evidence);
    }
    session->setup.authorized = true;

    if (session->setup.post_auth_actions_sent) {
        return true;
    }
    session->setup.post_auth_actions_sent = true;

    GlobalState *GLOBAL_STATE = session->gs;
    if (session->pool->difficulty > 0 &&
        STRATUM_V1_suggest_difficulty(session->transport, stratum_get_next_uid(GLOBAL_STATE),
                                      session->pool->difficulty) < 0) {
        return false;
    }
    if (session->pool->extranonce_subscribe &&
        STRATUM_V1_extranonce_subscribe(session->transport, stratum_get_next_uid(GLOBAL_STATE)) < 0) {
        return false;
    }
    return true;
}

static v1_line_result_t v1_handle_notify(v1_session_t *session)
{
    GlobalState *GLOBAL_STATE = session->gs;

    // Some V1 pools never answer mining.authorize. A valid job proves that the
    // pool accepted this session; setup completion still needs an extranonce.
    if (!v1_mark_authorized(session, "Pool is streaming work; treating authorization as implicit")) {
        return V1_LINE_FAIL;
    }

    miner_job_t *job = &s_parse_job;
    if (job->job_id[0] != '\0') {
        if (job->clean_jobs) {
            clear_active_job_ids(s_v1_conn->active_job_ids, &s_v1_conn->active_job_ids_count);
        }
        if (!add_active_job_id(s_v1_conn->active_job_ids, &s_v1_conn->active_job_ids_count, job->job_id)) {
            ESP_LOGW(TAG, "Ignoring duplicate notify for job %s", job->job_id);
            return V1_LINE_CONTINUE;
        }
    }

    GLOBAL_STATE->SYSTEM_MODULE.work_received++;
    SYSTEM_notify_new_ntime(GLOBAL_STATE, job->ntime);

    // Keep the accepted template; the parse buffer takes over the old one.
    miner_job_t previous = s_latest_job;
    s_latest_job = s_parse_job;
    s_parse_job = previous;
    s_latest_job_valid = true;

    if (s_latest_job.clean_jobs) {
        // A clean notification invalidates the work running on the ASIC and
        // every queued share, not only the template it replaces.
        SYSTEM_clean_jobs_queue(GLOBAL_STATE);
    }
    if (!v1_publish_latest(session)) {
        ESP_LOGW(TAG, "Holding job %s until the pool provides an extranonce", s_latest_job.job_id);
    }
    return V1_LINE_CONTINUE;
}

static v1_line_result_t v1_handle_extranonce(v1_session_t *session)
{
    StratumApiV1Message *message = s_v1_msg;

    if (message->method == STRATUM_RESULT_SUBSCRIBE) {
        if (session->setup.subscribe_message_id < 0 && session->setup.extranonce_ready) {
            ESP_LOGW(TAG, "Ignoring duplicate mining.subscribe result");
            return V1_LINE_CONTINUE;
        }
        if (session->setup.subscribe_message_id >= 0 && message->has_message_id &&
            !v1_setup_id_matches(message, session->setup.subscribe_message_id)) {
            ESP_LOGW(TAG, "Ignoring unexpected mining.subscribe result id %d", message->message_id);
            return V1_LINE_CONTINUE;
        }
    }

    if (!v1_apply_extranonce(session, message)) {
        ESP_LOGE(TAG, "Pool did not establish a usable extranonce; reconnecting");
        v1_negotiation_interrupted(session);
        return V1_LINE_FAIL;
    }

    // A valid subscribe array or mining.set_extranonce proves the subscription
    // completed. Retire the request ID so a later unrelated message reusing
    // this small value cannot be mistaken for setup traffic.
    session->setup.extranonce_ready = true;
    session->setup.subscribe_message_id = -1;
    if (session->setup.unidentified_success_received && !session->setup.authorized &&
        !v1_mark_authorized(session, "Treating an id-less successful response as implicit authorization")) {
        return V1_LINE_FAIL;
    }
    v1_publish_latest(session);
    return V1_LINE_CONTINUE;
}

static v1_line_result_t v1_handle_version_mask(v1_session_t *session)
{
    StratumApiV1Message *message = s_v1_msg;

    if (!s_version_rolling_enabled) {
        ESP_LOGW(TAG, "Ignoring mining.set_version_mask before successful BIP310 negotiation");
        return V1_LINE_CONTINUE;
    }

    uint32_t updated_mask = message->version_mask & BIP320_VERSION_ROLLING_MASK;
    if (updated_mask != message->version_mask) {
        ESP_LOGW(TAG, "Pool version mask %08lx exceeds hardware mask; using %08lx",
                 (unsigned long)message->version_mask, (unsigned long)updated_mask);
    }
    if (!version_mask_meets_minimum(updated_mask)) {
        ESP_LOGW(TAG, "Pool version mask does not satisfy the negotiated minimum bit count; disabling version rolling");
        STRATUM_V1_bip310_mark_unsupported(&s_bip310.state);
        v1_set_rolling_state(session, false, 0);
        v1_publish_latest(session);
        return V1_LINE_CONTINUE;
    }
    if (updated_mask == s_v1_conn->version_mask) {
        // Some pools re-broadcast the active mask. Nothing observable changes,
        // so skip the invalidation and refresh a real change requires.
        ESP_LOGD(TAG, "Ignoring redundant identical version mask");
        return V1_LINE_CONTINUE;
    }

    ESP_LOGI(TAG, "Set version mask: %08lx", (unsigned long)updated_mask);
    v1_set_rolling_state(session, true, updated_mask);
    v1_publish_latest(session);
    return V1_LINE_CONTINUE;
}

static v1_line_result_t v1_handle_configure(v1_session_t *session)
{
    StratumApiV1Message *message = s_v1_msg;

    if (!session->configure_pending || message->message_id != session->configure_message_id) {
        ESP_LOGW(TAG, "Ignoring unexpected mining.configure result id %d", message->message_id);
        return V1_LINE_CONTINUE;
    }
    session->configure_pending = false;

    if (!message->response_success) {
        STRATUM_V1_bip310_mark_unsupported(&s_bip310.state);
        ESP_LOGW(TAG, "Configure result rejected; continuing in legacy V1 mode: %s",
                 message->error_str != NULL ? message->error_str : "unknown");
        return V1_LINE_CONTINUE;
    }

    uint32_t negotiated_mask = message->version_mask & BIP320_VERSION_ROLLING_MASK;
    if (negotiated_mask != message->version_mask) {
        ESP_LOGW(TAG, "Pool returned unsupported version bits %08lx; using intersection %08lx",
                 (unsigned long)message->version_mask, (unsigned long)negotiated_mask);
    }
    if (!version_mask_meets_minimum(negotiated_mask)) {
        ESP_LOGW(TAG, "Pool accepted BIP310 with fewer than %u usable version bits; continuing in legacy V1 mode",
                 (unsigned int)STRATUM_VERSION_ROLLING_MIN_BIT_COUNT);
        STRATUM_V1_bip310_mark_unsupported(&s_bip310.state);
        v1_set_rolling_state(session, false, 0);
        v1_publish_latest(session);
        return V1_LINE_CONTINUE;
    }

    ESP_LOGI(TAG, "Configure result accepted, version mask: %08lx", (unsigned long)negotiated_mask);
    STRATUM_V1_bip310_mark_supported(&s_bip310.state);
    v1_set_rolling_state(session, true, negotiated_mask);
    v1_publish_latest(session);
    return V1_LINE_CONTINUE;
}

static v1_line_result_t v1_handle_result(v1_session_t *session, int64_t receive_time_us)
{
    GlobalState *GLOBAL_STATE = session->gs;
    StratumApiV1Message *message = s_v1_msg;

    if (session->configure_pending && message->has_message_id &&
        message->message_id == session->configure_message_id) {
        session->configure_pending = false;
        STRATUM_V1_bip310_mark_unsupported(&s_bip310.state);
        ESP_LOGW(TAG, "mining.configure rejected; continuing in legacy V1 mode: %s",
                 message->error_str != NULL ? message->error_str : "unsupported");
        return V1_LINE_CONTINUE;
    }

    if (v1_setup_id_matches(message, session->setup.subscribe_message_id)) {
        session->setup.subscribe_message_id = -1;
        if (!message->response_success) {
            ESP_LOGE(TAG, "mining.subscribe was rejected: %s",
                     message->error_str != NULL ? message->error_str : "unknown");
            v1_negotiation_interrupted(session);
            return V1_LINE_FAIL;
        }
        // A few pools acknowledge subscribe with true and deliver the fields
        // via mining.set_extranonce. The setup deadline still requires them.
        ESP_LOGI(TAG, "mining.subscribe accepted%s",
                 session->setup.extranonce_ready ? "" : "; awaiting extranonce");
        if (session->setup.extranonce_ready && session->setup.unidentified_success_received &&
            !session->setup.authorized &&
            !v1_mark_authorized(session, "Treating an id-less successful response as implicit authorization")) {
            return V1_LINE_FAIL;
        }
        return V1_LINE_CONTINUE;
    }

    if (v1_setup_id_matches(message, session->setup.authorize_message_id)) {
        session->setup.authorize_message_id = -1;
        if (!message->response_success) {
            ESP_LOGE(TAG, "mining.authorize rejected: %s",
                     message->error_str != NULL ? message->error_str : "unknown");
            snprintf(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info,
                     sizeof(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info), "SV1: Auth rejected");
            v1_negotiation_interrupted(session);
            return V1_LINE_FAIL;
        }
        return v1_mark_authorized(session, "mining.authorize accepted") ? V1_LINE_CONTINUE : V1_LINE_FAIL;
    }

    if (!message->has_message_id && message->response_success) {
        // With no usable ID the reply cannot be matched to a request. Keep only
        // positive evidence; an id-less rejection stays ambiguous and the setup
        // deadline arbitrates it.
        session->setup.unidentified_success_received = true;
        if (session->setup.extranonce_ready && !session->setup.authorized &&
            !v1_mark_authorized(session, "Treating an id-less successful response as implicit authorization")) {
            return V1_LINE_FAIL;
        }
    }

    float response_time_ms = STRATUM_V1_get_response_time_ms(message->message_id, receive_time_us);
    if (response_time_ms >= 0) {
        if (GLOBAL_STATE->SYSTEM_MODULE.shares_pending > 0) {
            GLOBAL_STATE->SYSTEM_MODULE.shares_pending--;
        }
        if (message->response_success) {
            ESP_LOGI(TAG, "message result accepted");
            ESP_LOGI(TAG, "Stratum response time: %.1f ms", response_time_ms);
            GLOBAL_STATE->SYSTEM_MODULE.response_time = response_time_ms;
            SYSTEM_notify_accepted_share(GLOBAL_STATE);
        } else {
            char unknown_reason[] = "unknown";
            ESP_LOGW(TAG, "message result rejected: %s",
                     message->error_str != NULL ? message->error_str : unknown_reason);
            SYSTEM_notify_rejected_share(GLOBAL_STATE,
                                         message->error_str != NULL ? message->error_str : unknown_reason);
        }
    } else if (message->response_success) {
        ESP_LOGI(TAG, "Untracked request accepted");
    } else {
        ESP_LOGW(TAG, "Untracked request rejected: %s",
                 message->error_str != NULL ? message->error_str : "unknown");
    }
    return V1_LINE_CONTINUE;
}

static v1_line_result_t v1_handle_line(v1_session_t *session, const char *line, int64_t receive_time_us)
{
    GlobalState *GLOBAL_STATE = session->gs;
    StratumApiV1Message *message = s_v1_msg;

    if (!STRATUM_V1_parse(message, line, &s_parse_job)) {
        if (session->configure_pending && message->is_response && message->has_message_id &&
            message->message_id == session->configure_message_id) {
            ESP_LOGW(TAG, "Pool returned an invalid mining.configure response; using legacy V1");
            session->configure_pending = false;
        }
        bool invalid_subscribe = message->is_response &&
                                 v1_setup_id_matches(message, session->setup.subscribe_message_id);
        bool invalid_authorize = message->is_response &&
                                 v1_setup_id_matches(message, session->setup.authorize_message_id);
        if (invalid_subscribe || invalid_authorize) {
            ESP_LOGE(TAG, "Invalid %s response; reconnecting",
                     invalid_subscribe ? "mining.subscribe" : "mining.authorize");
            v1_negotiation_interrupted(session);
            return V1_LINE_FAIL;
        }
        ESP_LOGE(TAG, "Failed to parse Stratum message, ignoring");
        return V1_LINE_CONTINUE;
    }

    switch (message->method) {
        case METHOD_UNKNOWN:
            break;

        case MINING_NOTIFY:
            return v1_handle_notify(session);

        case MINING_SET_DIFFICULTY: {
            // Keep the per-job snapshot for spec-compliant pools; the submit
            // path also enforces increases immediately.
            double requested_diff = message->new_difficulty;
            double asic_diff = GLOBAL_STATE->DEVICE_CONFIG.family.asic.difficulty;
            double effective_diff = (requested_diff < asic_diff) ? asic_diff : requested_diff;
            pthread_mutex_lock(&GLOBAL_STATE->transport_mutex);
            s_v1_conn->pool_difficulty = effective_diff;
            pthread_mutex_unlock(&GLOBAL_STATE->transport_mutex);
            GLOBAL_STATE->SYSTEM_MODULE.pool_difficulty = effective_diff;
            ESP_LOGI(TAG, "Set effective pool difficulty: %.2f (requested: %.2f)",
                     effective_diff, requested_diff);
            break;
        }

        case MINING_SET_VERSION_MASK:
            return v1_handle_version_mask(session);

        case STRATUM_RESULT_CONFIGURE:
            return v1_handle_configure(session);

        case MINING_SET_EXTRANONCE:
        case STRATUM_RESULT_SUBSCRIBE:
            return v1_handle_extranonce(session);

        case MINING_PING:
            if (STRATUM_V1_pong(session->transport, message->message_id) < 0) {
                return V1_LINE_FAIL;
            }
            break;

        case CLIENT_RECONNECT:
            ESP_LOGW(TAG, "Pool requested client reconnect, pausing 1s before reconnecting...");
            vTaskDelay(pdMS_TO_TICKS(1000));
            return V1_LINE_POOL_RECONNECT;

        case CLIENT_SHOW_MESSAGE:
            break;

        case CLIENT_GET_VERSION:
            if (STRATUM_V1_send_version(session->transport, message->message_id) < 0) {
                return V1_LINE_FAIL;
            }
            break;

        case STRATUM_RESULT:
            return v1_handle_result(session, receive_time_us);
    }
    return V1_LINE_CONTINUE;
}

int stratum_v1_submit_share_checked(GlobalState *GLOBAL_STATE, const bm_job *job,
                                    uint32_t expected_generation, uint32_t nonce,
                                    uint32_t version_bits, double share_difficulty,
                                    uint64_t *sent_time_us)
{
    if (!GLOBAL_STATE || !job || !job->jobid || !job->extranonce2) {
        return -1;
    }

    pthread_mutex_lock(&GLOBAL_STATE->transport_mutex);
    esp_transport_handle_t transport = GLOBAL_STATE->transport;
    sv1_conn_t *conn = s_v1_conn;
    if (transport == NULL || conn == NULL || s_closing ||
        job->session_id != s_session_id ||
        expected_generation != ASIC_result_task_get_job_generation()) {
        pthread_mutex_unlock(&GLOBAL_STATE->transport_mutex);
        return STRATUM_V1_SUBMIT_STALE;
    }

    double required_difficulty =
        mining_v1_effective_share_difficulty(job->pool_diff, conn->pool_difficulty);
    if (!(share_difficulty >= required_difficulty)) {
        pthread_mutex_unlock(&GLOBAL_STATE->transport_mutex);
        return STRATUM_V1_SUBMIT_FILTERED;
    }

    int uid = conn->send_uid++;
    char user[sizeof(conn->user)];
    strlcpy(user, conn->user, sizeof(user));
    s_active_writers++;
    pthread_mutex_unlock(&GLOBAL_STATE->transport_mutex);

    int ret = STRATUM_V1_submit_share(transport, uid, user, job->jobid, job->extranonce2,
                                      job->ntime, nonce, job->version_rolling_enabled,
                                      version_bits, sent_time_us);
    int submit_errno = errno;

    pthread_mutex_lock(&GLOBAL_STATE->transport_mutex);
    if (ret >= 0) {
        if (GLOBAL_STATE->SYSTEM_MODULE.shares_pending < UINT16_MAX) {
            GLOBAL_STATE->SYSTEM_MODULE.shares_pending++;
        }
    } else if ((ret == STRATUM_SOCKET_WRITE_ERROR || ret == STRATUM_SOCKET_WRITE_TRUNCATED) &&
               GLOBAL_STATE->transport == transport && !s_closing) {
        // Bind the failure to the exact handle used above so a reconnect that
        // raced this write can never close the replacement connection. A
        // truncated request would otherwise prefix the next one on the wire.
        s_closing = true;
        int sock = esp_transport_get_socket(transport);
        if (sock >= 0) {
            shutdown(sock, SHUT_RDWR);
        }
    }
    s_active_writers--;
    if (s_active_writers == 0) {
        pthread_cond_broadcast(&s_writers_drained);
    }
    pthread_mutex_unlock(&GLOBAL_STATE->transport_mutex);

    errno = submit_errno;
    return ret;
}

int stratum_v1_submit_share(GlobalState *GLOBAL_STATE, const bm_job *active_job,
                            uint32_t nonce, uint32_t rolled_version, uint64_t *sent_time_us)
{
    if (!GLOBAL_STATE || !active_job) return -1;

    // BIP310: the pool rebuilds the block version from the masked field.
    uint32_t version_bits = rolled_version & active_job->version_mask;
    double share_difficulty = test_nonce_value(active_job, nonce, rolled_version);
    return stratum_v1_submit_share_checked(GLOBAL_STATE, active_job,
                                           ASIC_result_task_get_job_generation(), nonce,
                                           version_bits, share_difficulty, sent_time_us);
}

double stratum_v1_get_current_difficulty(GlobalState *GLOBAL_STATE)
{
    if (!GLOBAL_STATE) return 0.0;

    pthread_mutex_lock(&GLOBAL_STATE->transport_mutex);
    double difficulty = (s_v1_conn != NULL && !s_closing) ? s_v1_conn->pool_difficulty : 0.0;
    pthread_mutex_unlock(&GLOBAL_STATE->transport_mutex);
    return difficulty;
}

void stratum_v1_close_connection(GlobalState *GLOBAL_STATE)
{
    pthread_mutex_lock(&GLOBAL_STATE->transport_mutex);
    s_closing = true;
    s_session_id = 0;
    esp_transport_handle_t transport = GLOBAL_STATE->transport;
    GLOBAL_STATE->transport = NULL;
    sv1_conn_t *conn = s_v1_conn;
    s_v1_conn = NULL;
    if (transport != NULL) {
        // Wake a share writer blocked in send before waiting for it.
        int sock = esp_transport_get_socket(transport);
        if (sock >= 0) {
            shutdown(sock, SHUT_RDWR);
        }
    }
    while (s_active_writers != 0) {
        pthread_cond_wait(&s_writers_drained, &GLOBAL_STATE->transport_mutex);
    }
    pthread_mutex_unlock(&GLOBAL_STATE->transport_mutex);

    if (transport != NULL) {
        esp_transport_close(transport);
        esp_transport_destroy(transport);
    }
    if (conn != NULL) {
        clear_active_job_ids(conn->active_job_ids, &conn->active_job_ids_count);
        free(conn);
    }
    s_latest_job_valid = false;
    s_version_rolling_enabled = false;

    GLOBAL_STATE->SYSTEM_MODULE.shares_pending = 0;
    SYSTEM_clean_jobs_queue(GLOBAL_STATE);
    SYSTEM_reset_coinbase_ui_state(GLOBAL_STATE, "");
}

static esp_err_t v1_open_session(GlobalState *GLOBAL_STATE, uint16_t pool_idx,
                                 const PoolConfig *pool, v1_session_t *session)
{
    const char *stratum_url = pool->url;
    uint16_t port = pool->port;
    tls_mode tls = (tls_mode)pool->tls;
    const char *cert = (pool->cert != NULL && pool->cert[0] != '\0') ? pool->cert : NULL;

    sv1_conn_t *conn = calloc(1, sizeof(sv1_conn_t));
    if (!conn) {
        ESP_LOGE(TAG, "Failed to allocate sv1_conn");
        return ESP_ERR_NO_MEM;
    }
    conn->send_uid = 1;
    strlcpy(conn->user, pool->user != NULL ? pool->user : "", sizeof(conn->user));
    conn->pool_difficulty = (double)GLOBAL_STATE->DEVICE_CONFIG.family.asic.difficulty;
    conn->version_mask = 0;

    stratum_connection_info_t conn_info;
    if (stratum_socket_resolve(stratum_url, port, &conn_info) != ESP_OK) {
        ESP_LOGE(TAG, "Address resolution failed for %s", stratum_url);
        snprintf(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info,
                 sizeof(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info), "SV1: Pool unreachable");
        free(conn);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Connecting to: stratum+tcp://%s:%d (%s)", stratum_url, port, conn_info.host_ip);

    esp_transport_handle_t transport = STRATUM_V1_transport_init(tls, cert);
    if (!transport) {
        ESP_LOGE(TAG, "Transport initialization failed.");
        snprintf(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info,
                 sizeof(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info), "SV1: Internal error");
        free(conn);
        return ESP_FAIL;
    }

    if (tls != DISABLED) {
        esp_transport_ssl_set_common_name(transport, stratum_url);
    }

    // Use the already-resolved IP so a slow DNS lookup cannot block the lwIP stack.
    esp_err_t ret = esp_transport_connect(transport, conn_info.host_ip, port, TRANSPORT_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Transport unable to connect to %s:%d (errno %d)", stratum_url, port, ret);
        snprintf(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info,
                 sizeof(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info), "SV1: Connection failed");
        esp_transport_close(transport);
        esp_transport_destroy(transport);
        free(conn);
        return ESP_FAIL;
    }

    stratum_socket_set_options(transport);

    uint32_t session_id = stratum_next_session_id();
    pthread_mutex_lock(&GLOBAL_STATE->transport_mutex);
    GLOBAL_STATE->transport = transport;
    s_v1_conn = conn;
    s_session_id = session_id;
    s_closing = false;
    pthread_mutex_unlock(&GLOBAL_STATE->transport_mutex);

    s_latest_job_valid = false;
    s_version_rolling_enabled = false;

    const char *protocol = (conn_info.addr_family == AF_INET6) ? "IPv6" : "IPv4";
    const char *tls_status = (tls == BUNDLED_CRT) ? " (TLS)" : (tls == CUSTOM_CRT) ? " (TLS Cert)" : "";
    snprintf(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info,
             sizeof(GLOBAL_STATE->SYSTEM_MODULE.pool_connection_info),
             "%s%s", protocol, tls_status);

    // Work from any previous session must never reach the ASIC again.
    SYSTEM_clean_jobs_queue(GLOBAL_STATE);

    *session = (v1_session_t) {
        .gs = GLOBAL_STATE,
        .transport = transport,
        .pool = pool,
        .pool_idx = pool_idx,
        .session_id = session_id,
        .setup = {
            .subscribe_message_id = -1,
            .authorize_message_id = -1,
        },
        .configure_message_id = -1,
    };
    return ESP_OK;
}

// Send mining.configure (when probing BIP310), subscribe and authorize.
static bool v1_send_setup(v1_session_t *session)
{
    GlobalState *GLOBAL_STATE = session->gs;

    if (STRATUM_V1_bip310_should_probe(&s_bip310.state)) {
        session->configure_message_id = stratum_get_next_uid(GLOBAL_STATE);
        if (STRATUM_V1_configure_version_rolling(session->transport, session->configure_message_id,
                                                 BIP320_VERSION_ROLLING_MASK,
                                                 STRATUM_VERSION_ROLLING_MIN_BIT_COUNT) < 0) {
            // A partial configure can leave the JSON stream unusable. Reconnect
            // once in legacy mode instead of repeating the request forever.
            ESP_LOGW(TAG, "Unable to send mining.configure; retrying in legacy V1 mode");
            STRATUM_V1_bip310_transient_failure(&s_bip310.state);
            return false;
        }
        session->configure_pending = true;
        session->configure_sent_us = esp_timer_get_time();
    } else {
        ESP_LOGI(TAG, "Using legacy Stratum V1 without BIP310 version rolling");
    }

    session->setup.subscribe_message_id = stratum_get_next_uid(GLOBAL_STATE);
    if (STRATUM_V1_subscribe(session->transport, session->setup.subscribe_message_id,
                             GLOBAL_STATE->DEVICE_CONFIG.family.asic.name) < 0) {
        ESP_LOGE(TAG, "Unable to send mining.subscribe; reconnecting");
        v1_negotiation_interrupted(session);
        return false;
    }

    session->setup.authorize_message_id = stratum_get_next_uid(GLOBAL_STATE);
    if (STRATUM_V1_authorize(session->transport, session->setup.authorize_message_id,
                             session->pool->user != NULL ? session->pool->user : "",
                             session->pool->pass != NULL ? session->pool->pass : "") < 0) {
        ESP_LOGE(TAG, "Unable to send mining.authorize; reconnecting");
        v1_negotiation_interrupted(session);
        return false;
    }

    session->setup.deadline_us = esp_timer_get_time() + SETUP_RESPONSE_TIMEOUT_US;
    return true;
}

static esp_err_t v1_session_loop(v1_session_t *session)
{
    GlobalState *GLOBAL_STATE = session->gs;

    while (1) {
        if (stratum_reconnect_requested()) {
            ESP_LOGI(TAG, "Reconnect requested");
            return ESP_OK;
        }

        if (GLOBAL_STATE->SYSTEM_MODULE.mining_paused || GLOBAL_STATE->SYSTEM_MODULE.hardware_fault) {
            ESP_LOGI(TAG, "Mining paused, disconnecting from pool");
            return ESP_OK;
        }

        char *line = NULL;
        stratum_v1_receive_status_t status =
            STRATUM_V1_receive_jsonrpc_line_status(session->transport, &line);
        if (status == STRATUM_V1_RECEIVE_ERROR) {
            if (stratum_reconnect_requested()) {
                ESP_LOGI(TAG, "Reconnect requested during read");
                return ESP_OK;
            }
            if (session->configure_pending) {
                ESP_LOGW(TAG, "Pool closed before mining.configure completed; falling back to legacy V1");
                v1_negotiation_interrupted(session);
            }
            ESP_LOGE(TAG, "Failed to receive JSON-RPC line, reconnecting...");
            return ESP_FAIL;
        }

        int64_t now_us = esp_timer_get_time();
        if (status == STRATUM_V1_RECEIVE_LINE) {
            v1_line_result_t result = v1_handle_line(session, line, now_us);
            free(line);
            STRATUM_V1_reset_message(s_v1_msg);
            if (result == V1_LINE_FAIL || result == V1_LINE_POOL_RECONNECT) {
                return ESP_FAIL;
            }
            v1_maybe_complete_setup(&session->setup);
        }

        v1_check_configure_timeout(session, now_us);
        if (v1_setup_timed_out(&session->setup, now_us)) {
            ESP_LOGE(TAG, "Stratum V1 subscription/authorization timed out; reconnecting");
            v1_negotiation_interrupted(session);
            return ESP_FAIL;
        }
    }
}

static esp_err_t v1_run_pool(GlobalState *GLOBAL_STATE, uint16_t pool_idx, const PoolConfig *pool)
{
    if (pool->url == NULL || pool->url[0] == '\0' || pool->port == 0) {
        ESP_LOGE(TAG, "Invalid pool configuration for pool %u", pool_idx);
        return ESP_ERR_INVALID_ARG;
    }

    GLOBAL_STATE->SYSTEM_MODULE.shares_pending = 0;
    // Buffered data and request timings are connection-scoped and must never
    // cross a reconnect.
    if (!STRATUM_V1_initialize_buffer() || !v1_ensure_buffers()) {
        ESP_LOGE(TAG, "Failed to allocate Stratum V1 buffers");
        return ESP_ERR_NO_MEM;
    }

    if (!s_bip310.valid || s_bip310.port != pool->port || strcmp(s_bip310.url, pool->url) != 0) {
        s_bip310.state = (stratum_v1_bip310_state_t)STRATUM_V1_BIP310_STATE_INITIALIZER;
        strlcpy(s_bip310.url, pool->url, sizeof(s_bip310.url));
        s_bip310.port = pool->port;
        s_bip310.valid = true;
    }

    v1_session_t session;
    esp_err_t err = v1_open_session(GLOBAL_STATE, pool_idx, pool, &session);
    if (err != ESP_OK) {
        return err;
    }

    err = v1_send_setup(&session) ? v1_session_loop(&session) : ESP_FAIL;
    stratum_v1_close_connection(GLOBAL_STATE);
    return err;
}

esp_err_t stratum_v1_run(GlobalState *GLOBAL_STATE, uint16_t pool_idx)
{
    if (!GLOBAL_STATE || pool_idx >= MAX_POOLS) {
        return ESP_ERR_INVALID_ARG;
    }

    PoolConfig pool = {0};
    if (!SYSTEM_get_pool_config_snapshot(GLOBAL_STATE, pool_idx, &pool)) {
        ESP_LOGE(TAG, "Unable to snapshot pool %u configuration", pool_idx);
        return ESP_ERR_NO_MEM;
    }
    esp_err_t err = v1_run_pool(GLOBAL_STATE, pool_idx, &pool);
    SYSTEM_release_pool_config_snapshot(&pool);
    return err;
}

static bool v1_probe(GlobalState *GLOBAL_STATE, const PoolConfig *pool)
{
    const char *url = pool->url;
    uint16_t port = pool->port;
    tls_mode tls = (tls_mode)pool->tls;
    const char *cert = (pool->cert != NULL && pool->cert[0] != '\0') ? pool->cert : NULL;

    if (url == NULL || url[0] == '\0' || port == 0) return false;

    stratum_connection_info_t conn_info;
    if (stratum_socket_resolve(url, port, &conn_info) != ESP_OK) {
        return false;
    }

    esp_transport_handle_t transport = STRATUM_V1_transport_init(tls, cert);
    if (!transport) return false;

    if (tls != DISABLED) {
        esp_transport_ssl_set_common_name(transport, url);
    }

    esp_err_t err = esp_transport_connect(transport, conn_info.host_ip, port, TRANSPORT_TIMEOUT_MS);
    if (err != ESP_OK) {
        esp_transport_close(transport);
        esp_transport_destroy(transport);
        return false;
    }

    bool success = false;
    if (STRATUM_V1_subscribe(transport, 1, GLOBAL_STATE->DEVICE_CONFIG.family.asic.name) >= 0 &&
        STRATUM_V1_authorize(transport, 2, pool->user != NULL ? pool->user : "",
                             pool->pass != NULL ? pool->pass : "") >= 0) {
        char recv_buf[PROBE_RECV_BUFFER_SIZE];
        int total_bytes = 0;
        int64_t start_time = esp_timer_get_time();

        while (total_bytes < (int)sizeof(recv_buf) - 1) {
            int64_t elapsed_ms = (esp_timer_get_time() - start_time) / 1000;
            int remaining_ms = TRANSPORT_TIMEOUT_MS - (int)elapsed_ms;
            if (remaining_ms <= 0) break;

            int n = esp_transport_read(transport, recv_buf + total_bytes, sizeof(recv_buf) - 1 - total_bytes, remaining_ms);
            if (n <= 0) break;
            total_bytes += n;
            recv_buf[total_bytes] = '\0';

            // Reject immediately if an error payload is returned
            if (strstr(recv_buf, "\"error\":[") != NULL ||
                strstr(recv_buf, "\"error\": {") != NULL ||
                strstr(recv_buf, "\"error\":{") != NULL) {
                success = false;
                break;
            }

            // Accept if a result is returned for subscribe (id: 1) or authorize (id: 2)
            if (strstr(recv_buf, "\"result\"") != NULL &&
                (strstr(recv_buf, "\"id\":1") != NULL || strstr(recv_buf, "\"id\": 1") != NULL ||
                 strstr(recv_buf, "\"id\":2") != NULL || strstr(recv_buf, "\"id\": 2") != NULL)) {
                success = true;
                break;
            }
        }
    }

    esp_transport_close(transport);
    esp_transport_destroy(transport);
    return success;
}

bool stratum_v1_probe_pool(GlobalState *GLOBAL_STATE, uint16_t pool_idx)
{
    if (!GLOBAL_STATE || pool_idx >= MAX_POOLS) return false;

    PoolConfig pool = {0};
    if (!SYSTEM_get_pool_config_snapshot(GLOBAL_STATE, pool_idx, &pool)) {
        return false;
    }
    bool reachable = v1_probe(GLOBAL_STATE, &pool);
    SYSTEM_release_pool_config_snapshot(&pool);
    return reachable;
}
