#ifndef STRATUM_API_H
#define STRATUM_API_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <sys/time.h>
#include <esp_transport.h>

#define MAX_MERKLE_BRANCHES 32
#define HASH_SIZE 32
#define COINBASE_SIZE 100
#define COINBASE2_SIZE 128
#define MAX_REQUEST_IDS 1024
#define MAX_EXTRANONCE_2_LEN 32
#define MAX_POOL_MESSAGE_LEN 256
#define STRATUM_VERSION_ROLLING_MIN_BIT_COUNT 2

// STRATUM_V1_submit_share() could not encode the request in its bounded
// buffer. Kept distinct from the stratum_socket write results so callers do
// not tear down a healthy connection for a local formatting failure.
#define STRATUM_V1_SUBMIT_FORMAT_ERROR (-3)

typedef enum
{
    STRATUM_V1_BIP310_PROBE = 0,
    STRATUM_V1_BIP310_SKIP_ONCE,
    STRATUM_V1_BIP310_EXPLICIT_LEGACY,
} stratum_v1_bip310_mode_t;

typedef struct
{
    stratum_v1_bip310_mode_t mode;
} stratum_v1_bip310_state_t;

#define STRATUM_V1_BIP310_STATE_INITIALIZER \
    { .mode = STRATUM_V1_BIP310_PROBE }

static inline bool STRATUM_V1_bip310_should_probe(
    stratum_v1_bip310_state_t *state)
{
    if (state == NULL) {
        return false;
    }
    if (state->mode == STRATUM_V1_BIP310_SKIP_ONCE) {
        state->mode = STRATUM_V1_BIP310_PROBE;
        return false;
    }
    return state->mode == STRATUM_V1_BIP310_PROBE;
}

static inline void STRATUM_V1_bip310_transient_failure(
    stratum_v1_bip310_state_t *state)
{
    if (state != NULL &&
        state->mode != STRATUM_V1_BIP310_EXPLICIT_LEGACY) {
        state->mode = STRATUM_V1_BIP310_SKIP_ONCE;
    }
}

static inline void STRATUM_V1_bip310_mark_unsupported(
    stratum_v1_bip310_state_t *state)
{
    if (state != NULL) {
        state->mode = STRATUM_V1_BIP310_EXPLICIT_LEGACY;
    }
}

static inline void STRATUM_V1_bip310_mark_supported(
    stratum_v1_bip310_state_t *state)
{
    if (state != NULL) {
        state->mode = STRATUM_V1_BIP310_PROBE;
    }
}

typedef enum
{
    METHOD_UNKNOWN,
    MINING_NOTIFY,
    MINING_SET_DIFFICULTY,
    MINING_SET_VERSION_MASK,
    MINING_SET_EXTRANONCE,
    MINING_PING,
    STRATUM_RESULT,
    STRATUM_RESULT_SUBSCRIBE,
    STRATUM_RESULT_CONFIGURE,
    CLIENT_RECONNECT,
    CLIENT_SHOW_MESSAGE,
    CLIENT_GET_VERSION,
} stratum_method;

typedef enum
{
    DISABLED = 0,
    BUNDLED_CRT = 1,
    CUSTOM_CRT = 2,
} tls_mode;

typedef struct mining_notify
{
    char *job_id;
    char *prev_block_hash;
    char *coinbase_1;
    char *coinbase_2;
    uint8_t *merkle_branches;
    size_t n_merkle_branches;
    uint32_t version;
    uint32_t target;
    uint32_t ntime;
    bool clean_jobs;
} mining_notify;

typedef struct StratumApiV1Message
{
    char *extranonce_str;
    int extranonce_2_len;

    int message_id;
    // Indicates the type of request the message represents.
    stratum_method method;

    // mining.notify
    mining_notify *mining_notification;
    // mining.set_difficulty
    double new_difficulty;
    // mining.set_version_mask
    uint32_t version_mask;
    // result
    bool response_success;
    char *error_str;
    char *show_message;
    char *version_string;
} StratumApiV1Message;

typedef struct RequestTiming
{
    int64_t timestamp_us;
    bool tracking;
} RequestTiming;

typedef enum
{
    STRATUM_V1_RECEIVE_LINE,
    STRATUM_V1_RECEIVE_INCOMPLETE,
    STRATUM_V1_RECEIVE_TIMEOUT,
    STRATUM_V1_RECEIVE_ERROR,
} stratum_v1_receive_status_t;

esp_transport_handle_t STRATUM_V1_transport_init(tls_mode tls, char * cert);

void STRATUM_V1_initialize_buffer(void);
void cleanup_stratum_buffer(void);

char *STRATUM_V1_receive_jsonrpc_line(esp_transport_handle_t transport);

stratum_v1_receive_status_t STRATUM_V1_receive_jsonrpc_line_status(
    esp_transport_handle_t transport, char **line_out);

int STRATUM_V1_subscribe(esp_transport_handle_t transport, int send_uid, const char * model);

bool STRATUM_V1_parse(StratumApiV1Message *message, const char *stratum_json);

void STRATUM_V1_reset_message(StratumApiV1Message *message);

void STRATUM_V1_free_mining_notify(mining_notify *mining_notify);

int STRATUM_V1_authorize(esp_transport_handle_t transport, int send_uid, const char *username, const char *pass);

int STRATUM_V1_configure_version_rolling(esp_transport_handle_t transport,
                                         int send_uid,
                                         uint32_t version_mask,
                                         uint8_t min_bit_count);

int STRATUM_V1_pong(esp_transport_handle_t transport, int message_id);

int STRATUM_V1_send_version(esp_transport_handle_t transport, int message_id);

int STRATUM_V1_suggest_difficulty(esp_transport_handle_t transport, int send_uid, uint32_t difficulty);

int STRATUM_V1_extranonce_subscribe(esp_transport_handle_t transport, int send_uid);

int STRATUM_V1_submit_share(esp_transport_handle_t transport, int send_uid, const char *username, const char *job_id,
                            const char *extranonce_2, const uint32_t ntime, const uint32_t nonce,
                            bool version_rolling_enabled, const uint32_t version_bits,
                            uint64_t *out_sent_time_us);

int STRATUM_V1_format_submit_request(char *buffer, size_t buffer_size,
                                     int send_uid, const char *username,
                                     const char *job_id,
                                     const char *extranonce_2, uint32_t ntime,
                                     uint32_t nonce,
                                     bool version_rolling_enabled,
                                     uint32_t version_bits);

int STRATUM_V1_format_configure_request(char *buffer, size_t buffer_size,
                                        int send_uid, uint32_t version_mask,
                                        uint8_t min_bit_count);

float STRATUM_V1_get_response_time_ms(int request_id, int64_t receive_time_us);

#endif // STRATUM_API_H
