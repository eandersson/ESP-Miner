#ifndef ASIC_COMMON_H_
#define ASIC_COMMON_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

static const double NONCE_SPACE = 4294967296.0; //  2^32

#define ASIC_RX_FRAME_MAX_SIZE 16

typedef struct
{
    uint8_t data[ASIC_RX_FRAME_MAX_SIZE];
    size_t length;
    uint32_t discarded_bytes;
    uint32_t frames_received;
    uint32_t crc_errors;
    uint32_t timeouts;
    uint32_t uart_errors;
} asic_rx_stream_t;

typedef struct
{
    uint32_t discarded_bytes;
    uint32_t frames_received;
    uint32_t crc_errors;
    uint32_t timeouts;
    uint32_t uart_errors;
} asic_rx_stats_t;

typedef enum
{
    REGISTER_INVALID = 0,
    REGISTER_HASHRATE,       // hashrate register (BM1397)
    REGISTER_TOTAL_COUNT,    // total counter (BM1366,BM1368,BM1370)
    REGISTER_DOMAIN_0_COUNT, // domain counters (BM1366,BM1368,BM1370)
    REGISTER_DOMAIN_1_COUNT,
    REGISTER_DOMAIN_2_COUNT,
    REGISTER_DOMAIN_3_COUNT,
    REGISTER_ERROR_COUNT,    // error count register (all)
    REGISTER_PLL_PARAM,      // PLL/clock config readback (BM1370)
} register_type_t;

typedef struct task_result
{
    // -- job result response
    uint8_t job_id;
    uint32_t nonce;
    uint32_t rolled_version;
    // ---- register response
    register_type_t register_type;
    uint8_t asic_nr;
    uint32_t value;
    uint8_t core_id;
    uint8_t small_core_id;
    // ---- timestamp
    uint64_t timestamp_us;
} task_result;

unsigned char _reverse_bits(unsigned char num);
int _largest_power_of_two(int num);
int _next_power_of_two(int num);
void clear_asic_chain_error(void);
const char *get_asic_chain_error(void);
int count_asic_chips(uint16_t asic_count, uint16_t chip_id, int chip_id_response_length);
void asic_rx_stream_reset(asic_rx_stream_t *stream);
bool asic_rx_stream_push(asic_rx_stream_t *stream, uint8_t byte, uint8_t *frame, size_t frame_size);
void reset_work_rx_parser(void);
void get_work_rx_stats(asic_rx_stats_t *stats);
esp_err_t receive_work(uint8_t * buffer, int buffer_size, uint64_t *out_timestamp_us);
void get_difficulty_mask(double difficulty, uint8_t *job_difficulty_mask);
uint32_t calculate_bm_hcn(float frequency_mhz, size_t asic_count, size_t cores,
                          double nonce_fraction, double frequency_multiplier,
                          double correction);
double calculate_bm_timeout_ms(float frequency_mhz, size_t asic_count, size_t small_cores, size_t cores, size_t version_size, float timeout_percent, double default_time_ms);
double calculate_bm_job_interval_ms(float frequency_mhz, size_t asic_count,
                                    size_t small_cores, size_t cores,
                                    size_t version_size, double max_refresh_ms);

#endif /* ASIC_COMMON_H_ */
