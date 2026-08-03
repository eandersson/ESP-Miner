#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>

#include "asic_common.h"
#include "serial.h"
#include "esp_log.h"
#include "crc.h"
#include "esp_timer.h"

#define PREAMBLE 0xAA55
#define ASIC_RX_TIMEOUT_MS 100

static const char * TAG = "common";
static char asic_chain_error[96];
static asic_rx_stream_t work_rx_stream;

static esp_err_t receive_stream_frame(asic_rx_stream_t *stream, uint8_t *buffer,
                                      size_t frame_size, uint32_t timeout_ms,
                                      uint64_t *out_timestamp_us);

register_type_t asic_register_map_lookup(const register_type_t *register_map,
                                         size_t register_map_size,
                                         uint8_t register_address)
{
    if (register_map == NULL || register_address >= register_map_size) {
        return REGISTER_INVALID;
    }
    return register_map[register_address];
}

static void format_asic_indices(char *buffer, size_t buffer_size, int first_index, int end_index)
{
    size_t offset = 0;

    for (int index = first_index; index < end_index; index++) {
        int written = snprintf(buffer + offset, buffer_size - offset, "%s%d", index == first_index ? "" : ",", index);
        if (written < 0 || (size_t) written >= buffer_size - offset) {
            snprintf(buffer, buffer_size, "%d-%d", first_index, end_index - 1);
            return;
        }

        offset += written;
    }
}

void clear_asic_chain_error(void)
{
    asic_chain_error[0] = '\0';
}

const char *get_asic_chain_error(void)
{
    return asic_chain_error[0] == '\0' ? NULL : asic_chain_error;
}

unsigned char _reverse_bits(unsigned char num)
{
    unsigned char reversed = 0;
    int i;

    for (i = 0; i < 8; i++) {
        reversed <<= 1;      // Left shift the reversed variable by 1
        reversed |= num & 1; // Use bitwise OR to set the rightmost bit of reversed to the current bit of num
        num >>= 1;           // Right shift num by 1 to get the next bit
    }

    return reversed;
}

int _largest_power_of_two(int num)
{
    int power = 0;

    while (num > 1) {
        num = num >> 1;
        power++;
    }

    return 1 << power;
}

int _next_power_of_two(int num)
{
    if (num <= 1)
        return 1;

    int power = 1;

    while (power < num) {
        power <<= 1;
    }

    return power;
}

int count_asic_chips(uint16_t asic_count, uint16_t chip_id, int chip_id_response_length)
{
    uint8_t buffer[11] = {0};
    asic_rx_stream_t chip_id_stream;

    clear_asic_chain_error();
    if (chip_id_response_length < 3 ||
        chip_id_response_length > (int)sizeof(buffer)) {
        ESP_LOGE(TAG, "Invalid CHIP_ID response length: %d",
                 chip_id_response_length);
        return 0;
    }
    asic_rx_stream_reset(&chip_id_stream);

    int chip_counter = 0;
    while (true) {
        esp_err_t receive_result = receive_stream_frame(
            &chip_id_stream, buffer, chip_id_response_length, 1000, NULL);
        if (receive_result == ESP_ERR_TIMEOUT) {
            break;
        }
        if (receive_result != ESP_OK) {
            ESP_LOGE(TAG, "Error reading CHIP_ID");
            break;
        }

        uint16_t received_preamble = (buffer[0] << 8) | buffer[1];
        if (received_preamble != PREAMBLE) {
            ESP_LOGW(TAG, "Preamble mismatch: expected 0x%04x, got 0x%04x", PREAMBLE, received_preamble);
            ESP_LOG_BUFFER_HEX(TAG, buffer, chip_id_response_length);
            continue;
        }

        uint16_t received_chip_id = (buffer[2] << 8) | buffer[3];
        if (received_chip_id != chip_id) {
            ESP_LOGW(TAG, "CHIP_ID response mismatch: expected 0x%04x, got 0x%04x", chip_id, received_chip_id);
            ESP_LOG_BUFFER_HEX(TAG, buffer, chip_id_response_length);
            continue;
        }

        if (crc5(buffer + 2, chip_id_response_length - 2) != 0) {
            ESP_LOGW(TAG, "Checksum failed on CHIP_ID response");
            ESP_LOG_BUFFER_HEX(TAG, buffer, chip_id_response_length);
            continue;
        }

        ESP_LOGI(TAG, "Chip %d detected: CORE_NUM: 0x%02x ADDR: 0x%02x", chip_counter, buffer[4], buffer[5]);

        chip_counter++;
    }    
    
    if (chip_counter != asic_count) {
        ESP_LOGE(TAG, "%i chip(s) detected on the chain, expected %i", chip_counter, asic_count);

        char asic_indices[64];
        if (chip_counter < asic_count) {
            format_asic_indices(asic_indices, sizeof(asic_indices), chip_counter, asic_count);
            snprintf(asic_chain_error, sizeof(asic_chain_error), "ASIC %s not found", asic_indices);
        } else {
            format_asic_indices(asic_indices, sizeof(asic_indices), asic_count, chip_counter);
            snprintf(asic_chain_error, sizeof(asic_chain_error), "Unexpected ASIC %s detected", asic_indices);
        }
        ESP_LOGE(TAG, "%s", asic_chain_error);

        return 0;
    }

    return chip_counter;
}

void asic_rx_stream_reset(asic_rx_stream_t *stream)
{
    if (stream == NULL) {
        return;
    }

    memset(stream, 0, sizeof(*stream));
}

static void discard_first_stream_byte(asic_rx_stream_t *stream)
{
    if (stream->length > 1) {
        memmove(stream->data, stream->data + 1, stream->length - 1);
    }
    if (stream->length > 0) {
        stream->length--;
        stream->discarded_bytes++;
    }
}

bool asic_rx_stream_push(asic_rx_stream_t *stream, uint8_t byte, uint8_t *frame, size_t frame_size)
{
    if (stream == NULL || frame == NULL || frame_size < 3 || frame_size > ASIC_RX_FRAME_MAX_SIZE) {
        return false;
    }

    if (stream->length == ASIC_RX_FRAME_MAX_SIZE) {
        discard_first_stream_byte(stream);
    }
    stream->data[stream->length++] = byte;

    while (stream->length >= 2 &&
           (stream->data[0] != 0xAA || stream->data[1] != 0x55)) {
        discard_first_stream_byte(stream);
    }

    if (stream->length < frame_size) {
        return false;
    }

    if (crc5(stream->data + 2, frame_size - 2) == 0) {
        memcpy(frame, stream->data, frame_size);
        stream->length = 0;
        stream->frames_received++;
        return true;
    }

    // Keep scanning instead of flushing bytes that may contain the next frame.
    stream->crc_errors++;
    discard_first_stream_byte(stream);
    while (stream->length >= 2 &&
           (stream->data[0] != 0xAA || stream->data[1] != 0x55)) {
        discard_first_stream_byte(stream);
    }
    return false;
}

static esp_err_t receive_stream_frame(asic_rx_stream_t *stream, uint8_t *buffer,
                                      size_t frame_size, uint32_t timeout_ms,
                                      uint64_t *out_timestamp_us)
{
    if (stream == NULL || buffer == NULL || frame_size < 3 ||
        frame_size > ASIC_RX_FRAME_MAX_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    int64_t deadline_us = esp_timer_get_time() + ((int64_t)timeout_ms * 1000);
    uint32_t discarded_before = stream->discarded_bytes;
    uint8_t rx_chunk[ASIC_RX_FRAME_MAX_SIZE];

    while (esp_timer_get_time() < deadline_us) {
        int64_t remaining_us = deadline_us - esp_timer_get_time();
        if (remaining_us <= 0) {
            break;
        }

        // Read enough bytes to complete the current candidate frame. In the
        // normal case this turns one UART-driver call per byte into one call
        // per ASIC response without consuming bytes past the first frame.
        size_t wanted = frame_size > stream->length
                            ? frame_size - stream->length
                            : 1;
        uint32_t remaining_ms_32 = (uint32_t)((remaining_us + 999) / 1000);
        uint16_t remaining_ms = remaining_ms_32 > UINT16_MAX
                                    ? UINT16_MAX
                                    : (uint16_t)remaining_ms_32;
        int received = SERIAL_rx(rx_chunk, (uint16_t)wanted, remaining_ms);

        if (received < 0) {
            stream->uart_errors++;
            ESP_LOGE(TAG, "UART error in serial RX");
            return ESP_FAIL;
        }
        if (received == 0) {
            break;
        }

        for (int i = 0; i < received; i++) {
            if (!asic_rx_stream_push(stream, rx_chunk[i], buffer, frame_size)) {
                continue;
            }

            if (out_timestamp_us != NULL) {
                *out_timestamp_us = esp_timer_get_time();
            }
            uint32_t discarded = stream->discarded_bytes - discarded_before;
            if (discarded > 0) {
                ESP_LOGD(TAG, "ASIC RX resynchronized after discarding %lu byte(s)",
                         (unsigned long)discarded);
            }
            return ESP_OK;
        }
    }

    stream->timeouts++;
    return ESP_ERR_TIMEOUT;
}

void reset_work_rx_parser(void)
{
    asic_rx_stream_reset(&work_rx_stream);
}

void get_work_rx_stats(asic_rx_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }

    stats->discarded_bytes = work_rx_stream.discarded_bytes;
    stats->frames_received = work_rx_stream.frames_received;
    stats->crc_errors = work_rx_stream.crc_errors;
    stats->timeouts = work_rx_stream.timeouts;
    stats->uart_errors = work_rx_stream.uart_errors;
}

esp_err_t receive_work(uint8_t * buffer, int buffer_size, uint64_t *out_timestamp_us)
{
    if (buffer == NULL || buffer_size < 3 || buffer_size > ASIC_RX_FRAME_MAX_SIZE) {
        ESP_LOGE(TAG, "Invalid ASIC response size %d", buffer_size);
        return ESP_FAIL;
    }

    esp_err_t result = receive_stream_frame(
        &work_rx_stream, buffer, (size_t)buffer_size, ASIC_RX_TIMEOUT_MS,
        out_timestamp_us);
    if (result == ESP_ERR_TIMEOUT) {
        ESP_LOGD(TAG, "UART timeout in serial RX");
    }
    return result == ESP_OK ? ESP_OK : ESP_FAIL;
}

void get_difficulty_mask(double difficulty, uint8_t *job_difficulty_mask)
{
    // The mask must be a power of 2 so there are no holes
    // Correct:   {0b00000000, 0b00000000, 0b11111111, 0b11111111}
    // Incorrect: {0b00000000, 0b00000000, 0b11100111, 0b11111111}

    // Round up to ensure we don't make difficulty harder than requested, then convert to int
    uint32_t diff_int = (uint32_t)ceil(difficulty);

    // Calculate largest power of 2 <= diff_int (inline of former _largest_power_of_two)
    int power = 0;
    while (diff_int > 1) {
        diff_int = diff_int >> 1;
        power++;
    }
    uint32_t mask = (1 << power) - 1;

    job_difficulty_mask[0] = 0x00;
    job_difficulty_mask[1] = 0x14; // TICKET_MASK

    // convert difficulty into char array
    // Ex: 256 = {0b00000000, 0b00000000, 0b00000000, 0b11111111}, {0x00, 0x00, 0x00, 0xff}
    // Ex: 512 = {0b00000000, 0b00000000, 0b00000001, 0b11111111}, {0x00, 0x00, 0x01, 0xff}
    job_difficulty_mask[2] = _reverse_bits((mask >> 24) & 0xFF);
    job_difficulty_mask[3] = _reverse_bits((mask >> 16) & 0xFF);
    job_difficulty_mask[4] = _reverse_bits((mask >>  8) & 0xFF);
    job_difficulty_mask[5] = _reverse_bits( mask        & 0xFF);
}

uint32_t calculate_bm_hcn(float frequency_mhz, size_t asic_count, size_t cores,
                          double nonce_fraction, double frequency_multiplier,
                          double correction)
{
    if (frequency_mhz <= 0.0f || asic_count == 0 || cores == 0 ||
        nonce_fraction <= 0.0 || frequency_multiplier <= 0.0) {
        return 0;
    }

    double nonce_space_per_core = NONCE_SPACE /
                                  (double)_next_power_of_two((int)cores) /
                                  (double)_next_power_of_two((int)asic_count);
    double hcn = nonce_space_per_core * frequency_multiplier /
                 (double)frequency_mhz * 0.5;
    hcn = nonce_fraction * (hcn - correction);

    if (hcn <= 0.0) {
        return 0;
    }
    if (hcn >= (double)UINT32_MAX) {
        return UINT32_MAX;
    }
    return (uint32_t)hcn;
}

double calculate_bm_timeout_ms(float frequency_mhz, size_t asic_count, size_t small_cores, size_t cores, size_t version_size, float timeout_percent, double default_time_ms)
{
    if (asic_count <= 0)
        return default_time_ms;

    // Round up to the nearest power of 2 some asic constants
    int cores_up = _next_power_of_two((int)cores);
    int small_cores_up = _next_power_of_two((int)small_cores);
    int asic_count_up = _next_power_of_two((int)asic_count);

    if ((small_cores_up < cores_up) || (frequency_mhz <= 0.0f))
        return default_time_ms;

    // Calulate the time to scan the full nonce * version space
    // effectively how many iterations we have to do
    // First we remove the paralell nonces/versions
    // then we end up with `time = space / frequency`
    double midstates = (double)small_cores_up / (double)cores_up;
    double serial_versions = (double)version_size / midstates;
    double serial_nonces = (double)NONCE_SPACE / (double)cores_up / (double)asic_count_up;
    double fullspace_timeout_ms = serial_versions * serial_nonces / ((double)frequency_mhz * 1000.0);

    if (!(fullspace_timeout_ms > 0.0))
        return default_time_ms;

    return (double)timeout_percent * fullspace_timeout_ms;
}

double calculate_bm_job_interval_ms(float frequency_mhz, size_t asic_count,
                                    size_t small_cores, size_t cores,
                                    size_t version_size, double max_refresh_ms)
{
    if (max_refresh_ms < 1.0) {
        max_refresh_ms = 1.0;
    }
    if (frequency_mhz <= 0.0f || asic_count == 0 || small_cores == 0 ||
        cores == 0 || version_size == 0) {
        return max_refresh_ms;
    }

    double full_scan_ms = calculate_bm_timeout_ms(
        frequency_mhz, asic_count, small_cores, cores, version_size, 1.0f,
        max_refresh_ms);
    return fmax(1.0, fmin(max_refresh_ms, full_scan_ms));
}
