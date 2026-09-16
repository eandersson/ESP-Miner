#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum
{
    JOB_PACKET = 0,
    CMD_PACKET = 1,
} packet_type_t;

typedef struct
{
    uint32_t tx_packets;
    uint32_t tx_bytes;
    uint32_t tx_failures;
    uint32_t tx_partial_writes;
    uint32_t rx_bytes;
    uint32_t rx_failures;
    uint32_t rx_buffer_high_watermark;
    uint32_t fifo_overflows;
    uint32_t buffer_full_events;
    uint32_t parity_errors;
    uint32_t frame_errors;
    uint32_t break_events;
    uint32_t baud_failures;
} serial_stats_t;

#define UART_FREQ 115200

/**
 * Queue one complete packet for UART transmission.
 *
 * A true return means the UART driver accepted every byte into its TX pipeline.
 * It does not imply an application-level acknowledgement from the ASIC.
 */
bool SERIAL_send(const uint8_t *data, size_t len, bool debug);
/**
 * Reject new ASIC UART writes and wait for already accepted TX data to drain.
 *
 * Calls that were already waiting to enter SERIAL_send() are invalidated so
 * they cannot leak into a later resume/startup cycle.
 */
esp_err_t SERIAL_pause_tx(uint32_t drain_timeout_ms);

/** Re-enable ASIC UART writes for a new initialization epoch. */
void SERIAL_resume_tx(void);
esp_err_t SERIAL_init(void);
void SERIAL_debug_rx(void);
int16_t SERIAL_rx(uint8_t *, uint16_t, uint16_t);
esp_err_t SERIAL_clear_buffer(void);
esp_err_t SERIAL_set_baud(int baud);
bool SERIAL_is_initialized(void);
void SERIAL_get_stats(serial_stats_t *stats);

#endif /* SERIAL_H_ */
