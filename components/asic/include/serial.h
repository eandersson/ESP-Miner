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

#define UART_FREQ 115200

/**
 * Queue one complete packet for UART transmission.
 *
 * A true return means the UART driver accepted every byte into its TX pipeline.
 * It does not imply an application-level acknowledgement from the ASIC.
 */
bool SERIAL_send(const uint8_t *data, size_t len, bool debug);
esp_err_t SERIAL_init(void);
void SERIAL_debug_rx(void);
int16_t SERIAL_rx(uint8_t *, uint16_t, uint16_t);
void SERIAL_clear_buffer(void);
esp_err_t SERIAL_set_baud(int baud);
bool SERIAL_is_initialized(void);

#endif /* SERIAL_H_ */
