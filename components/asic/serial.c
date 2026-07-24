#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"

#include "esp_log.h"
#include "soc/uart_struct.h"

#include "serial.h"
#include "utils.h"

#define ECHO_TEST_TXD (17)
#define ECHO_TEST_RXD (18)
#define BUF_SIZE (1024)

static const char *TAG = "serial";

esp_err_t SERIAL_init(void)
{
    ESP_LOGI(TAG, "Initializing serial");
    // Configure UART1 parameters
    uart_config_t uart_config = {
        .baud_rate = UART_FREQ,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    // Configure UART1 parameters
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_param_config(UART_NUM_1, &uart_config));
    // Set UART1 pins(TX: IO17, RX: I018)
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Install the UART driver without an event queue. The bounded TX ring lets
    // the short command/job packets be queued without waiting for every byte to
    // leave the wire. SERIAL_send still requires the complete packet to be
    // accepted before reporting success.
    return uart_driver_install(UART_NUM_1, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
}

bool SERIAL_is_initialized(void)
{
    return uart_is_driver_installed(UART_NUM_1);
}

esp_err_t SERIAL_set_baud(int baud)
{
    ESP_LOGI(TAG, "Changing UART baud to %i", baud);

    // Make sure that we are done writing before setting a new baudrate.
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_wait_tx_done(UART_NUM_1, 1000 / portTICK_PERIOD_MS));

    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_set_baudrate(UART_NUM_1, baud));

    return ESP_OK;
}

bool SERIAL_send(const uint8_t *data, size_t len, bool debug)
{
    if (data == NULL || len == 0) {
        ESP_LOGE(TAG, "Refusing invalid UART write (data=%p, len=%u)",
                 (const void *)data, (unsigned int)len);
        return false;
    }

    if (!uart_is_driver_installed(UART_NUM_1)) {
        ESP_LOGE(TAG, "UART write attempted before driver initialization");
        return false;
    }

    if (debug)
    {
        printf("tx: ");
        prettyHex((unsigned char *)data, (int)len);
        printf("\n");
    }

    int written = uart_write_bytes(UART_NUM_1, data, len);
    if (written < 0) {
        ESP_LOGE(TAG, "UART write failed for %u-byte packet",
                 (unsigned int)len);
        return false;
    }
    if ((size_t)written != len) {
        ESP_LOGE(TAG, "Incomplete UART write: %d of %u bytes accepted",
                 written, (unsigned int)len);
        return false;
    }

    return true;
}

/// @brief waits for a serial response from the device
/// @param buf buffer to read data into
/// @param buf number of ms to wait before timing out
/// @return number of bytes read, or -1 on error
int16_t SERIAL_rx(uint8_t *buf, uint16_t size, uint16_t timeout_ms)
{
    int16_t bytes_read = uart_read_bytes(UART_NUM_1, buf, size, timeout_ms / portTICK_PERIOD_MS);

    #if BM1397_SERIALRX_DEBUG || BM1366_SERIALRX_DEBUG || BM1368_SERIALRX_DEBUG || BM1370_SERIALRX_DEBUG
    size_t buff_len = 0;
    if (bytes_read > 0) {
        uart_get_buffered_data_len(UART_NUM_1, &buff_len);
        printf("rx: ");
        prettyHex((unsigned char*) buf, bytes_read);
        printf(" [%d]\n", buff_len);
    }
    #endif

    return bytes_read;
}

void SERIAL_debug_rx(void)
{
    int ret;
    uint8_t buf[100];

    ret = SERIAL_rx(buf, 100, 20);
    if (ret < 0)
    {
        fprintf(stderr, "unable to read data\n");
        return;
    }

    memset(buf, 0, 100);
}

void SERIAL_clear_buffer(void)
{
    uart_flush(UART_NUM_1);
}
