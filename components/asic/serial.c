#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/uart.h"

#include "esp_log.h"
#include "soc/uart_struct.h"

#include "serial.h"
#include "utils.h"

#define ECHO_TEST_TXD (17)
#define ECHO_TEST_RXD (18)
#define BUF_SIZE (1024)
#define UART_EVENT_QUEUE_SIZE (32)

static const char *TAG = "serial";
static QueueHandle_t uart_event_queue;

static atomic_uint_fast32_t tx_packets;
static atomic_uint_fast32_t tx_bytes;
static atomic_uint_fast32_t tx_failures;
static atomic_uint_fast32_t tx_partial_writes;
static atomic_uint_fast32_t rx_bytes;
static atomic_uint_fast32_t rx_failures;
static atomic_uint_fast32_t rx_buffer_high_watermark;
static atomic_uint_fast32_t fifo_overflows;
static atomic_uint_fast32_t buffer_full_events;
static atomic_uint_fast32_t parity_errors;
static atomic_uint_fast32_t frame_errors;
static atomic_uint_fast32_t break_events;
static atomic_uint_fast32_t baud_failures;

static void update_atomic_max(atomic_uint_fast32_t *maximum, uint32_t value)
{
    uint_fast32_t current = atomic_load(maximum);
    while (value > current &&
           !atomic_compare_exchange_weak(maximum, &current, value)) {
    }
}

static void update_rx_high_watermark(void)
{
    size_t buffered = 0;
    if (uart_is_driver_installed(UART_NUM_1) &&
        uart_get_buffered_data_len(UART_NUM_1, &buffered) == ESP_OK) {
        uint32_t value = buffered > UINT32_MAX ? UINT32_MAX : (uint32_t)buffered;
        update_atomic_max(&rx_buffer_high_watermark, value);
    }
}

static void process_uart_events(void)
{
    if (uart_event_queue == NULL) {
        return;
    }

    uart_event_t event;
    bool input_lost = false;
    while (xQueueReceive(uart_event_queue, &event, 0) == pdTRUE) {
        switch (event.type) {
            case UART_DATA:
                // Event size is captured by the driver without another UART
                // lock/query in the hot RX path.
                update_atomic_max(&rx_buffer_high_watermark,
                                  event.size > UINT32_MAX
                                      ? UINT32_MAX
                                      : (uint32_t)event.size);
                break;
            case UART_FIFO_OVF:
                atomic_fetch_add(&fifo_overflows, 1);
                input_lost = true;
                break;
            case UART_BUFFER_FULL:
                atomic_fetch_add(&buffer_full_events, 1);
                input_lost = true;
                break;
            case UART_PARITY_ERR:
                atomic_fetch_add(&parity_errors, 1);
                break;
            case UART_FRAME_ERR:
                atomic_fetch_add(&frame_errors, 1);
                break;
            case UART_BREAK:
                atomic_fetch_add(&break_events, 1);
                break;
            default:
                break;
        }
    }

    if (input_lost) {
        // Once the hardware or driver ring has overflowed, retaining a partial
        // frame only delays resynchronization. Drop the damaged input and let
        // the ASIC stream parser lock onto the next AA55 preamble.
        uart_flush_input(UART_NUM_1);
        xQueueReset(uart_event_queue);
        ESP_LOGW(TAG, "ASIC UART input overflow; discarded damaged RX data");
    }
}

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
    esp_err_t err = uart_param_config(UART_NUM_1, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Unable to configure ASIC UART: %s", esp_err_to_name(err));
        return err;
    }
    // Set UART1 pins(TX: IO17, RX: I018)
    err = uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Unable to configure ASIC UART pins: %s", esp_err_to_name(err));
        return err;
    }

    // The bounded TX ring lets
    // the short command/job packets be queued without waiting for every byte to
    // leave the wire. SERIAL_send still requires the complete packet to be
    // accepted before reporting success.
    err = uart_driver_install(UART_NUM_1, BUF_SIZE * 2, BUF_SIZE * 2,
                              UART_EVENT_QUEUE_SIZE, &uart_event_queue, 0);
    if (err != ESP_OK) {
        uart_event_queue = NULL;
        ESP_LOGE(TAG, "Unable to install ASIC UART driver: %s", esp_err_to_name(err));
    }
    return err;
}

bool SERIAL_is_initialized(void)
{
    return uart_is_driver_installed(UART_NUM_1);
}

esp_err_t SERIAL_set_baud(int baud)
{
    if (baud <= 0) {
        atomic_fetch_add(&baud_failures, 1);
        return ESP_ERR_INVALID_ARG;
    }
    if (!uart_is_driver_installed(UART_NUM_1)) {
        atomic_fetch_add(&baud_failures, 1);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Changing UART baud to %i", baud);

    // Make sure that we are done writing before setting a new baudrate.
    esp_err_t err = uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        atomic_fetch_add(&baud_failures, 1);
        ESP_LOGE(TAG, "ASIC UART did not drain before baud change: %s",
                 esp_err_to_name(err));
        return err;
    }

    err = uart_set_baudrate(UART_NUM_1, baud);
    if (err != ESP_OK) {
        atomic_fetch_add(&baud_failures, 1);
        ESP_LOGE(TAG, "Unable to change ASIC UART baud: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

bool SERIAL_send(const uint8_t *data, size_t len, bool debug)
{
    if (data == NULL || len == 0) {
        ESP_LOGE(TAG, "Refusing invalid UART write (data=%p, len=%u)",
                 (const void *)data, (unsigned int)len);
        atomic_fetch_add(&tx_failures, 1);
        return false;
    }

    if (!uart_is_driver_installed(UART_NUM_1)) {
        ESP_LOGE(TAG, "UART write attempted before driver initialization");
        atomic_fetch_add(&tx_failures, 1);
        return false;
    }

    if (debug)
    {
        printf("tx: ");
        prettyHex((unsigned char *)data, (int)len);
        printf("\n");
    }

    process_uart_events();
    int written = uart_write_bytes(UART_NUM_1, data, len);
    if (written < 0) {
        ESP_LOGE(TAG, "UART write failed for %u-byte packet",
                 (unsigned int)len);
        atomic_fetch_add(&tx_failures, 1);
        return false;
    }
    if ((size_t)written != len) {
        ESP_LOGE(TAG, "Incomplete UART write: %d of %u bytes accepted",
                 written, (unsigned int)len);
        atomic_fetch_add(&tx_partial_writes, 1);
        return false;
    }

    atomic_fetch_add(&tx_packets, 1);
    atomic_fetch_add(&tx_bytes, (uint32_t)len);
    return true;
}

/// @brief waits for a serial response from the device
/// @param buf buffer to read data into
/// @param buf number of ms to wait before timing out
/// @return number of bytes read, or -1 on error
int16_t SERIAL_rx(uint8_t *buf, uint16_t size, uint16_t timeout_ms)
{
    if (buf == NULL || size == 0 || !uart_is_driver_installed(UART_NUM_1)) {
        atomic_fetch_add(&rx_failures, 1);
        return -1;
    }

    process_uart_events();
    int16_t bytes_read = uart_read_bytes(UART_NUM_1, buf, size, timeout_ms / portTICK_PERIOD_MS);
    if (bytes_read < 0) {
        atomic_fetch_add(&rx_failures, 1);
    } else if (bytes_read > 0) {
        atomic_fetch_add(&rx_bytes, (uint32_t)bytes_read);
    }
    process_uart_events();

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

esp_err_t SERIAL_clear_buffer(void)
{
    if (!uart_is_driver_installed(UART_NUM_1)) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = uart_flush_input(UART_NUM_1);
    if (uart_event_queue != NULL) {
        xQueueReset(uart_event_queue);
    }
    return err;
}

void SERIAL_get_stats(serial_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }

    process_uart_events();
    update_rx_high_watermark();
    *stats = (serial_stats_t) {
        .tx_packets = (uint32_t)atomic_load(&tx_packets),
        .tx_bytes = (uint32_t)atomic_load(&tx_bytes),
        .tx_failures = (uint32_t)atomic_load(&tx_failures),
        .tx_partial_writes = (uint32_t)atomic_load(&tx_partial_writes),
        .rx_bytes = (uint32_t)atomic_load(&rx_bytes),
        .rx_failures = (uint32_t)atomic_load(&rx_failures),
        .rx_buffer_high_watermark = (uint32_t)atomic_load(&rx_buffer_high_watermark),
        .fifo_overflows = (uint32_t)atomic_load(&fifo_overflows),
        .buffer_full_events = (uint32_t)atomic_load(&buffer_full_events),
        .parity_errors = (uint32_t)atomic_load(&parity_errors),
        .frame_errors = (uint32_t)atomic_load(&frame_errors),
        .break_events = (uint32_t)atomic_load(&break_events),
        .baud_failures = (uint32_t)atomic_load(&baud_failures),
    };
}
