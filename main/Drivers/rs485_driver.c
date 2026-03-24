#include "Drivers/rs485_driver.h"

#include "config.h"

#include "esp_log.h"

uart_port_t rs485GetUart1(void) { return RS485_1_UART; }
uart_port_t rs485GetUart2(void) { return RS485_2_UART; }

gpio_num_t rs485GetDir1(void) { return (gpio_num_t)RS485_1_DIR; }
gpio_num_t rs485GetDir2(void) { return (gpio_num_t)RS485_2_DIR; }

void rs485SetDirection(gpio_num_t dirPin, bool txEnable)
{
    gpio_set_level(dirPin, txEnable ? 1 : 0);
}

esp_err_t rs485WriteBytes(uart_port_t uart,
                          gpio_num_t dirPin,
                          const uint8_t *data,
                          int len,
                          TickType_t txTimeoutTicks)
{
    if (data == NULL || len <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    rs485SetDirection(dirPin, true);
    int written = uart_write_bytes(uart, (const char *)data, len);
    if (written != len) {
        rs485SetDirection(dirPin, false);
        return ESP_FAIL;
    }

    esp_err_t waitErr = uart_wait_tx_done(uart, txTimeoutTicks);
    rs485SetDirection(dirPin, false);
    return waitErr;
}

void rs485Init(void)
{
    uart_config_t uartConfig = {
        .baud_rate = RS485_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* RS485_1 */
    ESP_ERROR_CHECK(uart_param_config(RS485_1_UART, &uartConfig));
    ESP_ERROR_CHECK(uart_set_pin(RS485_1_UART, RS485_1_TX, RS485_1_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(RS485_1_UART,
                                        RS485_BUF_SIZE, RS485_BUF_SIZE,
                                        0, NULL, 0));
    gpio_reset_pin(RS485_1_DIR);
    gpio_set_direction(RS485_1_DIR, GPIO_MODE_OUTPUT);
    gpio_set_level(RS485_1_DIR, 0); /* RX default */

    /* RS485_2 */
    ESP_ERROR_CHECK(uart_param_config(RS485_2_UART, &uartConfig));
    ESP_ERROR_CHECK(uart_set_pin(RS485_2_UART, RS485_2_TX, RS485_2_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(RS485_2_UART,
                                        RS485_BUF_SIZE, RS485_BUF_SIZE,
                                        0, NULL, 0));
    gpio_reset_pin(RS485_2_DIR);
    gpio_set_direction(RS485_2_DIR, GPIO_MODE_OUTPUT);
    gpio_set_level(RS485_2_DIR, 0); /* RX default */

    ESP_LOGI(EXAMPLE_TAG, "RS485_1 & RS485_2 initialized (%d 8N1)", RS485_BAUDRATE);
}
