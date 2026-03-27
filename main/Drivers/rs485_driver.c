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

static void rs485SetupOne(uart_port_t uart, int txPin, int rxPin, int dirPin)
{
    uart_config_t uartConfig = {
        .baud_rate = RS485_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(uart, &uartConfig));
    ESP_ERROR_CHECK(uart_set_pin(uart, txPin, rxPin,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(uart,
                                        RS485_BUF_SIZE, RS485_BUF_SIZE,
                                        0, NULL, 0));
    gpio_reset_pin((gpio_num_t)dirPin);
    gpio_set_direction((gpio_num_t)dirPin, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)dirPin, 0); /* RX default */
}

void rs485Reinit(void)
{
    if (uart_is_driver_installed(RS485_1_UART)) {
        (void)uart_driver_delete(RS485_1_UART);
    }
    if (uart_is_driver_installed(RS485_2_UART)) {
        (void)uart_driver_delete(RS485_2_UART);
    }

    rs485SetupOne(RS485_1_UART, RS485_1_TX, RS485_1_RX, RS485_1_DIR);
    rs485SetupOne(RS485_2_UART, RS485_2_TX, RS485_2_RX, RS485_2_DIR);

    ESP_LOGI(EXAMPLE_TAG, "RS485 interfaces reinitialized (%d 8N1)", RS485_BAUDRATE);
}

void rs485Init(void)
{
    rs485Reinit();
}
