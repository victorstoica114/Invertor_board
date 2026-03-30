#include "Drivers/rs485_driver.h"

#include "config.h"

#include "esp_log.h"

static bool rs485PortUsesHalfDuplex(uart_port_t uart)
{
    if (uart == RS485_1_UART) return RS485_1_USE_HALF_DUPLEX != 0;
    if (uart == RS485_2_UART) return RS485_2_USE_HALF_DUPLEX != 0;
    return RS485_USE_HALF_DUPLEX != 0;
}

static int rs485PortTxLevel(uart_port_t uart)
{
    if (uart == RS485_1_UART) return RS485_1_DIR_TX_LEVEL;
    if (uart == RS485_2_UART) return RS485_2_DIR_TX_LEVEL;
    return 1;
}

static int rs485DirTxLevelByPin(gpio_num_t dirPin)
{
    if (dirPin == (gpio_num_t)RS485_1_DIR) return RS485_1_DIR_TX_LEVEL;
    if (dirPin == (gpio_num_t)RS485_2_DIR) return RS485_2_DIR_TX_LEVEL;
    return 1;
}

uart_port_t rs485GetUart1(void) { return RS485_1_UART; }
uart_port_t rs485GetUart2(void) { return RS485_2_UART; }

gpio_num_t rs485GetDir1(void) { return (gpio_num_t)RS485_1_DIR; }
gpio_num_t rs485GetDir2(void) { return (gpio_num_t)RS485_2_DIR; }

void rs485SetDirection(gpio_num_t dirPin, bool txEnable)
{
    int txLevel = rs485DirTxLevelByPin(dirPin);
    int level = txEnable ? txLevel : (txLevel ? 0 : 1);
    gpio_set_level(dirPin, level);
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

    if (!rs485PortUsesHalfDuplex(uart)) {
        rs485SetDirection(dirPin, true);
    }
    int written = uart_write_bytes(uart, (const char *)data, len);
    if (written != len) {
        if (!rs485PortUsesHalfDuplex(uart)) {
            rs485SetDirection(dirPin, false);
        }
        return ESP_FAIL;
    }

    esp_err_t waitErr = uart_wait_tx_done(uart, txTimeoutTicks);
    if (!rs485PortUsesHalfDuplex(uart)) {
        rs485SetDirection(dirPin, false);
    }
    return waitErr;
}

static void rs485SetupOne(uart_port_t uart, int txPin, int rxPin, int dirPin)
{
    bool useHalfDuplex = rs485PortUsesHalfDuplex(uart);

    uart_config_t uartConfig = {
        .baud_rate = RS485_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(uart, &uartConfig));
    if (useHalfDuplex) {
        ESP_ERROR_CHECK(uart_set_pin(uart, txPin, rxPin, dirPin, UART_PIN_NO_CHANGE));
    } else {
        ESP_ERROR_CHECK(uart_set_pin(uart, txPin, rxPin,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    }
    ESP_ERROR_CHECK(uart_driver_install(uart,
                                        RS485_BUF_SIZE, RS485_BUF_SIZE,
                                        0, NULL, 0));
    if (useHalfDuplex) {
        ESP_ERROR_CHECK(uart_set_mode(uart, UART_MODE_RS485_HALF_DUPLEX));
    } else {
        gpio_reset_pin((gpio_num_t)dirPin);
        gpio_set_direction((gpio_num_t)dirPin, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)dirPin, rs485PortTxLevel(uart) ? 0 : 1);
    }
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

    ESP_LOGI(EXAMPLE_TAG,
             "RS485 interfaces reinitialized (%d 8N1, hd1=%s hd2=%s)",
             RS485_BAUDRATE,
             RS485_1_USE_HALF_DUPLEX ? "ON" : "OFF",
             RS485_2_USE_HALF_DUPLEX ? "ON" : "OFF");
}

void rs485Init(void)
{
    rs485Reinit();
}
