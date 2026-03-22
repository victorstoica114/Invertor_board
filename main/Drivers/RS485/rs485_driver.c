#include "Drivers/RS485/rs485_driver.h"

#include "config.h"

#include "esp_log.h"

static void rs485ConfigurePort(uart_port_t uart,
                               int txPin,
                               int rxPin,
                               int dirPin,
                               bool useHalfDuplex,
                               bool dirTxLevel)
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
#if 1
    if (useHalfDuplex) {
        ESP_ERROR_CHECK(uart_set_pin(uart, txPin, rxPin, dirPin, UART_PIN_NO_CHANGE));
    } else {
        ESP_ERROR_CHECK(uart_set_pin(uart, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    }
#endif
    ESP_ERROR_CHECK(uart_driver_install(uart, RS485_BUF_SIZE, RS485_BUF_SIZE, 0, NULL, 0));
    if (useHalfDuplex) {
        ESP_ERROR_CHECK(uart_set_mode(uart, UART_MODE_RS485_HALF_DUPLEX));
    } else {
        gpio_reset_pin((gpio_num_t)dirPin);
        gpio_set_direction((gpio_num_t)dirPin, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)dirPin, dirTxLevel ? 0 : 1);
    }
}

uart_port_t rs485GetUart1(void)
{
    return RS485_1_UART;
}

uart_port_t rs485GetUart2(void)
{
    return RS485_2_UART;
}

gpio_num_t rs485GetDir1(void)
{
    return (gpio_num_t)RS485_1_DIR;
}

gpio_num_t rs485GetDir2(void)
{
    return (gpio_num_t)RS485_2_DIR;
}

void rs485DriverSetTx(gpio_num_t dirPin, bool txEnable)
{
    gpio_set_level(dirPin, txEnable ? 1 : 0);
}

void rs485DriverWriteFrame(uart_port_t uart, gpio_num_t dirPin, const uint8_t *frame, int len)
{
    if (frame == NULL || len <= 0) {
        return;
    }

    rs485DriverSetTx(dirPin, true);
    uart_write_bytes(uart, (const char *)frame, len);
    uart_wait_tx_done(uart, pdMS_TO_TICKS(100));
    rs485DriverSetTx(dirPin, false);
}

void rs485ResetPorts(void)
{
    (void)uart_flush_input(RS485_1_UART);
    (void)uart_flush_input(RS485_2_UART);
    rs485DriverSetTx((gpio_num_t)RS485_1_DIR, false);
    rs485DriverSetTx((gpio_num_t)RS485_2_DIR, false);
    ESP_LOGI(EXAMPLE_TAG, "RS485 ports flushed/reset");
}

void rs485Init(void)
{
    rs485ConfigurePort(RS485_1_UART,
                       RS485_1_TX,
                       RS485_1_RX,
                       RS485_1_DIR,
                       RS485_1_USE_HALF_DUPLEX,
                       RS485_1_DIR_TX_LEVEL);
    rs485ConfigurePort(RS485_2_UART,
                       RS485_2_TX,
                       RS485_2_RX,
                       RS485_2_DIR,
                       RS485_2_USE_HALF_DUPLEX,
                       RS485_2_DIR_TX_LEVEL);

    ESP_LOGI(EXAMPLE_TAG,
             "RS485_1 & RS485_2 initialized (%d 8N1, hd1=%s hd2=%s)",
             RS485_BAUDRATE,
             RS485_1_USE_HALF_DUPLEX ? "ON" : "OFF",
             RS485_2_USE_HALF_DUPLEX ? "ON" : "OFF");
}

void rs485Reinit(void)
{
    (void)uart_driver_delete(RS485_1_UART);
    (void)uart_driver_delete(RS485_2_UART);
    rs485Init();
    rs485ResetPorts();
    ESP_LOGI(EXAMPLE_TAG, "RS485 ports reinitialized");
}
