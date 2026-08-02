#include "Drivers/rs485_driver.h"

#include "config.h"
#include "runtime_settings.h"

#include "esp_log.h"
#include "freertos/task.h"

static uint32_t s_rs485Baud1 = RS485_DEFAULT_BAUDRATE;
static uint32_t s_rs485Baud2 = RS485_DEFAULT_BAUDRATE;

static uint32_t rs485BaudForPort(uint8_t port, const bridge_runtime_settings_t *settings)
{
    uint32_t baud = RS485_DEFAULT_BAUDRATE;
    bool haveBaud = false;

    if (settings == NULL) {
        return baud;
    }

    if ((settings->bms_line == LINE_RS485) && (settings->bms_port == port)) {
        baud = bridgeProtocolRs485Baudrate(settings->bms_protocol);
        haveBaud = true;
    }

    if (settings->dual_bms && settings->bms2_port == port) {
        uint32_t bms2Baud = bridgeProtocolRs485Baudrate(settings->bms2_protocol);
        if (haveBaud && bms2Baud != baud) {
            ESP_LOGW(EXAMPLE_TAG,
                     "RS485_%u requested by both BMS slots with mixed baud rates (%u and %u); using BMS 2",
                     (unsigned)port,
                     (unsigned)baud,
                     (unsigned)bms2Baud);
        }
        baud = bms2Baud;
        haveBaud = true;
    }

    if ((settings->inverter_line == LINE_RS485) && (settings->inverter_port == port)) {
        uint32_t inverterBaud = bridgeProtocolRs485Baudrate(settings->inverter_protocol);
        if (haveBaud && inverterBaud != baud) {
            ESP_LOGW(EXAMPLE_TAG,
                     "RS485_%u requested with mixed baud rates (%u and %u); using inverter side",
                     (unsigned)port,
                     (unsigned)baud,
                     (unsigned)inverterBaud);
        }
        baud = inverterBaud;
    }

    return baud;
}

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

static TickType_t rs485PortPreDelayTicks(uart_port_t uart)
{
    uint32_t delayMs = 0u;

    if (uart == RS485_1_UART) {
        delayMs = RS485_1_TX_PRE_DELAY_MS;
    } else if (uart == RS485_2_UART) {
        delayMs = RS485_2_TX_PRE_DELAY_MS;
    }

    return (delayMs > 0u) ? pdMS_TO_TICKS(delayMs) : 0;
}

static TickType_t rs485PortPostDelayTicks(uart_port_t uart)
{
    uint32_t delayMs = 0u;

    if (uart == RS485_1_UART) {
        delayMs = RS485_1_TX_POST_DELAY_MS;
    } else if (uart == RS485_2_UART) {
        delayMs = RS485_2_TX_POST_DELAY_MS;
    }

    return (delayMs > 0u) ? pdMS_TO_TICKS(delayMs) : 0;
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

uint32_t rs485GetBaudRate(uart_port_t uart)
{
    if (uart == RS485_1_UART) {
        return s_rs485Baud1;
    }
    if (uart == RS485_2_UART) {
        return s_rs485Baud2;
    }
    return RS485_DEFAULT_BAUDRATE;
}

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
        TickType_t preDelay = rs485PortPreDelayTicks(uart);
        rs485SetDirection(dirPin, true);
        if (preDelay > 0) {
            vTaskDelay(preDelay);
        }
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
        TickType_t postDelay = rs485PortPostDelayTicks(uart);
        if (postDelay > 0) {
            vTaskDelay(postDelay);
        }
        rs485SetDirection(dirPin, false);
    }
    return waitErr;
}

static void rs485SetupOne(uart_port_t uart, int txPin, int rxPin, int dirPin, uint32_t baudRate)
{
    bool useHalfDuplex = rs485PortUsesHalfDuplex(uart);

    uart_config_t uartConfig = {
        .baud_rate = (int)baudRate,
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
    bridge_runtime_settings_t settings = runtimeSettingsGet();

    if (uart_is_driver_installed(RS485_1_UART)) {
        (void)uart_driver_delete(RS485_1_UART);
    }
    if (uart_is_driver_installed(RS485_2_UART)) {
        (void)uart_driver_delete(RS485_2_UART);
    }

    s_rs485Baud1 = rs485BaudForPort(1u, &settings);
    s_rs485Baud2 = rs485BaudForPort(2u, &settings);

    rs485SetupOne(RS485_1_UART, RS485_1_TX, RS485_1_RX, RS485_1_DIR, s_rs485Baud1);
    rs485SetupOne(RS485_2_UART, RS485_2_TX, RS485_2_RX, RS485_2_DIR, s_rs485Baud2);

    ESP_LOGI(EXAMPLE_TAG,
             "RS485 interfaces reinitialized (RS485_1=%u 8N1, RS485_2=%u 8N1, hd1=%s hd2=%s)",
             (unsigned)s_rs485Baud1,
             (unsigned)s_rs485Baud2,
             RS485_1_USE_HALF_DUPLEX ? "ON" : "OFF",
             RS485_2_USE_HALF_DUPLEX ? "ON" : "OFF");
}

void rs485Init(void)
{
    rs485Reinit();
}
