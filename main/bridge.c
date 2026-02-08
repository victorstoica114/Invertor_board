#include "bridge.h"
#include "config.h"
#include "modbusDecoder.h"
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/twai.h"

#define TAG "SNIFFER_BRIDGE"

// ===================== RS485 bridge (sniffer + forward) =====================

typedef struct {
    const char   *ifName;
    uart_port_t   uartNum;
    int           txPin;
    int           rxPin;
    int           dirPin;
    uint32_t      baudrate;
} rs485If_t;

typedef struct {
    rs485If_t a;
    rs485If_t b;
    bool      enableForward;
} rs485BridgeCtx_t;

static rs485BridgeCtx_t gRs485Ctx;
static TaskHandle_t     gRs485TaskHandle = NULL;

static inline void rs485SetDirTx(const rs485If_t *iface, bool txEnable)
{
    // DE/RE assumed tied together on DIR pin:
    // 1 = TX enabled, 0 = RX enabled
    gpio_set_level(iface->dirPin, txEnable ? 1 : 0);
}

static esp_err_t rs485UartInit(const rs485If_t *iface)
{
    uart_config_t cfg = {
        .baud_rate  = (int)iface->baudrate,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(iface->uartNum, 4096, 0, 0, NULL, 0), TAG, "uart_driver_install");
    ESP_RETURN_ON_ERROR(uart_param_config(iface->uartNum, &cfg), TAG, "uart_param_config");
    ESP_RETURN_ON_ERROR(uart_set_pin(iface->uartNum, iface->txPin, iface->rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "uart_set_pin");

    // DIR pin as output
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << iface->dirPin),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio_config(dir)");

    // start in RX mode
    rs485SetDirTx(iface, false);
    return ESP_OK;
}

static void rs485WriteFrame(const rs485If_t *dst, const uint8_t *data, int len)
{
    if (len <= 0) return;

    rs485SetDirTx(dst, true);
    uart_write_bytes(dst->uartNum, (const char *)data, len);
    uart_wait_tx_done(dst->uartNum, pdMS_TO_TICKS(50));
    rs485SetDirTx(dst, false);
}

static void rs485BridgeTask(void *arg)
{
    rs485BridgeCtx_t *ctx = (rs485BridgeCtx_t *)arg;

    modbusDecoder_t decA;
    modbusDecoder_t decB;

    // IMPORTANT: modbusDecoderInit expects baudrate as 3rd argument (per your header)
    modbusDecoderInit(&decA, ctx->a.ifName, ctx->a.baudrate);
    modbusDecoderInit(&decB, ctx->b.ifName, ctx->b.baudrate);

    uint8_t bufA[256];
    uint8_t bufB[256];

    // We use a short read timeout; if no data arrives we flush the decoder to close a frame on gap.
    const TickType_t readTimeout = pdMS_TO_TICKS(20);

    while (1) {
        int nA = uart_read_bytes(ctx->a.uartNum, bufA, sizeof(bufA), readTimeout);
        if (nA > 0) {
            // Feed decoder (3 args only, per your modbusDecoder.h)
            modbusDecoderFeed(&decA, bufA, nA);

            // Forward A->B if enabled
            if (ctx->enableForward) {
                rs485WriteFrame(&ctx->b, bufA, nA);
            }
        } else {
            // no bytes for a while => treat as inter-frame gap
            modbusDecoderFlush(&decA);
        }

        int nB = uart_read_bytes(ctx->b.uartNum, bufB, sizeof(bufB), readTimeout);
        if (nB > 0) {
            modbusDecoderFeed(&decB, bufB, nB);

            if (ctx->enableForward) {
                rs485WriteFrame(&ctx->a, bufB, nB);
            }
        } else {
            modbusDecoderFlush(&decB);
        }

        // yield
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Public API (must match bridge.h: void rs485BridgeEnable(void);)
void rs485BridgeEnable(void)
{
    // Configure from config.h/config.c – adapt these symbols to what you already have.
    // I’m assuming you have something like RS485_1_UART, RS485_1_TX, RS485_1_RX, RS485_1_DIR, RS485_BAUDRATE etc.
    gRs485Ctx.a.ifName   = "RS485_1";
    gRs485Ctx.a.uartNum  = RS485_1_UART;
    gRs485Ctx.a.txPin    = RS485_1_TX;
    gRs485Ctx.a.rxPin    = RS485_1_RX;
    gRs485Ctx.a.dirPin   = RS485_1_DIR;
    gRs485Ctx.a.baudrate = RS485_BAUDRATE;

    gRs485Ctx.b.ifName   = "RS485_2";
    gRs485Ctx.b.uartNum  = RS485_2_UART;
    gRs485Ctx.b.txPin    = RS485_2_TX;
    gRs485Ctx.b.rxPin    = RS485_2_RX;
    gRs485Ctx.b.dirPin   = RS485_2_DIR;
    gRs485Ctx.b.baudrate = RS485_BAUDRATE;

    gRs485Ctx.enableForward = true;

    ESP_ERROR_CHECK(rs485UartInit(&gRs485Ctx.a));
    ESP_ERROR_CHECK(rs485UartInit(&gRs485Ctx.b));

    if (gRs485TaskHandle == NULL) {
        xTaskCreate(rs485BridgeTask, "rs485Bridge", 4096, &gRs485Ctx, 10, &gRs485TaskHandle);
    }

    ESP_LOGI(TAG, "RS485 bridge enabled (%s<->%s)", gRs485Ctx.a.ifName, gRs485Ctx.b.ifName);
}

// ===================== CAN bridge =====================

typedef struct {
    bool enableForward;
} canBridgeCtx_t;

static canBridgeCtx_t gCanCtx;
static TaskHandle_t   gCanTaskHandle = NULL;

static void canBridgeTask(void *arg)
{
    (void)arg;

    twai_message_t msg;

    while (1) {
        // IDF 5.5 signature: twai_receive(twai_message_t *message, TickType_t ticks_to_wait)
        esp_err_t err = twai_receive(&msg, pdMS_TO_TICKS(1000));
        if (err == ESP_OK) {
            // If you want to forward between two controllers using handles, that’s not the legacy twai API.
            // With legacy twai driver, there is ONE controller instance.
            // So here we only log; bridging CAN1<->CAN2 requires two TWAI instances or a different driver layer.
            // Keep it safe: just log RX for now.
            ESP_LOGI(TAG, "CAN RX: id=0x%lx dlc=%d", (unsigned long)msg.identifier, msg.data_length_code);
        }
    }
}

void canBridgeEnable(void)
{
    // NOTE: With the classic TWAI driver in IDF, you can only run one TWAI peripheral instance.
    // If your board truly has two CAN controllers, you need two separate drivers/instances; that’s not this API.
    // For now, we start the one configured in canInit() and just run RX logging task.

    gCanCtx.enableForward = true;

    if (gCanTaskHandle == NULL) {
        xTaskCreate(canBridgeTask, "canBridge", 4096, &gCanCtx, 9, &gCanTaskHandle);
    }

    ESP_LOGI(TAG, "CAN bridge enabled (legacy TWAI: RX task running)");
}
