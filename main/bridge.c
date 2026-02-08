// bridge.c (integral)

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

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/twai.h"

#define TAG "SNIFFER_BRIDGE"

// ============ RS485 sniffer / optional forward ============

// Sniffer only by default (sa nu injectezi trafic pe linie).
// Pune 1 dacă vrei să forwardezi efectiv bytes între UART-uri.
#define RS485_FORWARD_ENABLE 1

typedef struct {
    const char  *ifName;
    uart_port_t  uartNum;
    gpio_num_t   dirPin;
} rs485If_t;

typedef struct {
    rs485If_t a;
    rs485If_t b;
} rs485BridgeCtx_t;

static rs485BridgeCtx_t gRs485Ctx;
static TaskHandle_t gRs485TaskHandle = NULL;

static inline void rs485SetDirTx(const rs485If_t *iface, bool txEnable)
{
    // DE/RE pe același pin (DIR): 1=TX, 0=RX
    gpio_set_level(iface->dirPin, txEnable ? 1 : 0);
}

static void rs485WriteBytes(const rs485If_t *dst, const uint8_t *data, int len)
{
#if RS485_FORWARD_ENABLE
    if (len <= 0) return;

    rs485SetDirTx(dst, true);
    uart_write_bytes(dst->uartNum, (const char *)data, len);
    // asigură drain TX; timeout suficient pentru 9600 (len mic)
    uart_wait_tx_done(dst->uartNum, pdMS_TO_TICKS(50));
    rs485SetDirTx(dst, false);
#else
    (void)dst; (void)data; (void)len;
#endif
}

static void rs485BridgeTask(void *arg)
{
    rs485BridgeCtx_t *ctx = (rs485BridgeCtx_t *)arg;

    modbusDecoder_t decA;
    modbusDecoder_t decB;

    // modbusDecoderInit(modbusDecoder_t*, ifName, baudrate)
    modbusDecoderInit(&decA, ctx->a.ifName, RS485_BAUDRATE);
    modbusDecoderInit(&decB, ctx->b.ifName, RS485_BAUDRATE);

    uint8_t bufA[256];
    uint8_t bufB[256];

    // Timeout mic: dacă nu vin bytes, flush => închide cadrul pe gap
    const TickType_t readTimeout = pdMS_TO_TICKS(20);

    while (1) {
        int nA = uart_read_bytes(ctx->a.uartNum, bufA, sizeof(bufA), readTimeout);
        if (nA > 0) {
            modbusDecoderFeed(&decA, bufA, nA);
            rs485WriteBytes(&ctx->b, bufA, nA);
        } else {
            modbusDecoderFlush(&decA);
        }

        int nB = uart_read_bytes(ctx->b.uartNum, bufB, sizeof(bufB), readTimeout);
        if (nB > 0) {
            modbusDecoderFeed(&decB, bufB, nB);
            rs485WriteBytes(&ctx->a, bufB, nB);
        } else {
            modbusDecoderFlush(&decB);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void rs485BridgeEnable(void)
{
    // IMPORTANT:
    // rs485Init() din config.c instalează deja driverul UART.
    // AICI nu mai chemăm uart_driver_install() deloc.

    gRs485Ctx.a.ifName  = "RS485_1";
    gRs485Ctx.a.uartNum = rs485GetUart1();
    gRs485Ctx.a.dirPin  = rs485GetDir1();

    gRs485Ctx.b.ifName  = "RS485_2";
    gRs485Ctx.b.uartNum = rs485GetUart2();
    gRs485Ctx.b.dirPin  = rs485GetDir2();

    // asigură DIR ca output + RX default (în caz că alt cod l-a lăsat aiurea)
    gpio_set_direction(gRs485Ctx.a.dirPin, GPIO_MODE_OUTPUT);
    gpio_set_direction(gRs485Ctx.b.dirPin, GPIO_MODE_OUTPUT);
    rs485SetDirTx(&gRs485Ctx.a, false);
    rs485SetDirTx(&gRs485Ctx.b, false);

    if (gRs485TaskHandle == NULL) {
        xTaskCreate(rs485BridgeTask, "rs485Bridge", 4096, &gRs485Ctx, 10, &gRs485TaskHandle);
    }

#if RS485_FORWARD_ENABLE
    ESP_LOGI(TAG, "RS485 bridge enabled (%s<->%s) [FORWARD=ON]", gRs485Ctx.a.ifName, gRs485Ctx.b.ifName);
#else
    ESP_LOGI(TAG, "RS485 sniffer enabled (%s & %s) [FORWARD=OFF]", gRs485Ctx.a.ifName, gRs485Ctx.b.ifName);
#endif
}

// ============ CAN bridge (TWAI v2 handles) ============

typedef struct {
    const char    *name;
    twai_handle_t  rx;
    twai_handle_t  tx;
} canLink_t;

static TaskHandle_t gCanTask1 = NULL;
static TaskHandle_t gCanTask2 = NULL;

static void canLinkTask(void *arg)
{
    canLink_t *lnk = (canLink_t *)arg;
    twai_message_t msg;

    while (1) {
        // v2 API: twai_receive_v2(handle, &msg, timeout)
        esp_err_t err = twai_receive_v2(lnk->rx, &msg, pdMS_TO_TICKS(1000));
        if (err == ESP_OK) {
            // forward
            (void)twai_transmit_v2(lnk->tx, &msg, pdMS_TO_TICKS(100));

            ESP_LOGI(TAG, "%s: id=0x%lx dlc=%u",
                     lnk->name, (unsigned long)msg.identifier, (unsigned)msg.data_length_code);
        }
    }
}

void canBridgeEnable(void)
{
    // Consistent cu canInit() din config.c (care folosește twai_driver_install_v2)
    twai_handle_t can1 = canGetBus0();
    twai_handle_t can2 = canGetBus1();

    static canLink_t link12;
    static canLink_t link21;

    link12.name = "CAN1->CAN2";
    link12.rx   = can1;
    link12.tx   = can2;

    link21.name = "CAN2->CAN1";
    link21.rx   = can2;
    link21.tx   = can1;

    if (gCanTask1 == NULL) {
        xTaskCreate(canLinkTask, "can12", 4096, &link12, 9, &gCanTask1);
    }
    if (gCanTask2 == NULL) {
        xTaskCreate(canLinkTask, "can21", 4096, &link21, 9, &gCanTask2);
    }

    ESP_LOGI(TAG, "CAN bridge enabled (TWAI v2: CAN1<->CAN2)");
}
