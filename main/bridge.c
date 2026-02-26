#include "bridge.h"
#include "config.h"
#include "modbusDecoder.h"
#include "CAN_Decoder.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"


/* ---------- CAN bridge task ---------- */
typedef struct {
    const char   *rxName;
    const char   *txName;
    twai_handle_t rxBus;
    twai_handle_t txBus;
} canBridgeCtx_t;

static void canPeriodicSnapshotTask(void *pv)
{
    const char *ifname = (const char *)pv;

    vTaskDelay(pdMS_TO_TICKS(CAN_DECODER_SNAPSHOT_PRINT_PERIOD_MS));

    while (1) {
        canDecoderPrintCachedSnapshot(ifname);
        vTaskDelay(pdMS_TO_TICKS(CAN_DECODER_SNAPSHOT_PRINT_PERIOD_MS));
    }
}

static void canBridgeTask(void *pv)
{
    canBridgeCtx_t *ctx = (canBridgeCtx_t*)pv;
    twai_message_t rx;

    while (1) {
        if (twai_receive_v2(ctx->rxBus, &rx, portMAX_DELAY) == ESP_OK) {

#ifdef TWAI_MSG_FLAG_SELF
            if (rx.flags & TWAI_MSG_FLAG_SELF) {
                continue;
            }
#endif
            canDecoderOnFrame(ctx->rxName, &rx);

            esp_err_t e = twai_transmit_v2(ctx->txBus, &rx, pdMS_TO_TICKS(50));
            if (e != ESP_OK) {
                ESP_LOGW(EXAMPLE_TAG, "CAN forward %s -> %s failed (err=0x%x)",
                         ctx->rxName, ctx->txName, (unsigned)e);
            }
        }
    }
}

/* ---------- RS485 log (HEX only) ---------- */
static void logRs485Bytes(const char *ifname, const uint8_t *buf, int len)
{
    const int maxHexBytes = 64;
    int n = (len < maxHexBytes) ? len : maxHexBytes;

    char hex[3 * maxHexBytes + 1];
    int pos = 0;

    for (int i = 0; i < n; i++) {
        pos += snprintf(&hex[pos], sizeof(hex) - pos, "%02X ", buf[i]);
        if (pos >= (int)sizeof(hex)) break;
    }

    if (pos > 0) hex[pos - 1] = 0;
    else hex[0] = 0;

    if (len > maxHexBytes) {
        ESP_LOGI(EXAMPLE_TAG,
                 "RX on %s: len=%d HEX(first %d)=[%s] ...",
                 ifname, len, maxHexBytes, hex);
    } else {
        ESP_LOGI(EXAMPLE_TAG,
                 "RX on %s: len=%d HEX=[%s]",
                 ifname, len, hex);
    }
}

/* ---------- RS485 bridge ---------- */
typedef struct {
    const char *rxName;
    const char *txName;
    uart_port_t rxUart;
    uart_port_t txUart;
    gpio_num_t  txDirPin;
} rs485BridgeCtx_t;

static inline void rs485SetTx(gpio_num_t dirPin, bool txEn)
{
    gpio_set_level(dirPin, txEn ? 1 : 0); // 1=TX, 0=RX (cum ai avut)
}

static void rs485BridgeTask(void *pv)
{
    rs485BridgeCtx_t *ctx = (rs485BridgeCtx_t*)pv;
    uint8_t buf[RS485_BUF_SIZE];

    // Prag gap: 5000us (~5ms) e ok la 9600bps pt Modbus RTU
    static modbusDecoder_t dec1;
    static modbusDecoder_t dec2;
    modbusDecoder_t *dec = NULL;

    // alegem instanța după nume ca să nu alocăm dinamic
    if (strcmp(ctx->rxName, "RS485_1") == 0) {
        dec = &dec1;
        if (dec->ifName == NULL) modbusDecoderInit(dec, "RS485_1", 5000);
    } else {
        dec = &dec2;
        if (dec->ifName == NULL) modbusDecoderInit(dec, "RS485_2", 5000);
    }

    while (1) {
        // poți reduce la 5ms dacă vrei; decoderul oricum reface cadrele.
        int len = uart_read_bytes(ctx->rxUart, buf, RS485_BUF_SIZE, pdMS_TO_TICKS(5));
        if (len > 0) {
            int64_t nowUs = esp_timer_get_time();
            modbusDecoderFeed(dec, buf, len, nowUs);

            // forward ca înainte
            rs485SetTx(ctx->txDirPin, true);
            uart_write_bytes(ctx->txUart, (const char*)buf, len);
            uart_wait_tx_done(ctx->txUart, pdMS_TO_TICKS(100));
            rs485SetTx(ctx->txDirPin, false);
        } else {
            // dacă n-au venit bytes, verificăm dacă a trecut gap-ul și flush-uim
            if (dec->haveLastByte) {
                int64_t nowUs = esp_timer_get_time();
                if ((nowUs - dec->lastByteUs) > (int64_t)dec->gapUs) {
                    modbusDecoderFlush(dec);
                }
            }
        }
    }
}

/* ---------- Enable functions (create tasks) ---------- */
void canBridgeEnable(void)
{
    static canBridgeCtx_t can12;
    static canBridgeCtx_t can21;

    can12.rxName = "CAN1";
    can12.txName = "CAN2";
    can12.rxBus  = canGetBus0();
    can12.txBus  = canGetBus1();

    can21.rxName = "CAN2";
    can21.txName = "CAN1";
    can21.rxBus  = canGetBus1();
    can21.txBus  = canGetBus0();

    xTaskCreate(canBridgeTask, "can1_to_can2", 4096, &can12, 10, NULL);
    xTaskCreate(canBridgeTask, "can2_to_can1", 4096, &can21, 10, NULL);
    xTaskCreate(canPeriodicSnapshotTask, "can_snapshot", 4096, (void *)"CAN1", 7, NULL);

    ESP_LOGI(EXAMPLE_TAG, "CAN periodic snapshot enabled (%d ms)", CAN_DECODER_SNAPSHOT_PRINT_PERIOD_MS);
    ESP_LOGI(EXAMPLE_TAG, "CAN bridge enabled (CAN1<->CAN2)");
}

void rs485BridgeEnable(void)
{
    static rs485BridgeCtx_t rs12;
    static rs485BridgeCtx_t rs21;

    rs12.rxName   = "RS485_1";
    rs12.txName   = "RS485_2";
    rs12.rxUart   = rs485GetUart1();
    rs12.txUart   = rs485GetUart2();
    rs12.txDirPin = rs485GetDir2();

    rs21.rxName   = "RS485_2";
    rs21.txName   = "RS485_1";
    rs21.rxUart   = rs485GetUart2();
    rs21.txUart   = rs485GetUart1();
    rs21.txDirPin = rs485GetDir1();

    xTaskCreate(rs485BridgeTask, "rs485_1_to_2", 4096, &rs12, 9, NULL);
    xTaskCreate(rs485BridgeTask, "rs485_2_to_1", 4096, &rs21, 9, NULL);

    ESP_LOGI(EXAMPLE_TAG, "RS485 bridge enabled (RS485_1<->RS485_2)");
}
