#include "bridge.h"
#include "config.h"
#include "modbusDecoder.h"
#include "CAN_Decoder.h"
#include "rs485_can_bridge.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define RS485_VERBOSE_LOGI(...) do { if (RS485_FORWARD_VERBOSE_LOGS) { ESP_LOGI(EXAMPLE_TAG, __VA_ARGS__); } } while (0)


static bool canIdExcludedToInverter(uint32_t id)
{
    for (size_t i = 0; i < (g_can1ToCan2ExcludeIdsCount); i++) {
        if (g_can1ToCan2ExcludeIds[i] == id) {
            return true;
        }
    }
    return false;
}

/* ---------- CAN bridge task ---------- */
typedef struct {
    const char   *rxName;
    const char   *txName;
    twai_handle_t rxBus;
    twai_handle_t txBus;
    bool          applyExcludeList;
    bool          forwardEnabled;
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
    canBridgeCtx_t *ctx = (canBridgeCtx_t *)pv;
    twai_message_t rx;

    while (1) {
        if (twai_receive_v2(ctx->rxBus, &rx, portMAX_DELAY) == ESP_OK) {
#ifdef TWAI_MSG_FLAG_SELF
            if (rx.flags & TWAI_MSG_FLAG_SELF) {
                continue;
            }
#endif
            canDecoderOnFrame(ctx->rxName, &rx);

            if (!ctx->forwardEnabled) {
                continue;
            }
            if (ctx->applyExcludeList && canIdExcludedToInverter((uint32_t)rx.identifier)) {
                continue;
            }

            esp_err_t e = twai_transmit_v2(ctx->txBus, &rx, pdMS_TO_TICKS(50));
            if (e != ESP_OK) {
                ESP_LOGW(EXAMPLE_TAG,
                         "CAN forward %s -> %s failed (err=0x%x)",
                         ctx->rxName,
                         ctx->txName,
                         (unsigned)e);
            }
        }
    }
}

/* ---------- RS485 bridge ---------- */

typedef struct {
    const char *rxName;
    const char *txName;
    uart_port_t rxUart;
    uart_port_t txUart;
    gpio_num_t  txDirPin;
    bool        applyRegExcludeList;
    bool        forwardEnabled;
} rs485BridgeCtx_t;

static modbusDecoder_t gRsDec1;
static modbusDecoder_t gRsDec2;

static inline void rs485SetTx(gpio_num_t dirPin, bool txEn)
{
    gpio_set_level(dirPin, txEn ? 1 : 0);
}

static uint16_t modbusBe16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint16_t modbusCrc16(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFFu;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}

static bool modbusCheckCrc(const uint8_t *frame, int len)
{
    if (len < 4) {
        return false;
    }
    uint16_t got = (uint16_t)(frame[len - 2] | (frame[len - 1] << 8));
    uint16_t calc = modbusCrc16(frame, len - 2);
    return got == calc;
}

static bool modbusParseRequestRange(const uint8_t *frame,
                                    int len,
                                    uint8_t *funcOut,
                                    uint16_t *startOut,
                                    uint16_t *countOut)
{
    if (len < 8 || !modbusCheckCrc(frame, len)) {
        return false;
    }

    const uint8_t func = frame[1];
    uint16_t start = 0;
    uint16_t count = 0;

    switch (func) {
        case 0x03:
        case 0x04:
            if (len != 8) {
                return false;
            }
            start = modbusBe16(&frame[2]);
            count = modbusBe16(&frame[4]);
            break;
        case 0x06:
            if (len != 8) {
                return false;
            }
            start = modbusBe16(&frame[2]);
            count = 1;
            break;
        case 0x10: {
            if (len < 9) {
                return false;
            }
            uint16_t qty = modbusBe16(&frame[4]);
            uint8_t byteCount = frame[6];
            if (len != (int)(7 + byteCount + 2)) {
                return false;
            }
            start = modbusBe16(&frame[2]);
            count = qty;
            break;
        }
        default:
            return false;
    }

    if (count == 0) {
        return false;
    }

    if (funcOut) {
        *funcOut = func;
    }
    if (startOut) {
        *startOut = start;
    }
    if (countOut) {
        *countOut = count;
    }
    return true;
}

static bool modbusIsExcludedRegister(uint16_t reg)
{
    for (size_t i = 0; i < (g_rs485ForwardExcludeRegsCount); i++) {
        if (g_rs485ForwardExcludeRegs[i] == reg) {
            return true;
        }
    }
    return false;
}

static bool modbusHasExcludedRegisterInRange(uint16_t start, uint16_t count)
{
    uint32_t reqStart = start;
    uint32_t reqEnd = reqStart + (uint32_t)count - 1u;
    if (reqEnd > 0xFFFFu) {
        reqEnd = 0xFFFFu;
    }

    for (size_t i = 0; i < (g_rs485ForwardExcludeRegsCount); i++) {
        uint32_t reg = g_rs485ForwardExcludeRegs[i];
        if (reg >= reqStart && reg <= reqEnd) {
            return true;
        }
    }

    return false;
}

static bool modbusFrameExcluded(const uint8_t *frame,
                                int len,
                                uint8_t *funcOut,
                                uint16_t *startOut,
                                uint16_t *countOut)
{
    uint8_t func = 0;
    uint16_t start = 0;
    uint16_t count = 0;

    if (!modbusParseRequestRange(frame, len, &func, &start, &count)) {
        return false;
    }

    /* For read requests we sanitize response payload instead of dropping whole ranges. */
    if (func == 0x03 || func == 0x04) {
        return false;
    }

    if (!modbusHasExcludedRegisterInRange(start, count)) {
        return false;
    }

    if (funcOut) {
        *funcOut = func;
    }
    if (startOut) {
        *startOut = start;
    }
    if (countOut) {
        *countOut = count;
    }
    return true;
}

static modbusDecoder_t *rs485PeerDecoderForRx(const rs485BridgeCtx_t *ctx)
{
    if (ctx == NULL || ctx->rxName == NULL) {
        return NULL;
    }

    if (strcmp(ctx->rxName, "RS485_2") == 0) {
        return &gRsDec1;
    }
    if (strcmp(ctx->rxName, "RS485_1") == 0) {
        return &gRsDec2;
    }
    return NULL;
}

static void rs485MirrorRequestToPeerDecoder(const rs485BridgeCtx_t *ctx,
                                            const uint8_t *frame,
                                            int len)
{
    uint8_t func = 0;
    uint16_t start = 0;
    uint16_t count = 0;

    if (!modbusParseRequestRange(frame, len, &func, &start, &count)) {
        return;
    }

    modbusDecoder_t *peer = rs485PeerDecoderForRx(ctx);
    if (peer == NULL || peer->ifName == NULL) {
        return;
    }

    peer->lastReqValid = true;
    peer->lastReqSlave = frame[0];
    peer->lastReqFunc = func;
    peer->lastReqStart = start;
    peer->lastReqCount = count;
    peer->lastReqUs = esp_timer_get_time();
}

static bool rs485SanitizeResponseToInverter(const rs485BridgeCtx_t *ctx,
                                            uint8_t *frame,
                                            int len)
{
    if (ctx == NULL || frame == NULL || len < 5) {
        return false;
    }

    if (!ctx->applyRegExcludeList) {
        return false;
    }

    if (strcmp(ctx->rxName, "RS485_1") != 0 || strcmp(ctx->txName, "RS485_2") != 0) {
        return false;
    }

    if (!modbusCheckCrc(frame, len)) {
        return false;
    }

    const uint8_t slave = frame[0];
    const uint8_t func = frame[1];
    if (func != 0x03 && func != 0x04) {
        return false;
    }

    const uint8_t byteCount = frame[2];
    if ((byteCount & 0x01u) != 0u || len != (int)(byteCount + 5u)) {
        return false;
    }

    modbusDecoder_t *peer = rs485PeerDecoderForRx(ctx);
    if (peer == NULL || !peer->lastReqValid) {
        return false;
    }
    if (peer->lastReqSlave != slave || peer->lastReqFunc != func) {
        return false;
    }

    const uint16_t regCount = (uint16_t)(byteCount / 2u);
    uint16_t startReg = peer->lastReqStart;

    /* Guard against mismatched reply size; still sanitize the overlapping part. */
    uint16_t maxCount = regCount;
    if (peer->lastReqCount > 0 && peer->lastReqCount < maxCount) {
        maxCount = peer->lastReqCount;
    }

    uint16_t maskedCount = 0;
    for (uint16_t i = 0; i < maxCount; i++) {
        uint16_t reg = (uint16_t)(startReg + i);
        if (!modbusIsExcludedRegister(reg)) {
            continue;
        }

        int dataPos = 3 + (int)(i * 2u);
        frame[dataPos] = 0x00u;
        frame[dataPos + 1] = 0x00u;
        maskedCount++;
    }

    if (maskedCount == 0) {
        return false;
    }

    uint16_t crc = modbusCrc16(frame, len - 2);
    frame[len - 2] = (uint8_t)(crc & 0xFFu);
    frame[len - 1] = (uint8_t)((crc >> 8) & 0xFFu);

    RS485_VERBOSE_LOGI("RS485 sanitized RESP %s -> %s: masked %u regs from start=0x%04X",
             ctx->rxName,
             ctx->txName,
             (unsigned)maskedCount,
             (unsigned)startReg);

    return true;
}
static void rs485ForwardFrame(rs485BridgeCtx_t *ctx, const uint8_t *frame, int len)
{
    if (!ctx->forwardEnabled) {
        return;
    }

    uint8_t func = 0;
    uint16_t start = 0;
    uint16_t count = 0;
    uint8_t txFrame[256];
    const uint8_t *txPtr = frame;

    if (ctx->applyRegExcludeList && modbusFrameExcluded(frame, len, &func, &start, &count)) {
        RS485_VERBOSE_LOGI("RS485 forward %s -> %s dropped req func=0x%02X start=0x%04X count=0x%04X (excluded)",
                 ctx->rxName,
                 ctx->txName,
                 (unsigned)func,
                 (unsigned)start,
                 (unsigned)count);
        return;
    }

    rs485MirrorRequestToPeerDecoder(ctx, frame, len);

    if (len > 0 && (size_t)len <= sizeof(txFrame)) {
        memcpy(txFrame, frame, (size_t)len);
        rs485SanitizeResponseToInverter(ctx, txFrame, len);
        txPtr = txFrame;
    }

    rs485SetTx(ctx->txDirPin, true);
    uart_write_bytes(ctx->txUart, (const char *)txPtr, len);
    uart_wait_tx_done(ctx->txUart, pdMS_TO_TICKS(100));
    rs485SetTx(ctx->txDirPin, false);
}

static void rs485PeriodicSnapshotTask(void *pv)
{
    (void)pv;

    vTaskDelay(pdMS_TO_TICKS(MODBUS_DECODER_SNAPSHOT_PRINT_PERIOD_MS));
    while (1) {
        if (gRsDec1.ifName != NULL) {
            modbusDecoderPrintSnapshot(&gRsDec1);
        }
        if (gRsDec2.ifName != NULL) {
            modbusDecoderPrintSnapshot(&gRsDec2);
        }
        vTaskDelay(pdMS_TO_TICKS(MODBUS_DECODER_SNAPSHOT_PRINT_PERIOD_MS));
    }
}

static void rs485BridgeTask(void *pv)
{
    rs485BridgeCtx_t *ctx = (rs485BridgeCtx_t *)pv;
    uint8_t rxChunk[RS485_BUF_SIZE];
    uint8_t frameBuf[256];
    uint16_t frameLen = 0;
    bool frameHaveLastByte = false;
    int64_t frameLastByteUs = 0;

    modbusDecoder_t *dec = NULL;
    if (strcmp(ctx->rxName, "RS485_1") == 0) {
        dec = &gRsDec1;
        if (dec->ifName == NULL) {
            modbusDecoderInit(dec, "RS485_1", 5000);
        }
    } else {
        dec = &gRsDec2;
        if (dec->ifName == NULL) {
            modbusDecoderInit(dec, "RS485_2", 5000);
        }
    }

    while (1) {
        int len = uart_read_bytes(ctx->rxUart, rxChunk, RS485_BUF_SIZE, pdMS_TO_TICKS(5));
        int64_t nowUs = esp_timer_get_time();

        if (len > 0) {
            if (frameHaveLastByte && ((nowUs - frameLastByteUs) > (int64_t)dec->gapUs)) {
                if (frameLen > 0) {
                    rs485ForwardFrame(ctx, frameBuf, frameLen);
                    frameLen = 0;
                }
                frameHaveLastByte = false;
            }

            if ((size_t)frameLen + (size_t)len > sizeof(frameBuf)) {
                ESP_LOGW(EXAMPLE_TAG,
                         "RS485 frame overflow on %s, dropping buffered frame",
                         ctx->rxName);
                frameLen = 0;
                frameHaveLastByte = false;
            }

            if ((size_t)frameLen + (size_t)len <= sizeof(frameBuf)) {
                memcpy(&frameBuf[frameLen], rxChunk, (size_t)len);
                frameLen = (uint16_t)(frameLen + len);
                frameLastByteUs = nowUs;
                frameHaveLastByte = true;
            }

            modbusDecoderFeed(dec, rxChunk, len, nowUs);
        } else {
            if (dec->haveLastByte && ((nowUs - dec->lastByteUs) > (int64_t)dec->gapUs)) {
                modbusDecoderFlush(dec);
            }

            if (frameHaveLastByte && ((nowUs - frameLastByteUs) > (int64_t)dec->gapUs)) {
                if (frameLen > 0) {
                    rs485ForwardFrame(ctx, frameBuf, frameLen);
                    frameLen = 0;
                }
                frameHaveLastByte = false;
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
    can12.rxBus = canGetBus0();
    can12.txBus = canGetBus1();
    can12.applyExcludeList = true;
    can12.forwardEnabled = CAN_FORWARD_ENABLE;

    can21.rxName = "CAN2";
    can21.txName = "CAN1";
    can21.rxBus = canGetBus1();
    can21.txBus = canGetBus0();
    can21.applyExcludeList = false;
    can21.forwardEnabled = CAN_FORWARD_ENABLE;

    xTaskCreate(canBridgeTask, "can1_to_can2", 4096, &can12, 10, NULL);
    xTaskCreate(canBridgeTask, "can2_to_can1", 4096, &can21, 10, NULL);
    xTaskCreate(canPeriodicSnapshotTask, "can_snapshot", 4096, (void *)"CAN1", 7, NULL);

    ESP_LOGI(EXAMPLE_TAG,
             "CAN periodic snapshot enabled (%d ms)",
             CAN_DECODER_SNAPSHOT_PRINT_PERIOD_MS);
    ESP_LOGI(EXAMPLE_TAG,
             "CAN bridge enabled (CAN1<->CAN2), forward=%s",
             CAN_FORWARD_ENABLE ? "ON" : "OFF");
}

void rs485BridgeEnable(void)
{
    static rs485BridgeCtx_t rs12;
    static rs485BridgeCtx_t rs21;

    rs12.rxName = "RS485_1";
    rs12.txName = "RS485_2";
    rs12.rxUart = rs485GetUart1();
    rs12.txUart = rs485GetUart2();
    rs12.txDirPin = rs485GetDir2();
    rs12.applyRegExcludeList = true;
    rs12.forwardEnabled = RS485_FORWARD_ENABLE;

    rs21.rxName = "RS485_2";
    rs21.txName = "RS485_1";
    rs21.rxUart = rs485GetUart2();
    rs21.txUart = rs485GetUart1();
    rs21.txDirPin = rs485GetDir1();
    rs21.applyRegExcludeList = true;
    rs21.forwardEnabled = RS485_FORWARD_ENABLE;

    xTaskCreate(rs485BridgeTask, "rs485_1_to_2", 4096, &rs12, 9, NULL);
    xTaskCreate(rs485BridgeTask, "rs485_2_to_1", 4096, &rs21, 9, NULL);
    xTaskCreate(rs485PeriodicSnapshotTask, "rs485_snapshot", 4096, NULL, 7, NULL);

    ESP_LOGI(EXAMPLE_TAG,
             "RS485 periodic snapshot enabled (%d ms)",
             MODBUS_DECODER_SNAPSHOT_PRINT_PERIOD_MS);
    ESP_LOGI(EXAMPLE_TAG,
             "RS485 reg exclude entries configured: %u",
             (unsigned)(g_rs485ForwardExcludeRegsCount));
    ESP_LOGI(EXAMPLE_TAG,
             "RS485 bridge enabled (RS485_1<->RS485_2), forward=%s",
             RS485_FORWARD_ENABLE ? "ON" : "OFF");

    rs485Can322BridgeEnable(&gRsDec1, canGetBus1(), "CAN2");
}




