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
    bool        bmsToInverterDir;
} rs485BridgeCtx_t;

static modbusDecoder_t gRsDec1;
static modbusDecoder_t gRsDec2;

static const char *canNameByPort(int port)
{
    return (port == 1) ? "CAN1" : "CAN2";
}

static twai_handle_t canBusByPort(int port)
{
    return (port == 1) ? canGetBus0() : canGetBus1();
}

static const char *rsNameByPort(int port)
{
    return (port == 1) ? "RS485_1" : "RS485_2";
}

static uart_port_t rsUartByPort(int port)
{
    return (port == 1) ? rs485GetUart1() : rs485GetUart2();
}

static gpio_num_t rsDirByPort(int port)
{
    return (port == 1) ? rs485GetDir1() : rs485GetDir2();
}

static modbusDecoder_t *rsDecoderByPort(int port)
{
    return (port == 1) ? &gRsDec1 : &gRsDec2;
}

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

typedef struct {
    const char *txName;
    uart_port_t txUart;
    gpio_num_t  txDirPin;
    uint8_t     slaveId;
    uint32_t    periodMs;
    modbusDecoder_t *targetDec;
} rs485BmsPollerCtx_t;

static void modbusBuildReadReq(uint8_t slave, uint16_t start, uint16_t count, uint8_t out[8])
{
    out[0] = slave;
    out[1] = 0x03u;
    out[2] = (uint8_t)((start >> 8) & 0xFFu);
    out[3] = (uint8_t)(start & 0xFFu);
    out[4] = (uint8_t)((count >> 8) & 0xFFu);
    out[5] = (uint8_t)(count & 0xFFu);

    uint16_t crc = modbusCrc16(out, 6);
    out[6] = (uint8_t)(crc & 0xFFu);
    out[7] = (uint8_t)((crc >> 8) & 0xFFu);
}

static void rs485SendRawFrame(uart_port_t txUart, gpio_num_t txDirPin, const uint8_t *frame, int len)
{
    rs485SetTx(txDirPin, true);
    uart_write_bytes(txUart, (const char *)frame, len);
    uart_wait_tx_done(txUart, pdMS_TO_TICKS(100));
    rs485SetTx(txDirPin, false);
}

static void rs485BmsPollerTask(void *pv)
{
    rs485BmsPollerCtx_t *ctx = (rs485BmsPollerCtx_t *)pv;
    static const struct {
        uint16_t start;
        uint16_t count;
        const char *name;
    } kPollReqs[] = {
        { GROWATT_MB_REG_MAIN_START, 0x001Bu, "main" },
        { GROWATT_MB_REG_CELL_BASE,  0x0011u, "cells" },
        { GROWATT_MB_REG_INFO_0001,  0x000Fu, "info" },
    };

    size_t reqIdx = 0;
    uint32_t sentCount = 0;

    while (1) {
        uint8_t req[8];
        const uint16_t start = kPollReqs[reqIdx].start;
        const uint16_t count = kPollReqs[reqIdx].count;

        int64_t nowUs = esp_timer_get_time();

        /* TX requests are not always visible on RX path; prime decoder context for upcoming response. */
        if (ctx->targetDec != NULL) {
            ctx->targetDec->lastReqValid = true;
            ctx->targetDec->lastReqSlave = ctx->slaveId;
            ctx->targetDec->lastReqFunc = 0x03u;
            ctx->targetDec->lastReqStart = start;
            ctx->targetDec->lastReqCount = count;
            ctx->targetDec->lastReqUs = nowUs;
        }
        modbusBuildReadReq(ctx->slaveId, start, count, req);
        rs485SendRawFrame(ctx->txUart, ctx->txDirPin, req, sizeof(req));

        sentCount++;
#if RS485_BMS_POLL_LOG_EVERY_N > 0
        if ((sentCount % RS485_BMS_POLL_LOG_EVERY_N) == 0u) {
            ESP_LOGI(EXAMPLE_TAG,
                     "RS485 poller TX on %s: slave=%u req=%s start=0x%04X count=0x%04X cnt=%lu",
                     ctx->txName,
                     (unsigned)ctx->slaveId,
                     kPollReqs[reqIdx].name,
                     (unsigned)start,
                     (unsigned)count,
                     (unsigned long)sentCount);
        }
#endif

        reqIdx = (reqIdx + 1u) % (sizeof(kPollReqs) / sizeof(kPollReqs[0]));
        vTaskDelay(pdMS_TO_TICKS(ctx->periodMs));
    }
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

    if (!ctx->bmsToInverterDir) {
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
    static canBridgeCtx_t bmsToInv;
    static canBridgeCtx_t invToBms;
    static canBridgeCtx_t bmsSniff;
    static canBridgeCtx_t invSniff;

    const bool bmsOnCan = (BMS_line == LINE_CAN);
    const bool invOnCan = (Inverter_line == LINE_CAN);

    if (bmsOnCan || invOnCan) {
        const char *snapIf = bmsOnCan ? canNameByPort(BMS_PORT) : canNameByPort(Inverter_PORT);
        xTaskCreate(canPeriodicSnapshotTask, "can_snapshot", 4096, (void *)snapIf, 7, NULL);
        ESP_LOGI(EXAMPLE_TAG,
                 "CAN periodic snapshot enabled (%d ms)",
                 CAN_DECODER_SNAPSHOT_PRINT_PERIOD_MS);
    }

    if (bmsOnCan && invOnCan) {
        bmsToInv.rxName = canNameByPort(BMS_PORT);
        bmsToInv.txName = canNameByPort(Inverter_PORT);
        bmsToInv.rxBus = canBusByPort(BMS_PORT);
        bmsToInv.txBus = canBusByPort(Inverter_PORT);
        bmsToInv.applyExcludeList = true;
        bmsToInv.forwardEnabled = CAN_FORWARD_ENABLE;

        invToBms.rxName = canNameByPort(Inverter_PORT);
        invToBms.txName = canNameByPort(BMS_PORT);
        invToBms.rxBus = canBusByPort(Inverter_PORT);
        invToBms.txBus = canBusByPort(BMS_PORT);
        invToBms.applyExcludeList = false;
        invToBms.forwardEnabled = CAN_FORWARD_ENABLE;

        xTaskCreate(canBridgeTask, "can_bms_to_inv", 4096, &bmsToInv, 10, NULL);
        xTaskCreate(canBridgeTask, "can_inv_to_bms", 4096, &invToBms, 10, NULL);

        ESP_LOGI(EXAMPLE_TAG,
                 "CAN bridge enabled (%s[P%d] <-> %s[P%d]), forward=%s",
                 bmsToInv.rxName,
                 BMS_PORT,
                 bmsToInv.txName,
                 Inverter_PORT,
                 CAN_FORWARD_ENABLE ? "ON" : "OFF");
        return;
    }

    if (bmsOnCan) {
        bmsSniff.rxName = canNameByPort(BMS_PORT);
        bmsSniff.txName = canNameByPort(BMS_PORT);
        bmsSniff.rxBus = canBusByPort(BMS_PORT);
        bmsSniff.txBus = canBusByPort(BMS_PORT);
        bmsSniff.applyExcludeList = false;
        bmsSniff.forwardEnabled = false;
        xTaskCreate(canBridgeTask, "can_bms_sniff", 4096, &bmsSniff, 10, NULL);
        ESP_LOGI(EXAMPLE_TAG, "CAN sniffer enabled on BMS side (%s[P%d])", bmsSniff.rxName, BMS_PORT);
    }

    if (invOnCan) {
        invSniff.rxName = canNameByPort(Inverter_PORT);
        invSniff.txName = canNameByPort(Inverter_PORT);
        invSniff.rxBus = canBusByPort(Inverter_PORT);
        invSniff.txBus = canBusByPort(Inverter_PORT);
        invSniff.applyExcludeList = false;
        invSniff.forwardEnabled = false;
        xTaskCreate(canBridgeTask, "can_inv_sniff", 4096, &invSniff, 10, NULL);
        ESP_LOGI(EXAMPLE_TAG, "CAN sniffer enabled on inverter side (%s[P%d])", invSniff.rxName, Inverter_PORT);
    }

    if (!bmsOnCan && !invOnCan) {
        ESP_LOGI(EXAMPLE_TAG, "CAN bridge/sniffer not used by current topology");
    }
}

void rs485BridgeEnable(void)
{
    static rs485BridgeCtx_t bmsToInv;
    static rs485BridgeCtx_t invToBms;
    static rs485BridgeCtx_t bmsSniff;
    static rs485BridgeCtx_t invSniff;
    static rs485BmsPollerCtx_t poller;

    const bool bmsOnRs = (BMS_line == LINE_RS485);
    const bool invOnRs = (Inverter_line == LINE_RS485);
    const bool inverseCanToRs = (BMS_line == LINE_CAN) &&
                              (Inverter_line == LINE_RS485) &&
                              (BMS_protocol == PROTOCOL_CAN_GROWATT) &&
                              (Inverter_protocol == PROTOCOL_RS485_GROWATT);

    xTaskCreate(rs485PeriodicSnapshotTask, "rs485_snapshot", 4096, NULL, 7, NULL);

    if (gRsDec1.ifName == NULL) {
        modbusDecoderInit(&gRsDec1, "RS485_1", 5000);
    }
    if (gRsDec2.ifName == NULL) {
        modbusDecoderInit(&gRsDec2, "RS485_2", 5000);
    }

    ESP_LOGI(EXAMPLE_TAG,
             "RS485 periodic snapshot enabled (%d ms)",
             MODBUS_DECODER_SNAPSHOT_PRINT_PERIOD_MS);
    ESP_LOGI(EXAMPLE_TAG,
             "RS485 reg exclude entries configured: %u",
             (unsigned)(g_rs485ForwardExcludeRegsCount));

    if (bmsOnRs && invOnRs) {
        bmsToInv.rxName = rsNameByPort(BMS_PORT);
        bmsToInv.txName = rsNameByPort(Inverter_PORT);
        bmsToInv.rxUart = rsUartByPort(BMS_PORT);
        bmsToInv.txUart = rsUartByPort(Inverter_PORT);
        bmsToInv.txDirPin = rsDirByPort(Inverter_PORT);
        bmsToInv.applyRegExcludeList = true;
        bmsToInv.forwardEnabled = RS485_FORWARD_ENABLE;
        bmsToInv.bmsToInverterDir = true;

        invToBms.rxName = rsNameByPort(Inverter_PORT);
        invToBms.txName = rsNameByPort(BMS_PORT);
        invToBms.rxUart = rsUartByPort(Inverter_PORT);
        invToBms.txUart = rsUartByPort(BMS_PORT);
        invToBms.txDirPin = rsDirByPort(BMS_PORT);
        invToBms.applyRegExcludeList = true;
        invToBms.forwardEnabled = RS485_FORWARD_ENABLE;
        invToBms.bmsToInverterDir = false;

        xTaskCreate(rs485BridgeTask, "rs485_bms_to_inv", 4096, &bmsToInv, 9, NULL);
        xTaskCreate(rs485BridgeTask, "rs485_inv_to_bms", 4096, &invToBms, 9, NULL);

        ESP_LOGI(EXAMPLE_TAG,
                 "RS485 bridge enabled (%s[P%d] <-> %s[P%d]), forward=%s",
                 bmsToInv.rxName,
                 BMS_PORT,
                 bmsToInv.txName,
                 Inverter_PORT,
                 RS485_FORWARD_ENABLE ? "ON" : "OFF");
    } else {
        if (bmsOnRs) {
            bmsSniff.rxName = rsNameByPort(BMS_PORT);
            bmsSniff.txName = rsNameByPort(BMS_PORT);
            bmsSniff.rxUart = rsUartByPort(BMS_PORT);
            bmsSniff.txUart = rsUartByPort(BMS_PORT);
            bmsSniff.txDirPin = rsDirByPort(BMS_PORT);
            bmsSniff.applyRegExcludeList = true;
            bmsSniff.forwardEnabled = false;
            bmsSniff.bmsToInverterDir = false;
            xTaskCreate(rs485BridgeTask, "rs485_bms_sniff", 4096, &bmsSniff, 9, NULL);
            ESP_LOGI(EXAMPLE_TAG, "RS485 sniffer enabled on BMS side (%s[P%d])", bmsSniff.rxName, BMS_PORT);
        }

        if (invOnRs && !inverseCanToRs) {
            invSniff.rxName = rsNameByPort(Inverter_PORT);
            invSniff.txName = rsNameByPort(Inverter_PORT);
            invSniff.rxUart = rsUartByPort(Inverter_PORT);
            invSniff.txUart = rsUartByPort(Inverter_PORT);
            invSniff.txDirPin = rsDirByPort(Inverter_PORT);
            invSniff.applyRegExcludeList = true;
            invSniff.forwardEnabled = false;
            invSniff.bmsToInverterDir = false;
            xTaskCreate(rs485BridgeTask, "rs485_inv_sniff", 4096, &invSniff, 9, NULL);
            ESP_LOGI(EXAMPLE_TAG, "RS485 sniffer enabled on inverter side (%s[P%d])", invSniff.rxName, Inverter_PORT);
        } else if (inverseCanToRs) {
            ESP_LOGI(EXAMPLE_TAG, "RS485 inverter side reserved for CAN->RS485 translator (%s[P%d])",
                     rsNameByPort(Inverter_PORT),
                     Inverter_PORT);
        }
    }

#if RS485_BMS_POLLER_ENABLE
    if ((BMS_line == LINE_RS485) && (Inverter_line == LINE_CAN)) {
#if RS485_FORWARD_ENABLE
        ESP_LOGW(EXAMPLE_TAG,
                 "RS485 BMS poller disabled because RS485 forward is ON (avoid collisions)");
#else
        poller.txName = rsNameByPort(BMS_PORT);
        poller.txUart = rsUartByPort(BMS_PORT);
        poller.txDirPin = rsDirByPort(BMS_PORT);
        poller.slaveId = (uint8_t)RS485_BMS_SLAVE_ID;
        poller.periodMs = RS485_BMS_POLL_PERIOD_MS;
        poller.targetDec = rsDecoderByPort(BMS_PORT);

        xTaskCreate(rs485BmsPollerTask, "rs485_bms_poller", 3072, &poller, 8, NULL);
        ESP_LOGI(EXAMPLE_TAG,
                 "RS485 BMS poller enabled (tx=%s slave=%u period=%ums)",
                 poller.txName,
                 (unsigned)poller.slaveId,
                 (unsigned)poller.periodMs);
#endif
    }
#endif

    if ((BMS_line == LINE_RS485) &&
        (Inverter_line == LINE_CAN) &&
        (BMS_protocol == PROTOCOL_RS485_GROWATT) &&
        (Inverter_protocol == PROTOCOL_CAN_GROWATT)) {
        rs485Can322BridgeEnable(rsDecoderByPort(BMS_PORT),
                                canBusByPort(Inverter_PORT),
                                canNameByPort(Inverter_PORT));
    } else if ((BMS_line == LINE_CAN) &&
               (Inverter_line == LINE_RS485) &&
               (BMS_protocol == PROTOCOL_CAN_GROWATT) &&
               (Inverter_protocol == PROTOCOL_RS485_GROWATT)) {
        canRs485SocBridgeEnable(rsUartByPort(Inverter_PORT),
                                rsDirByPort(Inverter_PORT),
                                rsNameByPort(Inverter_PORT));
    }
}
