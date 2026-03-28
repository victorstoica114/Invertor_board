#include "Operation_Modes/rs485_forward_sniffer.h"

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "Drivers/RS485/rs485_driver.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define RS485_VERBOSE_LOGI(...) do { if (RS485_FORWARD_VERBOSE_LOGS) { ESP_LOGI(EXAMPLE_TAG, __VA_ARGS__); } } while (0)

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

typedef struct {
    const char *txName;
    uart_port_t txUart;
    gpio_num_t  txDirPin;
    uint8_t     slaveId;
    uint32_t    periodMs;
    modbusDecoder_t *targetDec;
} rs485BmsPollerCtx_t;

static TaskHandle_t s_rs485TaskA = NULL;
static TaskHandle_t s_rs485TaskB = NULL;
static TaskHandle_t s_rs485SnapshotTask = NULL;
static TaskHandle_t s_rs485PollerTask = NULL;

static bool createRsTask(TaskFunction_t fn,
                         const char *name,
                         uint32_t stack,
                         void *arg,
                         UBaseType_t prio,
                         TaskHandle_t *outHandle)
{
    BaseType_t rc = xTaskCreate(fn, name, stack, arg, prio, outHandle);
    if (rc != pdPASS) {
        ESP_LOGE(EXAMPLE_TAG, "xTaskCreate failed for %s", name);
        if (outHandle != NULL) {
            *outHandle = NULL;
        }
        return false;
    }
    return true;
}
static modbusDecoder_t gRsDec1;
static modbusDecoder_t gRsDec2;

static void deleteTaskIfRunning(TaskHandle_t *handle)
{
    if (handle != NULL && *handle != NULL) {
        vTaskDelete(*handle);
        *handle = NULL;
    }
}

modbusDecoder_t *rs485ForwardSnifferGetDecoder(int port)
{
    return (port == 1) ? &gRsDec1 : &gRsDec2;
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
            crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

static bool modbusCheckCrc(const uint8_t *frame, int len)
{
    if (len < 4) {
        return false;
    }
    return (uint16_t)(frame[len - 2] | (frame[len - 1] << 8)) == modbusCrc16(frame, len - 2);
}

static void modbusBuildReadReq(uint8_t slave, uint16_t start, uint16_t count, uint8_t out[8])
{
    out[0] = slave;
    out[1] = 0x03u;
    out[2] = (uint8_t)((start >> 8) & 0xFFu);
    out[3] = (uint8_t)(start & 0xFFu);
    out[4] = (uint8_t)((count >> 8) & 0xFFu);
    out[5] = (uint8_t)(count & 0xFFu);

    {
        uint16_t crc = modbusCrc16(out, 6);
        out[6] = (uint8_t)(crc & 0xFFu);
        out[7] = (uint8_t)((crc >> 8) & 0xFFu);
    }
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
    uint32_t silentCount = 0;
    const TickType_t responseWaitTicks = pdMS_TO_TICKS(120);

    ESP_LOGI(EXAMPLE_TAG,
             "RS485 poller task running on %s slave=%u period=%ums",
             ctx->txName,
             (unsigned)ctx->slaveId,
             (unsigned)ctx->periodMs);

    while (1) {
        uint8_t req[8];
        const uint16_t start = kPollReqs[reqIdx].start;
        const uint16_t count = kPollReqs[reqIdx].count;
        int64_t nowUs = esp_timer_get_time();

        if (ctx->targetDec != NULL) {
            ctx->targetDec->lastReqValid = true;
            ctx->targetDec->lastReqSlave = ctx->slaveId;
            ctx->targetDec->lastReqFunc = 0x03u;
            ctx->targetDec->lastReqStart = start;
            ctx->targetDec->lastReqCount = count;
            ctx->targetDec->lastReqUs = nowUs;
        }

        modbusBuildReadReq(ctx->slaveId, start, count, req);
        rs485DriverWriteFrame(ctx->txUart, ctx->txDirPin, req, sizeof(req));

        sentCount++;
#if RS485_BMS_POLL_LOG_EVERY_N > 0
        if (sentCount <= 3u || (sentCount % RS485_BMS_POLL_LOG_EVERY_N) == 0u) {
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

        vTaskDelay(responseWaitTicks);

        if (ctx->targetDec != NULL) {
            if (ctx->targetDec->lastByteUs <= nowUs) {
                silentCount++;
                if (silentCount <= 5u || (silentCount % 10u) == 0u) {
                    ESP_LOGW(EXAMPLE_TAG,
                             "RS485 poller no response on %s: slave=%u req=%s start=0x%04X count=0x%04X silent=%lu",
                             ctx->txName,
                             (unsigned)ctx->slaveId,
                             kPollReqs[(reqIdx + (sizeof(kPollReqs) / sizeof(kPollReqs[0])) - 1u) % (sizeof(kPollReqs) / sizeof(kPollReqs[0]))].name,
                             (unsigned)start,
                             (unsigned)count,
                             (unsigned long)silentCount);
                }
            } else {
                silentCount = 0;
                if (sentCount <= 3u) {
                    ESP_LOGI(EXAMPLE_TAG,
                             "RS485 poller saw RX activity on %s after req=%s",
                             ctx->txName,
                             kPollReqs[(reqIdx + (sizeof(kPollReqs) / sizeof(kPollReqs[0])) - 1u) % (sizeof(kPollReqs) / sizeof(kPollReqs[0]))].name);
                }
            }
        }

        {
            TickType_t periodTicks = pdMS_TO_TICKS(ctx->periodMs);
            if (periodTicks > responseWaitTicks) {
                vTaskDelay(periodTicks - responseWaitTicks);
            }
        }
    }
}

static bool modbusParseRequestRange(const uint8_t *frame,
                                    int len,
                                    uint8_t *funcOut,
                                    uint16_t *startOut,
                                    uint16_t *countOut)
{
    uint8_t func = 0;
    uint16_t start = 0;
    uint16_t count = 0;

    if (len < 8 || !modbusCheckCrc(frame, len)) {
        return false;
    }

    func = frame[1];
    switch (func) {
        case 0x03:
        case 0x04:
        case 0x06:
            if (len != 8) {
                return false;
            }
            start = modbusBe16(&frame[2]);
            count = (func == 0x06) ? 1u : modbusBe16(&frame[4]);
            break;
        case 0x10: {
            uint16_t qty = 0;
            uint8_t byteCount = 0;
            if (len < 9) {
                return false;
            }
            qty = modbusBe16(&frame[4]);
            byteCount = frame[6];
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
    for (size_t i = 0; i < g_rs485ForwardExcludeRegsCount; i++) {
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

    for (size_t i = 0; i < g_rs485ForwardExcludeRegsCount; i++) {
        uint32_t reg = g_rs485ForwardExcludeRegs[i];
        if (reg >= reqStart && reg <= reqEnd) {
            return true;
        }
    }
    return false;
}

static bool modbusFrameExcluded(const uint8_t *frame, int len, uint8_t *funcOut, uint16_t *startOut, uint16_t *countOut)
{
    uint8_t func = 0;
    uint16_t start = 0;
    uint16_t count = 0;

    if (!modbusParseRequestRange(frame, len, &func, &start, &count)) {
        return false;
    }
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

static void rs485MirrorRequestToPeerDecoder(const rs485BridgeCtx_t *ctx, const uint8_t *frame, int len)
{
    uint8_t func = 0;
    uint16_t start = 0;
    uint16_t count = 0;
    modbusDecoder_t *peer = NULL;

    if (!modbusParseRequestRange(frame, len, &func, &start, &count)) {
        return;
    }

    peer = rs485PeerDecoderForRx(ctx);
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

static bool rs485SanitizeResponseToInverter(const rs485BridgeCtx_t *ctx, uint8_t *frame, int len)
{
    uint8_t slave = 0;
    uint8_t func = 0;
    uint8_t byteCount = 0;
    modbusDecoder_t *peer = NULL;
    uint16_t regCount = 0;
    uint16_t maxCount = 0;
    uint16_t startReg = 0;
    uint16_t maskedCount = 0;

    if (ctx == NULL || frame == NULL || len < 5 || !ctx->applyRegExcludeList || !ctx->bmsToInverterDir || !modbusCheckCrc(frame, len)) {
        return false;
    }

    slave = frame[0];
    func = frame[1];
    if (func != 0x03 && func != 0x04) {
        return false;
    }

    byteCount = frame[2];
    if ((byteCount & 0x01u) != 0u || len != (int)(byteCount + 5u)) {
        return false;
    }

    peer = rs485PeerDecoderForRx(ctx);
    if (peer == NULL || !peer->lastReqValid || peer->lastReqSlave != slave || peer->lastReqFunc != func) {
        return false;
    }

    regCount = (uint16_t)(byteCount / 2u);
    startReg = peer->lastReqStart;
    maxCount = regCount;
    if (peer->lastReqCount > 0 && peer->lastReqCount < maxCount) {
        maxCount = peer->lastReqCount;
    }

    for (uint16_t i = 0; i < maxCount; i++) {
        int dataPos = 3 + (int)(i * 2u);
        if (!modbusIsExcludedRegister((uint16_t)(startReg + i))) {
            continue;
        }
        frame[dataPos] = 0x00u;
        frame[dataPos + 1] = 0x00u;
        maskedCount++;
    }

    if (maskedCount == 0) {
        return false;
    }

    {
        uint16_t crc = modbusCrc16(frame, len - 2);
        frame[len - 2] = (uint8_t)(crc & 0xFFu);
        frame[len - 1] = (uint8_t)((crc >> 8) & 0xFFu);
    }

    RS485_VERBOSE_LOGI("RS485 sanitized RESP %s -> %s: masked %u regs from start=0x%04X",
                       ctx->rxName,
                       ctx->txName,
                       (unsigned)maskedCount,
                       (unsigned)startReg);
    return true;
}

static void rs485ForwardFrame(rs485BridgeCtx_t *ctx, const uint8_t *frame, int len)
{
    uint8_t func = 0;
    uint16_t start = 0;
    uint16_t count = 0;
    uint8_t txFrame[256];
    const uint8_t *txPtr = frame;

    if (!ctx->forwardEnabled) {
        return;
    }
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

    rs485DriverWriteFrame(ctx->txUart, ctx->txDirPin, txPtr, len);
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
                ESP_LOGW(EXAMPLE_TAG, "RS485 frame overflow on %s, dropping buffered frame", ctx->rxName);
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

void rs485ForwardSnifferStop(void)
{
    deleteTaskIfRunning(&s_rs485TaskA);
    deleteTaskIfRunning(&s_rs485TaskB);
    deleteTaskIfRunning(&s_rs485SnapshotTask);
    deleteTaskIfRunning(&s_rs485PollerTask);
}

void rs485ForwardSnifferResetDecoders(void)
{
    modbusDecoderInit(&gRsDec1, "RS485_1", 5000);
    modbusDecoderInit(&gRsDec2, "RS485_2", 5000);
}

void rs485ForwardSnifferStart(const bridge_runtime_settings_t *settings)
{
    static rs485BridgeCtx_t bmsToInv;
    static rs485BridgeCtx_t invToBms;
    static rs485BridgeCtx_t bmsSniff;
    static rs485BridgeCtx_t invSniff;
    static rs485BmsPollerCtx_t poller;
    const bool bmsOnRs = (settings != NULL) && (settings->bms_line == LINE_RS485);
    const bool invOnRs = (settings != NULL) && (settings->inverter_line == LINE_RS485);
    const bool rs485DirectPassthroughInBridge = (settings != NULL) &&
                                                (settings->mode == MODE_BRIDGE) &&
                                                bmsOnRs &&
                                                invOnRs &&
                                                (settings->bms_protocol == PROTOCOL_RS485_GROWATT) &&
                                                (settings->inverter_protocol == PROTOCOL_RS485_GROWATT);
    const bool rs485ForwardEnabled = (settings != NULL) &&
                                     ((settings->mode == MODE_FORWARD) || rs485DirectPassthroughInBridge);
    const bool inverseCanToRs = (settings != NULL) &&
                                (settings->bms_line == LINE_CAN) &&
                                (settings->inverter_line == LINE_RS485) &&
                                ((settings->bms_protocol == PROTOCOL_CAN_GROWATT) ||
                                 (settings->bms_protocol == PROTOCOL_CAN_PYLON) ||
                                 (settings->bms_protocol == PROTOCOL_CAN_GOODWE) ||
                                 (settings->bms_protocol == PROTOCOL_CAN_SOFAR) ||
                                 (settings->bms_protocol == PROTOCOL_CAN_SMA)) &&
                                (settings->inverter_protocol == PROTOCOL_RS485_GROWATT);

    rs485ForwardSnifferStop();

    if (settings == NULL) {
        return;
    }

    if (createRsTask(rs485PeriodicSnapshotTask, "rs485_snapshot", 4096, NULL, 7, &s_rs485SnapshotTask)) {
        if (gRsDec1.ifName == NULL || gRsDec2.ifName == NULL) {
            rs485ForwardSnifferResetDecoders();
        }

        ESP_LOGI(EXAMPLE_TAG, "RS485 periodic snapshot enabled (%d ms)", MODBUS_DECODER_SNAPSHOT_PRINT_PERIOD_MS);
    }
    ESP_LOGI(EXAMPLE_TAG, "RS485 reg exclude list: %s (%u entries configured)",
             RS485_REG_EXCLUDE_LIST_ENABLE ? "ON" : "OFF",
             (unsigned)g_rs485ForwardExcludeRegsCount);
    if (rs485DirectPassthroughInBridge) {
        ESP_LOGI(EXAMPLE_TAG,
                 "RS485 bridge mode uses direct pass-through because both sides are RS485_GROWATT");
    }

    if (bmsOnRs && invOnRs) {
        bmsToInv.rxName = rsNameByPort(settings->bms_port);
        bmsToInv.txName = rsNameByPort(settings->inverter_port);
        bmsToInv.rxUart = rsUartByPort(settings->bms_port);
        bmsToInv.txUart = rsUartByPort(settings->inverter_port);
        bmsToInv.txDirPin = rsDirByPort(settings->inverter_port);
        bmsToInv.applyRegExcludeList = RS485_REG_EXCLUDE_LIST_ENABLE;
        bmsToInv.forwardEnabled = rs485ForwardEnabled;
        bmsToInv.bmsToInverterDir = true;

        invToBms.rxName = rsNameByPort(settings->inverter_port);
        invToBms.txName = rsNameByPort(settings->bms_port);
        invToBms.rxUart = rsUartByPort(settings->inverter_port);
        invToBms.txUart = rsUartByPort(settings->bms_port);
        invToBms.txDirPin = rsDirByPort(settings->bms_port);
        invToBms.applyRegExcludeList = RS485_REG_EXCLUDE_LIST_ENABLE;
        invToBms.forwardEnabled = rs485ForwardEnabled;
        invToBms.bmsToInverterDir = false;

        createRsTask(rs485BridgeTask, "rs485_bms_to_inv", 4096, &bmsToInv, 9, &s_rs485TaskA);
        createRsTask(rs485BridgeTask, "rs485_inv_to_bms", 4096, &invToBms, 9, &s_rs485TaskB);

        ESP_LOGI(EXAMPLE_TAG,
                 "RS485 bridge enabled (%s[P%d] <-> %s[P%d]), forward=%s, exclude=%s",
                 bmsToInv.rxName,
                 settings->bms_port,
                 bmsToInv.txName,
                 settings->inverter_port,
                 rs485ForwardEnabled ? "ON" : "OFF",
                 RS485_REG_EXCLUDE_LIST_ENABLE ? "ON" : "OFF");
    } else {
        if (bmsOnRs) {
            bmsSniff.rxName = rsNameByPort(settings->bms_port);
            bmsSniff.txName = rsNameByPort(settings->bms_port);
            bmsSniff.rxUart = rsUartByPort(settings->bms_port);
            bmsSniff.txUart = rsUartByPort(settings->bms_port);
            bmsSniff.txDirPin = rsDirByPort(settings->bms_port);
            bmsSniff.applyRegExcludeList = RS485_REG_EXCLUDE_LIST_ENABLE;
            bmsSniff.forwardEnabled = false;
            bmsSniff.bmsToInverterDir = false;
            if (createRsTask(rs485BridgeTask, "rs485_bms_sniff", 4096, &bmsSniff, 9, &s_rs485TaskA)) {
                ESP_LOGI(EXAMPLE_TAG, "RS485 sniffer enabled on BMS side (%s[P%d])", bmsSniff.rxName, settings->bms_port);
            }
        }

        if (invOnRs && !inverseCanToRs) {
            invSniff.rxName = rsNameByPort(settings->inverter_port);
            invSniff.txName = rsNameByPort(settings->inverter_port);
            invSniff.rxUart = rsUartByPort(settings->inverter_port);
            invSniff.txUart = rsUartByPort(settings->inverter_port);
            invSniff.txDirPin = rsDirByPort(settings->inverter_port);
            invSniff.applyRegExcludeList = RS485_REG_EXCLUDE_LIST_ENABLE;
            invSniff.forwardEnabled = false;
            invSniff.bmsToInverterDir = false;
            if (createRsTask(rs485BridgeTask, "rs485_inv_sniff", 4096, &invSniff, 9, &s_rs485TaskB)) {
                ESP_LOGI(EXAMPLE_TAG, "RS485 sniffer enabled on inverter side (%s[P%d])", invSniff.rxName, settings->inverter_port);
            }
        } else if (inverseCanToRs) {
            ESP_LOGI(EXAMPLE_TAG, "RS485 inverter side reserved for CAN->RS485 translator (%s[P%d])",
                     rsNameByPort(settings->inverter_port),
                     settings->inverter_port);
        }
    }

    if ((settings->bms_line == LINE_RS485) &&
        (settings->bms_protocol == PROTOCOL_RS485_GROWATT)) {
#if RS485_BMS_POLLER_ENABLE
        if (rs485ForwardEnabled) {
            ESP_LOGW(EXAMPLE_TAG, "RS485 BMS poller disabled because RS485 forward is ON (avoid collisions)");
        } else {
            poller.txName = rsNameByPort(settings->bms_port);
            poller.txUart = rsUartByPort(settings->bms_port);
            poller.txDirPin = rsDirByPort(settings->bms_port);
            poller.slaveId = (uint8_t)RS485_BMS_SLAVE_ID;
            poller.periodMs = RS485_BMS_POLL_PERIOD_MS;
            poller.targetDec = rs485ForwardSnifferGetDecoder(settings->bms_port);
            if (createRsTask(rs485BmsPollerTask, "rs485_bms_poller", 3072, &poller, 8, &s_rs485PollerTask)) {
                ESP_LOGI(EXAMPLE_TAG,
                         "RS485 BMS poller enabled (tx=%s slave=%u period=%ums prot=RS485_GROWATT inv_line=%u inv_prot=%u)",
                         poller.txName,
                         (unsigned)poller.slaveId,
                         (unsigned)poller.periodMs,
                         (unsigned)settings->inverter_line,
                         (unsigned)settings->inverter_protocol);
            }
        }
#endif
    }
}
