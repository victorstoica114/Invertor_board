#include "protocols/seplos_rs485/seplos_rs485_bms_task.h"

#include <limits.h>
#include <string.h>

#include "Drivers/rs485_driver.h"
#include "config.h"
#include "protocols/common/battery_model.h"
#include "runtime_settings.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    QueueHandle_t outQueue;
    uint32_t sequence;
    int64_t lastPollUs;
    int64_t lastFrameUs;
    uint8_t nextCid2;
    uint8_t lastRequestedCid2;
    uint8_t rxBuf[SEPLOS_RS485_MAX_FRAME_LEN];
    size_t rxLen;
    seplos_rs485_snapshot_t workingSnapshot;
} seplosRs485BmsTaskCtx_t;

static seplosRs485BmsTaskCtx_t g_seplosRs485BmsCtx;
static TaskHandle_t g_seplosRs485BmsTaskHandle;
static portMUX_TYPE g_latestMux = portMUX_INITIALIZER_UNLOCKED;
static bool g_haveLatestPacket;
static bms_decoded_packet_t g_latestPacket;
static bool g_haveLatestSnapshot;
static seplos_rs485_snapshot_t g_latestSnapshot;
static int64_t g_lastSourceStaleLogUs;

static uint8_t pctFromDeci(uint16_t deciPct)
{
    uint16_t pct = (uint16_t)((deciPct + 5u) / 10u);
    return (uint8_t)((pct > 100u) ? 100u : pct);
}

static void storeLatestPacket(const bms_decoded_packet_t *packet)
{
    if (packet == NULL) {
        return;
    }

    portENTER_CRITICAL(&g_latestMux);
    g_latestPacket = *packet;
    g_haveLatestPacket = true;
    portEXIT_CRITICAL(&g_latestMux);
}

static void storeLatestSnapshot(const seplos_rs485_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    portENTER_CRITICAL(&g_latestMux);
    g_latestSnapshot = *snapshot;
    g_haveLatestSnapshot = true;
    portEXIT_CRITICAL(&g_latestMux);
}

static void clearLatestData(void)
{
    portENTER_CRITICAL(&g_latestMux);
    g_haveLatestPacket = false;
    memset(&g_latestPacket, 0, sizeof(g_latestPacket));
    g_haveLatestSnapshot = false;
    memset(&g_latestSnapshot, 0, sizeof(g_latestSnapshot));
    portEXIT_CRITICAL(&g_latestMux);
}

static void publishBatteryModel(const seplos_rs485_snapshot_t *snapshot, int64_t sourceUs)
{
    battery_model_t model = {0};

    if (snapshot == NULL || !snapshot->valid || !snapshot->hasTelemetry) {
        return;
    }

    model.valid = true;
    model.updatedMs = (uint32_t)(sourceUs / 1000LL);

    if (snapshot->hasPackVoltageCv) {
        model.packVoltageV = (float)snapshot->packVoltageCv / 100.0f;
    }
    if (snapshot->hasPackCurrentCa) {
        model.packCurrentA = (float)snapshot->packCurrentCa / 100.0f;
    }
    if (snapshot->hasSocDeciPct) {
        model.socPct = pctFromDeci(snapshot->socDeciPct);
    }
    if (snapshot->hasSohDeciPct) {
        model.sohPct = pctFromDeci(snapshot->sohDeciPct);
    } else {
        model.sohPct = 100u;
    }
    if (snapshot->hasCycles) {
        model.cycleCount = snapshot->cycles;
    }
    if (snapshot->hasCellExtremes) {
        model.cellMaxV = (float)snapshot->maxCellMv / 1000.0f;
        model.cellMinV = (float)snapshot->minCellMv / 1000.0f;
        model.cellMaxIdx = snapshot->maxCellIndex;
        model.cellMinIdx = snapshot->minCellIndex;
    }
    if (snapshot->hasCellDiffMv) {
        model.cellDeltaV = (float)snapshot->cellDiffMv / 1000.0f;
    }

    for (uint8_t i = 0u; i < UNIVERSAL_BATTERY_TEMP_SENSORS; i++) {
        model.temperaturesC[i] = -100.0f;
    }
    if (snapshot->tempCount > 5u) {
        model.temperaturesC[0] = (float)snapshot->tempDeciC[5] / 10.0f; /* MOS */
    }
    if (snapshot->tempCount > 0u) {
        model.temperaturesC[1] = (float)snapshot->tempDeciC[0] / 10.0f;
    }
    if (snapshot->tempCount > 1u) {
        model.temperaturesC[2] = (float)snapshot->tempDeciC[1] / 10.0f;
    }
    if (snapshot->tempCount > 2u) {
        model.temperaturesC[3] = (float)snapshot->tempDeciC[2] / 10.0f;
    }
    if (snapshot->tempCount > 4u) {
        model.temperaturesC[4] = (float)snapshot->tempDeciC[4] / 10.0f; /* Environment */
    }

    if (snapshot->hasAlarms) {
        model.chargeEnabled = snapshot->chargeEnabled;
        model.dischargeEnabled = snapshot->dischargeEnabled;
        model.balanceEnabled = snapshot->balanceFlags != 0u;
        model.protocolState = snapshot->systemStatus;

        uint32_t rawWarnings = ((uint32_t)snapshot->warningBytes[0]) |
                               ((uint32_t)snapshot->warningBytes[1] << 8) |
                               ((uint32_t)snapshot->warningBytes[2] << 16) |
                               ((uint32_t)snapshot->warningBytes[3] << 24);
        uint32_t rawAlarms = ((uint32_t)snapshot->warningBytes[4]) |
                             ((uint32_t)snapshot->warningBytes[5] << 8) |
                             ((uint32_t)snapshot->warningBytes[6] << 16) |
                             ((uint32_t)snapshot->warningBytes[7] << 24);
        model.warningsMask = rawWarnings;
        model.alarmsMask = rawAlarms;
    }

    batteryModelSet(&model);
}

static bool buildPacketFromSnapshot(const seplos_rs485_snapshot_t *snapshot,
                                    uint32_t sequence,
                                    int64_t sourceUs,
                                    bms_decoded_packet_t *outPacket)
{
    if (snapshot == NULL || outPacket == NULL || !snapshot->valid || !snapshot->hasTelemetry) {
        return false;
    }

    memset(outPacket, 0, sizeof(*outPacket));
    outPacket->sourceProtocol = PROTOCOL_ID_SEPLOS;
    outPacket->sequence = sequence;
    outPacket->timestampUs = sourceUs;

    if (snapshot->hasSocDeciPct) {
        outPacket->hasSoc = true;
        outPacket->socPct = pctFromDeci(snapshot->socDeciPct);
    }
    if (snapshot->tempCount > 0u) {
        outPacket->hasTemperatureC = true;
        outPacket->temperatureC = (int16_t)(snapshot->tempDeciC[0] / 10);
        outPacket->tempCount = snapshot->tempCount;
        for (uint8_t i = 0u; i < snapshot->tempCount && i < BMS_DECODED_PACKET_MAX_TEMPS; i++) {
            outPacket->tempDeciC[i] = snapshot->tempDeciC[i];
        }
    }
    if (snapshot->hasPackVoltageCv) {
        outPacket->hasPackVoltageCv = true;
        outPacket->packVoltageCv = snapshot->packVoltageCv;
    }
    if (snapshot->hasCellExtremes) {
        outPacket->hasCellExtremes = true;
        outPacket->minCellMv = snapshot->minCellMv;
        outPacket->maxCellMv = snapshot->maxCellMv;
        outPacket->minCellIndex = snapshot->minCellIndex;
        outPacket->maxCellIndex = snapshot->maxCellIndex;
    }
    if (snapshot->cellCount > 0u) {
        uint8_t limit = (snapshot->cellCount > BMS_DECODED_PACKET_MAX_CELLS)
                            ? BMS_DECODED_PACKET_MAX_CELLS
                            : snapshot->cellCount;
        outPacket->cellCount = limit;
        memcpy(outPacket->cellMv, snapshot->cellMv, (size_t)limit * sizeof(outPacket->cellMv[0]));
    }
    if (snapshot->hasAlarms) {
        outPacket->hasWarningFlags = true;
        outPacket->warningFlags = (uint16_t)(((uint16_t)snapshot->warningBytes[0]) |
                                             ((uint16_t)snapshot->warningBytes[1] << 8));
        outPacket->hasProtectionFlags = true;
        outPacket->protectionFlags = (uint16_t)(((uint16_t)snapshot->warningBytes[2]) |
                                                ((uint16_t)snapshot->warningBytes[3] << 8));
        outPacket->hasStatusFlags = true;
        outPacket->statusFlags = snapshot->systemStatus;
        outPacket->hasBalanceFlags = true;
        outPacket->balanceFlags = snapshot->balanceFlags;
    }

    return outPacket->hasSoc ||
           outPacket->hasTemperatureC ||
           outPacket->hasPackVoltageCv ||
           outPacket->hasCellExtremes ||
           (outPacket->cellCount > 0u) ||
           outPacket->hasWarningFlags ||
           outPacket->hasStatusFlags;
}

static void publishSnapshot(seplosRs485BmsTaskCtx_t *ctx,
                            const seplos_rs485_snapshot_t *snapshot,
                            int64_t sourceUs)
{
    bms_decoded_packet_t packet = {0};

    if (ctx == NULL || snapshot == NULL || !snapshot->valid) {
        return;
    }

    storeLatestSnapshot(snapshot);
    publishBatteryModel(snapshot, sourceUs);
    ctx->lastFrameUs = sourceUs;

    if (buildPacketFromSnapshot(snapshot, ++ctx->sequence, sourceUs, &packet)) {
        storeLatestPacket(&packet);
        if (xQueueOverwrite(ctx->outQueue, &packet) != pdPASS) {
            ESP_LOGW(EXAMPLE_TAG, "Seplos RS485 output queue overwrite failed");
        }
    }
}

static void dropRxPrefix(seplosRs485BmsTaskCtx_t *ctx, size_t count)
{
    if (ctx == NULL || count == 0u) {
        return;
    }
    if (count >= ctx->rxLen) {
        ctx->rxLen = 0u;
        return;
    }
    memmove(ctx->rxBuf, &ctx->rxBuf[count], ctx->rxLen - count);
    ctx->rxLen -= count;
}

static uint8_t inferCid2FromFrame(const seplos_rs485_frame_t *frame, uint8_t lastRequestedCid2)
{
    if (frame == NULL) {
        return lastRequestedCid2;
    }
    if (frame->infoLen >= 60u) {
        return SEPLOS_RS485_CID2_TELEMETRY;
    }
    if (frame->infoLen >= 24u) {
        return SEPLOS_RS485_CID2_ALARMS;
    }
    return lastRequestedCid2;
}

static void consumeRx(seplosRs485BmsTaskCtx_t *ctx, const uint8_t *data, size_t len, int64_t nowUs)
{
    if (ctx == NULL || data == NULL || len == 0u) {
        return;
    }

    if (len > sizeof(ctx->rxBuf) - ctx->rxLen) {
        ctx->rxLen = 0u;
    }
    if (len > sizeof(ctx->rxBuf)) {
        data += len - sizeof(ctx->rxBuf);
        len = sizeof(ctx->rxBuf);
    }

    memcpy(&ctx->rxBuf[ctx->rxLen], data, len);
    ctx->rxLen += len;

    while (ctx->rxLen > 0u) {
        size_t start = 0u;
        while (start < ctx->rxLen && ctx->rxBuf[start] != '~') {
            start++;
        }
        if (start > 0u) {
            dropRxPrefix(ctx, start);
        }
        if (ctx->rxLen < 2u) {
            return;
        }

        size_t end = 0u;
        while (end < ctx->rxLen && ctx->rxBuf[end] != '\r') {
            end++;
        }
        if (end >= ctx->rxLen) {
            return;
        }

        const size_t frameLen = end + 1u;
        seplos_rs485_frame_t frame = {0};
        if (seplosRs485DecodeFrame(ctx->rxBuf, frameLen, &frame)) {
            uint8_t cid2 = inferCid2FromFrame(&frame, ctx->lastRequestedCid2);
            if (cid2 == SEPLOS_RS485_CID2_TELEMETRY) {
                seplos_rs485_snapshot_t next = {0};
                bool hadAlarms = ctx->workingSnapshot.hasAlarms;
                seplos_rs485_snapshot_t previous = ctx->workingSnapshot;
                if (seplosRs485DecodeTelemetryInfo(frame.info, frame.infoLen, &next)) {
                    if (hadAlarms) {
                        memcpy(next.cellAlarmFlags,
                               previous.cellAlarmFlags,
                               sizeof(next.cellAlarmFlags));
                        memcpy(next.tempAlarmFlags,
                               previous.tempAlarmFlags,
                               sizeof(next.tempAlarmFlags));
                        next.currentAlarmFlags = previous.currentAlarmFlags;
                        next.voltageAlarmFlags = previous.voltageAlarmFlags;
                        next.customAlarmFlags = previous.customAlarmFlags;
                        memcpy(next.warningBytes, previous.warningBytes, sizeof(next.warningBytes));
                        next.powerStatus = previous.powerStatus;
                        next.balanceFlags = previous.balanceFlags;
                        next.systemStatus = previous.systemStatus;
                        next.chargeEnabled = previous.chargeEnabled;
                        next.dischargeEnabled = previous.dischargeEnabled;
                        next.sleepMode = previous.sleepMode;
                        next.hasAlarms = true;
                    }
                    ctx->workingSnapshot = next;
                    publishSnapshot(ctx, &ctx->workingSnapshot, nowUs);
                    ESP_LOGI(EXAMPLE_TAG,
                             "Seplos telemetry: cells=%u soc=%u.%u%% pack=%.2fV current=%.2fA",
                             (unsigned)ctx->workingSnapshot.cellCount,
                             (unsigned)(ctx->workingSnapshot.hasSocDeciPct
                                            ? (ctx->workingSnapshot.socDeciPct / 10u)
                                            : 0u),
                             (unsigned)(ctx->workingSnapshot.hasSocDeciPct
                                            ? (ctx->workingSnapshot.socDeciPct % 10u)
                                            : 0u),
                             ctx->workingSnapshot.hasPackVoltageCv
                                 ? ((double)ctx->workingSnapshot.packVoltageCv / 100.0)
                                 : 0.0,
                             ctx->workingSnapshot.hasPackCurrentCa
                                 ? ((double)ctx->workingSnapshot.packCurrentCa / 100.0)
                                 : 0.0);
                } else {
                    ESP_LOGW(EXAMPLE_TAG, "Seplos telemetry decode failed (info_len=%u)", (unsigned)frame.infoLen);
                }
            } else if (cid2 == SEPLOS_RS485_CID2_ALARMS) {
                if (seplosRs485DecodeAlarmInfo(frame.info, frame.infoLen, &ctx->workingSnapshot)) {
                    publishSnapshot(ctx, &ctx->workingSnapshot, nowUs);
                } else {
                    ESP_LOGW(EXAMPLE_TAG, "Seplos alarm decode failed (info_len=%u)", (unsigned)frame.infoLen);
                }
            }
        } else {
            ESP_LOGW(EXAMPLE_TAG, "Seplos RS485 frame decode failed (len=%u)", (unsigned)frameLen);
        }

        dropRxPrefix(ctx, frameLen);
    }
}

static void pollSeplos(uart_port_t uart, gpio_num_t dirPin, seplosRs485BmsTaskCtx_t *ctx, int64_t nowUs)
{
    uint8_t req[32];
    size_t reqLen = 0u;
    uint8_t cid2 = 0u;

    if (ctx == NULL) {
        return;
    }

    if (ctx->lastPollUs != 0 &&
        (nowUs - ctx->lastPollUs) < ((int64_t)SEPLOS_BMS_QUERY_PERIOD_MS * 1000LL)) {
        return;
    }

    cid2 = (ctx->nextCid2 == SEPLOS_RS485_CID2_ALARMS)
               ? SEPLOS_RS485_CID2_ALARMS
               : SEPLOS_RS485_CID2_TELEMETRY;
    reqLen = seplosRs485BuildRequest(cid2,
                                     SEPLOS_BMS_ADDRESS,
                                     SEPLOS_RS485_PROTOCOL_VERSION,
                                     req,
                                     sizeof(req));
    if (reqLen == 0u) {
        ESP_LOGW(EXAMPLE_TAG, "Seplos request build failed");
        return;
    }

    esp_err_t err = rs485WriteBytes(uart, dirPin, req, (int)reqLen, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGW(EXAMPLE_TAG, "Seplos poll TX failed (cid2=0x%02X err=0x%x)", (unsigned)cid2, (unsigned)err);
        return;
    }

    ctx->lastRequestedCid2 = cid2;
    ctx->nextCid2 = (cid2 == SEPLOS_RS485_CID2_TELEMETRY)
                        ? SEPLOS_RS485_CID2_ALARMS
                        : SEPLOS_RS485_CID2_TELEMETRY;
    ctx->lastPollUs = nowUs;
}

static void maybeClearStaleSource(seplosRs485BmsTaskCtx_t *ctx, int64_t nowUs)
{
    bool haveData = false;

    if (ctx == NULL) {
        return;
    }

    if (ctx->lastFrameUs == 0) {
        if ((nowUs - g_lastSourceStaleLogUs) >= 1000000LL) {
            ESP_LOGW(EXAMPLE_TAG,
                     "Seplos RS485 source stale: clearing published data (no valid BMS response yet)");
            g_lastSourceStaleLogUs = nowUs;
        }
        return;
    }

    portENTER_CRITICAL(&g_latestMux);
    haveData = g_haveLatestSnapshot || g_haveLatestPacket;
    portEXIT_CRITICAL(&g_latestMux);

    if (haveData &&
        (nowUs - ctx->lastFrameUs) > ((int64_t)BRIDGE_SOURCE_STALE_MS * 1000LL)) {
        batteryModelClear();
        clearLatestData();
        memset(&ctx->workingSnapshot, 0, sizeof(ctx->workingSnapshot));
        if ((nowUs - g_lastSourceStaleLogUs) >= 1000000LL) {
            const uint32_t ageMs = (uint32_t)((nowUs - ctx->lastFrameUs) / 1000LL);
            ESP_LOGW(EXAMPLE_TAG,
                     "Seplos RS485 source stale: clearing published data (last_rx_age=%u ms)",
                     (unsigned)ageMs);
            g_lastSourceStaleLogUs = nowUs;
        }
    }
}

static void seplosRs485BmsTask(void *pv)
{
    seplosRs485BmsTaskCtx_t *ctx = (seplosRs485BmsTaskCtx_t *)pv;
    uint8_t rxChunk[128];
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    const uint8_t bmsPort = (settings.bms_port == 2u) ? 2u : 1u;
    const uart_port_t rxUart = (bmsPort == 2u) ? rs485GetUart2() : rs485GetUart1();
    const gpio_num_t dirPin = (bmsPort == 2u) ? rs485GetDir2() : rs485GetDir1();

    uart_flush_input(rxUart);

    while (1) {
        int len = uart_read_bytes(rxUart, rxChunk, sizeof(rxChunk), pdMS_TO_TICKS(10));
        int64_t nowUs = esp_timer_get_time();

        if (len > 0) {
            consumeRx(ctx, rxChunk, (size_t)len, nowUs);
        }

        pollSeplos(rxUart, dirPin, ctx, nowUs);
        maybeClearStaleSource(ctx, nowUs);
    }
}

esp_err_t seplosRs485BmsTaskStart(QueueHandle_t outQueue)
{
    if (outQueue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_seplosRs485BmsTaskHandle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&g_seplosRs485BmsCtx, 0, sizeof(g_seplosRs485BmsCtx));
    g_seplosRs485BmsCtx.outQueue = outQueue;
    g_seplosRs485BmsCtx.nextCid2 = SEPLOS_RS485_CID2_TELEMETRY;
    g_lastSourceStaleLogUs = 0;
    batteryModelClear();
    clearLatestData();

    BaseType_t taskOk =
        xTaskCreate(seplosRs485BmsTask,
                    "seplos_rs485",
                    SEPLOS_BMS_TASK_STACK,
                    &g_seplosRs485BmsCtx,
                    SEPLOS_BMS_TASK_PRIORITY,
                    &g_seplosRs485BmsTaskHandle);
    if (taskOk != pdPASS) {
        g_seplosRs485BmsTaskHandle = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "Seplos RS485 BMS task started (addr=0x%02X poll=%dms)",
             (unsigned)SEPLOS_BMS_ADDRESS,
             SEPLOS_BMS_QUERY_PERIOD_MS);
    return ESP_OK;
}

esp_err_t seplosRs485BmsTaskStop(void)
{
    if (g_seplosRs485BmsTaskHandle == NULL) {
        return ESP_OK;
    }

    vTaskDelete(g_seplosRs485BmsTaskHandle);
    g_seplosRs485BmsTaskHandle = NULL;
    memset(&g_seplosRs485BmsCtx, 0, sizeof(g_seplosRs485BmsCtx));
    batteryModelClear();
    clearLatestData();
    return ESP_OK;
}

bool seplosRs485BmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket)
{
    bool hasPacket = false;

    if (outPacket == NULL) {
        return false;
    }

    portENTER_CRITICAL(&g_latestMux);
    hasPacket = g_haveLatestPacket;
    if (hasPacket) {
        *outPacket = g_latestPacket;
    }
    portEXIT_CRITICAL(&g_latestMux);

    return hasPacket;
}

bool seplosRs485BmsTaskGetLatestSnapshot(seplos_rs485_snapshot_t *outSnapshot)
{
    bool hasSnapshot = false;

    if (outSnapshot == NULL) {
        return false;
    }

    portENTER_CRITICAL(&g_latestMux);
    hasSnapshot = g_haveLatestSnapshot;
    if (hasSnapshot) {
        *outSnapshot = g_latestSnapshot;
    }
    portEXIT_CRITICAL(&g_latestMux);

    return hasSnapshot;
}
