#include "protocols/jkbms_rs485/jkbms_rs485_bms_task.h"

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
    uint8_t rxBuf[JKBMS_RS485_NATIVE_MAX_FRAME_LEN];
    size_t rxLen;
} jkbmsRs485BmsTaskCtx_t;

static jkbmsRs485BmsTaskCtx_t g_jkbmsRs485BmsCtx;
static TaskHandle_t g_jkbmsRs485BmsTaskHandle;
static portMUX_TYPE g_latestMux = portMUX_INITIALIZER_UNLOCKED;
static bool g_haveLatestPacket;
static bms_decoded_packet_t g_latestPacket;
static bool g_haveLatestSnapshot;
static jkbms_rs485_native_snapshot_t g_latestSnapshot;
static int64_t g_lastSourceStaleLogUs;

static uint16_t be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
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

static void storeLatestSnapshot(const jkbms_rs485_native_snapshot_t *snapshot)
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

static void publishBatteryModel(const jkbms_rs485_native_snapshot_t *snapshot, int64_t sourceUs)
{
    battery_model_t model = {0};

    if (snapshot == NULL || !snapshot->valid) {
        return;
    }

    model.valid = true;
    model.updatedMs = (uint32_t)(sourceUs / 1000LL);

    if (snapshot->hasPackVoltageMv) {
        model.packVoltageV = (float)snapshot->packVoltageMv / 1000.0f;
    } else if (snapshot->hasCellAvgMv && snapshot->cellCount > 0u) {
        model.packVoltageV =
            ((float)snapshot->cellAvgMv * (float)snapshot->cellCount) / 1000.0f;
    }
    if (snapshot->hasPackCurrentMa) {
        model.packCurrentA = (float)snapshot->packCurrentMa / 1000.0f;
    }
    if (snapshot->hasSoc) {
        model.socPct = snapshot->socPct;
    }
    if (snapshot->hasSoh) {
        model.sohPct = snapshot->sohPct;
    } else {
        model.sohPct = 100u;
    }
    if (snapshot->hasCycles) {
        model.cycleCount =
            (snapshot->cycles > UINT16_MAX) ? UINT16_MAX : (uint16_t)snapshot->cycles;
    }
    if (snapshot->hasCellExtremes) {
        model.cellMaxV = (float)snapshot->maxCellMv / 1000.0f;
        model.cellMinV = (float)snapshot->minCellMv / 1000.0f;
        model.cellMaxIdx = snapshot->maxCellIndex;
        model.cellMinIdx = snapshot->minCellIndex;
    }
    if (snapshot->hasCellDiffMaxMv) {
        model.cellDeltaV = (float)snapshot->cellDiffMaxMv / 1000.0f;
    }
    if (snapshot->hasTempMosC) {
        model.temperaturesC[0] = (float)snapshot->tempMosC;
    }
    if (snapshot->hasTempBat1C) {
        model.temperaturesC[1] = (float)snapshot->tempBat1C;
    }
    if (snapshot->hasTempBat2C) {
        model.temperaturesC[2] = (float)snapshot->tempBat2C;
    }
    if (snapshot->hasStatusFlags) {
        model.chargeEnabled = snapshot->chargeEnabled;
        model.dischargeEnabled = snapshot->dischargeEnabled;
        model.balanceEnabled = snapshot->balanceActive;
        model.protocolState = snapshot->statusFlags;
    }
    if (snapshot->hasAlarmBits) {
        model.alarmsMask = snapshot->alarmBits;
        model.warningsMask = snapshot->alarmBits & 0x0001u;
    }

    batteryModelSet(&model);
}

static bool buildPacketFromSnapshot(const jkbms_rs485_native_snapshot_t *snapshot,
                                    uint32_t sequence,
                                    int64_t sourceUs,
                                    bms_decoded_packet_t *outPacket)
{
    if (snapshot == NULL || outPacket == NULL || !snapshot->valid) {
        return false;
    }

    memset(outPacket, 0, sizeof(*outPacket));
    outPacket->sourceProtocol = PROTOCOL_ID_JKBMS_NATIVE;
    outPacket->sequence = sequence;
    outPacket->timestampUs = sourceUs;

    if (snapshot->hasSoc) {
        outPacket->hasSoc = true;
        outPacket->socPct = snapshot->socPct;
    }

    if (snapshot->hasTempBat1C) {
        outPacket->hasTemperatureC = true;
        outPacket->temperatureC = snapshot->tempBat1C;
    } else if (snapshot->hasTempMosC) {
        outPacket->hasTemperatureC = true;
        outPacket->temperatureC = snapshot->tempMosC;
    }

    if (snapshot->hasTempMosC) {
        outPacket->tempDeciC[outPacket->tempCount++] = (int16_t)(snapshot->tempMosC * 10);
    }
    if (snapshot->hasTempBat1C && outPacket->tempCount < BMS_DECODED_PACKET_MAX_TEMPS) {
        outPacket->tempDeciC[outPacket->tempCount++] = (int16_t)(snapshot->tempBat1C * 10);
    }
    if (snapshot->hasTempBat2C && outPacket->tempCount < BMS_DECODED_PACKET_MAX_TEMPS) {
        outPacket->tempDeciC[outPacket->tempCount++] = (int16_t)(snapshot->tempBat2C * 10);
    }

    if (snapshot->hasPackVoltageMv) {
        const uint32_t cv = (snapshot->packVoltageMv + 5u) / 10u;
        outPacket->hasPackVoltageCv = true;
        outPacket->packVoltageCv = (uint16_t)((cv > UINT16_MAX) ? UINT16_MAX : cv);
    }

    if (snapshot->hasCellExtremes) {
        outPacket->hasCellExtremes = true;
        outPacket->minCellMv = snapshot->minCellMv;
        outPacket->maxCellMv = snapshot->maxCellMv;
        outPacket->minCellIndex = snapshot->minCellIndex;
        outPacket->maxCellIndex = snapshot->maxCellIndex;
    }
    if (snapshot->cellCount > 0u) {
        const uint8_t limit = (snapshot->cellCount > BMS_DECODED_PACKET_MAX_CELLS)
                                  ? BMS_DECODED_PACKET_MAX_CELLS
                                  : snapshot->cellCount;
        outPacket->cellCount = limit;
        memcpy(outPacket->cellMv, snapshot->cellMv, (size_t)limit * sizeof(outPacket->cellMv[0]));
    }

    if (snapshot->hasAlarmBits) {
        outPacket->hasWarningFlags = true;
        outPacket->warningFlags = snapshot->alarmBits & 0x0001u;
        outPacket->hasProtectionFlags = true;
        outPacket->protectionFlags = snapshot->alarmBits & 0x0FFEu;
    }

    if (snapshot->hasStatusFlags) {
        outPacket->hasStatusFlags = true;
        outPacket->statusFlags = snapshot->statusFlags;
        outPacket->hasBalanceFlags = true;
        outPacket->balanceFlags = snapshot->balanceActive ? 1u : 0u;
    }

    return outPacket->hasSoc ||
           outPacket->hasTemperatureC ||
           outPacket->hasPackVoltageCv ||
           outPacket->hasCellExtremes ||
           outPacket->cellCount > 0u ||
           outPacket->hasWarningFlags ||
           outPacket->hasProtectionFlags ||
           outPacket->hasStatusFlags;
}

static void publishSnapshot(jkbmsRs485BmsTaskCtx_t *ctx,
                            const jkbms_rs485_native_snapshot_t *snapshot,
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
            ESP_LOGW(EXAMPLE_TAG, "JKBMS native output queue overwrite failed");
        }
    }
}

static void dropRxPrefix(jkbmsRs485BmsTaskCtx_t *ctx, size_t count)
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

static void consumeRx(jkbmsRs485BmsTaskCtx_t *ctx, const uint8_t *data, size_t len, int64_t nowUs)
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

    while (ctx->rxLen >= 4u) {
        size_t start = 0u;
        while (start + 1u < ctx->rxLen &&
               !(ctx->rxBuf[start] == 0x4Eu && ctx->rxBuf[start + 1u] == 0x57u)) {
            start++;
        }
        if (start > 0u) {
            dropRxPrefix(ctx, start);
        }
        if (ctx->rxLen < 4u) {
            return;
        }

        const size_t expectedLen = (size_t)be16(&ctx->rxBuf[2]) + 2u;
        if (expectedLen < 13u || expectedLen > sizeof(ctx->rxBuf)) {
            dropRxPrefix(ctx, 1u);
            continue;
        }
        if (ctx->rxLen < expectedLen) {
            return;
        }

        jkbms_rs485_native_snapshot_t snapshot = {0};
        if (jkbmsRs485NativeDecodeFrame(ctx->rxBuf, expectedLen, &snapshot)) {
            publishSnapshot(ctx, &snapshot, nowUs);
        } else {
            ESP_LOGW(EXAMPLE_TAG, "JKBMS native frame decode failed (len=%u)", (unsigned)expectedLen);
        }
        dropRxPrefix(ctx, expectedLen);
    }
}

static void pollNative(uart_port_t uart, gpio_num_t dirPin, jkbmsRs485BmsTaskCtx_t *ctx, int64_t nowUs)
{
    uint8_t req[JKBMS_RS485_NATIVE_READ_ALL_REQUEST_LEN];
    size_t reqLen = 0u;

    if (ctx == NULL) {
        return;
    }

    if (ctx->lastPollUs != 0 &&
        (nowUs - ctx->lastPollUs) < ((int64_t)JKBMS_RS485_NATIVE_QUERY_PERIOD_MS * 1000LL)) {
        return;
    }

    reqLen = jkbmsRs485NativeBuildReadAllRequest(req, sizeof(req));
    if (reqLen == 0u) {
        ESP_LOGW(EXAMPLE_TAG, "JKBMS native read-all request build failed");
        return;
    }

    esp_err_t err = rs485WriteBytes(uart, dirPin, req, (int)reqLen, pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        ESP_LOGW(EXAMPLE_TAG, "JKBMS native poll TX failed (err=0x%x)", (unsigned)err);
        return;
    }

    ctx->lastPollUs = nowUs;
}

static void maybeClearStaleSource(jkbmsRs485BmsTaskCtx_t *ctx, int64_t nowUs)
{
    bool haveData = false;

    if (ctx == NULL) {
        return;
    }

    if (ctx->lastFrameUs == 0) {
        if ((nowUs - g_lastSourceStaleLogUs) >= 1000000LL) {
            ESP_LOGW(EXAMPLE_TAG,
                     "JKBMS native source stale: clearing published data (no valid BMS response yet)");
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
        if ((nowUs - g_lastSourceStaleLogUs) >= 1000000LL) {
            const uint32_t ageMs = (uint32_t)((nowUs - ctx->lastFrameUs) / 1000LL);
            ESP_LOGW(EXAMPLE_TAG,
                     "JKBMS native source stale: clearing published data (last_rx_age=%u ms)",
                     (unsigned)ageMs);
            g_lastSourceStaleLogUs = nowUs;
        }
    }
}

static void jkbmsRs485BmsTask(void *pv)
{
    jkbmsRs485BmsTaskCtx_t *ctx = (jkbmsRs485BmsTaskCtx_t *)pv;
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

        pollNative(rxUart, dirPin, ctx, nowUs);
        maybeClearStaleSource(ctx, nowUs);
    }
}

esp_err_t jkbmsRs485BmsTaskStart(QueueHandle_t outQueue)
{
    if (outQueue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_jkbmsRs485BmsTaskHandle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&g_jkbmsRs485BmsCtx, 0, sizeof(g_jkbmsRs485BmsCtx));
    g_jkbmsRs485BmsCtx.outQueue = outQueue;
    g_lastSourceStaleLogUs = 0;
    batteryModelClear();
    clearLatestData();

    BaseType_t taskOk =
        xTaskCreate(jkbmsRs485BmsTask,
                    "jkbms_native",
                    JKBMS_RS485_NATIVE_TASK_STACK,
                    &g_jkbmsRs485BmsCtx,
                    JKBMS_RS485_NATIVE_TASK_PRIORITY,
                    &g_jkbmsRs485BmsTaskHandle);
    if (taskOk != pdPASS) {
        g_jkbmsRs485BmsTaskHandle = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "JKBMS native RS485 task started (poll=%dms)",
             JKBMS_RS485_NATIVE_QUERY_PERIOD_MS);
    return ESP_OK;
}

esp_err_t jkbmsRs485BmsTaskStop(void)
{
    if (g_jkbmsRs485BmsTaskHandle == NULL) {
        return ESP_OK;
    }

    vTaskDelete(g_jkbmsRs485BmsTaskHandle);
    g_jkbmsRs485BmsTaskHandle = NULL;
    memset(&g_jkbmsRs485BmsCtx, 0, sizeof(g_jkbmsRs485BmsCtx));
    batteryModelClear();
    clearLatestData();
    return ESP_OK;
}

bool jkbmsRs485BmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket)
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

bool jkbmsRs485BmsTaskGetLatestSnapshot(jkbms_rs485_native_snapshot_t *outSnapshot)
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
