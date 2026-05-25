#include "protocols/rs485_growatt/rs485_growatt_bms_task.h"

#include <limits.h>
#include <string.h>

#include "Drivers/rs485_driver.h"
#include "config.h"
#include "decoders/modbusDecoder.h"
#include "orchestrator/protocol_types.h"
#include "protocols/common/battery_model.h"
#include "protocols/rs485_growatt/rs485_growatt_modbus_poller.h"
#include "protocols/rs485_growatt/rs485_growatt_registers_map.h"
#include "runtime_settings.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    QueueHandle_t outQueue;
    modbusDecoder_t decoder;
    rs485_growatt_modbus_poller_t poller;
    uint32_t sequence;
    int64_t lastPublishUs;
} rs485GrowattBmsTaskCtx_t;

static rs485GrowattBmsTaskCtx_t g_rs485GrowattBmsCtx;
static TaskHandle_t g_rs485GrowattBmsTaskHandle;
static portMUX_TYPE g_latestPacketMux = portMUX_INITIALIZER_UNLOCKED;
static bool g_haveLatestPacket;
static bms_decoded_packet_t g_latestPacket;
static int64_t g_lastSourceStaleLogUs;

#define GROWATT_MIN_CELL_MV 1000u
#define GROWATT_MAX_CELL_MV 6000u
#define GROWATT_WARNING_CODE_MASK 0x3FFFu
#define GROWATT_ERROR_CODE_MASK   0x7FFFu

static bool growattCellVoltageValid(uint16_t mv)
{
    return mv >= GROWATT_MIN_CELL_MV && mv <= GROWATT_MAX_CELL_MV;
}

static bool decoderCacheFresh(const modbusDecoder_t *decoder, int64_t nowUs, int64_t *newestCacheUsOut)
{
    const int64_t newestCacheUs = modbusDecoderGetNewestCacheTsUs(decoder);
    const int64_t maxAgeUs = (int64_t)BRIDGE_SOURCE_STALE_MS * 1000LL;
    const int64_t ageUs = (nowUs >= newestCacheUs) ? (nowUs - newestCacheUs) : 0;

    if (newestCacheUsOut != NULL) {
        *newestCacheUsOut = newestCacheUs;
    }
    return (newestCacheUs > 0) && (ageUs <= maxAgeUs);
}

static void rs485GrowattStoreLatestPacket(const bms_decoded_packet_t *packet)
{
    if (packet == NULL) {
        return;
    }

    portENTER_CRITICAL(&g_latestPacketMux);
    g_latestPacket = *packet;
    g_haveLatestPacket = true;
    portEXIT_CRITICAL(&g_latestPacketMux);
}

static void rs485GrowattClearLatestPacket(void)
{
    portENTER_CRITICAL(&g_latestPacketMux);
    g_haveLatestPacket = false;
    memset(&g_latestPacket, 0, sizeof(g_latestPacket));
    portEXIT_CRITICAL(&g_latestPacketMux);
}

static uint8_t clampU8(uint16_t v, uint8_t vmax)
{
    return (uint8_t)((v > vmax) ? vmax : v);
}

static uint16_t rs485GrowattPackVoltageRawToCv(const modbusDecoder_t *decoder, uint16_t raw)
{
    uint16_t status = 0u;
    uint32_t cellSumMv = 0u;
    uint8_t validCells = 0u;

    if (decoder == NULL || raw == 0u) {
        return raw;
    }

    for (uint8_t i = 0u; i < RS485_GROWATT_MB_CELL_COUNT; i++) {
        uint16_t mv = 0u;
        const uint16_t addr = (uint16_t)(RS485_GROWATT_MB_REG_CELL_BASE + i);
        if (!modbusDecoderGetCachedReg(decoder, addr, &mv) || !growattCellVoltageValid(mv)) {
            continue;
        }
        cellSumMv += mv;
        validCells++;
    }

    /*
     * Seplos HV in Growatt485 mode has been observed returning status 0x0069,
     * one 8-cell module in the cell table, and the full stack voltage at 0x0016
     * in 0.1 V units. Convert that raw value into the centivolt unit used by
     * the shared bridge model so the inverter-facing Pylon responder sees the
     * real stack voltage.
     */
    if (raw <= (UINT16_MAX / 10u) &&
        validCells > 0u &&
        validCells <= 8u &&
        modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_STATUS_FLAGS, &status) &&
        status == 0x0069u) {
        const uint32_t rawAsCvMv = (uint32_t)raw * 10u;
        const uint32_t lowMv = (cellSumMv * 9u) / 10u;
        const uint32_t highMv = (cellSumMv * 11u) / 10u;
        if (rawAsCvMv >= lowMv && rawAsCvMv <= highMv) {
            return (uint16_t)(raw * 10u);
        }
    }

    return raw;
}

static void rs485GrowattPublishBatteryModel(const modbusDecoder_t *decoder,
                                            const bms_decoded_packet_t *packet)
{
    battery_model_t model = {0};
    uint16_t regVal = 0;

    if (decoder == NULL || packet == NULL) {
        return;
    }

    model.valid = true;
    model.updatedMs = (uint32_t)(packet->timestampUs / 1000LL);

    if (packet->hasSoc) {
        model.socPct = packet->socPct;
    }
    if (packet->hasPackVoltageCv) {
        model.packVoltageV = (float)packet->packVoltageCv / 100.0f;
    }
    if (packet->hasCellExtremes) {
        model.cellMinV = (float)packet->minCellMv / 1000.0f;
        model.cellMaxV = (float)packet->maxCellMv / 1000.0f;
        model.cellMinIdx = packet->minCellIndex;
        model.cellMaxIdx = packet->maxCellIndex;
        model.cellDeltaV = (float)(packet->maxCellMv - packet->minCellMv) / 1000.0f;
    }
    if (packet->hasTemperatureC) {
        model.temperaturesC[0] = (float)packet->temperatureC;
        model.temperaturesC[1] = (float)packet->temperatureC;
        model.temperaturesC[2] = (float)packet->temperatureC;
    }

    if (modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_SOH_PCT, &regVal)) {
        model.sohPct = clampU8(regVal, 100u);
    }
    if (modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_CYCLE_COUNT_TENTATIVE, &regVal)) {
        model.cycleCount = regVal;
    }
    if (modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_CV_TARGET_CV, &regVal)) {
        model.chargeVoltageLimitV =
            (float)rs485GrowattPackVoltageRawToCv(decoder, regVal) / 100.0f;
    }
    if (modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_ICHG_LIM_CA_TENTATIVE, &regVal)) {
        model.chargeCurrentLimitA = (float)regVal / 100.0f;
    }
    if (modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_IDIS_LIM_CA_TENTATIVE, &regVal)) {
        model.dischargeCurrentLimitA = (float)regVal / 100.0f;
    }
    if (modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_PACK_I_ABS_CA_TENTATIVE, &regVal)) {
        model.packCurrentA = (float)regVal / 100.0f;
    }
    if (modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_STATUS_FLAGS, &regVal)) {
        model.protocolState = regVal;
    }

    batteryModelSet(&model);
}

bool rs485GrowattBuildDecodedPacket(const modbusDecoder_t *decoder,
                                    uint32_t sequence,
                                    bms_decoded_packet_t *outPacket)
{
    if (decoder == NULL || outPacket == NULL) {
        return false;
    }

    memset(outPacket, 0, sizeof(*outPacket));
    outPacket->sourceProtocol = PROTOCOL_ID_GROWATT;
    outPacket->sequence = sequence;
    outPacket->timestampUs = esp_timer_get_time();

    uint16_t regVal = 0;
    if (modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_SOC_PCT, &regVal)) {
        outPacket->hasSoc = true;
        outPacket->socPct = clampU8(regVal, 100u);
    }
    if (modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_TEMP_C, &regVal)) {
        outPacket->hasTemperatureC = true;
        outPacket->temperatureC = (int16_t)regVal;
    }
    if (modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_PACK_V_CV, &regVal)) {
        outPacket->hasPackVoltageCv = true;
        outPacket->packVoltageCv = rs485GrowattPackVoltageRawToCv(decoder, regVal);
    }
    if (modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_STATUS_FLAGS, &regVal)) {
        outPacket->hasStatusFlags = true;
        outPacket->statusFlags = regVal;
    }
    if (modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_ERROR_CODE, &regVal)) {
        outPacket->hasProtectionFlags = true;
        outPacket->protectionFlags = (uint16_t)(regVal & GROWATT_ERROR_CODE_MASK);
    }
    if (modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_WARNING_CODE, &regVal)) {
        outPacket->hasWarningFlags = true;
        outPacket->warningFlags = (uint16_t)(regVal & GROWATT_WARNING_CODE_MASK);
    }

    uint16_t minCell = 0;
    uint16_t maxCell = 0;
    uint16_t minIdx = 0;
    uint16_t maxIdx = 0;
    bool hasMinCell = modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_CELL_MIN_MV, &minCell);
    bool hasMaxCell = modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_CELL_MAX_MV, &maxCell);
    bool hasMinIdx = modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_CELL_MIN_IDX, &minIdx);
    bool hasMaxIdx = modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_CELL_MAX_IDX, &maxIdx);
    if (hasMinCell && hasMaxCell && hasMinIdx && hasMaxIdx) {
        outPacket->hasCellExtremes = true;
        outPacket->minCellMv = minCell;
        outPacket->maxCellMv = maxCell;
        outPacket->minCellIndex = clampU8(minIdx, UINT8_MAX);
        outPacket->maxCellIndex = clampU8(maxIdx, UINT8_MAX);
    }

    uint16_t computedMinCell = UINT16_MAX;
    uint16_t computedMaxCell = 0u;
    uint8_t computedMinIdx = 0u;
    uint8_t computedMaxIdx = 0u;
    uint8_t validCells = 0u;
    for (uint8_t i = 0u; i < RS485_GROWATT_MB_CELL_COUNT; i++) {
        const uint16_t addr = (uint16_t)(RS485_GROWATT_MB_REG_CELL_BASE + i);
        if (!modbusDecoderGetCachedReg(decoder, addr, &regVal)) {
            continue;
        }
        if (!growattCellVoltageValid(regVal)) {
            continue;
        }
        if (i < BMS_DECODED_PACKET_MAX_CELLS) {
            outPacket->cellMv[i] = regVal;
            outPacket->cellCount = (uint8_t)(i + 1u);
        }
        validCells++;
        if (regVal < computedMinCell) {
            computedMinCell = regVal;
            computedMinIdx = (uint8_t)(i + 1u);
        }
        if (regVal > computedMaxCell) {
            computedMaxCell = regVal;
            computedMaxIdx = (uint8_t)(i + 1u);
        }
    }
    if (validCells > 0u && !outPacket->hasCellExtremes) {
        outPacket->hasCellExtremes = true;
        outPacket->minCellMv = computedMinCell;
        outPacket->maxCellMv = computedMaxCell;
        outPacket->minCellIndex = computedMinIdx;
        outPacket->maxCellIndex = computedMaxIdx;
    }

    return outPacket->hasSoc ||
           outPacket->hasTemperatureC ||
           outPacket->hasPackVoltageCv ||
           outPacket->hasCellExtremes ||
           (outPacket->cellCount > 0u) ||
           outPacket->hasStatusFlags ||
           outPacket->hasProtectionFlags ||
           outPacket->hasWarningFlags;
}

static void rs485GrowattBmsTask(void *pv)
{
    rs485GrowattBmsTaskCtx_t *ctx = (rs485GrowattBmsTaskCtx_t *)pv;
    uint8_t rxChunk[RS485_BUF_SIZE];
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    const uint8_t bmsPort = (settings.bms_port == 2u) ? 2u : 1u;
    const uart_port_t rxUart = (bmsPort == 2u) ? rs485GetUart2() : rs485GetUart1();
    const gpio_num_t dirPin = (bmsPort == 2u) ? rs485GetDir2() : rs485GetDir1();
    const char *ifName = (bmsPort == 2u) ? "GROWATT_RS485_2" : "GROWATT_RS485_1";
    int64_t lastRecordedReqUs = 0;

    modbusDecoderInit(&ctx->decoder, ifName, GROWATT_BMS_MODBUS_GAP_US);
    rs485GrowattModbusPollerInit(&ctx->poller,
                                 rxUart,
                                 dirPin,
                                 (uint8_t)GROWATT_BMS_MODBUS_SLAVE_ADDR);
    uart_flush_input(rxUart);

    while (1) {
        int len = uart_read_bytes(rxUart, rxChunk, sizeof(rxChunk), pdMS_TO_TICKS(10));
        int64_t nowUs = esp_timer_get_time();

        if (len > 0) {
            modbusDecoderFeed(&ctx->decoder, rxChunk, len, nowUs);
        } else if (ctx->decoder.haveLastByte &&
                   ((nowUs - ctx->decoder.lastByteUs) > (int64_t)ctx->decoder.gapUs)) {
            modbusDecoderFlush(&ctx->decoder);
        }

        esp_err_t pollErr = rs485GrowattModbusPollerTick(&ctx->poller,
                                                         nowUs,
                                                         GROWATT_BMS_QUERY_PERIOD_MS);
        if (pollErr != ESP_OK && pollErr != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(EXAMPLE_TAG, "RS485 Growatt poll TX failed (err=0x%x)", (unsigned)pollErr);
        } else if (ctx->poller.lastReqValid && ctx->poller.lastReqUs != lastRecordedReqUs) {
            modbusDecoderRecordRequest(&ctx->decoder,
                                       ctx->poller.lastReqSlave,
                                       ctx->poller.lastReqFunc,
                                       ctx->poller.lastReqStart,
                                       ctx->poller.lastReqCount,
                                       ctx->poller.lastReqUs);
            lastRecordedReqUs = ctx->poller.lastReqUs;
        }

        if ((nowUs - ctx->lastPublishUs) >= ((int64_t)GROWATT_BMS_PUBLISH_PERIOD_MS * 1000LL)) {
            int64_t newestCacheUs = 0;
            if (!decoderCacheFresh(&ctx->decoder, nowUs, &newestCacheUs)) {
                batteryModelClear();
                rs485GrowattClearLatestPacket();
                if ((nowUs - g_lastSourceStaleLogUs) >= 1000000LL) {
                    if (newestCacheUs <= 0) {
                        ESP_LOGW(EXAMPLE_TAG,
                                 "RS485 Growatt source stale: clearing published data (no valid BMS response yet)");
                    } else {
                        const uint32_t ageMs = (uint32_t)((nowUs - newestCacheUs) / 1000LL);
                        ESP_LOGW(EXAMPLE_TAG,
                                 "RS485 Growatt source stale: clearing published data (last_rx_age=%u ms)",
                                 (unsigned)ageMs);
                    }
                    g_lastSourceStaleLogUs = nowUs;
                }
            } else {
                bms_decoded_packet_t packet = {0};
                if (rs485GrowattBuildDecodedPacket(&ctx->decoder, ++ctx->sequence, &packet)) {
                    rs485GrowattStoreLatestPacket(&packet);
                    rs485GrowattPublishBatteryModel(&ctx->decoder, &packet);
                    if (xQueueOverwrite(ctx->outQueue, &packet) != pdPASS) {
                        ESP_LOGW(EXAMPLE_TAG, "RS485 Growatt output queue overwrite failed");
                    }
                }
            }
            ctx->lastPublishUs = nowUs;
        }

    }
}

esp_err_t rs485GrowattBmsTaskStart(QueueHandle_t outQueue)
{
    if (outQueue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_rs485GrowattBmsTaskHandle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&g_rs485GrowattBmsCtx, 0, sizeof(g_rs485GrowattBmsCtx));
    g_rs485GrowattBmsCtx.outQueue = outQueue;
    g_lastSourceStaleLogUs = 0;
    batteryModelClear();
    rs485GrowattClearLatestPacket();

    BaseType_t taskOk =
        xTaskCreate(rs485GrowattBmsTask,
                    "rs485_gw_bms",
                    GROWATT_BMS_TASK_STACK,
                    &g_rs485GrowattBmsCtx,
                    GROWATT_BMS_TASK_PRIORITY,
                    &g_rs485GrowattBmsTaskHandle);
    if (taskOk != pdPASS) {
        g_rs485GrowattBmsTaskHandle = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "RS485 Growatt BMS task started (slave=%u poll=%dms, publish=%dms)",
             (unsigned)GROWATT_BMS_MODBUS_SLAVE_ADDR,
             GROWATT_BMS_QUERY_PERIOD_MS,
             GROWATT_BMS_PUBLISH_PERIOD_MS);
    return ESP_OK;
}

bool rs485GrowattBmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket)
{
    bool hasPacket = false;

    if (outPacket == NULL) {
        return false;
    }

    portENTER_CRITICAL(&g_latestPacketMux);
    hasPacket = g_haveLatestPacket;
    if (hasPacket) {
        *outPacket = g_latestPacket;
    }
    portEXIT_CRITICAL(&g_latestPacketMux);

    return hasPacket;
}

esp_err_t rs485GrowattBmsTaskStop(void)
{
    if (g_rs485GrowattBmsTaskHandle == NULL) {
        return ESP_OK;
    }

    vTaskDelete(g_rs485GrowattBmsTaskHandle);
    g_rs485GrowattBmsTaskHandle = NULL;
    memset(&g_rs485GrowattBmsCtx, 0, sizeof(g_rs485GrowattBmsCtx));
    batteryModelClear();

    portENTER_CRITICAL(&g_latestPacketMux);
    g_haveLatestPacket = false;
    memset(&g_latestPacket, 0, sizeof(g_latestPacket));
    portEXIT_CRITICAL(&g_latestPacketMux);

    return ESP_OK;
}
