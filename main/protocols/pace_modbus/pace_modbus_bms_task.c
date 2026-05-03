#include "protocols/pace_modbus/pace_modbus_bms_task.h"

#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "Drivers/rs485_driver.h"
#include "config.h"
#include "protocols/common/battery_model.h"
#include "protocols/pace_modbus/pace_modbus_poller.h"
#include "protocols/pace_modbus/pace_modbus_registers_map.h"
#include "runtime_settings.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    QueueHandle_t outQueue;
    modbusDecoder_t decoder;
    pace_modbus_poller_t poller;
    uint32_t sequence;
    int64_t lastPublishUs;
} paceModbusBmsTaskCtx_t;

static paceModbusBmsTaskCtx_t g_paceModbusBmsCtx;
static TaskHandle_t g_paceModbusBmsTaskHandle;
static portMUX_TYPE g_latestPacketMux = portMUX_INITIALIZER_UNLOCKED;
static bool g_haveLatestPacket;
static bms_decoded_packet_t g_latestPacket;
static int64_t g_lastSourceStaleLogUs;
static int64_t g_lastDecodeLogUs;

#define PACE_MIN_CELL_MV         1000u
#define PACE_MAX_CELL_MV         6000u

static uint8_t clampU8(uint16_t v, uint8_t vmax)
{
    return (uint8_t)((v > vmax) ? vmax : v);
}

static uint8_t decodePackedPct(uint16_t raw)
{
    uint16_t lo = raw & 0x00FFu;
    uint16_t hi = (raw >> 8) & 0x00FFu;

    if (hi <= 100u) {
        if (lo == 0u && hi > 0u) {
            return (uint8_t)hi;
        }
    }
    if (lo <= 100u) {
        return (uint8_t)lo;
    }
    if (hi <= 100u) {
        return (uint8_t)hi;
    }
    return clampU8(raw, 100u);
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

static void paceStoreLatestPacket(const bms_decoded_packet_t *packet)
{
    if (packet == NULL) {
        return;
    }

    portENTER_CRITICAL(&g_latestPacketMux);
    g_latestPacket = *packet;
    g_haveLatestPacket = true;
    portEXIT_CRITICAL(&g_latestPacketMux);
}

static void paceClearLatestPacket(void)
{
    portENTER_CRITICAL(&g_latestPacketMux);
    g_haveLatestPacket = false;
    memset(&g_latestPacket, 0, sizeof(g_latestPacket));
    portEXIT_CRITICAL(&g_latestPacketMux);
}

static bool getI16Reg(const modbusDecoder_t *decoder, uint16_t addr, int16_t *out)
{
    uint16_t raw = 0;
    if (out == NULL || !modbusDecoderGetCachedReg(decoder, addr, &raw)) {
        return false;
    }
    *out = (int16_t)raw;
    return true;
}

static void paceFormatRawReg(const modbusDecoder_t *decoder,
                             uint16_t addr,
                             char *out,
                             size_t outSize)
{
    uint16_t raw = 0;

    if (out == NULL || outSize == 0u) {
        return;
    }
    if (decoder != NULL && modbusDecoderGetCachedReg(decoder, addr, &raw)) {
        (void)snprintf(out, outSize, "0x%04X/%u", (unsigned)raw, (unsigned)raw);
    } else {
        (void)snprintf(out, outSize, "missing");
    }
}

static void paceFormatCellSlice(const modbusDecoder_t *decoder,
                                uint8_t firstCell,
                                uint8_t lastCell,
                                char *out,
                                size_t outSize)
{
    size_t pos = 0u;

    if (out == NULL || outSize == 0u) {
        return;
    }
    out[0] = '\0';

    for (uint8_t cell = firstCell; cell <= lastCell; cell++) {
        uint16_t raw = 0;
        uint16_t addr = (uint16_t)(PACE_MB_REG_CELL01_MV + (uint16_t)(cell - 1u));
        int written = 0;

        if (pos + 1u >= outSize) {
            break;
        }

        if (modbusDecoderGetCachedReg(decoder, addr, &raw)) {
            written = snprintf(&out[pos],
                               outSize - pos,
                               "%s%02u=%.3fV(0x%04X)",
                               (pos == 0u) ? "" : " ",
                               (unsigned)cell,
                               (double)raw / 1000.0,
                               (unsigned)raw);
        } else {
            written = snprintf(&out[pos],
                               outSize - pos,
                               "%s%02u=missing",
                               (pos == 0u) ? "" : " ",
                               (unsigned)cell);
        }

        if (written <= 0) {
            break;
        }
        pos += (size_t)written;
    }
}

static void paceFormatTempRegs(const modbusDecoder_t *decoder, char *out, size_t outSize)
{
    static const char *const labels[PACE_MB_TEMP_REG_COUNT] = {
        "T1", "T2", "T4", "T5", "MOS", "ENV",
    };
    size_t pos = 0u;

    if (out == NULL || outSize == 0u) {
        return;
    }
    out[0] = '\0';

    for (uint8_t i = 0u; i < PACE_MB_TEMP_REG_COUNT; i++) {
        int16_t deciC = 0;
        int written = 0;
        uint16_t addr = (uint16_t)(PACE_MB_REG_TEMP1_DECIC + i);

        if (pos + 1u >= outSize) {
            break;
        }

        if (getI16Reg(decoder, addr, &deciC)) {
            written = snprintf(&out[pos],
                               outSize - pos,
                               "%s%s=%.1fC",
                               (pos == 0u) ? "" : " ",
                               labels[i],
                               (double)deciC / 10.0);
        } else {
            written = snprintf(&out[pos],
                               outSize - pos,
                               "%s%s=missing",
                               (pos == 0u) ? "" : " ",
                               labels[i]);
        }

        if (written <= 0) {
            break;
        }
        pos += (size_t)written;
    }
}

static void paceLogDecodedPacket(const modbusDecoder_t *decoder,
                                 const bms_decoded_packet_t *packet,
                                 int64_t nowUs,
                                 bool accepted)
{
    char rawCurrent[24];
    char rawPack[24];
    char rawSoc[24];
    char rawSoh[24];
    char rawWarn[24];
    char rawProt[24];
    char rawStatus[24];
    char cellsA[220];
    char cellsB[220];
    char temps[140];

    if (decoder == NULL || packet == NULL) {
        return;
    }
    if ((nowUs - g_lastDecodeLogUs) < 5000000LL) {
        return;
    }
    g_lastDecodeLogUs = nowUs;

    paceFormatRawReg(decoder, PACE_MB_REG_CURRENT_10MA, rawCurrent, sizeof(rawCurrent));
    paceFormatRawReg(decoder, PACE_MB_REG_PACK_VOLTAGE_10MV, rawPack, sizeof(rawPack));
    paceFormatRawReg(decoder, PACE_MB_REG_SOC_PCT, rawSoc, sizeof(rawSoc));
    paceFormatRawReg(decoder, PACE_MB_REG_SOH_PCT, rawSoh, sizeof(rawSoh));
    paceFormatRawReg(decoder, PACE_MB_REG_WARNING_FLAGS, rawWarn, sizeof(rawWarn));
    paceFormatRawReg(decoder, PACE_MB_REG_PROTECTION_FLAGS, rawProt, sizeof(rawProt));
    paceFormatRawReg(decoder, PACE_MB_REG_STATUS_FLAGS, rawStatus, sizeof(rawStatus));
    paceFormatCellSlice(decoder, 1u, 8u, cellsA, sizeof(cellsA));
    paceFormatCellSlice(decoder, 9u, 16u, cellsB, sizeof(cellsB));
    paceFormatTempRegs(decoder, temps, sizeof(temps));

    ESP_LOGI(EXAMPLE_TAG,
             "PACE decoded %s: seq=%" PRIu32 " soc=%u%% pack=%.2fV temp=%dC cell_min=%.3fV#%u cell_max=%.3fV#%u",
             accepted ? "accepted" : "rejected",
             packet->sequence,
             packet->hasSoc ? (unsigned)packet->socPct : 0u,
             packet->hasPackVoltageCv ? ((double)packet->packVoltageCv / 100.0) : 0.0,
             packet->hasTemperatureC ? (int)packet->temperatureC : 0,
             packet->hasCellExtremes ? ((double)packet->minCellMv / 1000.0) : 0.0,
             packet->hasCellExtremes ? (unsigned)packet->minCellIndex : 0u,
             packet->hasCellExtremes ? ((double)packet->maxCellMv / 1000.0) : 0.0,
             packet->hasCellExtremes ? (unsigned)packet->maxCellIndex : 0u);
    ESP_LOGI(EXAMPLE_TAG,
             "PACE raw summary: current=%s pack=%s soc=%s soh=%s warn=%s prot=%s status=%s",
             rawCurrent,
             rawPack,
             rawSoc,
             rawSoh,
             rawWarn,
             rawProt,
             rawStatus);
    ESP_LOGI(EXAMPLE_TAG, "PACE raw cells 01-08: %s", cellsA);
    ESP_LOGI(EXAMPLE_TAG, "PACE raw cells 09-16: %s", cellsB);
    ESP_LOGI(EXAMPLE_TAG, "PACE raw temps: %s", temps);
}

static void pacePublishBatteryModel(const modbusDecoder_t *decoder,
                                    const bms_decoded_packet_t *packet)
{
    battery_model_t model = {0};
    uint16_t regVal = 0;
    int16_t i16 = 0;

    if (decoder == NULL || packet == NULL) {
        return;
    }

    model.valid = true;
    model.updatedMs = (uint32_t)(packet->timestampUs / 1000LL);

    if (packet->hasSoc) {
        model.socPct = packet->socPct;
    }
    if (modbusDecoderGetCachedReg(decoder, PACE_MB_REG_SOH_PCT, &regVal)) {
        model.sohPct = decodePackedPct(regVal);
    }
    if (packet->hasPackVoltageCv) {
        model.packVoltageV = (float)packet->packVoltageCv / 100.0f;
    }
    if (getI16Reg(decoder, PACE_MB_REG_CURRENT_10MA, &i16)) {
        model.packCurrentA = (float)i16 * 0.01f;
    }
    if (modbusDecoderGetCachedReg(decoder, PACE_MB_REG_CYCLE_COUNT, &regVal)) {
        model.cycleCount = regVal;
    }
    if (modbusDecoderGetCachedReg(decoder, PACE_MB_REG_WARNING_FLAGS, &regVal)) {
        model.warningsMask = regVal;
    }
    if (modbusDecoderGetCachedReg(decoder, PACE_MB_REG_PROTECTION_FLAGS, &regVal)) {
        model.alarmsMask = regVal;
    }
    if (modbusDecoderGetCachedReg(decoder, PACE_MB_REG_STATUS_FLAGS, &regVal)) {
        model.protocolState = regVal;
        model.chargeEnabled = (regVal & PACE_MB_STATUS_MOSFET_CHG) != 0u;
        model.dischargeEnabled = (regVal & PACE_MB_STATUS_MOSFET_DCHG) != 0u;
    }
    if (modbusDecoderGetCachedReg(decoder, PACE_MB_REG_BALANCE_STATUS, &regVal)) {
        model.balanceEnabled = regVal != 0u;
    }

    for (uint8_t i = 0u; i < UNIVERSAL_BATTERY_TEMP_SENSORS; i++) {
        model.temperaturesC[i] = -100.0f;
    }
    if (getI16Reg(decoder, PACE_MB_REG_MOS_TEMP_DECIC, &i16)) {
        model.temperaturesC[0] = (float)i16 / 10.0f;
    }
    if (getI16Reg(decoder, PACE_MB_REG_TEMP1_DECIC, &i16)) {
        model.temperaturesC[1] = (float)i16 / 10.0f;
    }
    if (getI16Reg(decoder, PACE_MB_REG_TEMP2_DECIC, &i16)) {
        model.temperaturesC[2] = (float)i16 / 10.0f;
    }
    if (getI16Reg(decoder, PACE_MB_REG_TEMP3_DECIC, &i16)) {
        model.temperaturesC[3] = (float)i16 / 10.0f;
    }
    if (getI16Reg(decoder, PACE_MB_REG_TEMP4_DECIC, &i16)) {
        model.temperaturesC[4] = (float)i16 / 10.0f;
    }

    if (packet->hasCellExtremes) {
        model.cellMinV = (float)packet->minCellMv / 1000.0f;
        model.cellMaxV = (float)packet->maxCellMv / 1000.0f;
        model.cellMinIdx = packet->minCellIndex;
        model.cellMaxIdx = packet->maxCellIndex;
        model.cellDeltaV = (float)(packet->maxCellMv - packet->minCellMv) / 1000.0f;
    }

    batteryModelSet(&model);
}

bool paceModbusBuildDecodedPacket(const modbusDecoder_t *decoder,
                                  uint32_t sequence,
                                  bms_decoded_packet_t *outPacket)
{
    uint16_t regVal = 0;

    if (decoder == NULL || outPacket == NULL) {
        return false;
    }

    memset(outPacket, 0, sizeof(*outPacket));
    outPacket->sourceProtocol = PROTOCOL_ID_PACE;
    outPacket->sequence = sequence;
    outPacket->timestampUs = esp_timer_get_time();

    if (modbusDecoderGetCachedReg(decoder, PACE_MB_REG_SOC_PCT, &regVal)) {
        outPacket->hasSoc = true;
        outPacket->socPct = decodePackedPct(regVal);
    }
    if (modbusDecoderGetCachedReg(decoder, PACE_MB_REG_PACK_VOLTAGE_10MV, &regVal)) {
        outPacket->hasPackVoltageCv = true;
        outPacket->packVoltageCv = regVal;
    }
    if (modbusDecoderGetCachedReg(decoder, PACE_MB_REG_WARNING_FLAGS, &regVal)) {
        outPacket->hasWarningFlags = true;
        outPacket->warningFlags = regVal;
    }
    if (modbusDecoderGetCachedReg(decoder, PACE_MB_REG_PROTECTION_FLAGS, &regVal)) {
        outPacket->hasProtectionFlags = true;
        outPacket->protectionFlags = regVal;
    }
    if (modbusDecoderGetCachedReg(decoder, PACE_MB_REG_STATUS_FLAGS, &regVal)) {
        outPacket->hasStatusFlags = true;
        outPacket->statusFlags = regVal;
    }
    if (modbusDecoderGetCachedReg(decoder, PACE_MB_REG_BALANCE_STATUS, &regVal)) {
        outPacket->hasBalanceFlags = true;
        outPacket->balanceFlags = regVal;
    }

    int32_t tempSumDeciC = 0;
    uint8_t tempSamples = 0;
    for (uint8_t i = 0u; i < PACE_MB_TEMP_REG_COUNT; i++) {
        int16_t tempDeciC = 0;
        uint16_t addr = (uint16_t)(PACE_MB_REG_TEMP1_DECIC + i);
        if (getI16Reg(decoder, addr, &tempDeciC)) {
            tempSumDeciC += tempDeciC;
            tempSamples++;
            if (i < BMS_DECODED_PACKET_MAX_TEMPS) {
                outPacket->tempDeciC[i] = tempDeciC;
                outPacket->tempCount = (uint8_t)(i + 1u);
            }
        }
    }
    if (tempSamples > 0u) {
        outPacket->hasTemperatureC = true;
        outPacket->temperatureC = (int16_t)((tempSumDeciC / (int32_t)tempSamples) / 10);
    }

    uint16_t minCell = UINT16_MAX;
    uint16_t maxCell = 0u;
    uint8_t minIdx = 0u;
    uint8_t maxIdx = 0u;
    uint8_t validCells = 0u;
    for (uint8_t i = 0u; i < PACE_MB_CELL_COUNT; i++) {
        const uint16_t addr = (uint16_t)(PACE_MB_REG_CELL01_MV + i);
        if (!modbusDecoderGetCachedReg(decoder, addr, &regVal)) {
            continue;
        }
        if (regVal < PACE_MIN_CELL_MV || regVal > PACE_MAX_CELL_MV) {
            continue;
        }
        if (i < BMS_DECODED_PACKET_MAX_CELLS) {
            outPacket->cellMv[i] = regVal;
            outPacket->cellCount = (uint8_t)(i + 1u);
        }

        validCells++;
        if (regVal < minCell) {
            minCell = regVal;
            minIdx = (uint8_t)(i + 1u);
        }
        if (regVal > maxCell) {
            maxCell = regVal;
            maxIdx = (uint8_t)(i + 1u);
        }
    }
    if (validCells > 0u) {
        outPacket->hasCellExtremes = true;
        outPacket->minCellMv = minCell;
        outPacket->maxCellMv = maxCell;
        outPacket->minCellIndex = minIdx;
        outPacket->maxCellIndex = maxIdx;
    }

    return outPacket->hasSoc ||
           outPacket->hasTemperatureC ||
           outPacket->hasPackVoltageCv ||
           outPacket->hasCellExtremes ||
           (outPacket->cellCount > 0u) ||
           outPacket->hasWarningFlags ||
           outPacket->hasProtectionFlags ||
           outPacket->hasStatusFlags ||
           outPacket->hasBalanceFlags;
}

static void paceModbusBmsTask(void *pv)
{
    paceModbusBmsTaskCtx_t *ctx = (paceModbusBmsTaskCtx_t *)pv;
    uint8_t rxChunk[RS485_BUF_SIZE];
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    const uint8_t bmsPort = (settings.bms_port == 2u) ? 2u : 1u;
    const uart_port_t rxUart = (bmsPort == 2u) ? rs485GetUart2() : rs485GetUart1();
    const gpio_num_t dirPin = (bmsPort == 2u) ? rs485GetDir2() : rs485GetDir1();
    const char *ifName = (bmsPort == 2u) ? "PACE_RS485_2" : "PACE_RS485_1";
    int64_t lastRecordedReqUs = 0;

    modbusDecoderInit(&ctx->decoder, ifName, PACE_BMS_MODBUS_GAP_US);
    paceModbusPollerInit(&ctx->poller,
                         rxUart,
                         dirPin,
                         (uint8_t)PACE_BMS_MODBUS_SLAVE_ADDR);
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

        esp_err_t pollErr = paceModbusPollerTick(&ctx->poller,
                                                 nowUs,
                                                 PACE_BMS_QUERY_PERIOD_MS);
        if (pollErr != ESP_OK && pollErr != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(EXAMPLE_TAG, "PACE Modbus poll TX failed (err=0x%x)", (unsigned)pollErr);
        } else if (ctx->poller.lastReqValid && ctx->poller.lastReqUs != lastRecordedReqUs) {
            modbusDecoderRecordRequest(&ctx->decoder,
                                       ctx->poller.lastReqSlave,
                                       ctx->poller.lastReqFunc,
                                       ctx->poller.lastReqStart,
                                       ctx->poller.lastReqCount,
                                       ctx->poller.lastReqUs);
            lastRecordedReqUs = ctx->poller.lastReqUs;
        }

        if ((nowUs - ctx->lastPublishUs) >= ((int64_t)PACE_BMS_PUBLISH_PERIOD_MS * 1000LL)) {
            int64_t newestCacheUs = 0;
            if (!decoderCacheFresh(&ctx->decoder, nowUs, &newestCacheUs)) {
                batteryModelClear();
                if ((nowUs - g_lastSourceStaleLogUs) >= 1000000LL) {
                    if (newestCacheUs <= 0) {
                        ESP_LOGW(EXAMPLE_TAG,
                                 "PACE Modbus source stale: clearing published data (no valid BMS response yet)");
                    } else {
                        const uint32_t ageMs = (uint32_t)((nowUs - newestCacheUs) / 1000LL);
                        ESP_LOGW(EXAMPLE_TAG,
                                 "PACE Modbus source stale: clearing published data (last_rx_age=%u ms)",
                                 (unsigned)ageMs);
                    }
                    g_lastSourceStaleLogUs = nowUs;
                }
            } else {
                bms_decoded_packet_t packet = {0};
                if (paceModbusBuildDecodedPacket(&ctx->decoder, ++ctx->sequence, &packet)) {
                    paceLogDecodedPacket(&ctx->decoder, &packet, nowUs, true);
                    paceStoreLatestPacket(&packet);
                    pacePublishBatteryModel(&ctx->decoder, &packet);
                    if (xQueueOverwrite(ctx->outQueue, &packet) != pdPASS) {
                        ESP_LOGW(EXAMPLE_TAG, "PACE output queue overwrite failed");
                    }
                }
            }
            ctx->lastPublishUs = nowUs;
        }
    }
}

esp_err_t paceModbusBmsTaskStart(QueueHandle_t outQueue)
{
    if (outQueue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_paceModbusBmsTaskHandle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&g_paceModbusBmsCtx, 0, sizeof(g_paceModbusBmsCtx));
    g_paceModbusBmsCtx.outQueue = outQueue;
    g_lastSourceStaleLogUs = 0;
    g_lastDecodeLogUs = 0;
    batteryModelClear();
    paceClearLatestPacket();

    BaseType_t taskOk =
        xTaskCreate(paceModbusBmsTask,
                    "pace_bms",
                    PACE_BMS_TASK_STACK,
                    &g_paceModbusBmsCtx,
                    PACE_BMS_TASK_PRIORITY,
                    &g_paceModbusBmsTaskHandle);
    if (taskOk != pdPASS) {
        g_paceModbusBmsTaskHandle = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "PACE Modbus BMS task started (slave=%u poll=%dms, publish=%dms)",
             (unsigned)PACE_BMS_MODBUS_SLAVE_ADDR,
             PACE_BMS_QUERY_PERIOD_MS,
             PACE_BMS_PUBLISH_PERIOD_MS);
    return ESP_OK;
}

bool paceModbusBmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket)
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

esp_err_t paceModbusBmsTaskStop(void)
{
    if (g_paceModbusBmsTaskHandle == NULL) {
        return ESP_OK;
    }

    vTaskDelete(g_paceModbusBmsTaskHandle);
    g_paceModbusBmsTaskHandle = NULL;
    memset(&g_paceModbusBmsCtx, 0, sizeof(g_paceModbusBmsCtx));
    g_lastDecodeLogUs = 0;
    batteryModelClear();
    paceClearLatestPacket();

    return ESP_OK;
}
