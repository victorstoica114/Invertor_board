#include "protocols/jkbms_modbus/jkbms_modbus_bms_task.h"

#include <limits.h>
#include <string.h>

#include "Drivers/rs485_driver.h"
#include "config.h"
#include "modbusDecoder.h"
#include "orchestrator/protocol_types.h"
#include "protocols/jkbms_modbus/jkbms_modbus_poller.h"
#include "protocols/jkbms_modbus/jkbms_modbus_register_map.h"
#include "runtime_settings.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    QueueHandle_t outQueue;
    modbusDecoder_t decoder;
    jkbms_modbus_poller_t poller;
    uint32_t sequence;
    int64_t lastPublishUs;
} jkbmsModbusBmsTaskCtx_t;

static jkbmsModbusBmsTaskCtx_t g_jkbmsModbusBmsCtx;
static TaskHandle_t g_jkbmsModbusBmsTaskHandle;
static portMUX_TYPE g_latestPacketMux = portMUX_INITIALIZER_UNLOCKED;
static bool g_haveLatestPacket;
static bms_decoded_packet_t g_latestPacket;
static bool g_haveLatestSnapshot;
static jkbms_modbus_snapshot_t g_latestSnapshot;

static void jkbmsStoreLatestPacket(const bms_decoded_packet_t *packet)
{
    if (packet == NULL) {
        return;
    }

    portENTER_CRITICAL(&g_latestPacketMux);
    g_latestPacket = *packet;
    g_haveLatestPacket = true;
    portEXIT_CRITICAL(&g_latestPacketMux);
}

static void jkbmsStoreLatestSnapshot(const jkbms_modbus_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    portENTER_CRITICAL(&g_latestPacketMux);
    g_latestSnapshot = *snapshot;
    g_haveLatestSnapshot = true;
    portEXIT_CRITICAL(&g_latestPacketMux);
}

static bool decoderGetU16(const modbusDecoder_t *decoder, uint16_t reg, uint16_t *out)
{
    return modbusDecoderGetCachedReg(decoder, reg, out);
}

static bool decoderGetI16(const modbusDecoder_t *decoder, uint16_t reg, int16_t *out)
{
    uint16_t raw = 0;
    if (!modbusDecoderGetCachedReg(decoder, reg, &raw)) {
        return false;
    }

    if (out != NULL) {
        *out = (int16_t)raw;
    }
    return true;
}

static bool decoderGetU32(const modbusDecoder_t *decoder, uint16_t reg, uint32_t *out)
{
    uint16_t hi = 0;
    uint16_t lo = 0;
    if (!modbusDecoderGetCachedReg(decoder, reg, &hi)) {
        return false;
    }
    if (!modbusDecoderGetCachedReg(decoder, (uint16_t)(reg + 1u), &lo)) {
        return false;
    }

    if (out != NULL) {
        *out = (((uint32_t)hi) << 16) | (uint32_t)lo;
    }
    return true;
}

static bool decoderGetI32(const modbusDecoder_t *decoder, uint16_t reg, int32_t *out)
{
    uint32_t raw = 0u;
    if (!decoderGetU32(decoder, reg, &raw)) {
        return false;
    }

    if (out != NULL) {
        *out = (int32_t)raw;
    }
    return true;
}

static bool decodePctFromU8x2(uint16_t raw, uint8_t *pctOut)
{
    const uint8_t hi = (uint8_t)((raw >> 8) & 0xFFu);
    const uint8_t lo = (uint8_t)(raw & 0xFFu);

    if (lo <= 100u) {
        if (pctOut != NULL) {
            *pctOut = lo;
        }
        return true;
    }
    if (hi <= 100u) {
        if (pctOut != NULL) {
            *pctOut = hi;
        }
        return true;
    }
    return false;
}

static bool decodeSocPct(const modbusDecoder_t *decoder, uint8_t *socOut)
{
    uint16_t raw = 0u;

    if (decoderGetU16(decoder, JKBMS_RT_REG_BALAN_SOC_U8X2, &raw) &&
        decodePctFromU8x2(raw, socOut)) {
        return true;
    }

    if (decoderGetU16(decoder, (uint16_t)(JKBMS_RT_REG_BALAN_SOC_U8X2 + 1u), &raw) &&
        decodePctFromU8x2(raw, socOut)) {
        return true;
    }

    /* Some JK firmwares expose SOC in the SOH/PRECHARGE word as fallback. */
    if (decoderGetU16(decoder, JKBMS_RT_REG_SOH_PRECHARGE_U8X2, &raw) &&
        decodePctFromU8x2(raw, socOut)) {
        return true;
    }

    return false;
}

static bool decodeSohPrecharge(const modbusDecoder_t *decoder,
                               uint8_t *sohOut,
                               uint8_t *prechargeOut)
{
    uint16_t raw = 0u;
    if (!decoderGetU16(decoder, JKBMS_RT_REG_SOH_PRECHARGE_U8X2, &raw)) {
        return false;
    }

    const uint8_t hi = (uint8_t)((raw >> 8) & 0xFFu);
    const uint8_t lo = (uint8_t)(raw & 0xFFu);
    bool hasSoh = false;

    if (hi <= 100u && lo <= 1u) {
        if (sohOut != NULL) {
            *sohOut = hi;
        }
        if (prechargeOut != NULL) {
            *prechargeOut = lo;
        }
        return true;
    }

    if (lo <= 100u && hi <= 1u) {
        if (sohOut != NULL) {
            *sohOut = lo;
        }
        if (prechargeOut != NULL) {
            *prechargeOut = hi;
        }
        return true;
    }

    if (hi <= 100u) {
        if (sohOut != NULL) {
            *sohOut = hi;
        }
        hasSoh = true;
    } else if (lo <= 100u) {
        if (sohOut != NULL) {
            *sohOut = lo;
        }
        hasSoh = true;
    }

    if (prechargeOut != NULL) {
        *prechargeOut = lo;
    }

    return hasSoh;
}

static bool buildDecodedSnapshot(const modbusDecoder_t *decoder, jkbms_modbus_snapshot_t *out)
{
    if (decoder == NULL || out == NULL) {
        return false;
    }

    memset(out, 0, sizeof(*out));

    uint8_t validCells = 0u;
    uint16_t minMv = UINT16_MAX;
    uint16_t maxMv = 0u;
    uint8_t minIdx = 0u;
    uint8_t maxIdx = 0u;

    for (uint8_t i = 0; i < JKBMS_MAX_CELLS; i++) {
        const uint16_t reg = (uint16_t)(JKBMS_RT_REG_CELL0_MV + ((uint16_t)i * JKBMS_RT_CELL_STEP));
        uint16_t mv = 0u;
        if (!decoderGetU16(decoder, reg, &mv)) {
            continue;
        }

        if (mv < 1000u || mv > 5000u) {
            continue;
        }

        out->cellMv[i] = mv;
        validCells++;

        if (mv < minMv) {
            minMv = mv;
            minIdx = (uint8_t)(i + 1u);
        }
        if (mv > maxMv) {
            maxMv = mv;
            maxIdx = (uint8_t)(i + 1u);
        }
    }

    out->cellCount = validCells;
    if (validCells > 0u) {
        out->hasCellExtremes = true;
        out->minCellMv = minMv;
        out->maxCellMv = maxMv;
        out->minCellIndex = minIdx;
        out->maxCellIndex = maxIdx;
    }

    uint16_t u16 = 0u;
    if (decoderGetU16(decoder, JKBMS_RT_REG_CELL_AVG_MV, &u16)) {
        out->hasCellAvgMv = true;
        out->cellAvgMv = u16;
    }
    if (decoderGetU16(decoder, JKBMS_RT_REG_CELL_VDIFF_MAX_MV, &u16)) {
        out->hasCellDiffMaxMv = true;
        out->cellDiffMaxMv = u16;
    }

    if (decoderGetU16(decoder, JKBMS_RT_REG_MAX_MIN_CELL_NBR_U8X2, &u16)) {
        const uint8_t idxA = (uint8_t)((u16 >> 8) & 0xFFu);
        const uint8_t idxB = (uint8_t)(u16 & 0xFFu);
        const bool idxAValid = (idxA >= 1u && idxA <= JKBMS_MAX_CELLS);
        const bool idxBValid = (idxB >= 1u && idxB <= JKBMS_MAX_CELLS);

        if (idxAValid && idxBValid) {
            if (out->hasCellExtremes && idxA == out->minCellIndex && idxB == out->maxCellIndex) {
                out->minCellIndex = idxA;
                out->maxCellIndex = idxB;
            } else if (out->hasCellExtremes && idxA == out->maxCellIndex && idxB == out->minCellIndex) {
                out->maxCellIndex = idxA;
                out->minCellIndex = idxB;
            } else {
                /* Protocol naming suggests max/min order. */
                out->maxCellIndex = idxA;
                out->minCellIndex = idxB;
            }
        }
    }

    int16_t i16 = 0;
    if (decoderGetI16(decoder, JKBMS_RT_REG_TEMP_MOS_DECIC, &i16)) {
        out->hasTempMosC = true;
        out->tempMosC = (int16_t)(i16 / 10);
    }
    if (decoderGetI16(decoder, JKBMS_RT_REG_TEMP_BAT1_DECIC, &i16)) {
        out->hasTempBat1C = true;
        out->tempBat1C = (int16_t)(i16 / 10);
    }
    if (decoderGetI16(decoder, JKBMS_RT_REG_TEMP_BAT2_DECIC, &i16)) {
        out->hasTempBat2C = true;
        out->tempBat2C = (int16_t)(i16 / 10);
    }

    uint32_t u32 = 0u;
    if (decoderGetU32(decoder, JKBMS_RT_REG_PACK_VOLT_MV_U32, &u32)) {
        out->hasPackVoltageMv = true;
        out->packVoltageMv = u32;
    }

    int32_t i32 = 0;
    if (decoderGetI32(decoder, JKBMS_RT_REG_PACK_CURRENT_MA_I32, &i32)) {
        out->hasPackCurrentMa = true;
        out->packCurrentMa = i32;
    }

    if (decoderGetI32(decoder, JKBMS_RT_REG_PACK_WATT_MW_U32, &i32)) {
        out->hasPackPowerMw = true;
        out->packPowerMw = i32;
    } else if (out->hasPackVoltageMv && out->hasPackCurrentMa) {
        int64_t mw = ((int64_t)out->packVoltageMv * (int64_t)out->packCurrentMa) / 1000LL;
        if (mw > INT32_MAX) {
            mw = INT32_MAX;
        }
        if (mw < INT32_MIN) {
            mw = INT32_MIN;
        }
        out->hasPackPowerMw = true;
        out->packPowerMw = (int32_t)mw;
    }

    if (decoderGetI16(decoder, JKBMS_RT_REG_BALAN_CURRENT_MA_I16, &i16)) {
        out->hasBalanceCurrentMa = true;
        out->balanceCurrentMa = i16;
    }

    if (decodeSocPct(decoder, &out->socPct)) {
        out->hasSoc = true;
    }

    if (decodeSohPrecharge(decoder, &out->sohPct, &out->prechargeState)) {
        out->hasSoh = true;
        out->hasPrecharge = true;
    }

    if (decoderGetI32(decoder, JKBMS_RT_REG_SOC_REMAIN_MAH_I32, &i32)) {
        out->hasRemainMah = true;
        out->remainMah = i32;
    }

    if (decoderGetU32(decoder, JKBMS_RT_REG_SOC_FULL_MAH_U32, &u32)) {
        out->hasFullMah = true;
        out->fullMah = u32;
    }

    if (decoderGetU32(decoder, JKBMS_RT_REG_CYCLE_COUNT_U32, &u32)) {
        out->hasCycles = true;
        out->cycles = u32;
    }

    if (decoderGetU32(decoder, JKBMS_RT_REG_ALARM_U32, &u32)) {
        out->hasAlarmBits = true;
        out->alarmBits = u32;
    }

    out->valid = out->hasSoc ||
                 out->hasSoh ||
                 out->hasTempMosC ||
                 out->hasTempBat1C ||
                 out->hasTempBat2C ||
                 out->hasPackVoltageMv ||
                 out->hasPackCurrentMa ||
                 out->hasPackPowerMw ||
                 out->hasBalanceCurrentMa ||
                 out->hasRemainMah ||
                 out->hasFullMah ||
                 out->hasCycles ||
                 out->hasCellExtremes ||
                 out->hasCellAvgMv ||
                 out->hasCellDiffMaxMv ||
                 out->hasAlarmBits;

    return out->valid;
}

static bool buildPacketFromSnapshot(const jkbms_modbus_snapshot_t *snapshot,
                                    uint32_t sequence,
                                    bms_decoded_packet_t *outPacket)
{
    if (snapshot == NULL || outPacket == NULL || !snapshot->valid) {
        return false;
    }

    memset(outPacket, 0, sizeof(*outPacket));
    outPacket->sourceProtocol = PROTOCOL_ID_JKBMS;
    outPacket->sequence = sequence;
    outPacket->timestampUs = esp_timer_get_time();

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

    return outPacket->hasSoc ||
           outPacket->hasTemperatureC ||
           outPacket->hasPackVoltageCv ||
           outPacket->hasCellExtremes;
}

static void jkbmsModbusBmsTask(void *pv)
{
    jkbmsModbusBmsTaskCtx_t *ctx = (jkbmsModbusBmsTaskCtx_t *)pv;
    uint8_t rxChunk[RS485_BUF_SIZE];
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    const uint8_t bmsPort = (settings.bms_port == 2u) ? 2u : 1u;
    const uart_port_t rxUart = (bmsPort == 2u) ? rs485GetUart2() : rs485GetUart1();
    const gpio_num_t dirPin = (bmsPort == 2u) ? rs485GetDir2() : rs485GetDir1();
    const char *ifName = (bmsPort == 2u) ? "JKBMS_RS485_2" : "JKBMS_RS485_1";

    modbusDecoderInit(&ctx->decoder, ifName, JKBMS_BMS_MODBUS_GAP_US);
    jkbmsModbusPollerInit(&ctx->poller,
                          rxUart,
                          dirPin,
                          (uint8_t)JKBMS_BMS_MODBUS_SLAVE_ADDR);
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

        esp_err_t pollErr = jkbmsModbusPollerTick(&ctx->poller,
                                                  nowUs,
                                                  JKBMS_BMS_QUERY_PERIOD_MS);
        if (pollErr != ESP_OK && pollErr != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(EXAMPLE_TAG, "JKBMS Modbus poll TX failed (err=0x%x)", (unsigned)pollErr);
        } else if (ctx->poller.lastReqValid) {
            /* On some RS485 transceivers TX bytes are not looped into RX. Seed decoder request context. */
            ctx->decoder.lastReqValid = true;
            ctx->decoder.lastReqSlave = ctx->poller.lastReqSlave;
            ctx->decoder.lastReqFunc = ctx->poller.lastReqFunc;
            ctx->decoder.lastReqStart = ctx->poller.lastReqStart;
            ctx->decoder.lastReqCount = ctx->poller.lastReqCount;
            ctx->decoder.lastReqUs = ctx->poller.lastReqUs;
        }

        if ((nowUs - ctx->lastPublishUs) >= ((int64_t)JKBMS_BMS_PUBLISH_PERIOD_MS * 1000LL)) {
            jkbms_modbus_snapshot_t snapshot = {0};
            if (buildDecodedSnapshot(&ctx->decoder, &snapshot)) {
                jkbmsStoreLatestSnapshot(&snapshot);

                bms_decoded_packet_t packet = {0};
                if (buildPacketFromSnapshot(&snapshot, ++ctx->sequence, &packet)) {
                    jkbmsStoreLatestPacket(&packet);
                    if (xQueueOverwrite(ctx->outQueue, &packet) != pdPASS) {
                        ESP_LOGW(EXAMPLE_TAG, "JKBMS output queue overwrite failed");
                    }
                }
            }
            ctx->lastPublishUs = nowUs;
        }
    }
}

esp_err_t jkbmsModbusBmsTaskStart(QueueHandle_t outQueue)
{
    if (outQueue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_jkbmsModbusBmsTaskHandle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&g_jkbmsModbusBmsCtx, 0, sizeof(g_jkbmsModbusBmsCtx));
    g_jkbmsModbusBmsCtx.outQueue = outQueue;

    BaseType_t taskOk =
        xTaskCreate(jkbmsModbusBmsTask,
                    "jkbms_modbus",
                    JKBMS_BMS_TASK_STACK,
                    &g_jkbmsModbusBmsCtx,
                    JKBMS_BMS_TASK_PRIORITY,
                    &g_jkbmsModbusBmsTaskHandle);
    if (taskOk != pdPASS) {
        g_jkbmsModbusBmsTaskHandle = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "JKBMS Modbus BMS task started (poll=%dms, publish=%dms)",
             JKBMS_BMS_QUERY_PERIOD_MS,
             JKBMS_BMS_PUBLISH_PERIOD_MS);
    return ESP_OK;
}

bool jkbmsModbusBmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket)
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

bool jkbmsModbusBmsTaskGetLatestSnapshot(jkbms_modbus_snapshot_t *outSnapshot)
{
    bool hasSnapshot = false;

    if (outSnapshot == NULL) {
        return false;
    }

    portENTER_CRITICAL(&g_latestPacketMux);
    hasSnapshot = g_haveLatestSnapshot;
    if (hasSnapshot) {
        *outSnapshot = g_latestSnapshot;
    }
    portEXIT_CRITICAL(&g_latestPacketMux);

    return hasSnapshot;
}

esp_err_t jkbmsModbusBmsTaskStop(void)
{
    if (g_jkbmsModbusBmsTaskHandle == NULL) {
        return ESP_OK;
    }

    vTaskDelete(g_jkbmsModbusBmsTaskHandle);
    g_jkbmsModbusBmsTaskHandle = NULL;
    memset(&g_jkbmsModbusBmsCtx, 0, sizeof(g_jkbmsModbusBmsCtx));

    portENTER_CRITICAL(&g_latestPacketMux);
    g_haveLatestPacket = false;
    memset(&g_latestPacket, 0, sizeof(g_latestPacket));
    g_haveLatestSnapshot = false;
    memset(&g_latestSnapshot, 0, sizeof(g_latestSnapshot));
    portEXIT_CRITICAL(&g_latestPacketMux);

    return ESP_OK;
}
