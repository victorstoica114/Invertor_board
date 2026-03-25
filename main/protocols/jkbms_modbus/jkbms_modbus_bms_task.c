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

static uint16_t bswap16(uint16_t v)
{
    return (uint16_t)((v >> 8) | (v << 8));
}

static bool normalizeCellMv(uint16_t raw, uint16_t *mvOut)
{
    uint16_t candidates[3];
    candidates[0] = raw;
    candidates[1] = bswap16(raw);
    candidates[2] = (raw > 5000u) ? (uint16_t)(raw / 10u) : raw;

    for (size_t i = 0u; i < (sizeof(candidates) / sizeof(candidates[0])); i++) {
        uint16_t mv = candidates[i];
        if (mv >= 1000u && mv <= 5000u) {
            if (mvOut != NULL) {
                *mvOut = mv;
            }
            return true;
        }
    }

    if (raw >= 100u && raw <= 600u) {
        uint16_t mv = (uint16_t)(raw * 10u);
        if (mvOut != NULL) {
            *mvOut = mv;
        }
        return true;
    }

    return false;
}

static bool normalizeSignedDeciC(int16_t raw, int16_t *outC)
{
    if (raw < -1000 || raw > 1500) {
        return false;
    }

    if (outC != NULL) {
        *outC = (int16_t)(raw / 10);
    }
    return true;
}

static bool decodePctBytePair(uint16_t raw, bool preferHigh, uint8_t *pctOut)
{
    const uint8_t hi = (uint8_t)((raw >> 8) & 0xFFu);
    const uint8_t lo = (uint8_t)(raw & 0xFFu);
    const bool hiOk = (hi <= 100u);
    const bool loOk = (lo <= 100u);

    if (!hiOk && !loOk) {
        return false;
    }

    if (hiOk && loOk) {
        if (hi == 0u && lo > 0u) {
            if (pctOut != NULL) {
                *pctOut = lo;
            }
            return true;
        }
        if (lo == 0u && hi > 0u) {
            if (pctOut != NULL) {
                *pctOut = hi;
            }
            return true;
        }
        if (pctOut != NULL) {
            *pctOut = preferHigh ? hi : lo;
        }
        return true;
    }

    if (pctOut != NULL) {
        *pctOut = hiOk ? hi : lo;
    }
    return true;
}

static bool decodeU32Best(const modbusDecoder_t *decoder,
                          uint16_t reg,
                          uint32_t minVal,
                          uint32_t maxVal,
                          uint32_t *out)
{
    uint16_t a = 0u;
    uint16_t b = 0u;
    if (!decoderGetU16(decoder, reg, &a) || !decoderGetU16(decoder, (uint16_t)(reg + 1u), &b)) {
        return false;
    }

    const uint32_t vAB = (((uint32_t)a) << 16) | (uint32_t)b;
    const uint32_t vBA = (((uint32_t)b) << 16) | (uint32_t)a;
    const bool abOk = (vAB >= minVal && vAB <= maxVal);
    const bool baOk = (vBA >= minVal && vBA <= maxVal);

    if (!abOk && !baOk) {
        return false;
    }

    if (out != NULL) {
        if (abOk && baOk) {
            *out = (vAB <= vBA) ? vAB : vBA;
        } else {
            *out = abOk ? vAB : vBA;
        }
    }

    return true;
}

static bool decodeI32Best(const modbusDecoder_t *decoder,
                          uint16_t reg,
                          int32_t absLimit,
                          int32_t *out)
{
    uint16_t a = 0u;
    uint16_t b = 0u;
    if (!decoderGetU16(decoder, reg, &a) || !decoderGetU16(decoder, (uint16_t)(reg + 1u), &b)) {
        return false;
    }

    const int32_t vAB = (int32_t)((((uint32_t)a) << 16) | (uint32_t)b);
    const int32_t vBA = (int32_t)((((uint32_t)b) << 16) | (uint32_t)a);
    const bool abOk = (vAB <= absLimit && vAB >= -absLimit);
    const bool baOk = (vBA <= absLimit && vBA >= -absLimit);

    if (!abOk && !baOk) {
        return false;
    }

    if (out != NULL) {
        if (abOk && baOk) {
            int64_t aAbs = (vAB >= 0) ? (int64_t)vAB : -(int64_t)vAB;
            int64_t bAbs = (vBA >= 0) ? (int64_t)vBA : -(int64_t)vBA;
            *out = (aAbs <= bAbs) ? vAB : vBA;
        } else {
            *out = abOk ? vAB : vBA;
        }
    }

    return true;
}

static bool decodeSocPct(const modbusDecoder_t *decoder, uint8_t *socOut)
{
    uint16_t raw = 0u;
    uint8_t chosen = 0u;
    bool have = false;
    uint8_t cand = 0u;

    if (decoderGetU16(decoder, JKBMS_RT_REG_BALAN_SOC_U8X2, &raw) &&
        decodePctBytePair(raw, false, &cand)) {
        chosen = cand;
        have = true;
        if (cand > 0u) {
            if (socOut != NULL) {
                *socOut = cand;
            }
            return true;
        }
    }

    if (decoderGetU16(decoder, (uint16_t)(JKBMS_RT_REG_BALAN_SOC_U8X2 + 1u), &raw) &&
        decodePctBytePair(raw, false, &cand)) {
        chosen = cand;
        have = true;
        if (cand > 0u) {
            if (socOut != NULL) {
                *socOut = cand;
            }
            return true;
        }
    }

    /* Some JK firmwares expose SOC in high byte of SOH/PRECHARGE word. */
    if (decoderGetU16(decoder, JKBMS_RT_REG_SOH_PRECHARGE_U8X2, &raw) &&
        decodePctBytePair(raw, true, &cand)) {
        chosen = cand;
        have = true;
        if (cand > 0u) {
            if (socOut != NULL) {
                *socOut = cand;
            }
            return true;
        }
    }

    if (have) {
        if (socOut != NULL) {
            *socOut = chosen;
        }
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
        if (lo <= 1u) {
            *prechargeOut = lo;
        } else if (hi <= 1u) {
            *prechargeOut = hi;
        }
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
    uint8_t highestCellIdx = 0u;
    uint16_t minMv = UINT16_MAX;
    uint16_t maxMv = 0u;
    uint8_t minIdx = 0u;
    uint8_t maxIdx = 0u;

    for (uint8_t i = 0; i < JKBMS_MAX_CELLS; i++) {
        const uint16_t regPrimary = (uint16_t)(JKBMS_RT_REG_CELL0_MV + ((uint16_t)i * JKBMS_RT_CELL_STEP));
        const uint16_t regCompact = (uint16_t)(JKBMS_RT_REG_CELL0_MV + (uint16_t)i);
        const uint16_t regOdd = (uint16_t)(regPrimary + 1u);
        uint16_t raw = 0u;
        uint16_t mv = 0u;
        bool ok = false;

        if (decoderGetU16(decoder, regPrimary, &raw) && normalizeCellMv(raw, &mv)) {
            ok = true;
        } else if (decoderGetU16(decoder, regCompact, &raw) && normalizeCellMv(raw, &mv)) {
            ok = true;
        } else if (decoderGetU16(decoder, regOdd, &raw) && normalizeCellMv(raw, &mv)) {
            ok = true;
        }

        if (!ok) {
            continue;
        }

        out->cellMv[i] = mv;
        validCells++;
        if ((uint8_t)(i + 1u) > highestCellIdx) {
            highestCellIdx = (uint8_t)(i + 1u);
        }

        if (mv < minMv) {
            minMv = mv;
            minIdx = (uint8_t)(i + 1u);
        }
        if (mv > maxMv) {
            maxMv = mv;
            maxIdx = (uint8_t)(i + 1u);
        }
    }

    out->cellCount = (validCells > 0u) ? highestCellIdx : 0u;
    if (validCells > 0u) {
        out->hasCellExtremes = true;
        out->minCellMv = minMv;
        out->maxCellMv = maxMv;
        out->minCellIndex = minIdx;
        out->maxCellIndex = maxIdx;
    }

    uint16_t u16 = 0u;
    if (decoderGetU16(decoder, JKBMS_RT_REG_CELL_AVG_MV, &u16) && normalizeCellMv(u16, &u16)) {
        out->hasCellAvgMv = true;
        out->cellAvgMv = u16;
    }
    if (decoderGetU16(decoder, JKBMS_RT_REG_CELL_VDIFF_MAX_MV, &u16)) {
        if (u16 > 5000u) {
            u16 = (uint16_t)(u16 / 10u);
        }
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
    if (decoderGetI16(decoder, JKBMS_RT_REG_TEMP_MOS_DECIC, &i16) && normalizeSignedDeciC(i16, &i16)) {
        out->hasTempMosC = true;
        out->tempMosC = i16;
    }
    if (decoderGetI16(decoder, JKBMS_RT_REG_TEMP_BAT1_DECIC, &i16) && normalizeSignedDeciC(i16, &i16)) {
        out->hasTempBat1C = true;
        out->tempBat1C = i16;
    }
    if (decoderGetI16(decoder, JKBMS_RT_REG_TEMP_BAT2_DECIC, &i16) && normalizeSignedDeciC(i16, &i16)) {
        out->hasTempBat2C = true;
        out->tempBat2C = i16;
    }

    uint32_t u32 = 0u;
    if (decodeU32Best(decoder, JKBMS_RT_REG_PACK_VOLT_MV_U32, 1000u, 200000u, &u32)) {
        out->hasPackVoltageMv = true;
        out->packVoltageMv = u32;
    }

    int32_t i32 = 0;
    if (decodeI32Best(decoder, JKBMS_RT_REG_PACK_CURRENT_MA_I32, 1000000, &i32)) {
        out->hasPackCurrentMa = true;
        out->packCurrentMa = i32;
    }

    if (decodeI32Best(decoder, JKBMS_RT_REG_PACK_WATT_MW_U32, 200000000, &i32)) {
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

    if (decoderGetI16(decoder, JKBMS_RT_REG_BALAN_CURRENT_MA_I16, &i16) &&
        i16 >= -30000 && i16 <= 30000) {
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

    if (decodeI32Best(decoder, JKBMS_RT_REG_SOC_REMAIN_MAH_I32, 500000000, &i32)) {
        out->hasRemainMah = true;
        out->remainMah = i32;
    }

    if (decodeU32Best(decoder, JKBMS_RT_REG_SOC_FULL_MAH_U32, 0u, 500000000u, &u32)) {
        out->hasFullMah = true;
        out->fullMah = u32;
    }

    if (decodeU32Best(decoder, JKBMS_RT_REG_CYCLE_COUNT_U32, 0u, 1000000u, &u32)) {
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
    int64_t lastRecordedReqUs = 0;

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
        } else if (ctx->poller.lastReqValid && ctx->poller.lastReqUs != lastRecordedReqUs) {
            /* On some RS485 transceivers TX bytes are not looped into RX. Seed decoder request queue. */
            modbusDecoderRecordRequest(&ctx->decoder,
                                       ctx->poller.lastReqSlave,
                                       ctx->poller.lastReqFunc,
                                       ctx->poller.lastReqStart,
                                       ctx->poller.lastReqCount,
                                       ctx->poller.lastReqUs);
            lastRecordedReqUs = ctx->poller.lastReqUs;
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

    portENTER_CRITICAL(&g_latestPacketMux);
    g_haveLatestPacket = false;
    memset(&g_latestPacket, 0, sizeof(g_latestPacket));
    g_haveLatestSnapshot = false;
    memset(&g_latestSnapshot, 0, sizeof(g_latestSnapshot));
    portEXIT_CRITICAL(&g_latestPacketMux);

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
