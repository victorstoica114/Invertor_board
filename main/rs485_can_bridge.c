#include "rs485_can_bridge.h"

#include "config.h"
#include "bridge.h"
#include "BMS_Protocols/Growatt/growatt_modbus_map.h"
#include "CAN_Decoder.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    modbusDecoder_t *src;
    twai_handle_t txBus;
    const char *txName;
    uint32_t txSetCount;
} rs485Can322Ctx_t;

static rs485Can322Ctx_t g_rsCanCtx;
static TaskHandle_t g_rsCanTaskHandle;

typedef struct {
    uart_port_t uart;
    gpio_num_t dirPin;
    const char *ifName;
    const char *srcCanIf;
    uint8_t slaveId;
    uint8_t fakeSocPct;
    uint32_t reqCount;
    uint32_t rspCount;
} canRs485GrowattCtx_t;

static canRs485GrowattCtx_t g_canRsGrowattCtx;
static TaskHandle_t g_canRsGrowattTaskHandle;

static inline void putBe16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)((v >> 8) & 0xFFu);
    p[1] = (uint8_t)(v & 0xFFu);
}

static inline void putLe16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static bool decoderGetCachedReg(const modbusDecoder_t *d, uint16_t addr, uint16_t *outVal)
{
    if (d == NULL) {
        return false;
    }

    for (int i = 0; i < MODBUS_DECODER_CACHE_MAX_REGS; i++) {
        if (!d->cacheValid[i]) {
            continue;
        }
        if (d->cacheAddr[i] != addr) {
            continue;
        }

        if (outVal != NULL) {
            *outVal = d->cacheVal[i];
        }
        return true;
    }

    return false;
}

static bool getRegOrFallback(const rs485Can322Ctx_t *ctx, uint16_t addr, uint16_t fallback, uint16_t *out)
{
    if (decoderGetCachedReg(ctx->src, addr, out)) {
        return true;
    }

#if RS485_CAN_BRIDGE_USE_FALLBACK
    if (out != NULL) {
        *out = fallback;
    }
    return true;
#else
    (void)fallback;
    return false;
#endif
}

static uint8_t clampPctU8(uint16_t v)
{
    if (v > 100u) {
        return 100u;
    }
    return (uint8_t)v;
}

static uint8_t clampCellIdxU8(uint16_t idx, uint8_t fallback)
{
    if (idx >= 1u && idx <= 16u) {
        return (uint8_t)idx;
    }
    return fallback;
}

static bool sendFrame(twai_handle_t txBus, uint32_t id, const uint8_t data[8], const char *txName)
{
    twai_message_t tx = {0};
    tx.identifier = id;
    tx.data_length_code = 8;
    memcpy(tx.data, data, 8);

    esp_err_t e = twai_transmit_v2(txBus, &tx, pdMS_TO_TICKS(20));
    if (e != ESP_OK) {
        ESP_LOGW(EXAMPLE_TAG,
                 "RS485->CAN 0x%03X TX failed on %s (err=0x%x)",
                 (unsigned)id,
                 txName,
                 (unsigned)e);
        return false;
    }

    return true;
}

static void rs485CanTelemetryTask(void *pv)
{
    rs485Can322Ctx_t *ctx = (rs485Can322Ctx_t *)pv;

    while (1) {
        uint16_t soc = 0;
        uint16_t tempCraw = 0;
        uint16_t packCv = 0;
        uint16_t packIAbsCa = 0;
        uint16_t soh = 0;
        uint16_t remain10mAh = 0;
        uint16_t fcc10mAh = 0;
        uint16_t cycles = 0;
        uint16_t cellMaxMv = 0;
        uint16_t cellMinMv = 0;
        uint16_t cellMaxIdx = 0;
        uint16_t cellMinIdx = 0;

        bool okSoc = getRegOrFallback(ctx, GROWATT_MB_REG_SOC_PCT, RS485_CAN_BRIDGE_FALLBACK_SOC_PCT, &soc);
        bool okTemp = getRegOrFallback(ctx, GROWATT_MB_REG_TEMP_C, (uint16_t)RS485_CAN_BRIDGE_FALLBACK_TEMP_C, &tempCraw);
        bool okPackV = getRegOrFallback(ctx, GROWATT_MB_REG_PACK_V_CV, RS485_CAN_BRIDGE_FALLBACK_PACK_V_CV, &packCv);
        bool okPackI = getRegOrFallback(ctx, GROWATT_MB_REG_PACK_I_ABS_CA_TENTATIVE, (uint16_t)(RS485_CAN_BRIDGE_FALLBACK_PACK_I_0P1 * 10), &packIAbsCa);
        bool okSoh = getRegOrFallback(ctx, GROWATT_MB_REG_SOH_PCT, RS485_CAN_BRIDGE_FALLBACK_SOH_PCT, &soh);
        bool okRm = getRegOrFallback(ctx, GROWATT_MB_REG_REMAIN_CAP_CAH, RS485_CAN_BRIDGE_FALLBACK_RM_10MAH, &remain10mAh);
        bool okFcc = getRegOrFallback(ctx, GROWATT_MB_REG_FULL_CAP_CAH, RS485_CAN_BRIDGE_FALLBACK_FCC_10MAH, &fcc10mAh);
        bool okCycle = getRegOrFallback(ctx, GROWATT_MB_REG_CYCLE_COUNT_TENTATIVE, RS485_CAN_BRIDGE_FALLBACK_CYCLE_COUNT, &cycles);
        bool okCmax = getRegOrFallback(ctx, GROWATT_MB_REG_CELL_MAX_MV, RS485_CAN_BRIDGE_FALLBACK_CELL_MAX_MV, &cellMaxMv);
        bool okCmin = getRegOrFallback(ctx, GROWATT_MB_REG_CELL_MIN_MV, RS485_CAN_BRIDGE_FALLBACK_CELL_MIN_MV, &cellMinMv);
        bool okCmaxIdx = getRegOrFallback(ctx, GROWATT_MB_REG_CELL_MAX_IDX, RS485_CAN_BRIDGE_FALLBACK_CELL_MAX_IDX, &cellMaxIdx);
        bool okCminIdx = getRegOrFallback(ctx, GROWATT_MB_REG_CELL_MIN_IDX, RS485_CAN_BRIDGE_FALLBACK_CELL_MIN_IDX, &cellMinIdx);

        int16_t tempC = (int16_t)tempCraw;
        int16_t tempDeci = (int16_t)(tempC * 10);

        int32_t i0p1 = (int32_t)(packIAbsCa / 10u);
        if (i0p1 > 32767) {
            i0p1 = 32767;
        }
        int16_t packI_0p1 = (int16_t)i0p1;

        uint8_t socPct = clampPctU8(soc);
        uint8_t sohPct = clampPctU8(soh);

        uint8_t cmaxIdx = clampCellIdxU8(cellMaxIdx, (uint8_t)RS485_CAN_BRIDGE_FALLBACK_CELL_MAX_IDX);
        uint8_t cminIdx = clampCellIdxU8(cellMinIdx, (uint8_t)RS485_CAN_BRIDGE_FALLBACK_CELL_MIN_IDX);
        if (cellMaxMv < cellMinMv) {
            uint16_t t = cellMaxMv;
            cellMaxMv = cellMinMv;
            cellMinMv = t;
        }
        uint16_t dVmv = (uint16_t)(cellMaxMv - cellMinMv);

        bool sent313 = false;
        bool sent314 = false;
        bool sent319 = false;
        bool sent322 = false;

        if (okPackV && okPackI && okTemp && okSoc && okSoh) {
            uint8_t d313[8] = {0};
            putBe16(&d313[0], packCv);
            putBe16(&d313[2], (uint16_t)packI_0p1);
            putBe16(&d313[4], (uint16_t)tempDeci);
            d313[6] = socPct;
            d313[7] = (uint8_t)(sohPct & 0x7Fu);
            sent313 = sendFrame(ctx->txBus, GROWATT_CAN_ID_313_V_I_SOC_SOH, d313, ctx->txName);
        }

        if (okRm && okFcc && okCycle && okCmax && okCmin) {
            uint8_t d314[8] = {0};
            putBe16(&d314[0], remain10mAh);
            putBe16(&d314[2], fcc10mAh);
            putBe16(&d314[4], dVmv);
            putBe16(&d314[6], cycles);
            sent314 = sendFrame(ctx->txBus, GROWATT_CAN_ID_314_RM_FCC_DV_CYCLES, d314, ctx->txName);
        }

        if (okCmax && okCmin && okCmaxIdx && okCminIdx) {
            uint8_t d319[8] = {0};
            putLe16(&d319[0], cellMaxMv);
            putLe16(&d319[2], cellMinMv);
            d319[4] = (uint8_t)RS485_CAN_BRIDGE_FALLBACK_FLAGS_319;
            d319[5] = cmaxIdx;
            d319[6] = cminIdx;
            d319[7] = (uint8_t)RS485_CAN_BRIDGE_FALLBACK_ADDR_319;
            sent319 = sendFrame(ctx->txBus, GROWATT_CAN_ID_319_CELL_REF_FLAGS, d319, ctx->txName);
        }

        if (okTemp && okSoc) {
            uint8_t d322[8] = {0};
            putBe16(&d322[0], (uint16_t)tempDeci);
            putBe16(&d322[2], (uint16_t)tempDeci);
            d322[4] = (uint8_t)RS485_CAN_BRIDGE_FALLBACK_TEMP_SENSOR_MAX;
            d322[5] = (uint8_t)RS485_CAN_BRIDGE_FALLBACK_TEMP_SENSOR_MIN;
            d322[6] = socPct;
            d322[7] = socPct;
            sent322 = sendFrame(ctx->txBus, GROWATT_CAN_ID_322_TEMP_SOC_MIN_MAX, d322, ctx->txName);
        }

        if (sent313 || sent314 || sent319 || sent322) {
            ctx->txSetCount++;
#if RS485_CAN_BRIDGE_TX_LOG_EVERY_N > 0
            if ((ctx->txSetCount % RS485_CAN_BRIDGE_TX_LOG_EVERY_N) == 0u) {
                ESP_LOGI(EXAMPLE_TAG,
                         "RS485->CAN TXSET on %s: 313=%c 314=%c 319=%c 322=%c | V=%.2fV SOC=%u%% T=%dC",
                         ctx->txName,
                         sent313 ? 'Y' : 'N',
                         sent314 ? 'Y' : 'N',
                         sent319 ? 'Y' : 'N',
                         sent322 ? 'Y' : 'N',
                         (double)packCv / 100.0,
                         (unsigned)socPct,
                         (int)tempC);
            }
#endif
        }

        vTaskDelay(pdMS_TO_TICKS(RS485_CAN_322_TX_PERIOD_MS));
    }
}

static uint16_t canRsBe16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint16_t canRsCrc16(const uint8_t *data, int len)
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

static bool canRsCheckCrc(const uint8_t *frame, int len)
{
    if (len < 4) {
        return false;
    }
    uint16_t got = (uint16_t)(frame[len - 2] | ((uint16_t)frame[len - 1] << 8));
    uint16_t calc = canRsCrc16(frame, len - 2);
    return got == calc;
}

static void canRsSetTx(gpio_num_t dirPin, bool txEn)
{
    gpio_set_level(dirPin, txEn ? 1 : 0);
}

static bool canRsModelLooksUsable(const universal_battery_model_t *model)
{
    return model != NULL && model->valid;
}

static uint16_t canRsRoundScaled(float value, float scale, uint16_t fallback)
{
    float scaled = value * scale;
    int32_t rounded = 0;

    if (!(value == value)) {
        return fallback;
    }

    if (scaled < 0.0f) {
        return fallback;
    }

    rounded = (int32_t)(scaled + 0.5f);
    if (rounded < 0) {
        return fallback;
    }
    if (rounded > 65535) {
        return 65535u;
    }
    return (uint16_t)rounded;
}

static uint16_t canRsAbsScaled(float value, float scale, uint16_t fallback)
{
    if (value < 0.0f) {
        value = -value;
    }
    return canRsRoundScaled(value, scale, fallback);
}

static uint16_t canRsSocFromSources(const canRs485GrowattCtx_t *ctx,
                                    const universal_battery_model_t *model,
                                    bool *fromModel,
                                    bool *fromCan)
{
    uint8_t socPct = (ctx != NULL) ? ctx->fakeSocPct : 0u;
    bool socFromCan = false;
    bool socFromModel = false;

    if (fromModel != NULL) {
        *fromModel = false;
    }
    if (fromCan != NULL) {
        *fromCan = false;
    }

    if (model != NULL && model->valid && model->socPct <= 100u) {
        socPct = model->socPct;
        socFromModel = true;
    } else if (ctx != NULL && ctx->srcCanIf != NULL) {
        socFromCan = canDecoderTryGetSocPct(ctx->srcCanIf, &socPct);
    }

    if (fromModel != NULL) {
        *fromModel = socFromModel;
    }
    if (fromCan != NULL) {
        *fromCan = socFromCan;
    }
    return socPct;
}

static void canRsSynthCellRegs(const universal_battery_model_t *model, uint16_t cells[16])
{
    uint16_t maxMv = RS485_CAN_BRIDGE_FALLBACK_CELL_MAX_MV;
    uint16_t minMv = RS485_CAN_BRIDGE_FALLBACK_CELL_MIN_MV;
    uint8_t maxIdx = (uint8_t)RS485_CAN_BRIDGE_FALLBACK_CELL_MAX_IDX;
    uint8_t minIdx = (uint8_t)RS485_CAN_BRIDGE_FALLBACK_CELL_MIN_IDX;
    uint16_t avgMv = 0;

    if (model != NULL) {
        if (model->cellMaxV > 0.0f) {
            maxMv = canRsRoundScaled(model->cellMaxV, 1000.0f, maxMv);
        }
        if (model->cellMinV > 0.0f) {
            minMv = canRsRoundScaled(model->cellMinV, 1000.0f, minMv);
        }
        if (model->cellMaxIdx >= 1u && model->cellMaxIdx <= 16u) {
            maxIdx = model->cellMaxIdx;
        }
        if (model->cellMinIdx >= 1u && model->cellMinIdx <= 16u) {
            minIdx = model->cellMinIdx;
        }
    }

    if (maxMv < minMv) {
        uint16_t tmpMv = maxMv;
        uint8_t tmpIdx = maxIdx;
        maxMv = minMv;
        minMv = tmpMv;
        maxIdx = minIdx;
        minIdx = tmpIdx;
    }

    if (model != NULL && model->packVoltageV > 0.0f) {
        avgMv = canRsRoundScaled(model->packVoltageV / 16.0f, 1000.0f, 0u);
    }
    if (avgMv == 0u) {
        avgMv = (uint16_t)((maxMv + minMv) / 2u);
    }
    if (avgMv < minMv) {
        avgMv = minMv;
    }
    if (avgMv > maxMv) {
        avgMv = maxMv;
    }

    for (int i = 0; i < 16; i++) {
        cells[i] = avgMv;
    }

    if (maxIdx >= 1u && maxIdx <= 16u) {
        cells[maxIdx - 1u] = maxMv;
    }
    if (minIdx >= 1u && minIdx <= 16u) {
        cells[minIdx - 1u] = minMv;
    }
}

static uint16_t canRsSynthStatusFlags(const universal_battery_model_t *model)
{
    uint16_t status = 0u;

    if (model != NULL && model->protocolState != 0u) {
        status = (uint16_t)(model->protocolState & 0xFFFFu);
    }

    if (model != NULL) {
        if (model->balanceEnabled) {
            status |= 0x0020u;
        }
        if (model->dischargeEnabled) {
            status |= 0x0040u;
        }
        if (model->chargeEnabled) {
            status |= 0x0080u;
        }
    }

    return status;
}

static uint16_t canRsSynthGrowattReg(const universal_battery_model_t *model,
                                     uint16_t addr,
                                     uint16_t socPct,
                                     uint16_t fullCapCah,
                                     const uint16_t cells[16])
{
    bool modelOk = canRsModelLooksUsable(model);

    switch (addr) {
        case GROWATT_MB_REG_INFO_0001:
            return 0x0001u;
        case GROWATT_MB_REG_INFO_0002:
            return 0x0010u;
        case GROWATT_MB_REG_INFO_0003:
            return 0x0001u;
        case GROWATT_MB_REG_INFO_0004:
            return 0x0000u;
        case GROWATT_MB_REG_STATUS_FLAGS:
            return canRsSynthStatusFlags(model);
        case GROWATT_MB_REG_SOC_PCT:
            return socPct;
        case GROWATT_MB_REG_PACK_V_CV:
            if (modelOk && model->packVoltageV > 0.0f) {
                return canRsRoundScaled(model->packVoltageV, 100.0f, RS485_CAN_BRIDGE_FALLBACK_PACK_V_CV);
            }
            return RS485_CAN_BRIDGE_FALLBACK_PACK_V_CV;
        case GROWATT_MB_REG_PACK_I_ABS_CA_TENTATIVE:
            if (modelOk) {
                return canRsAbsScaled(model->packCurrentA, 100.0f, (uint16_t)(RS485_CAN_BRIDGE_FALLBACK_PACK_I_0P1 * 10));
            }
            return (uint16_t)(RS485_CAN_BRIDGE_FALLBACK_PACK_I_0P1 * 10);
        case GROWATT_MB_REG_TEMP_C:
            if (modelOk) {
                return canRsRoundScaled(model->temperaturesC[0], 1.0f, (uint16_t)RS485_CAN_BRIDGE_FALLBACK_TEMP_C);
            }
            return (uint16_t)RS485_CAN_BRIDGE_FALLBACK_TEMP_C;
        case GROWATT_MB_REG_CYCLE_COUNT_TENTATIVE:
            if (modelOk && model->cycleCount > 0u) {
                return model->cycleCount;
            }
            return RS485_CAN_BRIDGE_FALLBACK_CYCLE_COUNT;
        case GROWATT_MB_REG_REMAIN_CAP_CAH:
            return (uint16_t)(((uint32_t)fullCapCah * (uint32_t)socPct) / 100u);
        case GROWATT_MB_REG_FULL_CAP_CAH:
            return fullCapCah;
        case GROWATT_MB_REG_SOH_PCT:
            if (modelOk && model->sohPct <= 100u) {
                return model->sohPct;
            }
            return RS485_CAN_BRIDGE_FALLBACK_SOH_PCT;
        case GROWATT_MB_REG_CV_TARGET_CV:
            if (modelOk && model->chargeVoltageLimitV > 0.0f) {
                return canRsRoundScaled(model->chargeVoltageLimitV, 100.0f, RS485_CAN_BRIDGE_FALLBACK_PACK_V_CV);
            }
            return canRsSynthGrowattReg(model, GROWATT_MB_REG_PACK_V_CV, socPct, fullCapCah, cells);
        case GROWATT_MB_REG_ICHG_LIM_CA_TENTATIVE:
            if (modelOk && model->chargeCurrentLimitA > 0.0f) {
                return canRsAbsScaled(model->chargeCurrentLimitA, 100.0f, 0u);
            }
            return 0u;
        case GROWATT_MB_REG_IDIS_LIM_CA_TENTATIVE:
            if (modelOk && model->dischargeCurrentLimitA > 0.0f) {
                return canRsAbsScaled(model->dischargeCurrentLimitA, 100.0f, 0u);
            }
            return 0u;
        case GROWATT_MB_REG_CELL_MAX_MV:
            return cells[clampCellIdxU8((modelOk ? model->cellMaxIdx : 0u), (uint8_t)RS485_CAN_BRIDGE_FALLBACK_CELL_MAX_IDX) - 1u];
        case GROWATT_MB_REG_CELL_MIN_MV:
            return cells[clampCellIdxU8((modelOk ? model->cellMinIdx : 0u), (uint8_t)RS485_CAN_BRIDGE_FALLBACK_CELL_MIN_IDX) - 1u];
        case GROWATT_MB_REG_CELL_MAX_IDX:
            return clampCellIdxU8((modelOk ? model->cellMaxIdx : 0u), (uint8_t)RS485_CAN_BRIDGE_FALLBACK_CELL_MAX_IDX);
        case GROWATT_MB_REG_CELL_MIN_IDX:
            return clampCellIdxU8((modelOk ? model->cellMinIdx : 0u), (uint8_t)RS485_CAN_BRIDGE_FALLBACK_CELL_MIN_IDX);
        case GROWATT_MB_REG_CELL_EXTRA:
            return 0u;
        default:
            if (addr >= GROWATT_MB_REG_CELL_BASE && addr <= GROWATT_MB_REG_CELL_LAST) {
                return cells[addr - GROWATT_MB_REG_CELL_BASE];
            }
            return 0u;
    }
}

static bool canRsParseReadReq(const uint8_t *frame,
                              int len,
                              uint8_t slaveId,
                              uint8_t *funcOut,
                              uint16_t *startOut,
                              uint16_t *countOut)
{
    if (frame == NULL || len != 8) {
        return false;
    }
    if (!canRsCheckCrc(frame, len)) {
        return false;
    }

    if (frame[0] != slaveId) {
        return false;
    }

    const uint8_t func = frame[1];
    if (func != 0x03u && func != 0x04u) {
        return false;
    }

    const uint16_t start = canRsBe16(&frame[2]);
    const uint16_t count = canRsBe16(&frame[4]);
    if (count == 0u || count > 125u) {
        return false;
    }

    if (funcOut != NULL) {
        *funcOut = func;
    }
    if (startOut != NULL) {
        *startOut = start;
    }
    if (countOut != NULL) {
        *countOut = count;
    }
    return true;
}

static bool canRsSendGrowattResponse(canRs485GrowattCtx_t *ctx,
                                     uint8_t func,
                                     uint16_t start,
                                     uint16_t count,
                                     const universal_battery_model_t *model,
                                     uint16_t socPct)
{
    uint16_t fullCapCah = RS485_CAN_BRIDGE_FALLBACK_FCC_10MAH;
    uint16_t cells[16];

    if (ctx == NULL) {
        return false;
    }

    const int respLen = (int)(3u + (count * 2u) + 2u);
    if (respLen <= 0 || respLen > 256) {
        return false;
    }

    uint8_t resp[256] = {0};
    resp[0] = ctx->slaveId;
    resp[1] = func;
    resp[2] = (uint8_t)(count * 2u);

    canRsSynthCellRegs(model, cells);

    for (uint16_t i = 0; i < count; i++) {
        uint16_t addr = (uint16_t)(start + i);
        uint16_t val = canRsSynthGrowattReg(model, addr, socPct, fullCapCah, cells);
        putBe16(&resp[3 + (i * 2u)], val);
    }

    uint16_t crc = canRsCrc16(resp, respLen - 2);
    resp[respLen - 2] = (uint8_t)(crc & 0xFFu);
    resp[respLen - 1] = (uint8_t)((crc >> 8) & 0xFFu);

    canRsSetTx(ctx->dirPin, true);
    uart_write_bytes(ctx->uart, (const char *)resp, respLen);
    uart_wait_tx_done(ctx->uart, pdMS_TO_TICKS(100));
    canRsSetTx(ctx->dirPin, false);

    ctx->rspCount++;
    return true;
}

static void canRs485GrowattTask(void *pv)
{
    canRs485GrowattCtx_t *ctx = (canRs485GrowattCtx_t *)pv;
    uint8_t rxChunk[64];
    uint8_t frameBuf[256];
    uint16_t frameLen = 0;
    bool frameHaveLastByte = false;
    int64_t frameLastByteUs = 0;

    while (1) {
        int len = uart_read_bytes(ctx->uart, rxChunk, sizeof(rxChunk), pdMS_TO_TICKS(5));
        int64_t nowUs = esp_timer_get_time();

        if (len > 0) {
            if (frameHaveLastByte && ((nowUs - frameLastByteUs) > (int64_t)CAN_RS485_SOC_RX_GAP_US)) {
                frameLen = 0;
                frameHaveLastByte = false;
            }

            if ((size_t)frameLen + (size_t)len > sizeof(frameBuf)) {
                frameLen = 0;
                frameHaveLastByte = false;
            } else {
                memcpy(&frameBuf[frameLen], rxChunk, (size_t)len);
                frameLen = (uint16_t)(frameLen + len);
                frameLastByteUs = nowUs;
                frameHaveLastByte = true;
            }
        }

        if (frameHaveLastByte && ((nowUs - frameLastByteUs) > (int64_t)CAN_RS485_SOC_RX_GAP_US)) {
            uint8_t func = 0;
            uint16_t start = 0;
            uint16_t count = 0;
            bool sent = false;
            universal_battery_model_t model = {0};
            bool socFromModel = false;
            bool socFromCan = false;
            uint16_t socPct = 0;

            if (canRsParseReadReq(frameBuf, frameLen, ctx->slaveId, &func, &start, &count)) {
                bridgeGetUniversalBatteryModel(&model);
                socPct = canRsSocFromSources(ctx, &model, &socFromModel, &socFromCan);
                ctx->reqCount++;
                sent = canRsSendGrowattResponse(ctx, func, start, count, &model, socPct);

#if CAN_RS485_SOC_LOG_EVERY_N > 0
                if (ctx->reqCount <= 3u || (ctx->reqCount % CAN_RS485_SOC_LOG_EVERY_N) == 0u) {
                    ESP_LOGI(EXAMPLE_TAG,
                             "CAN->RS485 Growatt translator on %s: req start=0x%04X count=0x%04X sent=%s src=%s soc=%u pack=%.2fV",
                             ctx->ifName,
                             (unsigned)start,
                             (unsigned)count,
                             sent ? "Y" : "N",
                             socFromModel ? "UNIVERSAL" : (socFromCan ? "CAN_SOC+FALLBACK" : "FALLBACK"),
                             (unsigned)socPct,
                             (double)model.packVoltageV);
                }
#endif
            }

            frameLen = 0;
            frameHaveLastByte = false;
        }
    }
}

void canRs485GrowattBridgeEnable(uart_port_t inverterUart,
                                 gpio_num_t inverterDir,
                                 const char *ifName,
                                 const char *srcCanIf)
{
#if !CAN_RS485_SOC_TRANSLATOR_ENABLE
    (void)inverterUart;
    (void)inverterDir;
    (void)ifName;
    (void)srcCanIf;
    ESP_LOGI(EXAMPLE_TAG, "CAN->RS485 Growatt translator disabled by config");
    return;
#else
    if (g_canRsGrowattTaskHandle != NULL) {
        ESP_LOGI(EXAMPLE_TAG, "CAN->RS485 Growatt translator already running");
        return;
    }

    memset(&g_canRsGrowattCtx, 0, sizeof(g_canRsGrowattCtx));
    g_canRsGrowattCtx.uart = inverterUart;
    g_canRsGrowattCtx.dirPin = inverterDir;
    g_canRsGrowattCtx.ifName = (ifName != NULL) ? ifName : "RS485";
    g_canRsGrowattCtx.srcCanIf = (srcCanIf != NULL) ? srcCanIf : "CAN1";
    g_canRsGrowattCtx.slaveId = (uint8_t)CAN_RS485_SOC_SLAVE_ID;
    g_canRsGrowattCtx.fakeSocPct = (uint8_t)((CAN_RS485_SOC_FAKE_PCT > 100u) ? 100u : CAN_RS485_SOC_FAKE_PCT);

    xTaskCreate(canRs485GrowattTask,
                "can_to_rs485_gw",
                4096,
                &g_canRsGrowattCtx,
                9,
                &g_canRsGrowattTaskHandle);

    ESP_LOGI(EXAMPLE_TAG,
             "CAN->RS485 Growatt translator enabled (if=%s src=%s slave=%u fallbackSOC=%u%%)",
             g_canRsGrowattCtx.ifName,
             g_canRsGrowattCtx.srcCanIf,
             (unsigned)g_canRsGrowattCtx.slaveId,
             (unsigned)g_canRsGrowattCtx.fakeSocPct);
#endif
}

void canRs485GrowattBridgeStop(void)
{
    if (g_canRsGrowattTaskHandle != NULL) {
        vTaskDelete(g_canRsGrowattTaskHandle);
        g_canRsGrowattTaskHandle = NULL;
    }
    memset(&g_canRsGrowattCtx, 0, sizeof(g_canRsGrowattCtx));
}

void rs485Can322BridgeEnable(modbusDecoder_t *srcDecoder, twai_handle_t txBus, const char *txName)
{
#if !RS485_CAN_322_TRANSLATOR_ENABLE
    ESP_LOGI(EXAMPLE_TAG, "RS485->CAN translator disabled by config");
    return;
#else
    if (g_rsCanTaskHandle != NULL) {
        ESP_LOGI(EXAMPLE_TAG, "RS485->CAN translator already running");
        return;
    }

    if (srcDecoder == NULL || txBus == NULL) {
        ESP_LOGW(EXAMPLE_TAG, "RS485->CAN translator not started: invalid source decoder or CAN bus");
        return;
    }

    memset(&g_rsCanCtx, 0, sizeof(g_rsCanCtx));
    g_rsCanCtx.src = srcDecoder;
    g_rsCanCtx.txBus = txBus;
    g_rsCanCtx.txName = (txName != NULL) ? txName : "CAN";

    xTaskCreate(rs485CanTelemetryTask,
                "rs485_to_can",
                4096,
                &g_rsCanCtx,
                8,
                &g_rsCanTaskHandle);

    ESP_LOGI(EXAMPLE_TAG,
             "RS485->CAN translator enabled (tx=%s, period=%dms, fallback=%s)",
             g_rsCanCtx.txName,
             RS485_CAN_322_TX_PERIOD_MS,
             RS485_CAN_BRIDGE_USE_FALLBACK ? "ON" : "OFF");
#endif
}

void rs485Can322BridgeStop(void)
{
    if (g_rsCanTaskHandle != NULL) {
        vTaskDelete(g_rsCanTaskHandle);
        g_rsCanTaskHandle = NULL;
    }
    memset(&g_rsCanCtx, 0, sizeof(g_rsCanCtx));
}



