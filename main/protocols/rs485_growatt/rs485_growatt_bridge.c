#include "protocols/rs485_growatt/rs485_growatt_bridge.h"

#include "decoders/CAN_Decoder.h"
#include "Drivers/rs485_driver.h"
#include "config.h"
#include "protocols/common/battery_model.h"
#include "protocols/goodwe/goodwe_registers_map.h"
#include "protocols/growatt/growatt_registers_map.h"
#include "protocols/pylon/pylon_registers_map.h"
#include "protocols/sofar/sofar_registers_map.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    bool hasSoc;
    uint8_t socPct;
    bool hasSoh;
    uint8_t sohPct;
    bool hasTempC;
    int16_t tempC;
    int16_t tempDeciC;
    bool hasTempRangeDeciC;
    int16_t tempMinDeciC;
    int16_t tempMaxDeciC;
    bool hasPackCv;
    uint16_t packCv;
    bool hasCycles;
    uint16_t cycles;
    bool hasLimits;
    uint16_t chargeVoltLimitCv;
    uint16_t chargeCurrentLimitCa;
    uint16_t dischargeCurrentLimitCa;
    bool hasCellExtremes;
    uint16_t cellMaxMv;
    uint16_t cellMinMv;
    uint8_t cellMaxIdx;
    uint8_t cellMinIdx;
} canGrowattCache_t;

typedef struct {
    uart_port_t uart;
    gpio_num_t dirPin;
    const char *ifName;
    twai_handle_t srcCanBus;
    const char *srcCanIf;
    uint8_t slaveId;
    uint8_t fakeSocPct;
    canGrowattCache_t cache;
    int64_t lastCanFrameUs;
    uint32_t rxBytes;
    uint32_t reqCount;
    uint32_t rspCount;
} canRs485GrowattCtx_t;

static canRs485GrowattCtx_t g_canRsGrowattCtx;
static TaskHandle_t g_canRsGrowattTaskHandle;

typedef struct {
    uart_port_t uart;
    gpio_num_t dirPin;
    const char *ifName;
    uint8_t slaveId;
    uint8_t fakeSocPct;
    uint32_t rxBytes;
    uint32_t reqCount;
    uint32_t rspCount;
} jkbmsRs485GrowattCtx_t;

static jkbmsRs485GrowattCtx_t g_jkbmsRsGrowattCtx;
static TaskHandle_t g_jkbmsRsGrowattTaskHandle;
static can_rs485_growatt_snapshot_t g_canRsGrowattSnapshot;
static portMUX_TYPE g_canRsGrowattSnapshotMux = portMUX_INITIALIZER_UNLOCKED;

static int drainUartRx(uart_port_t uart, int maxBytes)
{
    uint8_t tmp[64];
    int drained = 0;

    if (maxBytes <= 0) {
        return 0;
    }

    while (drained < maxBytes) {
        int toRead = (int)sizeof(tmp);
        if ((maxBytes - drained) < toRead) {
            toRead = maxBytes - drained;
        }

        int got = uart_read_bytes(uart, tmp, (uint32_t)toRead, 0);
        if (got <= 0) {
            break;
        }
        drained += got;
    }

    return drained;
}

static inline void putBe16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)((v >> 8) & 0xFFu);
    p[1] = (uint8_t)(v & 0xFFu);
}

static inline uint16_t be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static inline int16_t be16s(const uint8_t *p)
{
    return (int16_t)be16(p);
}

static inline uint16_t le16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[1] << 8) | (uint16_t)p[0]);
}

static inline int16_t deciCToIntC(int16_t tempDeciC)
{
    return (tempDeciC >= 0) ? (int16_t)((tempDeciC + 5) / 10) : (int16_t)((tempDeciC - 5) / 10);
}

static uint16_t crc16(const uint8_t *data, int len)
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

static bool checkCrc(const uint8_t *frame, int len)
{
    if (frame == NULL || len < 4) {
        return false;
    }
    const uint16_t got = (uint16_t)(frame[len - 2] | ((uint16_t)frame[len - 1] << 8));
    const uint16_t calc = crc16(frame, len - 2);
    return got == calc;
}

static void putLe16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void ensureCellExtremesFromPack(canGrowattCache_t *cache)
{
    if (cache == NULL) {
        return;
    }
    if (cache->hasCellExtremes || !cache->hasPackCv) {
        return;
    }
    if (cache->packCv < 3000u) {
        return;
    }

    /*
     * Some CAN profiles (including Sofar-compatible variants) do not expose
     * per-cell min/max in dedicated frames. Derive a stable fallback from
     * pack voltage so translator replies remain valid and inverter does not
     * enter fault due to missing data.
     */
    uint16_t avgMv = (uint16_t)(((uint32_t)cache->packCv * 10u + 8u) / 16u);
    if (avgMv < 1500u || avgMv > 5000u) {
        return;
    }

    cache->cellMinMv = (avgMv > 2u) ? (uint16_t)(avgMv - 2u) : avgMv;
    cache->cellMaxMv = (uint16_t)(avgMv + 2u);
    cache->cellMinIdx = 1u;
    cache->cellMaxIdx = 2u;
    cache->hasCellExtremes = true;
}

static void cacheFromCanFrame(canRs485GrowattCtx_t *ctx, const twai_message_t *m)
{
    if (ctx == NULL || m == NULL || m->data_length_code != 8u) {
        return;
    }

    const uint32_t id = (uint32_t)m->identifier;
    const uint8_t *d = m->data;
    bool handled = false;

    switch (id) {
        case GROWATT_CAN_ID_313_V_I_SOC_SOH: {
            ctx->cache.packCv = be16(&d[0]);
            ctx->cache.hasPackCv = true;

            const int16_t tDeci = be16s(&d[4]);
            ctx->cache.tempDeciC = tDeci;
            ctx->cache.tempC = deciCToIntC(tDeci);
            ctx->cache.hasTempC = true;

            ctx->cache.socPct = (uint8_t)((d[6] > 100u) ? 100u : d[6]);
            ctx->cache.hasSoc = true;

            ctx->cache.sohPct = (uint8_t)(d[7] & 0x7Fu);
            if (ctx->cache.sohPct > 100u) {
                ctx->cache.sohPct = 100u;
            }
            ctx->cache.hasSoh = true;
            handled = true;
            break;
        }
        case GROWATT_CAN_ID_314_RM_FCC_DV_CYCLES: {
            ctx->cache.cycles = be16(&d[6]);
            ctx->cache.hasCycles = true;
            handled = true;
            break;
        }
        case GROWATT_CAN_ID_319_CELL_REF_FLAGS: {
            ctx->cache.cellMaxMv = le16(&d[0]);
            ctx->cache.cellMinMv = le16(&d[2]);
            ctx->cache.cellMaxIdx = d[5];
            ctx->cache.cellMinIdx = d[6];
            ctx->cache.hasCellExtremes = true;
            handled = true;
            break;
        }
        case GROWATT_CAN_ID_322_TEMP_SOC_MIN_MAX: {
            const int16_t tDeci = be16s(&d[0]);
            ctx->cache.tempDeciC = tDeci;
            ctx->cache.tempC = deciCToIntC(tDeci);
            ctx->cache.hasTempC = true;
            ctx->cache.socPct = (uint8_t)((d[6] > 100u) ? 100u : d[6]);
            ctx->cache.hasSoc = true;
            handled = true;
            break;
        }
        case GOODWE_CAN_ID_LIMITS_456: {
            /* GoodWe CAN: limits are little-endian 0.1V/0.1A. */
            const uint16_t vLimDeciV = le16(&d[GOODWE_CAN_456_OFF_CHG_VLIM_DV]);
            const int16_t chgLimDeciA = (int16_t)le16(&d[GOODWE_CAN_456_OFF_CHG_ILIM_DA]);
            const int16_t disLimDeciA = (int16_t)le16(&d[GOODWE_CAN_456_OFF_DIS_ILIM_DA]);
            const uint16_t absChg = (uint16_t)((chgLimDeciA < 0) ? (-(int32_t)chgLimDeciA) : (int32_t)chgLimDeciA);
            const uint16_t absDis = (uint16_t)((disLimDeciA < 0) ? (-(int32_t)disLimDeciA) : (int32_t)disLimDeciA);
            ctx->cache.chargeVoltLimitCv = (uint16_t)(vLimDeciV * 10u);
            ctx->cache.chargeCurrentLimitCa = (uint16_t)(absChg * 10u);
            ctx->cache.dischargeCurrentLimitCa = (uint16_t)(absDis * 10u);
            ctx->cache.hasLimits = true;
            handled = true;
            break;
        }
        case GOODWE_CAN_ID_SOC_SOH_457: {
            /* GoodWe CAN: SOC/SOH encoded in 0.1%. */
            const uint16_t socDeciPct = le16(&d[GOODWE_CAN_457_OFF_SOC_DECIPCT]);
            const uint16_t sohDeciPct = le16(&d[GOODWE_CAN_457_OFF_SOH_DECIPCT]);
            const uint16_t socPct = (uint16_t)((socDeciPct + 5u) / 10u);
            const uint16_t sohPct = (uint16_t)((sohDeciPct + 5u) / 10u);
            ctx->cache.socPct = (uint8_t)((socPct > 100u) ? 100u : socPct);
            ctx->cache.sohPct = (uint8_t)((sohPct > 100u) ? 100u : sohPct);
            ctx->cache.hasSoc = true;
            ctx->cache.hasSoh = true;
            handled = true;
            break;
        }
        case GOODWE_CAN_ID_PACK_458: {
            /* GoodWe CAN: pack V/I/T are little-endian in 0.1 units. */
            const uint16_t packDeciV = le16(&d[GOODWE_CAN_458_OFF_PACK_V_DV]);
            const int16_t tempDeci = (int16_t)le16(&d[GOODWE_CAN_458_OFF_TEMP_DECIC]);
            ctx->cache.packCv = (uint16_t)(packDeciV * 10u);
            ctx->cache.hasPackCv = true;
            ctx->cache.tempDeciC = tempDeci;
            ctx->cache.tempC = deciCToIntC(tempDeci);
            ctx->cache.hasTempC = true;

            if (!ctx->cache.hasCellExtremes && ctx->cache.packCv >= 3000u) {
                /*
                 * GoodWe payload has no explicit per-cell extremes.
                 * Derive a neutral estimate from pack voltage to keep Modbus replies consistent.
                 */
                const uint16_t avgMv = (uint16_t)(((uint32_t)ctx->cache.packCv * 10u + 8u) / 16u);
                if (avgMv >= 1500u && avgMv <= 5000u) {
                    ctx->cache.cellMinMv = avgMv;
                    ctx->cache.cellMaxMv = avgMv;
                    ctx->cache.cellMinIdx = 0u;
                    ctx->cache.cellMaxIdx = 0u;
                    ctx->cache.hasCellExtremes = true;
                }
            }
            handled = true;
            break;
        }
        case PYLON_CAN_ID_LIMITS_351: {
            /* Pylon CAN: charge/discharge limits, little-endian 0.1 units. */
            const uint16_t vLimDeciV = le16(&d[PYLON_CAN_351_OFF_CHG_VLIM_DV]);
            const uint16_t chgLimDeciA = le16(&d[PYLON_CAN_351_OFF_CHG_ILIM_DA]);
            const uint16_t disLimDeciA = le16(&d[PYLON_CAN_351_OFF_DIS_ILIM_DA]);
            ctx->cache.chargeVoltLimitCv = (uint16_t)(vLimDeciV * 10u);
            ctx->cache.chargeCurrentLimitCa = (uint16_t)(chgLimDeciA * 10u);
            ctx->cache.dischargeCurrentLimitCa = (uint16_t)(disLimDeciA * 10u);
            ctx->cache.hasLimits = true;
            handled = true;
            break;
        }
        case PYLON_CAN_ID_SOC_SOH_355: {
            /* Pylon CAN: SOC/SOH, little-endian integer percent. */
            const uint16_t soc = le16(&d[PYLON_CAN_355_OFF_SOC_PCT]);
            const uint16_t soh = le16(&d[PYLON_CAN_355_OFF_SOH_PCT]);
            ctx->cache.socPct = (uint8_t)((soc > 100u) ? 100u : soc);
            ctx->cache.sohPct = (uint8_t)((soh > 100u) ? 100u : soh);
            ctx->cache.hasSoc = true;
            ctx->cache.hasSoh = true;
            handled = true;
            break;
        }
        case PYLON_CAN_ID_PACK_356: {
            /* Pylon CAN: pack voltage/current/average temp, little-endian. */
            const uint16_t rawPackCv = le16(&d[PYLON_CAN_356_OFF_PACK_V_CV]);
            uint16_t packCv = rawPackCv;
            if (ctx->cache.hasLimits && ctx->cache.chargeVoltLimitCv >= 3000u) {
                /* Some BMS variants publish half-pack in 0x356; infer by comparing against charge limit. */
                if (rawPackCv < (uint16_t)((ctx->cache.chargeVoltLimitCv / 2u) + 200u)) {
                    uint32_t scaled = (uint32_t)rawPackCv * 2u;
                    packCv = (scaled > 65535u) ? 65535u : (uint16_t)scaled;
                }
            }
            const int16_t tempDeci = (int16_t)le16(&d[PYLON_CAN_356_OFF_TEMP_DECIC]);
            ctx->cache.packCv = packCv;
            ctx->cache.hasPackCv = true;
            ctx->cache.tempDeciC = tempDeci;
            ctx->cache.tempC = deciCToIntC(tempDeci);
            ctx->cache.hasTempC = true;
            handled = true;
            break;
        }
        case PYLON_CAN_ID_CELL_TEMP_373: {
            /* Pylon CAN: inferred min/max cell voltage in mV (little-endian). */
            const uint16_t cellMinMv = le16(&d[PYLON_CAN_373_OFF_CELL_MIN_MV]);
            const uint16_t cellMaxMv = le16(&d[PYLON_CAN_373_OFF_CELL_MAX_MV]);
            if (cellMinMv >= 1500u && cellMinMv <= 5000u &&
                cellMaxMv >= 1500u && cellMaxMv <= 5000u) {
                ctx->cache.cellMinMv = cellMinMv;
                ctx->cache.cellMaxMv = cellMaxMv;
                /* No reliable per-cell index in 0x373; keep unknown. */
                ctx->cache.cellMinIdx = 0u;
                ctx->cache.cellMaxIdx = 0u;
                ctx->cache.hasCellExtremes = true;
            }
            {
                const int16_t t1Deci = (int16_t)le16(&d[PYLON_CAN_373_OFF_TEMP1_DECIC]);
                const int16_t t2Deci = (int16_t)le16(&d[PYLON_CAN_373_OFF_TEMP2_DECIC]);
                const bool t1Ok = (t1Deci >= -500 && t1Deci <= 1200);
                const bool t2Ok = (t2Deci >= -500 && t2Deci <= 1200);
                if (t1Ok && t2Ok) {
                    if (t1Deci <= t2Deci) {
                        ctx->cache.tempMinDeciC = t1Deci;
                        ctx->cache.tempMaxDeciC = t2Deci;
                    } else {
                        ctx->cache.tempMinDeciC = t2Deci;
                        ctx->cache.tempMaxDeciC = t1Deci;
                    }
                    ctx->cache.hasTempRangeDeciC = true;
                }
            }
            handled = true;
            break;
        }
        default:
            break;
    }

    if (handled) {
        ensureCellExtremesFromPack(&ctx->cache);
        ctx->lastCanFrameUs = esp_timer_get_time();
    }
}

static bool parseReadReq(const uint8_t *frame,
                         int len,
                         uint8_t slaveId,
                         uint8_t *funcOut,
                         uint16_t *startOut,
                         uint16_t *countOut)
{
    if (frame == NULL || len != 8 || !checkCrc(frame, len)) {
        return false;
    }

    if (frame[0] != slaveId) {
        return false;
    }

    const uint8_t func = frame[1];
    if (func != 0x03u && func != 0x04u) {
        return false;
    }

    const uint16_t start = be16(&frame[2]);
    const uint16_t count = be16(&frame[4]);
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

static uint16_t fallbackCell(uint8_t idx)
{
    static const uint16_t k_cells[16] = {
        3450u, 3450u, 3451u, 3452u,
        3450u, 3450u, 3451u, 3452u,
        3449u, 3448u, 3449u, 3448u,
        3450u, 3451u, 3450u, 3451u
    };
    if (idx >= 16u) {
        return 3450u;
    }
    return k_cells[idx];
}

typedef struct {
    uint16_t soc;
    uint16_t soh;
    uint16_t packCv;
    int16_t tempC;
    uint16_t cycles;
    uint16_t cellMaxMv;
    uint16_t cellMinMv;
    uint8_t cellMaxIdx;
    uint8_t cellMinIdx;
} growattSynthSnapshot_t;

static bool fillSnapshotFromBatteryModel(growattSynthSnapshot_t *snap, uint8_t fallbackSocPct)
{
    battery_model_t model = {0};

    if (snap == NULL) {
        return false;
    }

    snap->soc = fallbackSocPct;
    snap->soh = 100u;
    snap->packCv = 5120u;
    snap->tempC = 25;
    snap->cycles = 0u;
    snap->cellMaxMv = 3452u;
    snap->cellMinMv = 3448u;
    snap->cellMaxIdx = 4u;
    snap->cellMinIdx = 10u;

    batteryModelGet(&model);
    if (!model.valid || model.updatedMs == 0u) {
        return false;
    }

    if (model.socPct <= 100u) {
        snap->soc = model.socPct;
    }
    if (model.sohPct <= 100u) {
        snap->soh = model.sohPct;
    }
    if (model.temperaturesC[0] > -100.0f) {
        snap->tempC = (int16_t)model.temperaturesC[0];
    }
    if (model.packVoltageV > 0.0f) {
        uint32_t packCv = (uint32_t)(model.packVoltageV * 100.0f + 0.5f);
        snap->packCv = (packCv > UINT16_MAX) ? UINT16_MAX : (uint16_t)packCv;
    }
    if (model.cycleCount > 0u) {
        snap->cycles = model.cycleCount;
    }
    if (model.cellMaxV > 0.0f) {
        uint32_t cellMaxMv = (uint32_t)(model.cellMaxV * 1000.0f + 0.5f);
        snap->cellMaxMv = (cellMaxMv > UINT16_MAX) ? UINT16_MAX : (uint16_t)cellMaxMv;
    }
    if (model.cellMinV > 0.0f) {
        uint32_t cellMinMv = (uint32_t)(model.cellMinV * 1000.0f + 0.5f);
        snap->cellMinMv = (cellMinMv > UINT16_MAX) ? UINT16_MAX : (uint16_t)cellMinMv;
    }
    if (model.cellMaxIdx >= 1u && model.cellMaxIdx <= 16u) {
        snap->cellMaxIdx = model.cellMaxIdx;
    }
    if (model.cellMinIdx >= 1u && model.cellMinIdx <= 16u) {
        snap->cellMinIdx = model.cellMinIdx;
    }
    return true;
}

static void synthCellMap16(uint16_t *cellsOut,
                           uint16_t cellMaxMv,
                           uint16_t cellMinMv,
                           uint8_t cellMaxIdx,
                           uint8_t cellMinIdx)
{
    if (cellsOut == NULL) {
        return;
    }

    for (uint8_t i = 0; i < 16u; i++) {
        cellsOut[i] = fallbackCell(i);
    }

    if (cellMaxIdx >= 1u && cellMaxIdx <= 16u) {
        cellsOut[cellMaxIdx - 1u] = cellMaxMv;
    }
    if (cellMinIdx >= 1u && cellMinIdx <= 16u) {
        cellsOut[cellMinIdx - 1u] = cellMinMv;
    }
}

static void publishCanRsGrowattSnapshot(uint8_t socPct,
                                        uint8_t sohPct,
                                        int16_t tempC,
                                        int16_t tempDeciC,
                                        int16_t tempMinDeciC,
                                        int16_t tempMaxDeciC,
                                        uint16_t packCv,
                                        uint16_t cycles,
                                        uint16_t remCapCah,
                                        uint16_t fullCapCah,
                                        uint16_t cellMaxMv,
                                        uint16_t cellMinMv,
                                        uint8_t cellMaxIdx,
                                        uint8_t cellMinIdx)
{
    can_rs485_growatt_snapshot_t s = {0};
    s.valid = true;
    s.timestampUs = esp_timer_get_time();
    s.socPct = socPct;
    s.sohPct = sohPct;
    s.tempC = tempC;
    s.tempDeciC = tempDeciC;
    s.tempMinDeciC = tempMinDeciC;
    s.tempMaxDeciC = tempMaxDeciC;
    s.packCv = packCv;
    s.cycles = cycles;
    s.remainingCapCah = remCapCah;
    s.fullCapCah = fullCapCah;
    s.cellMaxMv = cellMaxMv;
    s.cellMinMv = cellMinMv;
    s.cellMaxIdx = cellMaxIdx;
    s.cellMinIdx = cellMinIdx;
    s.cellCount = 16u;
    synthCellMap16(s.cellMv, cellMaxMv, cellMinMv, cellMaxIdx, cellMinIdx);

    portENTER_CRITICAL(&g_canRsGrowattSnapshotMux);
    g_canRsGrowattSnapshot = s;
    portEXIT_CRITICAL(&g_canRsGrowattSnapshotMux);
}

static uint16_t synthRegFromSnapshot(const growattSynthSnapshot_t *snap, uint16_t addr)
{
    if (snap == NULL) {
        return 0u;
    }

    const uint16_t soc = snap->soc;
    const uint16_t soh = snap->soh;
    const uint16_t packCv = snap->packCv;
    const uint16_t tempC = (uint16_t)snap->tempC;
    const uint16_t cycles = snap->cycles;
    const uint16_t fullCap = 4000u; /* 40.00Ah in 0.01Ah units */
    const uint16_t remCap = (uint16_t)(((uint32_t)fullCap * (uint32_t)soc) / 100u);
    uint16_t cellMaxMv = snap->cellMaxMv;
    uint16_t cellMinMv = snap->cellMinMv;
    uint8_t cellMaxIdx = snap->cellMaxIdx;
    uint8_t cellMinIdx = snap->cellMinIdx;

    if (cellMaxIdx < 1u || cellMaxIdx > 16u) {
        cellMaxIdx = 4u;
    }
    if (cellMinIdx < 1u || cellMinIdx > 16u) {
        cellMinIdx = 10u;
    }
    if (cellMaxMv < cellMinMv) {
        uint16_t t = cellMaxMv;
        cellMaxMv = cellMinMv;
        cellMinMv = t;
    }

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
            return 0x0000u;
        case GROWATT_MB_REG_SOC_PCT:
            return soc;
        case GROWATT_MB_REG_PACK_V_CV:
            return packCv;
        case GROWATT_MB_REG_PACK_I_ABS_CA_TENTATIVE:
            return 0u;
        case GROWATT_MB_REG_TEMP_C:
            return tempC;
        case GROWATT_MB_REG_CYCLE_COUNT_TENTATIVE:
            return cycles;
        case GROWATT_MB_REG_REMAIN_CAP_CAH:
            return remCap;
        case GROWATT_MB_REG_FULL_CAP_CAH:
            return fullCap;
        case GROWATT_MB_REG_SOH_PCT:
            return soh;
        case GROWATT_MB_REG_CV_TARGET_CV:
            return packCv;
        case GROWATT_MB_REG_ICHG_LIM_CA_TENTATIVE:
            return 0u;
        case GROWATT_MB_REG_IDIS_LIM_CA_TENTATIVE:
            return 0u;
        case GROWATT_MB_REG_CELL_MAX_MV:
            return cellMaxMv;
        case GROWATT_MB_REG_CELL_MIN_MV:
            return cellMinMv;
        case GROWATT_MB_REG_CELL_MAX_IDX:
            return cellMaxIdx;
        case GROWATT_MB_REG_CELL_MIN_IDX:
            return cellMinIdx;
        case GROWATT_MB_REG_CELL_EXTRA:
            return 0u;
        default:
            if (addr >= GROWATT_MB_REG_CELL_BASE && addr <= GROWATT_MB_REG_CELL_LAST) {
                uint8_t idx = (uint8_t)(addr - GROWATT_MB_REG_CELL_BASE);
                if (idx == (uint8_t)(cellMaxIdx - 1u)) {
                    return cellMaxMv;
                }
                if (idx == (uint8_t)(cellMinIdx - 1u)) {
                    return cellMinMv;
                }
                return fallbackCell(idx);
            }
            return 0u;
    }
}

static uint16_t synthReg(const canRs485GrowattCtx_t *ctx, uint16_t addr)
{
    const uint16_t soc = ctx->cache.hasSoc ? ctx->cache.socPct : 0u;
    const uint16_t soh = ctx->cache.hasSoh ? ctx->cache.sohPct : 0u;
    const uint16_t packCv = ctx->cache.hasPackCv ? ctx->cache.packCv : 0u;
    const uint16_t tempC = (uint16_t)(ctx->cache.hasTempC ? ctx->cache.tempC : 0);
    const uint16_t cycles = ctx->cache.hasCycles ? ctx->cache.cycles : 0u;
    const uint16_t fullCap = ctx->cache.hasSoc ? 4000u : 0u; /* 40.00Ah in 0.01Ah units */
    const uint16_t remCap = (fullCap > 0u) ? (uint16_t)(((uint32_t)fullCap * (uint32_t)soc) / 100u) : 0u;
    uint16_t cellMaxMv = ctx->cache.hasCellExtremes ? ctx->cache.cellMaxMv : 0u;
    uint16_t cellMinMv = ctx->cache.hasCellExtremes ? ctx->cache.cellMinMv : 0u;
    uint8_t cellMaxIdx = ctx->cache.hasCellExtremes ? ctx->cache.cellMaxIdx : 0u;
    uint8_t cellMinIdx = ctx->cache.hasCellExtremes ? ctx->cache.cellMinIdx : 0u;

    if (cellMaxIdx > 16u) {
        cellMaxIdx = 0u;
    }
    if (cellMinIdx > 16u) {
        cellMinIdx = 0u;
    }
    if (cellMaxMv > 0u && cellMinMv > 0u && cellMaxMv < cellMinMv) {
        uint16_t tMv = cellMaxMv;
        cellMaxMv = cellMinMv;
        cellMinMv = tMv;
    }

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
            return 0x0000u;
        case GROWATT_MB_REG_SOC_PCT:
            return soc;
        case GROWATT_MB_REG_PACK_V_CV:
            return packCv;
        case GROWATT_MB_REG_PACK_I_ABS_CA_TENTATIVE:
            return 0u;
        case GROWATT_MB_REG_TEMP_C:
            return tempC;
        case GROWATT_MB_REG_CYCLE_COUNT_TENTATIVE:
            return cycles;
        case GROWATT_MB_REG_REMAIN_CAP_CAH:
            return remCap;
        case GROWATT_MB_REG_FULL_CAP_CAH:
            return fullCap;
        case GROWATT_MB_REG_SOH_PCT:
            return soh;
        case GROWATT_MB_REG_CV_TARGET_CV:
            return ctx->cache.hasLimits ? ctx->cache.chargeVoltLimitCv : packCv;
        case GROWATT_MB_REG_ICHG_LIM_CA_TENTATIVE:
            return ctx->cache.hasLimits ? ctx->cache.chargeCurrentLimitCa : 0u;
        case GROWATT_MB_REG_IDIS_LIM_CA_TENTATIVE:
            return ctx->cache.hasLimits ? ctx->cache.dischargeCurrentLimitCa : 0u;
        case GROWATT_MB_REG_CELL_MAX_MV:
            return cellMaxMv;
        case GROWATT_MB_REG_CELL_MIN_MV:
            return cellMinMv;
        case GROWATT_MB_REG_CELL_MAX_IDX:
            return cellMaxIdx;
        case GROWATT_MB_REG_CELL_MIN_IDX:
            return cellMinIdx;
        case GROWATT_MB_REG_CELL_EXTRA:
            return 0u;
        default:
            if (addr >= GROWATT_MB_REG_CELL_BASE && addr <= GROWATT_MB_REG_CELL_LAST) {
                uint8_t idx = (uint8_t)(addr - GROWATT_MB_REG_CELL_BASE);
                if (cellMaxIdx > 0u && idx == (uint8_t)(cellMaxIdx - 1u)) {
                    return cellMaxMv;
                }
                if (cellMinIdx > 0u && idx == (uint8_t)(cellMinIdx - 1u)) {
                    return cellMinMv;
                }
                return 0u;
            }
            return 0u;
    }
}

static bool sendGrowattResponse(canRs485GrowattCtx_t *ctx, uint8_t func, uint16_t start, uint16_t count)
{
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

    for (uint16_t i = 0; i < count; i++) {
        uint16_t addr = (uint16_t)(start + i);
        uint16_t val = synthReg(ctx, addr);
        putBe16(&resp[3 + (i * 2u)], val);
    }

    uint16_t crc = crc16(resp, respLen - 2);
    putLe16(&resp[respLen - 2], crc);

    rs485SetDirection(ctx->dirPin, true);
    int written = uart_write_bytes(ctx->uart, (const char *)resp, respLen);
    if (written == respLen) {
        (void)uart_wait_tx_done(ctx->uart, pdMS_TO_TICKS(100));
    }
    rs485SetDirection(ctx->dirPin, false);

    return written == respLen;
}

static void logDecodedRegisterSnapshot(const canRs485GrowattCtx_t *ctx,
                                       uint16_t reqStart,
                                       uint16_t reqCount,
                                       bool sent)
{
    if (ctx == NULL) {
        return;
    }

    const uint16_t soc = synthReg(ctx, GROWATT_MB_REG_SOC_PCT);
    const uint16_t soh = synthReg(ctx, GROWATT_MB_REG_SOH_PCT);
    const int16_t tempC = (int16_t)synthReg(ctx, GROWATT_MB_REG_TEMP_C);
    const uint16_t packCv = synthReg(ctx, GROWATT_MB_REG_PACK_V_CV);
    const uint16_t cycles = synthReg(ctx, GROWATT_MB_REG_CYCLE_COUNT_TENTATIVE);
    const uint16_t remCap = synthReg(ctx, GROWATT_MB_REG_REMAIN_CAP_CAH);
    const uint16_t fullCap = synthReg(ctx, GROWATT_MB_REG_FULL_CAP_CAH);
    const uint16_t cMaxMv = synthReg(ctx, GROWATT_MB_REG_CELL_MAX_MV);
    const uint16_t cMinMv = synthReg(ctx, GROWATT_MB_REG_CELL_MIN_MV);
    const uint16_t cMaxIdx = synthReg(ctx, GROWATT_MB_REG_CELL_MAX_IDX);
    const uint16_t cMinIdx = synthReg(ctx, GROWATT_MB_REG_CELL_MIN_IDX);
    const float tempAvgC = (float)(ctx->cache.hasTempC ? ctx->cache.tempDeciC : (int16_t)(tempC * 10)) / 10.0f;
    float tempMinC = tempAvgC;
    float tempMaxC = tempAvgC;
    if (ctx->cache.hasTempRangeDeciC) {
        tempMinC = (float)ctx->cache.tempMinDeciC / 10.0f;
        tempMaxC = (float)ctx->cache.tempMaxDeciC / 10.0f;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "CAN->RS485 req#%u on %s start=0x%04X count=%u sent=%s | "
             "SOC=%u%% SOH=%u%% Tavg=%.1fC Tmin/Tmax=%.1f/%.1fC "
             "Vpack=%.2fV Cycles=%u Rem/FCC=%.2f/%.2fAh "
             "Cmax=%.3fV(#%u) Cmin=%.3fV(#%u)",
             (unsigned)ctx->reqCount,
             ctx->ifName,
             (unsigned)reqStart,
             (unsigned)reqCount,
             sent ? "Y" : "N",
             (unsigned)soc,
             (unsigned)soh,
             (double)tempAvgC,
             (double)tempMinC,
             (double)tempMaxC,
             (double)packCv / 100.0,
             (unsigned)cycles,
             (double)remCap / 100.0,
             (double)fullCap / 100.0,
             (double)cMaxMv / 1000.0,
             (unsigned)cMaxIdx,
             (double)cMinMv / 1000.0,
             (unsigned)cMinIdx);
}

static void publishSnapshotFromCanCtx(const canRs485GrowattCtx_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    const uint16_t soc = synthReg(ctx, GROWATT_MB_REG_SOC_PCT);
    const uint16_t soh = synthReg(ctx, GROWATT_MB_REG_SOH_PCT);
    const int16_t tempC = (int16_t)synthReg(ctx, GROWATT_MB_REG_TEMP_C);
    const uint16_t packCv = synthReg(ctx, GROWATT_MB_REG_PACK_V_CV);
    const uint16_t cycles = synthReg(ctx, GROWATT_MB_REG_CYCLE_COUNT_TENTATIVE);
    const uint16_t remCap = synthReg(ctx, GROWATT_MB_REG_REMAIN_CAP_CAH);
    const uint16_t fullCap = synthReg(ctx, GROWATT_MB_REG_FULL_CAP_CAH);
    const uint16_t cMaxMv = synthReg(ctx, GROWATT_MB_REG_CELL_MAX_MV);
    const uint16_t cMinMv = synthReg(ctx, GROWATT_MB_REG_CELL_MIN_MV);
    const uint8_t cMaxIdx = (uint8_t)synthReg(ctx, GROWATT_MB_REG_CELL_MAX_IDX);
    const uint8_t cMinIdx = (uint8_t)synthReg(ctx, GROWATT_MB_REG_CELL_MIN_IDX);
    const int16_t tempDeciC = ctx->cache.hasTempC ? ctx->cache.tempDeciC : (int16_t)(tempC * 10);
    int16_t tempMinDeciC = tempDeciC;
    int16_t tempMaxDeciC = tempDeciC;
    if (ctx->cache.hasTempRangeDeciC) {
        tempMinDeciC = ctx->cache.tempMinDeciC;
        tempMaxDeciC = ctx->cache.tempMaxDeciC;
    }
    if (tempMinDeciC > tempMaxDeciC) {
        int16_t t = tempMinDeciC;
        tempMinDeciC = tempMaxDeciC;
        tempMaxDeciC = t;
    }

    publishCanRsGrowattSnapshot((uint8_t)soc,
                                (uint8_t)soh,
                                tempC,
                                tempDeciC,
                                tempMinDeciC,
                                tempMaxDeciC,
                                packCv,
                                cycles,
                                remCap,
                                fullCap,
                                cMaxMv,
                                cMinMv,
                                cMaxIdx,
                                cMinIdx);
}

static bool canSourceFresh(const canRs485GrowattCtx_t *ctx)
{
    if (ctx == NULL || ctx->lastCanFrameUs <= 0) {
        return false;
    }

    int64_t ageUs = esp_timer_get_time() - ctx->lastCanFrameUs;
    if (ageUs < 0) {
        ageUs = 0;
    }
    return ageUs <= ((int64_t)BRIDGE_SOURCE_STALE_MS * 1000LL);
}

static bool canSnapshotReadyForReply(const canRs485GrowattCtx_t *ctx)
{
    if (ctx == NULL) {
        return false;
    }

    return ctx->cache.hasSoc &&
           ctx->cache.hasSoh &&
           ctx->cache.hasPackCv &&
           ctx->cache.hasTempC &&
           ctx->cache.hasCellExtremes;
}

static void canRs485GrowattTask(void *pv)
{
    canRs485GrowattCtx_t *ctx = (canRs485GrowattCtx_t *)pv;
    uint8_t rxChunk[64];
    uint8_t streamBuf[256];
    uint16_t streamLen = 0u;
    TickType_t lastStatsTick = xTaskGetTickCount();

    while (1) {
        twai_message_t canMsg = {0};
        while (twai_receive_v2(ctx->srcCanBus, &canMsg, 0) == ESP_OK) {
#ifdef TWAI_MSG_FLAG_SELF
            if (canMsg.flags & TWAI_MSG_FLAG_SELF) {
                continue;
            }
#endif
            canDecoderOnFrame(ctx->srcCanIf, &canMsg);
            cacheFromCanFrame(ctx, &canMsg);
        }

        int len = uart_read_bytes(ctx->uart, rxChunk, sizeof(rxChunk), pdMS_TO_TICKS(2));
        if (len > 0) {
            ctx->rxBytes += (uint32_t)len;

            if ((size_t)streamLen + (size_t)len > sizeof(streamBuf)) {
                if (streamLen > 7u) {
                    const uint16_t keep = 7u;
                    memmove(streamBuf, &streamBuf[streamLen - keep], keep);
                    streamLen = keep;
                }
            }

            if ((size_t)streamLen + (size_t)len <= sizeof(streamBuf)) {
                memcpy(&streamBuf[streamLen], rxChunk, (size_t)len);
                streamLen = (uint16_t)(streamLen + len);
            }

            while (streamLen >= 8u) {
                bool found = false;
                for (uint16_t off = 0; off + 8u <= streamLen; off++) {
                    uint8_t func = 0u;
                    uint16_t start = 0u;
                    uint16_t count = 0u;
                    if (parseReadReq(&streamBuf[off], 8, ctx->slaveId, &func, &start, &count)) {
                        ctx->reqCount++;
                        const bool ready = canSnapshotReadyForReply(ctx);
                        bool sent = ready && canSourceFresh(ctx) && sendGrowattResponse(ctx, func, start, count);
                        if (sent) {
                            ctx->rspCount++;
                            publishSnapshotFromCanCtx(ctx);
                        } else if (ctx->reqCount <= 3u || (ctx->reqCount % 25u) == 0u) {
                            ESP_LOGW(EXAMPLE_TAG,
                                     "CAN->RS485 source not ready (ready=%s fresh=%s hasSoc=%u hasSoh=%u hasPack=%u hasTemp=%u hasCellExt=%u)",
                                     ready ? "Y" : "N",
                                     canSourceFresh(ctx) ? "Y" : "N",
                                     (unsigned)ctx->cache.hasSoc,
                                     (unsigned)ctx->cache.hasSoh,
                                     (unsigned)ctx->cache.hasPackCv,
                                     (unsigned)ctx->cache.hasTempC,
                                     (unsigned)ctx->cache.hasCellExtremes);
                        }
                        if (ctx->reqCount <= 3u || (ctx->reqCount % 25u) == 0u) {
                            logDecodedRegisterSnapshot(ctx, start, count, sent);
                        }

                        const uint16_t consume = (uint16_t)(off + 8u);
                        if (consume < streamLen) {
                            memmove(streamBuf, &streamBuf[consume], streamLen - consume);
                        }
                        streamLen = (uint16_t)(streamLen - consume);
                        found = true;
                        break;
                    }
                }

                if (!found) {
                    if (streamLen > 7u) {
                        const uint16_t drop = (uint16_t)(streamLen - 7u);
                        memmove(streamBuf, &streamBuf[drop], 7u);
                        streamLen = 7u;
                    }
                    break;
                }
            }
        }

        if ((xTaskGetTickCount() - lastStatsTick) >= pdMS_TO_TICKS(5000)) {
            ESP_LOGI(EXAMPLE_TAG,
                     "CAN->RS485 %s stats: rxBytes=%u req=%u rsp=%u",
                     ctx->ifName,
                     (unsigned)ctx->rxBytes,
                     (unsigned)ctx->reqCount,
                     (unsigned)ctx->rspCount);
            ctx->rxBytes = 0u;
            lastStatsTick = xTaskGetTickCount();
        }
    }
}

static bool sendGrowattResponseFromSnapshot(const jkbmsRs485GrowattCtx_t *ctx,
                                            const growattSynthSnapshot_t *snap,
                                            uint8_t func,
                                            uint16_t start,
                                            uint16_t count)
{
    if (ctx == NULL || snap == NULL) {
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

    for (uint16_t i = 0; i < count; i++) {
        uint16_t addr = (uint16_t)(start + i);
        uint16_t val = synthRegFromSnapshot(snap, addr);
        putBe16(&resp[3 + (i * 2u)], val);
    }

    uint16_t crc = crc16(resp, respLen - 2);
    putLe16(&resp[respLen - 2], crc);

    rs485SetDirection(ctx->dirPin, true);
    int written = uart_write_bytes(ctx->uart, (const char *)resp, respLen);
    if (written == respLen) {
        (void)uart_wait_tx_done(ctx->uart, pdMS_TO_TICKS(100));
    }
    rs485SetDirection(ctx->dirPin, false);
    return written == respLen;
}

static void jkbmsRs485GrowattTask(void *pv)
{
    jkbmsRs485GrowattCtx_t *ctx = (jkbmsRs485GrowattCtx_t *)pv;
    uint8_t rxChunk[64];
    uint8_t streamBuf[256];
    uint16_t streamLen = 0u;
    TickType_t lastStatsTick = xTaskGetTickCount();

    while (1) {
        int len = uart_read_bytes(ctx->uart, rxChunk, sizeof(rxChunk), pdMS_TO_TICKS(2));
        if (len > 0) {
            ctx->rxBytes += (uint32_t)len;

            if ((size_t)streamLen + (size_t)len > sizeof(streamBuf)) {
                if (streamLen > 7u) {
                    const uint16_t keep = 7u;
                    memmove(streamBuf, &streamBuf[streamLen - keep], keep);
                    streamLen = keep;
                }
            }

            if ((size_t)streamLen + (size_t)len <= sizeof(streamBuf)) {
                memcpy(&streamBuf[streamLen], rxChunk, (size_t)len);
                streamLen = (uint16_t)(streamLen + len);
            }

            while (streamLen >= 8u) {
                bool found = false;
                for (uint16_t off = 0; off + 8u <= streamLen; off++) {
                    uint8_t func = 0u;
                    uint16_t start = 0u;
                    uint16_t count = 0u;
                    if (parseReadReq(&streamBuf[off], 8, ctx->slaveId, &func, &start, &count)) {
                        ctx->reqCount++;

                        growattSynthSnapshot_t snap = {0};
                        bool fresh = fillSnapshotFromBatteryModel(&snap, ctx->fakeSocPct);

                        bool sent = fresh && sendGrowattResponseFromSnapshot(ctx, &snap, func, start, count);
                        if (sent) {
                            ctx->rspCount++;
                            const uint16_t fullCap = 4000u;
                            const uint16_t remCap = (uint16_t)(((uint32_t)fullCap * (uint32_t)snap.soc) / 100u);
                            publishCanRsGrowattSnapshot((uint8_t)snap.soc,
                                                        (uint8_t)snap.soh,
                                                        snap.tempC,
                                                        (int16_t)(snap.tempC * 10),
                                                        (int16_t)(snap.tempC * 10),
                                                        (int16_t)(snap.tempC * 10),
                                                        snap.packCv,
                                                        snap.cycles,
                                                        remCap,
                                                        fullCap,
                                                        snap.cellMaxMv,
                                                        snap.cellMinMv,
                                                        snap.cellMaxIdx,
                                                        snap.cellMinIdx);
                        }

                        if (ctx->reqCount <= 3u || (ctx->reqCount % 25u) == 0u) {
                            ESP_LOGI(EXAMPLE_TAG,
                                     "JKBMS->RS485 req#%u on %s start=0x%04X count=%u sent=%s | "
                                     "SOC=%u%% T=%dC Vpack=%.2fV Cmax=%.3fV(#%u) Cmin=%.3fV(#%u)",
                                     (unsigned)ctx->reqCount,
                                     ctx->ifName,
                                     (unsigned)start,
                                     (unsigned)count,
                                     sent ? "Y" : "N",
                                     (unsigned)snap.soc,
                                     (int)snap.tempC,
                                     (double)snap.packCv / 100.0,
                                     (double)snap.cellMaxMv / 1000.0,
                                     (unsigned)snap.cellMaxIdx,
                                     (double)snap.cellMinMv / 1000.0,
                                     (unsigned)snap.cellMinIdx);
                        }

                        const uint16_t consume = (uint16_t)(off + 8u);
                        if (consume < streamLen) {
                            memmove(streamBuf, &streamBuf[consume], streamLen - consume);
                        }
                        streamLen = (uint16_t)(streamLen - consume);
                        found = true;
                        break;
                    }
                }

                if (!found) {
                    if (streamLen > 7u) {
                        const uint16_t drop = (uint16_t)(streamLen - 7u);
                        memmove(streamBuf, &streamBuf[drop], 7u);
                        streamLen = 7u;
                    }
                    break;
                }
            }
        }

        if ((xTaskGetTickCount() - lastStatsTick) >= pdMS_TO_TICKS(5000)) {
            ESP_LOGI(EXAMPLE_TAG,
                     "JKBMS->RS485 %s stats: rxBytes=%u req=%u rsp=%u",
                     ctx->ifName,
                     (unsigned)ctx->rxBytes,
                     (unsigned)ctx->reqCount,
                     (unsigned)ctx->rspCount);
            ctx->rxBytes = 0u;
            lastStatsTick = xTaskGetTickCount();
        }
    }
}
esp_err_t canRs485GrowattBridgeEnable(uart_port_t inverterUart,
                                      gpio_num_t inverterDir,
                                      const char *ifName,
                                      twai_handle_t srcCanBus,
                                      const char *srcCanIf)
{
#if !CAN_RS485_SOC_TRANSLATOR_ENABLE
    (void)inverterUart;
    (void)inverterDir;
    (void)ifName;
    (void)srcCanBus;
    (void)srcCanIf;
    ESP_LOGI(EXAMPLE_TAG, "CAN->RS485 Growatt translator disabled by config");
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (g_canRsGrowattTaskHandle != NULL) {
        ESP_LOGI(EXAMPLE_TAG, "CAN->RS485 Growatt translator already running");
        return ESP_OK;
    }
    if (srcCanBus == NULL) {
        ESP_LOGW(EXAMPLE_TAG, "CAN->RS485 Growatt translator not started: source CAN bus is null");
        return ESP_ERR_INVALID_ARG;
    }

    memset(&g_canRsGrowattCtx, 0, sizeof(g_canRsGrowattCtx));
    g_canRsGrowattCtx.uart = inverterUart;
    g_canRsGrowattCtx.dirPin = inverterDir;
    g_canRsGrowattCtx.ifName = (ifName != NULL) ? ifName : "RS485";
    g_canRsGrowattCtx.srcCanBus = srcCanBus;
    g_canRsGrowattCtx.srcCanIf = (srcCanIf != NULL) ? srcCanIf : "CAN1";
    g_canRsGrowattCtx.slaveId = (uint8_t)CAN_RS485_SOC_SLAVE_ID;
    g_canRsGrowattCtx.fakeSocPct =
        (uint8_t)((CAN_RS485_SOC_FAKE_PCT > 100u) ? 100u : CAN_RS485_SOC_FAKE_PCT);

    rs485SetDirection(g_canRsGrowattCtx.dirPin, false);

    (void)drainUartRx(g_canRsGrowattCtx.uart, 4096);

    BaseType_t taskOk = xTaskCreate(canRs485GrowattTask,
                                    "can_to_rs485_gw",
                                    4096,
                                    &g_canRsGrowattCtx,
                                    9,
                                    &g_canRsGrowattTaskHandle);
    if (taskOk != pdPASS) {
        g_canRsGrowattTaskHandle = NULL;
        memset(&g_canRsGrowattCtx, 0, sizeof(g_canRsGrowattCtx));
        ESP_LOGE(EXAMPLE_TAG, "CAN->RS485 Growatt translator task create failed");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "CAN->RS485 Growatt translator enabled (if=%s src=%s slave=%u fallbackSOC=%u%%)",
             g_canRsGrowattCtx.ifName,
             g_canRsGrowattCtx.srcCanIf,
             (unsigned)g_canRsGrowattCtx.slaveId,
             (unsigned)g_canRsGrowattCtx.fakeSocPct);
    return ESP_OK;
#endif
}

esp_err_t jkbmsRs485GrowattBridgeEnable(uart_port_t inverterUart,
                                        gpio_num_t inverterDir,
                                        const char *ifName)
{
#if !CAN_RS485_SOC_TRANSLATOR_ENABLE
    (void)inverterUart;
    (void)inverterDir;
    (void)ifName;
    ESP_LOGI(EXAMPLE_TAG, "JKBMS->RS485 Growatt translator disabled by config");
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (g_jkbmsRsGrowattTaskHandle != NULL) {
        ESP_LOGI(EXAMPLE_TAG, "JKBMS->RS485 Growatt translator already running");
        return ESP_OK;
    }

    memset(&g_jkbmsRsGrowattCtx, 0, sizeof(g_jkbmsRsGrowattCtx));
    g_jkbmsRsGrowattCtx.uart = inverterUart;
    g_jkbmsRsGrowattCtx.dirPin = inverterDir;
    g_jkbmsRsGrowattCtx.ifName = (ifName != NULL) ? ifName : "RS485";
    g_jkbmsRsGrowattCtx.slaveId = (uint8_t)CAN_RS485_SOC_SLAVE_ID;
    g_jkbmsRsGrowattCtx.fakeSocPct =
        (uint8_t)((CAN_RS485_SOC_FAKE_PCT > 100u) ? 100u : CAN_RS485_SOC_FAKE_PCT);

    rs485SetDirection(g_jkbmsRsGrowattCtx.dirPin, false);
    (void)drainUartRx(g_jkbmsRsGrowattCtx.uart, 4096);

    BaseType_t taskOk = xTaskCreate(jkbmsRs485GrowattTask,
                                    "jkbms_to_rs485_gw",
                                    4096,
                                    &g_jkbmsRsGrowattCtx,
                                    9,
                                    &g_jkbmsRsGrowattTaskHandle);
    if (taskOk != pdPASS) {
        g_jkbmsRsGrowattTaskHandle = NULL;
        memset(&g_jkbmsRsGrowattCtx, 0, sizeof(g_jkbmsRsGrowattCtx));
        ESP_LOGE(EXAMPLE_TAG, "JKBMS->RS485 Growatt translator task create failed");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "JKBMS->RS485 Growatt translator enabled (if=%s slave=%u fallbackSOC=%u%%)",
             g_jkbmsRsGrowattCtx.ifName,
             (unsigned)g_jkbmsRsGrowattCtx.slaveId,
             (unsigned)g_jkbmsRsGrowattCtx.fakeSocPct);
    return ESP_OK;
#endif
}

void canRs485GrowattBridgeStop(void)
{
    if (g_canRsGrowattTaskHandle != NULL) {
        vTaskDelete(g_canRsGrowattTaskHandle);
        g_canRsGrowattTaskHandle = NULL;
    }
    if ((int)g_canRsGrowattCtx.dirPin >= 0) {
        rs485SetDirection(g_canRsGrowattCtx.dirPin, false);
    }
    memset(&g_canRsGrowattCtx, 0, sizeof(g_canRsGrowattCtx));

    if (g_jkbmsRsGrowattTaskHandle != NULL) {
        vTaskDelete(g_jkbmsRsGrowattTaskHandle);
        g_jkbmsRsGrowattTaskHandle = NULL;
    }
    if ((int)g_jkbmsRsGrowattCtx.dirPin >= 0) {
        rs485SetDirection(g_jkbmsRsGrowattCtx.dirPin, false);
    }
    memset(&g_jkbmsRsGrowattCtx, 0, sizeof(g_jkbmsRsGrowattCtx));

    portENTER_CRITICAL(&g_canRsGrowattSnapshotMux);
    memset(&g_canRsGrowattSnapshot, 0, sizeof(g_canRsGrowattSnapshot));
    portEXIT_CRITICAL(&g_canRsGrowattSnapshotMux);
}

bool canRs485GrowattBridgeGetLatestSnapshot(can_rs485_growatt_snapshot_t *out)
{
    can_rs485_growatt_snapshot_t snap = {0};

    if (out == NULL) {
        return false;
    }

    portENTER_CRITICAL(&g_canRsGrowattSnapshotMux);
    snap = g_canRsGrowattSnapshot;
    portEXIT_CRITICAL(&g_canRsGrowattSnapshotMux);

    if (!snap.valid || snap.timestampUs <= 0) {
        return false;
    }

    int64_t ageUs = esp_timer_get_time() - snap.timestampUs;
    if (ageUs < 0) {
        ageUs = 0;
    }
    if (ageUs > ((int64_t)BRIDGE_SOURCE_STALE_MS * 1000LL)) {
        return false;
    }

    *out = snap;
    return true;
}

