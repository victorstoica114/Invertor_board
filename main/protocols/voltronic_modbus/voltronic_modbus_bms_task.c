#include "protocols/voltronic_modbus/voltronic_modbus_bms_task.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "Drivers/rs485_driver.h"
#include "config.h"
#include "protocols/common/battery_model.h"
#include "protocols/voltronic_modbus/voltronic_modbus_poller.h"
#include "runtime_settings.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    QueueHandle_t outQueue;
    modbusDecoder_t decoder;
    voltronic_modbus_poller_t poller;
    uint32_t sequence;
    int64_t lastPublishUs;
} voltronicModbusBmsTaskCtx_t;

static voltronicModbusBmsTaskCtx_t g_voltronicModbusBmsCtx;
static TaskHandle_t g_voltronicModbusBmsTaskHandle;
static portMUX_TYPE g_latestPacketMux = portMUX_INITIALIZER_UNLOCKED;
static bool g_haveLatestPacket;
static bms_decoded_packet_t g_latestPacket;
static bool g_haveLatestSnapshot;
static voltronic_modbus_snapshot_t g_latestSnapshot;
static int64_t g_lastSourceStaleLogUs;
static int64_t g_lastDecodeLogUs;
static int64_t g_lastRawRxLogUs;
static int64_t g_lastTxLogUs;

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

static uint8_t clampU8(uint16_t v, uint8_t vmax)
{
    return (uint8_t)((v > vmax) ? vmax : v);
}

static bool decoderGetU16(const modbusDecoder_t *decoder, uint16_t reg, uint16_t *out)
{
    return modbusDecoderGetCachedReg(decoder, reg, out);
}

static bool decoderGetI16(const modbusDecoder_t *decoder, uint16_t reg, int16_t *out)
{
    uint16_t raw = 0u;
    if (!decoderGetU16(decoder, reg, &raw)) {
        return false;
    }
    if (out != NULL) {
        *out = (int16_t)raw;
    }
    return true;
}

static bool decoderGetU32(const modbusDecoder_t *decoder, uint16_t reg, uint32_t *out)
{
    uint16_t hi = 0u;
    uint16_t lo = 0u;
    if (!decoderGetU16(decoder, reg, &hi) ||
        !decoderGetU16(decoder, (uint16_t)(reg + 1u), &lo)) {
        return false;
    }

    if (out != NULL) {
        *out = (((uint32_t)hi) << 16) | (uint32_t)lo;
    }
    return true;
}

static bool decoderGetU32Best(const modbusDecoder_t *decoder,
                              uint16_t reg,
                              uint32_t minVal,
                              uint32_t maxVal,
                              uint32_t *out)
{
    uint16_t a = 0u;
    uint16_t b = 0u;
    if (!decoderGetU16(decoder, reg, &a) ||
        !decoderGetU16(decoder, (uint16_t)(reg + 1u), &b)) {
        return false;
    }

    const uint32_t ab = (((uint32_t)a) << 16) | (uint32_t)b;
    const uint32_t ba = (((uint32_t)b) << 16) | (uint32_t)a;
    const bool abOk = (ab >= minVal && ab <= maxVal);
    const bool baOk = (ba >= minVal && ba <= maxVal);
    if (!abOk && !baOk) {
        return false;
    }
    if (out != NULL) {
        *out = (abOk && baOk) ? ((ab <= ba) ? ab : ba) : (abOk ? ab : ba);
    }
    return true;
}

static bool decoderGetI32Best(const modbusDecoder_t *decoder,
                              uint16_t reg,
                              int32_t absLimit,
                              int32_t *out)
{
    uint16_t a = 0u;
    uint16_t b = 0u;
    if (!decoderGetU16(decoder, reg, &a) ||
        !decoderGetU16(decoder, (uint16_t)(reg + 1u), &b)) {
        return false;
    }

    const int32_t ab = (int32_t)((((uint32_t)a) << 16) | (uint32_t)b);
    const int32_t ba = (int32_t)((((uint32_t)b) << 16) | (uint32_t)a);
    const bool abOk = (ab <= absLimit && ab >= -absLimit);
    const bool baOk = (ba <= absLimit && ba >= -absLimit);
    if (!abOk && !baOk) {
        return false;
    }
    if (out != NULL) {
        const int64_t abAbs = (ab >= 0) ? (int64_t)ab : -(int64_t)ab;
        const int64_t baAbs = (ba >= 0) ? (int64_t)ba : -(int64_t)ba;
        *out = (abOk && baOk) ? ((abAbs <= baAbs) ? ab : ba) : (abOk ? ab : ba);
    }
    return true;
}

static bool normalizeCellMv(uint16_t raw, uint16_t *mvOut)
{
    uint16_t mv = 0u;

    if (raw >= 1000u && raw <= 6000u) {
        mv = raw;
    } else if (raw >= 100u && raw <= 600u) {
        mv = (uint16_t)(raw * 10u);
    } else if (raw >= 10u && raw <= 60u) {
        mv = (uint16_t)(raw * 100u);
    } else {
        return false;
    }

    if (mv < 1000u || mv > 6000u) {
        return false;
    }

    if (mvOut != NULL) {
        *mvOut = mv;
    }
    return true;
}

static bool normalizeDeciKelvinToDeciC(uint16_t raw, int16_t *deciCOut)
{
    int32_t deciC = 0;

    if (raw >= 2000u && raw <= 4500u) {
        deciC = (int32_t)raw - 2731;
    } else {
        const int16_t signedRaw = (int16_t)raw;
        if (signedRaw < -1000 || signedRaw > 1500) {
            return false;
        }
        deciC = signedRaw;
    }

    if (deciC < -1000 || deciC > 1500) {
        return false;
    }

    if (deciCOut != NULL) {
        *deciCOut = (int16_t)deciC;
    }
    return true;
}

static bool normalizeJkDeciC(int16_t raw, int16_t *deciCOut)
{
    if (raw < -1000 || raw > 1500) {
        return false;
    }
    if (deciCOut != NULL) {
        *deciCOut = raw;
    }
    return true;
}

static bool decodePctBytePair(uint16_t raw, bool preferHigh, uint8_t *pctOut)
{
    const uint8_t hi = (uint8_t)((raw >> 8) & 0xFFu);
    const uint8_t lo = (uint8_t)(raw & 0xFFu);
    const bool hiOk = hi <= 100u;
    const bool loOk = lo <= 100u;

    if (!hiOk && !loOk) {
        return false;
    }
    if (pctOut != NULL) {
        if (hiOk && loOk) {
            if (hi == 0u && lo > 0u) {
                *pctOut = lo;
            } else if (lo == 0u && hi > 0u) {
                *pctOut = hi;
            } else {
                *pctOut = preferHigh ? hi : lo;
            }
        } else {
            *pctOut = hiOk ? hi : lo;
        }
    }
    return true;
}

static void bytesToHex(const uint8_t *data, int len, char *out, size_t outCap)
{
    if (out == NULL || outCap == 0u) {
        return;
    }
    out[0] = '\0';
    if (data == NULL || len <= 0) {
        return;
    }

    int n = len;
    if (n > 128) {
        n = 128;
    }

    size_t pos = 0u;
    for (int i = 0; i < n && pos < outCap; i++) {
        int wrote = snprintf(&out[pos], outCap - pos, "%02X%s",
                             (unsigned)data[i],
                             (i + 1 < n) ? " " : "");
        if (wrote <= 0) {
            break;
        }
        pos += (size_t)wrote;
        if (pos >= outCap) {
            break;
        }
    }
    if (len > n && pos + 5u < outCap) {
        (void)snprintf(&out[pos], outCap - pos, " ...");
    }
}

static void decodePairStates(uint16_t raw, uint8_t *states, uint8_t firstIndex, uint8_t maxCount)
{
    if (states == NULL || firstIndex >= maxCount) {
        return;
    }

    states[firstIndex] = (uint8_t)((raw >> 8) & 0xFFu);
    if ((uint8_t)(firstIndex + 1u) < maxCount) {
        states[firstIndex + 1u] = (uint8_t)(raw & 0xFFu);
    }
}

static void storeLatestPacket(const bms_decoded_packet_t *packet)
{
    if (packet == NULL) {
        return;
    }

    portENTER_CRITICAL(&g_latestPacketMux);
    g_latestPacket = *packet;
    g_haveLatestPacket = true;
    portEXIT_CRITICAL(&g_latestPacketMux);
}

static void storeLatestSnapshot(const voltronic_modbus_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    portENTER_CRITICAL(&g_latestPacketMux);
    g_latestSnapshot = *snapshot;
    g_haveLatestSnapshot = true;
    portEXIT_CRITICAL(&g_latestPacketMux);
}

static void clearLatestData(void)
{
    portENTER_CRITICAL(&g_latestPacketMux);
    g_haveLatestPacket = false;
    memset(&g_latestPacket, 0, sizeof(g_latestPacket));
    g_haveLatestSnapshot = false;
    memset(&g_latestSnapshot, 0, sizeof(g_latestSnapshot));
    portEXIT_CRITICAL(&g_latestPacketMux);
}

static void publishBatteryModel(const voltronic_modbus_snapshot_t *snapshot)
{
    battery_model_t model = {0};

    if (snapshot == NULL || !snapshot->valid) {
        return;
    }

    model.valid = true;
    model.updatedMs = (uint32_t)(snapshot->timestampUs / 1000LL);
    model.sohPct = 100u;

    if (snapshot->hasPackVoltage) {
        model.packVoltageV = snapshot->packVoltageV;
    }
    if (snapshot->hasPackCurrent) {
        model.packCurrentA = snapshot->packCurrentA;
    }
    if (snapshot->hasSoc) {
        model.socPct = snapshot->socPct;
    }
    if (snapshot->hasChargeLimits) {
        model.chargeVoltageLimitV = snapshot->chargeVoltageLimitV;
        model.chargeCurrentLimitA = snapshot->chargeCurrentLimitA;
        model.dischargeCurrentLimitA = snapshot->dischargeCurrentLimitA;
    }
    if (snapshot->hasCellExtremes) {
        model.cellMinV = (float)snapshot->minCellMv / 1000.0f;
        model.cellMaxV = (float)snapshot->maxCellMv / 1000.0f;
        model.cellMinIdx = snapshot->minCellIndex;
        model.cellMaxIdx = snapshot->maxCellIndex;
        model.cellDeltaV = (float)(snapshot->maxCellMv - snapshot->minCellMv) / 1000.0f;
    }
    for (uint8_t i = 0u; i < snapshot->tempCount && i < UNIVERSAL_BATTERY_TEMP_SENSORS; i++) {
        model.temperaturesC[i] = (float)snapshot->tempDeciC[i] / 10.0f;
    }
    if (snapshot->hasStatusFlags) {
        model.protocolState = snapshot->statusFlags;
        model.chargeEnabled = snapshot->chargeEnabled;
        model.dischargeEnabled = snapshot->dischargeEnabled;
        model.balanceEnabled = false;
    }
    if (snapshot->hasAlarmRegisters) {
        model.alarmsMask = ((uint32_t)snapshot->chargeProtect << 16) |
                           (uint32_t)snapshot->dischargeProtect;
        model.warningsMask = ((uint32_t)snapshot->chargeAlarm << 16) |
                             (uint32_t)snapshot->dischargeAlarm;
    }

    batteryModelSet(&model);
}

static void logDecodedSnapshot(const voltronic_modbus_snapshot_t *snapshot, int64_t nowUs)
{
    if (snapshot == NULL || !snapshot->valid) {
        return;
    }
    if ((nowUs - g_lastDecodeLogUs) < 5000000LL) {
        return;
    }
    g_lastDecodeLogUs = nowUs;

    ESP_LOGI(EXAMPLE_TAG,
             "Voltronic decoded: soc=%u%% pack=%.2fV current=%.2fA cells=%u min=%.3fV#%u max=%.3fV#%u status=0x%04X",
             snapshot->hasSoc ? (unsigned)snapshot->socPct : 0u,
             snapshot->hasPackVoltage ? (double)snapshot->packVoltageV : 0.0,
             snapshot->hasPackCurrent ? (double)snapshot->packCurrentA : 0.0,
             (unsigned)snapshot->cellCount,
             snapshot->hasCellExtremes ? ((double)snapshot->minCellMv / 1000.0) : 0.0,
             snapshot->hasCellExtremes ? (unsigned)snapshot->minCellIndex : 0u,
             snapshot->hasCellExtremes ? ((double)snapshot->maxCellMv / 1000.0) : 0.0,
             snapshot->hasCellExtremes ? (unsigned)snapshot->maxCellIndex : 0u,
             snapshot->hasStatusFlags ? (unsigned)snapshot->statusFlags : 0u);

    if (snapshot->hasAlarmRegisters) {
        ESP_LOGI(EXAMPLE_TAG,
                 "Voltronic alerts: CA=0x%04X DA=0x%04X CP=0x%04X CP2=0x%04X DP=0x%04X DP2=0x%04X BMS=0x%04X",
                 (unsigned)snapshot->chargeAlarm,
                 (unsigned)snapshot->dischargeAlarm,
                 (unsigned)snapshot->chargeProtect,
                 (unsigned)snapshot->chargeProtect2,
                 (unsigned)snapshot->dischargeProtect,
                 (unsigned)snapshot->dischargeProtect2,
                 (unsigned)snapshot->bmsState);
    }
}

static void decodeJkCompatSnapshot(const modbusDecoder_t *decoder,
                                   voltronic_modbus_snapshot_t *outSnapshot)
{
    uint16_t u16 = 0u;
    uint32_t u32 = 0u;
    int32_t i32 = 0;
    int16_t i16 = 0;

    if (decoder == NULL || outSnapshot == NULL) {
        return;
    }

    if (decoderGetU16(decoder, VOLTRONIC_JK_REG_CELL_COUNT, &u16)) {
        uint8_t count = clampU8(u16, VOLTRONIC_MB_MAX_CELLS);
        if (count > outSnapshot->cellCount) {
            outSnapshot->cellCount = count;
        }
    }

    uint16_t minCell = outSnapshot->hasCellExtremes ? outSnapshot->minCellMv : UINT16_MAX;
    uint16_t maxCell = outSnapshot->hasCellExtremes ? outSnapshot->maxCellMv : 0u;
    uint8_t minIdx = outSnapshot->hasCellExtremes ? outSnapshot->minCellIndex : 0u;
    uint8_t maxIdx = outSnapshot->hasCellExtremes ? outSnapshot->maxCellIndex : 0u;
    uint8_t validCells = 0u;
    for (uint8_t i = 0u; i < VOLTRONIC_JK_MAX_CELLS && i < VOLTRONIC_MB_MAX_CELLS; i++) {
        uint16_t mv = 0u;
        const uint16_t addr = (uint16_t)(VOLTRONIC_JK_REG_CELL01_MV +
                                         ((uint16_t)i * VOLTRONIC_JK_REG_CELL_STEP));
        if (!decoderGetU16(decoder, addr, &u16) || !normalizeCellMv(u16, &mv)) {
            continue;
        }
        if (outSnapshot->cellMv[i] == 0u) {
            outSnapshot->cellMv[i] = mv;
        }
        if (outSnapshot->cellCount < (uint8_t)(i + 1u)) {
            outSnapshot->cellCount = (uint8_t)(i + 1u);
        }
        validCells++;
        if (mv < minCell) {
            minCell = mv;
            minIdx = (uint8_t)(i + 1u);
        }
        if (mv > maxCell) {
            maxCell = mv;
            maxIdx = (uint8_t)(i + 1u);
        }
    }
    if (validCells >= 2u && minIdx != 0u && maxIdx != 0u) {
        outSnapshot->hasCellExtremes = true;
        outSnapshot->minCellMv = minCell;
        outSnapshot->maxCellMv = maxCell;
        outSnapshot->minCellIndex = minIdx;
        outSnapshot->maxCellIndex = maxIdx;
    }

    if (!outSnapshot->hasPackVoltage &&
        decoderGetU16(decoder, VOLTRONIC_JK_REG_PACK_VOLTAGE_CV, &u16)) {
        if (u16 >= 1000u && u16 <= 20000u) {
            outSnapshot->hasPackVoltage = true;
            outSnapshot->packVoltageV = (float)u16 / 100.0f;
        } else if (u16 >= 100u && u16 <= 2000u) {
            outSnapshot->hasPackVoltage = true;
            outSnapshot->packVoltageV = (float)u16 / 10.0f;
        }
    }
    if (!outSnapshot->hasPackVoltage &&
        decoderGetU32Best(decoder, VOLTRONIC_JK_REG_PACK_VOLTAGE_MV_U32, 1000u, 200000u, &u32)) {
        outSnapshot->hasPackVoltage = true;
        outSnapshot->packVoltageV = (float)u32 / 1000.0f;
    }

    if (!outSnapshot->hasPackCurrent &&
        decoderGetI16(decoder, (uint16_t)(VOLTRONIC_JK_REG_PACK_VOLTAGE_CV + 2u), &i16)) {
        if (i16 >= -30000 && i16 <= 30000) {
            outSnapshot->hasPackCurrent = true;
            outSnapshot->packCurrentA = (float)i16 / 100.0f;
        }
    }
    if (!outSnapshot->hasPackCurrent &&
        decoderGetI32Best(decoder, VOLTRONIC_JK_REG_PACK_CURRENT_MA_I32, 1000000, &i32)) {
        outSnapshot->hasPackCurrent = true;
        outSnapshot->packCurrentA = (float)i32 / 1000.0f;
    }

    if (!outSnapshot->hasPackPower &&
        decoderGetI32Best(decoder, VOLTRONIC_JK_REG_PACK_POWER_MW_U32, 200000000, &i32)) {
        outSnapshot->hasPackPower = true;
        outSnapshot->packPowerW = (float)i32 / 1000.0f;
    } else if (!outSnapshot->hasPackPower &&
               outSnapshot->hasPackVoltage &&
               outSnapshot->hasPackCurrent) {
        outSnapshot->hasPackPower = true;
        outSnapshot->packPowerW = outSnapshot->packVoltageV * outSnapshot->packCurrentA;
    }

    if (!outSnapshot->hasSoc) {
        uint8_t soc = 0u;
        if ((decoderGetU16(decoder, VOLTRONIC_JK_REG_BALANCE_SOC_U8X2, &u16) &&
             decodePctBytePair(u16, false, &soc)) ||
            (decoderGetU16(decoder, VOLTRONIC_JK_REG_SOH_PRECHARGE_U8X2, &u16) &&
             decodePctBytePair(u16, true, &soc))) {
            outSnapshot->hasSoc = true;
            outSnapshot->socPct = soc;
        }
    }

    if (outSnapshot->tempCount == 0u) {
        if (decoderGetI16(decoder, VOLTRONIC_JK_REG_MOS_TEMP_DECIC, &i16) &&
            normalizeJkDeciC(i16, &i16)) {
            outSnapshot->tempDeciC[outSnapshot->tempCount++] = i16;
        }
        if (decoderGetI16(decoder, VOLTRONIC_JK_REG_TEMP1_DECIC, &i16) &&
            normalizeJkDeciC(i16, &i16) &&
            outSnapshot->tempCount < VOLTRONIC_MB_MAX_TEMPS) {
            outSnapshot->tempDeciC[outSnapshot->tempCount++] = i16;
        }
        if (decoderGetI16(decoder, VOLTRONIC_JK_REG_TEMP2_DECIC, &i16) &&
            normalizeJkDeciC(i16, &i16) &&
            outSnapshot->tempCount < VOLTRONIC_MB_MAX_TEMPS) {
            outSnapshot->tempDeciC[outSnapshot->tempCount++] = i16;
        }
    }

    if (!outSnapshot->hasFullMah &&
        decoderGetU32Best(decoder, VOLTRONIC_JK_REG_FULL_MAH_U32, 0u, 500000000u, &u32)) {
        outSnapshot->hasFullMah = true;
        outSnapshot->fullMah = u32;
    }
    if (!outSnapshot->hasFullMah &&
        decoderGetU16(decoder, VOLTRONIC_JK_REG_RATED_CAPACITY_MAH, &u16) &&
        u16 > 0u) {
        outSnapshot->hasFullMah = true;
        outSnapshot->fullMah = u16;
    }
    if (!outSnapshot->hasRemainMah &&
        decoderGetI32Best(decoder, VOLTRONIC_JK_REG_REMAIN_MAH_I32, 500000000, &i32) &&
        i32 >= 0) {
        outSnapshot->hasRemainMah = true;
        outSnapshot->remainMah = (uint32_t)i32;
    }

    if (!outSnapshot->hasStatusFlags) {
        uint16_t status = 0u;
        if (decoderGetU16(decoder, VOLTRONIC_JK_REG_CHARGE_MOS, &u16) && u16 != 0u) {
            status |= VOLTRONIC_MB_STATUS_CHARGE_ENABLE;
        }
        if (decoderGetU16(decoder, VOLTRONIC_JK_REG_DISCHARGE_MOS, &u16) && u16 != 0u) {
            status |= VOLTRONIC_MB_STATUS_DISCHARGE_ENABLE;
        }
        if (status != 0u) {
            outSnapshot->hasStatusFlags = true;
            outSnapshot->statusFlags = status;
            outSnapshot->chargeEnabled = (status & VOLTRONIC_MB_STATUS_CHARGE_ENABLE) != 0u;
            outSnapshot->dischargeEnabled = (status & VOLTRONIC_MB_STATUS_DISCHARGE_ENABLE) != 0u;
        }
    }

    if (!outSnapshot->hasAlarmRegisters &&
        decoderGetU32(decoder, VOLTRONIC_JK_REG_ALARM_U32, &u32)) {
        outSnapshot->hasAlarmRegisters = true;
        outSnapshot->chargeAlarm = (uint16_t)((u32 >> 16) & 0xFFFFu);
        outSnapshot->dischargeAlarm = (uint16_t)(u32 & 0xFFFFu);
        outSnapshot->chargeProtect = (uint16_t)((u32 >> 16) & 0xFFFFu);
        outSnapshot->dischargeProtect = (uint16_t)(u32 & 0xFFFFu);
        outSnapshot->bmsState = 0u;
    }
}

bool voltronicModbusBuildDecodedSnapshot(const modbusDecoder_t *decoder,
                                         int64_t sourceUs,
                                         voltronic_modbus_snapshot_t *outSnapshot)
{
    uint16_t regVal = 0u;

    if (decoder == NULL || outSnapshot == NULL) {
        return false;
    }

    memset(outSnapshot, 0, sizeof(*outSnapshot));
    outSnapshot->timestampUs = sourceUs;

    if (decoderGetU16(decoder, VOLTRONIC_MB_REG_CELL_COUNT, &regVal)) {
        outSnapshot->cellCount = clampU8(regVal, VOLTRONIC_MB_MAX_CELLS);
    }

    uint16_t minCell = UINT16_MAX;
    uint16_t maxCell = 0u;
    uint8_t minIdx = 0u;
    uint8_t maxIdx = 0u;
    uint8_t validCells = 0u;
    for (uint8_t i = 0u; i < VOLTRONIC_MB_MAX_CELLS; i++) {
        uint16_t mv = 0u;
        uint16_t addr = (uint16_t)(VOLTRONIC_MB_REG_CELL01 + i);
        if (!decoderGetU16(decoder, addr, &regVal) || !normalizeCellMv(regVal, &mv)) {
            continue;
        }

        outSnapshot->cellMv[i] = mv;
        if (outSnapshot->cellCount < (uint8_t)(i + 1u)) {
            outSnapshot->cellCount = (uint8_t)(i + 1u);
        }
        validCells++;

        if (mv < minCell) {
            minCell = mv;
            minIdx = (uint8_t)(i + 1u);
        }
        if (mv > maxCell) {
            maxCell = mv;
            maxIdx = (uint8_t)(i + 1u);
        }
    }
    if (validCells > 0u) {
        outSnapshot->hasCellExtremes = true;
        outSnapshot->minCellMv = minCell;
        outSnapshot->maxCellMv = maxCell;
        outSnapshot->minCellIndex = minIdx;
        outSnapshot->maxCellIndex = maxIdx;
    }

    uint8_t tempLimit = VOLTRONIC_MB_MAX_TEMPS;
    if (decoderGetU16(decoder, VOLTRONIC_MB_REG_TEMP_COUNT, &regVal)) {
        outSnapshot->tempCount = clampU8(regVal, VOLTRONIC_MB_MAX_TEMPS);
        tempLimit = outSnapshot->tempCount;
    }
    for (uint8_t i = 0u; i < tempLimit; i++) {
        int16_t deciC = 0;
        uint16_t addr = (uint16_t)(VOLTRONIC_MB_REG_TEMP01_DECIK + i);
        if (!decoderGetU16(decoder, addr, &regVal) ||
            !normalizeDeciKelvinToDeciC(regVal, &deciC)) {
            continue;
        }
        outSnapshot->tempDeciC[i] = deciC;
        if (outSnapshot->tempCount < (uint8_t)(i + 1u)) {
            outSnapshot->tempCount = (uint8_t)(i + 1u);
        }
    }

    uint16_t chargeDa = 0u;
    uint16_t dischargeDa = 0u;
    bool haveChargeCurrent = decoderGetU16(decoder, VOLTRONIC_MB_REG_CHARGE_CURRENT_DA, &chargeDa);
    bool haveDischargeCurrent = decoderGetU16(decoder, VOLTRONIC_MB_REG_DISCHARGE_CURRENT_DA, &dischargeDa);
    if (haveChargeCurrent || haveDischargeCurrent) {
        outSnapshot->hasPackCurrent = true;
        outSnapshot->packCurrentA = ((float)chargeDa - (float)dischargeDa) / 10.0f;
    }
    if (decoderGetU16(decoder, VOLTRONIC_MB_REG_MODULE_VOLTAGE_DV, &regVal)) {
        const float packVoltageV = (float)regVal / 10.0f;
        if (packVoltageV >= 10.0f && packVoltageV <= 200.0f) {
            outSnapshot->hasPackVoltage = true;
            outSnapshot->packVoltageV = packVoltageV;
        }
    }
    if (decoderGetU16(decoder, VOLTRONIC_MB_REG_SOC_PCT, &regVal) && regVal <= 100u) {
        outSnapshot->hasSoc = true;
        outSnapshot->socPct = (uint8_t)regVal;
    }

    uint32_t u32 = 0u;
    if (decoderGetU32(decoder, VOLTRONIC_MB_REG_TOTAL_CAPACITY_MAH, &u32)) {
        outSnapshot->hasFullMah = true;
        outSnapshot->fullMah = u32;
    }
    if (decoderGetU32(decoder, VOLTRONIC_MB_REG_DESIGN_CAPACITY_MAH, &u32)) {
        outSnapshot->hasDesignMah = true;
        outSnapshot->designMah = u32;
    }

    if (decoderGetU16(decoder, VOLTRONIC_MB_REG_CHARGE_ALARM, &outSnapshot->chargeAlarm) |
        decoderGetU16(decoder, VOLTRONIC_MB_REG_DISCHARGE_ALARM, &outSnapshot->dischargeAlarm) |
        decoderGetU16(decoder, VOLTRONIC_MB_REG_CHARGE_PROTECT, &outSnapshot->chargeProtect) |
        decoderGetU16(decoder, VOLTRONIC_MB_REG_CHARGE_PROTECT2, &outSnapshot->chargeProtect2) |
        decoderGetU16(decoder, VOLTRONIC_MB_REG_DISCHARGE_PROTECT, &outSnapshot->dischargeProtect) |
        decoderGetU16(decoder, VOLTRONIC_MB_REG_DISCHARGE_PROTECT2, &outSnapshot->dischargeProtect2) |
        decoderGetU16(decoder, VOLTRONIC_MB_REG_BMS_STATE, &outSnapshot->bmsState)) {
        outSnapshot->hasAlarmRegisters = true;
    }

    for (uint8_t pair = 0u; pair < 10u; pair++) {
        uint16_t addr = (uint16_t)(VOLTRONIC_MB_REG_CELL_STATE_PAIR01 + pair);
        if (decoderGetU16(decoder, addr, &regVal)) {
            decodePairStates(regVal, outSnapshot->cellState, (uint8_t)(pair * 2u), VOLTRONIC_MB_MAX_CELLS);
            if (outSnapshot->cellStateCount < (uint8_t)((pair + 1u) * 2u)) {
                outSnapshot->cellStateCount = (uint8_t)((pair + 1u) * 2u);
            }
        }
    }
    for (uint8_t pair = 0u; pair < 5u; pair++) {
        uint16_t addr = (uint16_t)(VOLTRONIC_MB_REG_TEMP_STATE_PAIR01 + pair);
        if (decoderGetU16(decoder, addr, &regVal)) {
            decodePairStates(regVal, outSnapshot->tempState, (uint8_t)(pair * 2u), VOLTRONIC_MB_MAX_TEMPS);
            if (outSnapshot->tempStateCount < (uint8_t)((pair + 1u) * 2u)) {
                outSnapshot->tempStateCount = (uint8_t)((pair + 1u) * 2u);
            }
        }
    }
    for (uint8_t i = 0u; i < VOLTRONIC_MB_MODULE_STATE_COUNT; i++) {
        uint16_t addr = (uint16_t)(VOLTRONIC_MB_REG_MODULE_CHG_V_STATE + i);
        if (decoderGetU16(decoder, addr, &regVal)) {
            outSnapshot->moduleState[i] = (uint8_t)(regVal & 0xFFu);
        }
    }

    if (decoderGetU16(decoder, VOLTRONIC_MB_REG_CHARGE_V_LIMIT_DV, &regVal)) {
        outSnapshot->hasChargeLimits = true;
        outSnapshot->chargeVoltageLimitV = (float)regVal / 10.0f;
    }
    if (decoderGetU16(decoder, VOLTRONIC_MB_REG_DISCHARGE_V_LIMIT_DV, &regVal)) {
        outSnapshot->hasChargeLimits = true;
        outSnapshot->dischargeVoltageLimitV = (float)regVal / 10.0f;
    }
    if (decoderGetU16(decoder, VOLTRONIC_MB_REG_CHARGE_I_LIMIT_DA, &regVal)) {
        outSnapshot->hasChargeLimits = true;
        outSnapshot->chargeCurrentLimitA = (float)regVal / 10.0f;
    }
    if (decoderGetU16(decoder, VOLTRONIC_MB_REG_DISCHARGE_I_LIMIT_DA, &regVal)) {
        outSnapshot->hasChargeLimits = true;
        outSnapshot->dischargeCurrentLimitA = (float)regVal / 10.0f;
    }
    if (decoderGetU16(decoder, VOLTRONIC_MB_REG_CHG_DSG_STATUS, &regVal)) {
        outSnapshot->hasStatusFlags = true;
        outSnapshot->statusFlags = regVal;
        outSnapshot->chargeEnabled = (regVal & VOLTRONIC_MB_STATUS_CHARGE_ENABLE) != 0u;
        outSnapshot->dischargeEnabled = (regVal & VOLTRONIC_MB_STATUS_DISCHARGE_ENABLE) != 0u;
        outSnapshot->chargeImmediately = (regVal & VOLTRONIC_MB_STATUS_CHARGE_NOW) != 0u;
        outSnapshot->chargeImmediately2 = (regVal & VOLTRONIC_MB_STATUS_CHARGE_NOW2) != 0u;
        outSnapshot->fullChargeRequested = (regVal & VOLTRONIC_MB_STATUS_FULL_CHARGE_REQ) != 0u;
    }
    if (decoderGetU32(decoder, VOLTRONIC_MB_REG_REMAIN_CAPACITY_MAH, &u32)) {
        outSnapshot->hasRemainMah = true;
        outSnapshot->remainMah = u32;
    }

    decodeJkCompatSnapshot(decoder, outSnapshot);

    if (validCells >= 2u) {
        uint32_t packMvSum = 0u;
        for (uint8_t i = 0u; i < outSnapshot->cellCount && i < VOLTRONIC_MB_MAX_CELLS; i++) {
            packMvSum += outSnapshot->cellMv[i];
        }

        const float sumVoltageV = (float)packMvSum / 1000.0f;
        if (sumVoltageV >= 10.0f &&
            (!outSnapshot->hasPackVoltage ||
             outSnapshot->packVoltageV < (sumVoltageV * 0.5f) ||
             outSnapshot->packVoltageV > (sumVoltageV * 1.5f))) {
            outSnapshot->hasPackVoltage = true;
            outSnapshot->packVoltageV = sumVoltageV;
        }
    }

    if (!outSnapshot->hasSoc && outSnapshot->hasRemainMah &&
        outSnapshot->hasFullMah && outSnapshot->fullMah > 0u) {
        const uint32_t soc = (uint32_t)(((uint64_t)outSnapshot->remainMah * 100u +
                                         (outSnapshot->fullMah / 2u)) /
                                        outSnapshot->fullMah);
        outSnapshot->hasSoc = true;
        outSnapshot->socPct = clampU8((uint16_t)soc, 100u);
    }

    if (outSnapshot->hasPackVoltage && outSnapshot->hasPackCurrent) {
        outSnapshot->hasPackPower = true;
        outSnapshot->packPowerW = outSnapshot->packVoltageV * outSnapshot->packCurrentA;
    }

    outSnapshot->valid = outSnapshot->hasSoc ||
                         outSnapshot->hasPackVoltage ||
                         outSnapshot->hasPackCurrent ||
                         outSnapshot->hasCellExtremes ||
                         outSnapshot->tempCount > 0u ||
                         outSnapshot->hasStatusFlags ||
                         outSnapshot->hasAlarmRegisters;
    return outSnapshot->valid;
}

bool voltronicModbusBuildDecodedPacket(const voltronic_modbus_snapshot_t *snapshot,
                                       uint32_t sequence,
                                       bms_decoded_packet_t *outPacket)
{
    if (snapshot == NULL || outPacket == NULL || !snapshot->valid) {
        return false;
    }

    memset(outPacket, 0, sizeof(*outPacket));
    outPacket->sourceProtocol = PROTOCOL_ID_VOLTRONIC;
    outPacket->sequence = sequence;
    outPacket->timestampUs = snapshot->timestampUs;

    if (snapshot->hasSoc) {
        outPacket->hasSoc = true;
        outPacket->socPct = snapshot->socPct;
    }
    if (snapshot->tempCount > 0u) {
        int32_t tempSum = 0;
        for (uint8_t i = 0u; i < snapshot->tempCount; i++) {
            tempSum += snapshot->tempDeciC[i];
            if (i < BMS_DECODED_PACKET_MAX_TEMPS) {
                outPacket->tempDeciC[i] = snapshot->tempDeciC[i];
                outPacket->tempCount = (uint8_t)(i + 1u);
            }
        }
        outPacket->hasTemperatureC = true;
        outPacket->temperatureC = (int16_t)((tempSum / (int32_t)snapshot->tempCount) / 10);
    }
    if (snapshot->hasPackVoltage) {
        uint32_t cv = (uint32_t)(snapshot->packVoltageV * 100.0f + 0.5f);
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
        uint8_t limit = snapshot->cellCount;
        if (limit > BMS_DECODED_PACKET_MAX_CELLS) {
            limit = BMS_DECODED_PACKET_MAX_CELLS;
        }
        outPacket->cellCount = limit;
        memcpy(outPacket->cellMv, snapshot->cellMv, (size_t)limit * sizeof(outPacket->cellMv[0]));
    }
    if (snapshot->hasStatusFlags) {
        outPacket->hasStatusFlags = true;
        outPacket->statusFlags = snapshot->statusFlags;
    }
    if (snapshot->hasAlarmRegisters) {
        outPacket->hasWarningFlags = true;
        outPacket->warningFlags = (uint16_t)(snapshot->chargeAlarm | snapshot->dischargeAlarm);
        outPacket->hasProtectionFlags = true;
        outPacket->protectionFlags = (uint16_t)(snapshot->chargeProtect | snapshot->dischargeProtect);
    }

    return outPacket->hasSoc ||
           outPacket->hasTemperatureC ||
           outPacket->hasPackVoltageCv ||
           outPacket->hasCellExtremes ||
           outPacket->cellCount > 0u ||
           outPacket->hasStatusFlags ||
           outPacket->hasWarningFlags ||
           outPacket->hasProtectionFlags;
}

static void voltronicModbusBmsTask(void *pv)
{
    voltronicModbusBmsTaskCtx_t *ctx = (voltronicModbusBmsTaskCtx_t *)pv;
    uint8_t rxChunk[RS485_BUF_SIZE];
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    const uint8_t bmsPort = (settings.bms_port == 2u) ? 2u : 1u;
    const uart_port_t rxUart = (bmsPort == 2u) ? rs485GetUart2() : rs485GetUart1();
    const gpio_num_t dirPin = (bmsPort == 2u) ? rs485GetDir2() : rs485GetDir1();
    const char *ifName = (bmsPort == 2u) ? "VOLTRONIC_RS485_2" : "VOLTRONIC_RS485_1";
    int64_t lastRecordedReqUs = 0;

    modbusDecoderInit(&ctx->decoder, ifName, VOLTRONIC_BMS_MODBUS_GAP_US);
    voltronicModbusPollerInit(&ctx->poller,
                              rxUart,
                              dirPin,
                              (uint8_t)VOLTRONIC_BMS_MODBUS_SLAVE_ADDR);
    uart_flush_input(rxUart);

    while (1) {
        int len = uart_read_bytes(rxUart, rxChunk, sizeof(rxChunk), pdMS_TO_TICKS(10));
        int64_t nowUs = esp_timer_get_time();

        if (len > 0) {
            if ((nowUs - g_lastRawRxLogUs) >= 500000LL) {
                char hex[3 * 128 + 8];
                bytesToHex(rxChunk, len, hex, sizeof(hex));
                ESP_LOGI(EXAMPLE_TAG, "Voltronic RX raw len=%d: %s", len, hex);
                g_lastRawRxLogUs = nowUs;
            }
            modbusDecoderFeed(&ctx->decoder, rxChunk, len, nowUs);
        } else if (ctx->decoder.haveLastByte &&
                   ((nowUs - ctx->decoder.lastByteUs) > (int64_t)ctx->decoder.gapUs)) {
            modbusDecoderFlush(&ctx->decoder);
        }

        esp_err_t pollErr = voltronicModbusPollerTick(&ctx->poller,
                                                      nowUs,
                                                      VOLTRONIC_BMS_QUERY_PERIOD_MS);
        if (pollErr != ESP_OK && pollErr != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(EXAMPLE_TAG, "Voltronic Modbus poll TX failed (err=0x%x)", (unsigned)pollErr);
        } else if (ctx->poller.lastReqValid && ctx->poller.lastReqUs != lastRecordedReqUs) {
            modbusDecoderRecordRequest(&ctx->decoder,
                                       ctx->poller.lastReqSlave,
                                       ctx->poller.lastReqFunc,
                                       ctx->poller.lastReqStart,
                                       ctx->poller.lastReqCount,
                                       ctx->poller.lastReqUs);
            lastRecordedReqUs = ctx->poller.lastReqUs;
            if ((nowUs - g_lastTxLogUs) >= 1000000LL) {
                char hex[3 * 8 + 8];
                bytesToHex(ctx->poller.lastReqFrame,
                           ctx->poller.lastReqLen,
                           hex,
                           sizeof(hex));
                ESP_LOGI(EXAMPLE_TAG,
                         "Voltronic TX %s start=0x%04X count=%u slave=%u: %s",
                         (ctx->poller.lastReqFrameOrder == VOLTRONIC_MB_FRAME_FUNCTION_FIRST) ?
                             "fn-first" : "classic",
                         (unsigned)ctx->poller.lastReqStart,
                         (unsigned)ctx->poller.lastReqCount,
                         (unsigned)ctx->poller.lastReqSlave,
                         hex);
                g_lastTxLogUs = nowUs;
            }
        }

        if ((nowUs - ctx->lastPublishUs) >= ((int64_t)VOLTRONIC_BMS_PUBLISH_PERIOD_MS * 1000LL)) {
            int64_t newestCacheUs = 0;
            if (!decoderCacheFresh(&ctx->decoder, nowUs, &newestCacheUs)) {
                batteryModelClear();
                clearLatestData();
                if ((nowUs - g_lastSourceStaleLogUs) >= 1000000LL) {
                    if (newestCacheUs <= 0) {
                        ESP_LOGW(EXAMPLE_TAG,
                                 "Voltronic Modbus source stale: clearing published data (no valid BMS response yet)");
                    } else {
                        const uint32_t ageMs = (uint32_t)((nowUs - newestCacheUs) / 1000LL);
                        ESP_LOGW(EXAMPLE_TAG,
                                 "Voltronic Modbus source stale: clearing published data (last_rx_age=%u ms)",
                                 (unsigned)ageMs);
                    }
                    g_lastSourceStaleLogUs = nowUs;
                }
            } else {
                voltronic_modbus_snapshot_t snapshot = {0};
                if (voltronicModbusBuildDecodedSnapshot(&ctx->decoder, newestCacheUs, &snapshot)) {
                    bms_decoded_packet_t packet = {0};
                    storeLatestSnapshot(&snapshot);
                    publishBatteryModel(&snapshot);
                    logDecodedSnapshot(&snapshot, nowUs);
                    if (voltronicModbusBuildDecodedPacket(&snapshot, ++ctx->sequence, &packet)) {
                        storeLatestPacket(&packet);
                        if (xQueueOverwrite(ctx->outQueue, &packet) != pdPASS) {
                            ESP_LOGW(EXAMPLE_TAG, "Voltronic output queue overwrite failed");
                        }
                    }
                }
            }
            ctx->lastPublishUs = nowUs;
        }
    }
}

esp_err_t voltronicModbusBmsTaskStart(QueueHandle_t outQueue)
{
    if (outQueue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_voltronicModbusBmsTaskHandle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&g_voltronicModbusBmsCtx, 0, sizeof(g_voltronicModbusBmsCtx));
    g_voltronicModbusBmsCtx.outQueue = outQueue;
    g_lastSourceStaleLogUs = 0;
    g_lastDecodeLogUs = 0;
    g_lastRawRxLogUs = 0;
    g_lastTxLogUs = 0;
    batteryModelClear();
    clearLatestData();

    BaseType_t taskOk =
        xTaskCreate(voltronicModbusBmsTask,
                    "voltronic_bms",
                    VOLTRONIC_BMS_TASK_STACK,
                    &g_voltronicModbusBmsCtx,
                    VOLTRONIC_BMS_TASK_PRIORITY,
                    &g_voltronicModbusBmsTaskHandle);
    if (taskOk != pdPASS) {
        g_voltronicModbusBmsTaskHandle = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "Voltronic Modbus BMS task started (slave=%u poll=%dms, publish=%dms)",
             (unsigned)VOLTRONIC_BMS_MODBUS_SLAVE_ADDR,
             VOLTRONIC_BMS_QUERY_PERIOD_MS,
             VOLTRONIC_BMS_PUBLISH_PERIOD_MS);
    return ESP_OK;
}

bool voltronicModbusBmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket)
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

bool voltronicModbusBmsTaskGetLatestSnapshot(voltronic_modbus_snapshot_t *outSnapshot)
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

esp_err_t voltronicModbusBmsTaskStop(void)
{
    if (g_voltronicModbusBmsTaskHandle == NULL) {
        return ESP_OK;
    }

    vTaskDelete(g_voltronicModbusBmsTaskHandle);
    g_voltronicModbusBmsTaskHandle = NULL;
    memset(&g_voltronicModbusBmsCtx, 0, sizeof(g_voltronicModbusBmsCtx));
    g_lastDecodeLogUs = 0;
    g_lastRawRxLogUs = 0;
    g_lastTxLogUs = 0;
    batteryModelClear();
    clearLatestData();

    return ESP_OK;
}
