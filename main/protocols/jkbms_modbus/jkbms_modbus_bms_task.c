#include "protocols/jkbms_modbus/jkbms_modbus_bms_task.h"

#include <limits.h>
#include <string.h>

#include "Drivers/rs485_driver.h"
#include "config.h"
#include "decoders/modbusDecoder.h"
#include "orchestrator/protocol_types.h"
#include "protocols/common/battery_model.h"
#include "protocols/jkbms_modbus/jkbms_modbus_poller.h"
#include "protocols/jkbms_modbus/jkbms_modbus_registers_map.h"
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
static int64_t g_lastCellDebugLogUs;
static int64_t g_lastDecodeSourceLogUs;
static int64_t g_lastModelPublishDebugLogUs;
static float g_lastPublishedPackVoltageV;
static struct {
    bool valid;
    uint8_t cellCount;
    uint16_t cellMv[JKBMS_MAX_CELLS];
    uint16_t minCellMv;
    uint16_t maxCellMv;
    uint8_t minCellIndex;
    uint8_t maxCellIndex;
    uint16_t cellAvgMv;
    uint16_t cellDiffMaxMv;
} g_lastGoodCellMap;

static void logJkbmsModelPublishDebug(const jkbms_modbus_snapshot_t *snapshot,
                                      int64_t nowUs,
                                      const char *voltageSource,
                                      float publishedPackVoltageV,
                                      bool forceLog)
{
    if (snapshot == NULL) {
        return;
    }
    if (!forceLog && (nowUs - g_lastModelPublishDebugLogUs) < 1000000LL) {
        return;
    }
    g_lastModelPublishDebugLogUs = nowUs;

    if (forceLog) {
        ESP_LOGW(EXAMPLE_TAG,
                 "JKBMS model publish jump: source=%s resultV=%.2fV hasPack=%s packMv=%lu hasAvg=%s avgMv=%u count=%u hasExt=%s min=%u max=%u soc=%s/%u curr=%s/%ld",
                 (voltageSource != NULL) ? voltageSource : "none",
                 publishedPackVoltageV,
                 snapshot->hasPackVoltageMv ? "YES" : "NO",
                 (unsigned long)snapshot->packVoltageMv,
                 snapshot->hasCellAvgMv ? "YES" : "NO",
                 (unsigned)snapshot->cellAvgMv,
                 (unsigned)snapshot->cellCount,
                 snapshot->hasCellExtremes ? "YES" : "NO",
                 (unsigned)snapshot->minCellMv,
                 (unsigned)snapshot->maxCellMv,
                 snapshot->hasSoc ? "YES" : "NO",
                 (unsigned)snapshot->socPct,
                 snapshot->hasPackCurrentMa ? "YES" : "NO",
                 (long)snapshot->packCurrentMa);
    } else {
        ESP_LOGI(EXAMPLE_TAG,
                 "JKBMS model publish: source=%s resultV=%.2fV hasPack=%s packMv=%lu hasAvg=%s avgMv=%u count=%u hasExt=%s min=%u max=%u soc=%s/%u curr=%s/%ld",
                 (voltageSource != NULL) ? voltageSource : "none",
                 publishedPackVoltageV,
                 snapshot->hasPackVoltageMv ? "YES" : "NO",
                 (unsigned long)snapshot->packVoltageMv,
                 snapshot->hasCellAvgMv ? "YES" : "NO",
                 (unsigned)snapshot->cellAvgMv,
                 (unsigned)snapshot->cellCount,
                 snapshot->hasCellExtremes ? "YES" : "NO",
                 (unsigned)snapshot->minCellMv,
                 (unsigned)snapshot->maxCellMv,
                 snapshot->hasSoc ? "YES" : "NO",
                 (unsigned)snapshot->socPct,
                 snapshot->hasPackCurrentMa ? "YES" : "NO",
                 (long)snapshot->packCurrentMa);
    }
}

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

static bool jkbmsCellAvgLooksConsistent(const jkbms_modbus_snapshot_t *snapshot)
{
    const uint16_t toleranceMv = 50u;

    if (snapshot == NULL || !snapshot->hasCellAvgMv) {
        return false;
    }
    if (!snapshot->hasCellExtremes) {
        return true;
    }
    if (snapshot->minCellMv == 0u || snapshot->maxCellMv == 0u) {
        return true;
    }
    if (snapshot->cellAvgMv + toleranceMv < snapshot->minCellMv) {
        return false;
    }
    if (snapshot->cellAvgMv > snapshot->maxCellMv + toleranceMv) {
        return false;
    }
    return true;
}

static void jkbmsPublishBatteryModel(const jkbms_modbus_snapshot_t *snapshot, int64_t nowUs)
{
    battery_model_t model = {0};
    const char *voltageSource = "none";
    bool forceLog = false;
    const float lastPublishedVoltageV = g_lastPublishedPackVoltageV;
    const bool avgLooksConsistent = jkbmsCellAvgLooksConsistent(snapshot);

    if (snapshot == NULL || !snapshot->valid) {
        return;
    }

    model.valid = true;
    model.updatedMs = (uint32_t)(nowUs / 1000LL);

    if (snapshot->hasPackVoltageMv) {
        model.packVoltageV = (float)snapshot->packVoltageMv / 1000.0f;
        voltageSource = "pack_mv";
    } else if (snapshot->hasCellAvgMv && snapshot->cellCount > 0u && avgLooksConsistent) {
        model.packVoltageV = ((float)snapshot->cellAvgMv * (float)snapshot->cellCount) / 1000.0f;
        voltageSource = "cell_avg_x_count";
    } else if (snapshot->hasCellExtremes && snapshot->cellCount > 0u) {
        const float avgCellV =
            ((float)snapshot->maxCellMv + (float)snapshot->minCellMv) / 2000.0f;
        model.packVoltageV = avgCellV * (float)snapshot->cellCount;
        voltageSource = avgLooksConsistent ? "cell_extremes_x_count" : "cell_extremes_x_count_reject_avg";
    }
    if (snapshot->hasPackCurrentMa) {
        model.packCurrentA = (float)snapshot->packCurrentMa / 1000.0f;
    }
    if (snapshot->hasSoc) {
        model.socPct = snapshot->socPct;
    }
    if (snapshot->hasSoh) {
        model.sohPct = snapshot->sohPct;
    }
    if (snapshot->hasCycles) {
        model.cycleCount = (snapshot->cycles > UINT16_MAX) ? UINT16_MAX : (uint16_t)snapshot->cycles;
    }
    if (snapshot->hasCellExtremes) {
        model.cellMaxV = (float)snapshot->maxCellMv / 1000.0f;
        model.cellMinV = (float)snapshot->minCellMv / 1000.0f;
        model.cellMaxIdx = snapshot->maxCellIndex;
        model.cellMinIdx = snapshot->minCellIndex;
    }
    if (snapshot->hasCellDiffMaxMv) {
        model.cellDeltaV = (float)snapshot->cellDiffMaxMv / 1000.0f;
    } else if (snapshot->hasCellExtremes && snapshot->maxCellMv >= snapshot->minCellMv) {
        model.cellDeltaV = (float)(snapshot->maxCellMv - snapshot->minCellMv) / 1000.0f;
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
    if (snapshot->hasBalanceCurrentMa) {
        model.balanceEnabled = (snapshot->balanceCurrentMa != 0);
    }
    if (snapshot->hasAlarmBits) {
        model.alarmsMask = snapshot->alarmBits & 0x0000FFFFu;
        model.warningsMask = snapshot->alarmBits >> 16;
        model.protocolState = snapshot->alarmBits;
    }

    batteryModelSet(&model);
    if (lastPublishedVoltageV > 0.0f) {
        float deltaV = model.packVoltageV - lastPublishedVoltageV;
        if (deltaV < 0.0f) {
            deltaV = -deltaV;
        }
        if (deltaV >= 5.0f) {
            forceLog = true;
        }
    }
    if (strcmp(voltageSource, "pack_mv") == 0) {
        forceLog = true;
    }
    if (!avgLooksConsistent && snapshot->hasCellAvgMv && snapshot->hasCellExtremes) {
        forceLog = true;
    }
    g_lastPublishedPackVoltageV = model.packVoltageV;
    logJkbmsModelPublishDebug(snapshot, nowUs, voltageSource, model.packVoltageV, forceLog);
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

static uint16_t absDiffU16(uint16_t a, uint16_t b)
{
    return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static uint32_t absDiffU32(uint32_t a, uint32_t b)
{
    return (a >= b) ? (uint32_t)(a - b) : (uint32_t)(b - a);
}

typedef enum {
    CELL_VALUE_RAW = 0,
    CELL_VALUE_SWAP = 1,
    CELL_VALUE_RAW_DIV10 = 2,
    CELL_VALUE_SWAP_DIV10 = 3,
} cell_value_mode_t;

typedef struct {
    bool hasExpectedCount;
    uint8_t expectedCount;
    bool hasCellAvg;
    uint16_t cellAvgMv;
    bool hasCellDiff;
    uint16_t cellDiffMv;
    bool hasPackMv;
    uint32_t packMv;
    bool hasMinIdx;
    uint8_t minIdx;
    bool hasMaxIdx;
    uint8_t maxIdx;
} cell_expectation_t;

typedef struct {
    uint8_t stride;
    uint8_t offset;
    cell_value_mode_t mode;
    uint8_t windowCount;
    uint8_t validInWindow;
    uint8_t headCount;
    uint8_t highestValidIdx;
    uint8_t gapCount;
    int32_t score;
    bool cellValid[JKBMS_MAX_CELLS];
    uint16_t cellMv[JKBMS_MAX_CELLS];
} cell_decode_candidate_t;

/*
 * Current JK setup used with this bridge is 16S. When pack-derived hint is
 * missing/noisy, keep a stable default to avoid sparse maps drifting to 32 cells.
 */
#define JKBMS_DEFAULT_CELL_COUNT_HINT 16u

static bool decodeCellRawWithMode(uint16_t raw, cell_value_mode_t mode, uint16_t *mvOut)
{
    uint16_t candidate = 0u;
    const uint16_t rawSwap = bswap16(raw);

    switch (mode) {
        case CELL_VALUE_RAW:
            candidate = raw;
            break;
        case CELL_VALUE_SWAP:
            candidate = rawSwap;
            break;
        case CELL_VALUE_RAW_DIV10:
            candidate = (uint16_t)(raw / 10u);
            break;
        case CELL_VALUE_SWAP_DIV10:
            candidate = (uint16_t)(rawSwap / 10u);
            break;
        default:
            return false;
    }

    /*
     * Keep only realistic Li-ion/LFP per-cell voltages.
     * Wider ranges produced false positives (e.g. 5.9V phantom cells).
     */
    if (candidate < 2000u || candidate > 5000u) {
        return false;
    }

    if (mvOut != NULL) {
        *mvOut = candidate;
    }
    return true;
}

static bool normalizeCellMv(uint16_t raw, uint16_t *mvOut)
{
    uint16_t best = 0u;
    bool have = false;

    for (int mode = CELL_VALUE_RAW; mode <= CELL_VALUE_SWAP_DIV10; mode++) {
        uint16_t mv = 0u;
        if (!decodeCellRawWithMode(raw, (cell_value_mode_t)mode, &mv)) {
            continue;
        }
        if (!have || absDiffU16(mv, 4000u) < absDiffU16(best, 4000u)) {
            best = mv;
            have = true;
        }
    }

    if (!have) {
        return false;
    }

    if (mvOut != NULL) {
        *mvOut = best;
    }
    return true;
}

static void formatCellVoltageSlice(const jkbms_modbus_snapshot_t *snapshot,
                                   uint8_t firstCell,
                                   uint8_t lastCell,
                                   char *out,
                                   size_t outSize)
{
    size_t pos = 0u;

    if (snapshot == NULL || out == NULL || outSize == 0u || firstCell < 1u || lastCell < firstCell) {
        return;
    }

    out[0] = '\0';
    for (uint8_t c = firstCell; c <= lastCell; c++) {
        const uint8_t idx = (uint8_t)(c - 1u);
        const bool hasCell = (idx < snapshot->cellCount &&
                              snapshot->cellMv[idx] >= 2000u &&
                              snapshot->cellMv[idx] <= 5000u);
        int w = 0;

        if (hasCell) {
            w = snprintf(out + pos,
                         outSize - pos,
                         "%sC%02u=%.3fV",
                         (pos == 0u) ? "" : " ",
                         (unsigned)c,
                         (double)((float)snapshot->cellMv[idx] / 1000.0f));
        } else {
            w = snprintf(out + pos,
                         outSize - pos,
                         "%sC%02u=-",
                         (pos == 0u) ? "" : " ",
                         (unsigned)c);
        }

        if (w <= 0 || (size_t)w >= (outSize - pos)) {
            out[outSize - 1u] = '\0';
            return;
        }
        pos += (size_t)w;
    }
}

static void formatCellRawSlice(const modbusDecoder_t *decoder,
                               uint8_t firstCell,
                               uint8_t lastCell,
                               char *out,
                               size_t outSize)
{
    size_t pos = 0u;

    if (decoder == NULL || out == NULL || outSize == 0u || firstCell < 1u || lastCell < firstCell) {
        return;
    }

    out[0] = '\0';
    for (uint8_t c = firstCell; c <= lastCell; c++) {
        const uint16_t addr = JKBMS_RT_REG_CELL_N_MV((uint16_t)(c - 1u));
        uint16_t raw = 0u;
        const bool ok = decoderGetU16(decoder, addr, &raw);
        int w = snprintf(out + pos,
                         outSize - pos,
                         "%sR%02u=%s",
                         (pos == 0u) ? "" : " ",
                         (unsigned)c,
                         ok ? "0x" : "--");
        if (w <= 0 || (size_t)w >= (outSize - pos)) {
            out[outSize - 1u] = '\0';
            return;
        }
        pos += (size_t)w;

        if (ok) {
            w = snprintf(out + pos, outSize - pos, "%04X", (unsigned)raw);
            if (w <= 0 || (size_t)w >= (outSize - pos)) {
                out[outSize - 1u] = '\0';
                return;
            }
            pos += (size_t)w;
        }
    }
}

static void logCellDebug(const modbusDecoder_t *decoder,
                         const jkbms_modbus_snapshot_t *snapshot,
                         int64_t nowUs)
{
    /*
     * Keep these buffers static so periodic debug formatting does not consume
     * several kilobytes of task stack inside the JKBMS polling task.
     */
    static char decA[320];
    static char decB[320];
    static char decC[320];
    static char decD[320];
    static char rawA[320];
    static char rawB[320];
    static char rawC[320];
    static char rawD[320];

    if (decoder == NULL || snapshot == NULL) {
        return;
    }

    if ((nowUs - g_lastCellDebugLogUs) < 5000000LL) {
        return;
    }
    g_lastCellDebugLogUs = nowUs;

    formatCellVoltageSlice(snapshot, 1u, 8u, decA, sizeof(decA));
    formatCellVoltageSlice(snapshot, 9u, 16u, decB, sizeof(decB));
    formatCellVoltageSlice(snapshot, 17u, 24u, decC, sizeof(decC));
    formatCellVoltageSlice(snapshot, 25u, 32u, decD, sizeof(decD));
    formatCellRawSlice(decoder, 1u, 8u, rawA, sizeof(rawA));
    formatCellRawSlice(decoder, 9u, 16u, rawB, sizeof(rawB));
    formatCellRawSlice(decoder, 17u, 24u, rawC, sizeof(rawC));
    formatCellRawSlice(decoder, 25u, 32u, rawD, sizeof(rawD));

    ESP_LOGI(EXAMPLE_TAG,
             "JKBMS decoded cells: valid=%s count=%u min=%.3fV(#%u) max=%.3fV(#%u)",
             snapshot->valid ? "YES" : "NO",
             (unsigned)snapshot->cellCount,
             snapshot->hasCellExtremes ? (double)((float)snapshot->minCellMv / 1000.0f) : 0.0,
             snapshot->hasCellExtremes ? (unsigned)snapshot->minCellIndex : 0u,
             snapshot->hasCellExtremes ? (double)((float)snapshot->maxCellMv / 1000.0f) : 0.0,
             snapshot->hasCellExtremes ? (unsigned)snapshot->maxCellIndex : 0u);
    ESP_LOGI(EXAMPLE_TAG, "JKBMS cells 01-08: %s", decA);
    ESP_LOGI(EXAMPLE_TAG, "JKBMS cells 09-16: %s", decB);
    ESP_LOGI(EXAMPLE_TAG, "JKBMS cells 17-24: %s", decC);
    ESP_LOGI(EXAMPLE_TAG, "JKBMS cells 25-32: %s", decD);
    ESP_LOGI(EXAMPLE_TAG, "JKBMS raw   01-08: %s", rawA);
    ESP_LOGI(EXAMPLE_TAG, "JKBMS raw   09-16: %s", rawB);
    ESP_LOGI(EXAMPLE_TAG, "JKBMS raw   17-24: %s", rawC);
    ESP_LOGI(EXAMPLE_TAG, "JKBMS raw   25-32: %s", rawD);
}

static void evaluateCellDecodeCandidate(const modbusDecoder_t *decoder,
                                        uint8_t stride,
                                        uint8_t offset,
                                        cell_value_mode_t mode,
                                        const cell_expectation_t *exp,
                                        cell_decode_candidate_t *out)
{
    uint32_t sumMv = 0u;
    uint16_t minMv = UINT16_MAX;
    uint16_t maxMv = 0u;

    if (out == NULL) {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->stride = stride;
    out->offset = offset;
    out->mode = mode;
    out->windowCount = (exp != NULL && exp->hasExpectedCount && exp->expectedCount > 0u)
                           ? exp->expectedCount
                           : JKBMS_MAX_CELLS;

    for (uint8_t i = 0u; i < JKBMS_MAX_CELLS; i++) {
        const uint16_t addr = (uint16_t)(JKBMS_RT_REG_CELL0_MV + (uint16_t)offset + ((uint16_t)i * (uint16_t)stride));
        uint16_t raw = 0u;
        uint16_t mv = 0u;
        if (!decoderGetU16(decoder, addr, &raw)) {
            continue;
        }
        if (!decodeCellRawWithMode(raw, mode, &mv)) {
            continue;
        }
        out->cellValid[i] = true;
        out->cellMv[i] = mv;
        out->highestValidIdx = (uint8_t)(i + 1u);
    }

    for (uint8_t i = 0u; i < JKBMS_MAX_CELLS; i++) {
        if (out->cellValid[i]) {
            out->headCount = (uint8_t)(i + 1u);
        } else {
            break;
        }
    }

    for (uint8_t i = 0u; i < out->windowCount; i++) {
        if (out->cellValid[i]) {
            const uint16_t mv = out->cellMv[i];
            out->validInWindow++;
            sumMv += mv;
            if (mv < minMv) {
                minMv = mv;
            }
            if (mv > maxMv) {
                maxMv = mv;
            }
        } else {
            out->gapCount++;
        }
    }

    if (out->validInWindow == 0u) {
        out->score = INT_MIN / 2;
        return;
    }

    int32_t score = ((int32_t)out->validInWindow * 200) +
                    ((int32_t)out->headCount * 20) -
                    ((int32_t)out->gapCount * 35);

    if (exp != NULL && exp->hasExpectedCount) {
        const int deltaCount = (int)out->validInWindow - (int)exp->expectedCount;
        score -= (int32_t)((deltaCount < 0 ? -deltaCount : deltaCount) * 90);
    }

    if (exp != NULL && exp->hasCellAvg) {
        const uint16_t avgMv = (uint16_t)(sumMv / (uint32_t)out->validInWindow);
        score -= (int32_t)(absDiffU16(avgMv, exp->cellAvgMv) / 2u);
    }

    if (exp != NULL && exp->hasPackMv && exp->hasExpectedCount &&
        out->validInWindow >= ((exp->expectedCount + 1u) / 2u)) {
        const uint16_t avgMv = (uint16_t)(sumMv / (uint32_t)out->validInWindow);
        const uint32_t estPackMv = (uint32_t)avgMv * (uint32_t)exp->expectedCount;
        score -= (int32_t)(absDiffU32(estPackMv, exp->packMv) / 20u);
    }

    if (exp != NULL && exp->hasCellDiff && out->validInWindow >= 2u && minMv <= maxMv) {
        const uint16_t spreadMv = (uint16_t)(maxMv - minMv);
        score -= (int32_t)(absDiffU16(spreadMv, exp->cellDiffMv) * 2u);
    }

    if (exp != NULL && exp->hasMaxIdx && exp->maxIdx >= 1u && exp->maxIdx <= out->windowCount &&
        out->cellValid[exp->maxIdx - 1u] && out->cellMv[exp->maxIdx - 1u] >= maxMv) {
        score += 40;
    }

    if (exp != NULL && exp->hasMinIdx && exp->minIdx >= 1u && exp->minIdx <= out->windowCount &&
        out->cellValid[exp->minIdx - 1u] && out->cellMv[exp->minIdx - 1u] <= minMv) {
        score += 40;
    }

    if (out->headCount < 4u) {
        score -= 400;
    }

    /* Slight preference for protocol-documented map start (offset 0). */
    if (offset != 0u) {
        score -= 10;
    }

    out->score = score;
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

static void logJkbmsDecodeSourceDebug(const modbusDecoder_t *decoder,
                                      const jkbms_modbus_snapshot_t *snapshot,
                                      const cell_expectation_t *exp,
                                      int64_t nowUs,
                                      bool forceLog)
{
    uint16_t rawCellAvg = 0u;
    uint16_t rawPackHi = 0u;
    uint16_t rawPackLo = 0u;
    const bool haveRawCellAvg =
        (decoder != NULL) && decoderGetU16(decoder, JKBMS_RT_REG_CELL_AVG_MV, &rawCellAvg);
    const bool haveRawPackHi =
        (decoder != NULL) && decoderGetU16(decoder, JKBMS_RT_REG_PACK_VOLT_MV_U32, &rawPackHi);
    const bool haveRawPackLo =
        (decoder != NULL) && decoderGetU16(decoder, (uint16_t)(JKBMS_RT_REG_PACK_VOLT_MV_U32 + 1u), &rawPackLo);

    if (snapshot == NULL) {
        return;
    }
    if (!forceLog && (nowUs - g_lastDecodeSourceLogUs) < 1000000LL) {
        return;
    }
    g_lastDecodeSourceLogUs = nowUs;

    if (forceLog) {
        ESP_LOGW(EXAMPLE_TAG,
                 "JKBMS decode source jump: rawAvg=%s/0x%04X decAvg=%s/%u rawPack=%s/0x%04X:0x%04X decPack=%s/%lu expPack=%s/%lu expCount=%s/%u cells=%u min=%u max=%u",
                 haveRawCellAvg ? "YES" : "NO",
                 (unsigned)rawCellAvg,
                 snapshot->hasCellAvgMv ? "YES" : "NO",
                 (unsigned)snapshot->cellAvgMv,
                 (haveRawPackHi && haveRawPackLo) ? "YES" : "NO",
                 (unsigned)rawPackHi,
                 (unsigned)rawPackLo,
                 snapshot->hasPackVoltageMv ? "YES" : "NO",
                 (unsigned long)snapshot->packVoltageMv,
                 (exp != NULL && exp->hasPackMv) ? "YES" : "NO",
                 (unsigned long)((exp != NULL) ? exp->packMv : 0u),
                 (exp != NULL && exp->hasExpectedCount) ? "YES" : "NO",
                 (unsigned)((exp != NULL) ? exp->expectedCount : 0u),
                 (unsigned)snapshot->cellCount,
                 (unsigned)snapshot->minCellMv,
                 (unsigned)snapshot->maxCellMv);
    } else {
        ESP_LOGI(EXAMPLE_TAG,
                 "JKBMS decode source: rawAvg=%s/0x%04X decAvg=%s/%u rawPack=%s/0x%04X:0x%04X decPack=%s/%lu expPack=%s/%lu expCount=%s/%u cells=%u min=%u max=%u",
                 haveRawCellAvg ? "YES" : "NO",
                 (unsigned)rawCellAvg,
                 snapshot->hasCellAvgMv ? "YES" : "NO",
                 (unsigned)snapshot->cellAvgMv,
                 (haveRawPackHi && haveRawPackLo) ? "YES" : "NO",
                 (unsigned)rawPackHi,
                 (unsigned)rawPackLo,
                 snapshot->hasPackVoltageMv ? "YES" : "NO",
                 (unsigned long)snapshot->packVoltageMv,
                 (exp != NULL && exp->hasPackMv) ? "YES" : "NO",
                 (unsigned long)((exp != NULL) ? exp->packMv : 0u),
                 (exp != NULL && exp->hasExpectedCount) ? "YES" : "NO",
                 (unsigned)((exp != NULL) ? exp->expectedCount : 0u),
                 (unsigned)snapshot->cellCount,
                 (unsigned)snapshot->minCellMv,
                 (unsigned)snapshot->maxCellMv);
    }
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
    /* JK runtime map stores cells at fixed stride from CELL0. */
    static const uint8_t k_strides[] = {1u, (uint8_t)JKBMS_RT_CELL_STEP};
    static const uint8_t k_offsets[] = {0u, 1u, 0x10u, 0x20u, 0x30u, 0x40u};

    if (decoder == NULL || out == NULL) {
        return false;
    }

    memset(out, 0, sizeof(*out));

    cell_expectation_t exp = {0};
    cell_decode_candidate_t bestCandidate = {0};
    bestCandidate.score = INT_MIN;

    uint16_t u16 = 0u;
    if (decoderGetU16(decoder, JKBMS_RT_REG_CELL_AVG_MV, &u16) && normalizeCellMv(u16, &u16)) {
        out->hasCellAvgMv = true;
        out->cellAvgMv = u16;
        exp.hasCellAvg = true;
        exp.cellAvgMv = u16;
    }
    if (decoderGetU16(decoder, JKBMS_RT_REG_CELL_VDIFF_MAX_MV, &u16)) {
        if (u16 > 5000u) {
            u16 = (uint16_t)(u16 / 10u);
        }
        out->hasCellDiffMaxMv = true;
        out->cellDiffMaxMv = u16;
        exp.hasCellDiff = true;
        exp.cellDiffMv = u16;
    }
    if (decoderGetU16(decoder, JKBMS_RT_REG_MAX_MIN_CELL_NBR_U8X2, &u16)) {
        const uint8_t idxA = (uint8_t)((u16 >> 8) & 0xFFu);
        const uint8_t idxB = (uint8_t)(u16 & 0xFFu);
        if (idxA >= 1u && idxA <= JKBMS_MAX_CELLS) {
            exp.hasMaxIdx = true;
            exp.maxIdx = idxA;
        }
        if (idxB >= 1u && idxB <= JKBMS_MAX_CELLS) {
            exp.hasMinIdx = true;
            exp.minIdx = idxB;
        }
    }

    uint32_t u32 = 0u;
    if (decodeU32Best(decoder, JKBMS_RT_REG_PACK_VOLT_MV_U32, 1000u, 200000u, &u32)) {
        out->hasPackVoltageMv = true;
        out->packVoltageMv = u32;
        exp.hasPackMv = true;
        exp.packMv = u32;
    }

    if (exp.hasPackMv && exp.hasCellAvg && exp.cellAvgMv > 0u) {
        const uint32_t estimatedCells = (exp.packMv + ((uint32_t)exp.cellAvgMv / 2u)) / (uint32_t)exp.cellAvgMv;
        if (estimatedCells >= 1u && estimatedCells <= JKBMS_MAX_CELLS) {
            exp.hasExpectedCount = true;
            exp.expectedCount = (uint8_t)estimatedCells;
        }
    }
    /*
     * Do not use max/min index word as cell-count hint. On some JK firmwares
     * this field is noisy while polling and creates bogus counts (27, 29, ...).
     */
    if (!exp.hasExpectedCount) {
        exp.hasExpectedCount = true;
        exp.expectedCount = JKBMS_DEFAULT_CELL_COUNT_HINT;
    }

    for (size_t si = 0u; si < (sizeof(k_strides) / sizeof(k_strides[0])); si++) {
        for (size_t oi = 0u; oi < (sizeof(k_offsets) / sizeof(k_offsets[0])); oi++) {
            for (int mode = CELL_VALUE_RAW; mode <= CELL_VALUE_SWAP_DIV10; mode++) {
                cell_decode_candidate_t candidate = {0};
                evaluateCellDecodeCandidate(decoder,
                                            k_strides[si],
                                            k_offsets[oi],
                                            (cell_value_mode_t)mode,
                                            &exp,
                                            &candidate);
                if (candidate.score > bestCandidate.score) {
                    bestCandidate = candidate;
                }
            }
        }
    }

    uint8_t targetCellCount = 0u;
    const uint8_t hintCount =
        (exp.hasExpectedCount && exp.expectedCount > 0u) ? exp.expectedCount : 0u;
    const uint8_t highestSeen = bestCandidate.highestValidIdx;
    if (hintCount > 0u) {
        /* Prefer physical cell-count hint (packV/cellAvg or index hints). */
        targetCellCount = hintCount;
    } else {
        targetCellCount = highestSeen;
    }
    if (targetCellCount > JKBMS_MAX_CELLS) {
        targetCellCount = JKBMS_MAX_CELLS;
    }
    out->cellCount = targetCellCount;
    memset(out->cellMv, 0, sizeof(out->cellMv));

    uint8_t validCells = 0u;
    uint16_t minMv = UINT16_MAX;
    uint16_t maxMv = 0u;
    uint8_t minIdx = 0u;
    uint8_t maxIdx = 0u;

    for (uint8_t i = 0u; i < out->cellCount; i++) {
        if (!bestCandidate.cellValid[i]) {
            continue;
        }
        const uint16_t mv = bestCandidate.cellMv[i];
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

    if (validCells >= 2u && minIdx != 0u && maxIdx != 0u) {
        out->hasCellExtremes = true;
        out->minCellMv = minMv;
        out->maxCellMv = maxMv;
        out->minCellIndex = minIdx;
        out->maxCellIndex = maxIdx;
    }

    if (out->hasCellExtremes && exp.hasMaxIdx && exp.hasMinIdx &&
        exp.maxIdx >= 1u && exp.maxIdx <= out->cellCount &&
        exp.minIdx >= 1u && exp.minIdx <= out->cellCount &&
        bestCandidate.cellValid[exp.maxIdx - 1u] &&
        bestCandidate.cellValid[exp.minIdx - 1u]) {
        const uint16_t vA = out->cellMv[exp.maxIdx - 1u];
        const uint16_t vB = out->cellMv[exp.minIdx - 1u];
        if (vA >= vB) {
            out->maxCellIndex = exp.maxIdx;
            out->minCellIndex = exp.minIdx;
            out->maxCellMv = vA;
            out->minCellMv = vB;
        } else {
            out->maxCellIndex = exp.minIdx;
            out->minCellIndex = exp.maxIdx;
            out->maxCellMv = vB;
            out->minCellMv = vA;
        }
    }

    uint8_t minStableCells = 4u;
    if (hintCount >= 8u) {
        /* Require at least 75% of expected cells before replacing stable map. */
        minStableCells = (uint8_t)((hintCount * 3u + 3u) / 4u);
    }

    if (validCells >= minStableCells) {
        g_lastGoodCellMap.valid = true;
        g_lastGoodCellMap.cellCount = out->cellCount;
        memcpy(g_lastGoodCellMap.cellMv, out->cellMv, sizeof(out->cellMv));
        g_lastGoodCellMap.minCellMv = out->minCellMv;
        g_lastGoodCellMap.maxCellMv = out->maxCellMv;
        g_lastGoodCellMap.minCellIndex = out->minCellIndex;
        g_lastGoodCellMap.maxCellIndex = out->maxCellIndex;
        g_lastGoodCellMap.cellAvgMv = out->cellAvgMv;
        g_lastGoodCellMap.cellDiffMaxMv = out->cellDiffMaxMv;
    } else if (g_lastGoodCellMap.valid && g_lastGoodCellMap.cellCount >= 4u) {
        uint8_t fallbackCount = g_lastGoodCellMap.cellCount;
        if (hintCount > 0u && fallbackCount > hintCount) {
            fallbackCount = hintCount;
        }
        out->cellCount = fallbackCount;
        memcpy(out->cellMv, g_lastGoodCellMap.cellMv, sizeof(out->cellMv));
        {
            uint16_t fMin = UINT16_MAX;
            uint16_t fMax = 0u;
            uint8_t fMinIdx = 0u;
            uint8_t fMaxIdx = 0u;
            for (uint8_t i = 0u; i < out->cellCount; i++) {
                const uint16_t mv = out->cellMv[i];
                if (mv < 500u || mv > 6000u) {
                    continue;
                }
                if (mv < fMin) {
                    fMin = mv;
                    fMinIdx = (uint8_t)(i + 1u);
                }
                if (mv > fMax) {
                    fMax = mv;
                    fMaxIdx = (uint8_t)(i + 1u);
                }
            }
            if (fMinIdx != 0u && fMaxIdx != 0u) {
                out->hasCellExtremes = true;
                out->minCellMv = fMin;
                out->maxCellMv = fMax;
                out->minCellIndex = fMinIdx;
                out->maxCellIndex = fMaxIdx;
            } else {
                out->hasCellExtremes = false;
            }
        }
        if (!out->hasCellAvgMv) {
            out->hasCellAvgMv = true;
            out->cellAvgMv = g_lastGoodCellMap.cellAvgMv;
        }
        if (!out->hasCellDiffMaxMv) {
            out->hasCellDiffMaxMv = true;
            out->cellDiffMaxMv = g_lastGoodCellMap.cellDiffMaxMv;
        }
    } else if (validCells < 2u) {
        out->cellCount = 0u;
        memset(out->cellMv, 0, sizeof(out->cellMv));
        out->hasCellExtremes = false;
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
                 out->hasCellAvgMv ||
                 out->hasCellDiffMaxMv ||
                 out->hasAlarmBits ||
                 (out->hasCellExtremes && out->cellCount >= 4u);

    {
        bool forceDecodeLog = false;
        if (out->hasPackVoltageMv && out->hasCellAvgMv && out->cellCount > 0u) {
            uint32_t derivedPackMv = (uint32_t)out->cellAvgMv * (uint32_t)out->cellCount;
            uint32_t diffMv = (out->packVoltageMv >= derivedPackMv)
                                  ? (out->packVoltageMv - derivedPackMv)
                                  : (derivedPackMv - out->packVoltageMv);
            if (diffMv >= 5000u) {
                forceDecodeLog = true;
            }
        }
        logJkbmsDecodeSourceDebug(decoder, out, &exp, esp_timer_get_time(), forceDecodeLog);
    }

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
                jkbmsPublishBatteryModel(&snapshot, nowUs);
                logCellDebug(&ctx->decoder, &snapshot, nowUs);

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
    g_lastCellDebugLogUs = 0;
    g_lastDecodeSourceLogUs = 0;
    g_lastModelPublishDebugLogUs = 0;
    g_lastPublishedPackVoltageV = 0.0f;
    memset(&g_lastGoodCellMap, 0, sizeof(g_lastGoodCellMap));
    batteryModelClear();

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
    g_lastCellDebugLogUs = 0;
    memset(&g_lastGoodCellMap, 0, sizeof(g_lastGoodCellMap));
    batteryModelClear();

    portENTER_CRITICAL(&g_latestPacketMux);
    g_haveLatestPacket = false;
    memset(&g_latestPacket, 0, sizeof(g_latestPacket));
    g_haveLatestSnapshot = false;
    memset(&g_latestSnapshot, 0, sizeof(g_latestSnapshot));
    portEXIT_CRITICAL(&g_latestPacketMux);

    return ESP_OK;
}
