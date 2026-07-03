#include "jkbms_can_protocol.h"

#include "../../Web_interface/web_bridge_api.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "protocols/common/battery_model.h"

#include "esp_log.h"

static const char *TAG = "SNIFFER_BRIDGE";
static char s_jkbmsCanLogText[2048];

#define JKBMS_CAN_LOG_ALERT_LIMIT 160
#define JKBMS_CAN_EXT_CELL_VALUES_PER_FRAME 4u
#define JKBMS_CAN_EXT_CELL_LIMIT 25u

typedef struct {
    bridgeTelemetrySnapshot_t snap;
    universal_battery_model_t model;
    uint16_t dischargeTimeHours;
    uint8_t tempMaxIdx;
    uint8_t tempMinIdx;
    uint8_t tempExtCount;
    float tempSummaryMaxC;
    float tempSummaryMinC;
    float tempSummaryAvgC;
    bool haveCell;
    bool haveTemp;
    bool haveTempExt;
    bool haveAlarm;
    char raw02F4[32];
    char raw04F4[32];
    char raw05F4[32];
    char raw07F4[32];
    char raw18F228F4[32];
    char raw18E[7][32];
    char extCellText[128];
    char extTempText[96];
} jkbms_can_decoded_t;

typedef struct {
    const char *name;
} jkbms_alarm_desc_t;

static const jkbms_alarm_desc_t kJkbmsCanAlarmFields[] = {
    {"Cell overvoltage"},
    {"Cell undervoltage"},
    {"Pack overvoltage"},
    {"Pack undervoltage"},
    {"Cell voltage delta high"},
    {"Discharge overcurrent"},
    {"Charge overcurrent"},
    {"Temperature high"},
    {"Temperature low"},
    {"Temperature delta high"},
    {"SOC low"},
    {"Insulation low"},
    {"High-voltage interlock fault"},
    {"External communication failure"},
    {"Internal communication failure"},
};

int jkbmsCanCacheIndex(uint32_t id)
{
    switch (id) {
        case JKBMS_CAN_ID_BATT_ST:
            return 0;
        case JKBMS_CAN_ID_CELL_VOLT:
            return 1;
        case JKBMS_CAN_ID_CELL_TEMP:
            return 2;
        case JKBMS_CAN_ID_ALM_INFO:
            return 3;
        case JKBMS_CAN_ID_CELL_TEMP_EXT:
            return 11;
        default:
            if (id >= JKBMS_CAN_ID_CELL_VOLT_EXT_BASE && id <= JKBMS_CAN_ID_CELL_VOLT_EXT_LAST &&
                ((id - JKBMS_CAN_ID_CELL_VOLT_EXT_BASE) % 0x00010000u) == 0u) {
                return 4 + (int)((id - JKBMS_CAN_ID_CELL_VOLT_EXT_BASE) / 0x00010000u);
            }
            return -1;
    }
}

static const jkbms_can_frame_t *jkbmsCanFrameById(const jkbms_can_frame_t *cache,
                                                  size_t count,
                                                  uint32_t id)
{
    int idx = jkbmsCanCacheIndex(id);
    if (cache == NULL || idx < 0 || (size_t)idx >= count) {
        return NULL;
    }
    return cache[idx].valid ? &cache[idx] : NULL;
}

static inline uint16_t jkbmsCanLe16(const uint8_t *p)
{
    return (uint16_t)((((uint16_t)p[1]) << 8) | (uint16_t)p[0]);
}

static float jkbmsCanCurrentToA(uint16_t raw)
{
    return ((float)raw / 10.0f) - 400.0f;
}

static float jkbmsCanTempToC(uint8_t raw)
{
    return ((float)raw) - 50.0f;
}

static uint8_t jkbmsCanDecodeExtendedTemps(const jkbms_can_frame_t *frame,
                                           float temps[UNIVERSAL_BATTERY_TEMP_SENSORS])
{
    uint8_t mask = 0u;
    uint8_t src = 1u;
    uint8_t count = 0u;

    if (temps != NULL) {
        for (uint8_t i = 0u; i < UNIVERSAL_BATTERY_TEMP_SENSORS; i++) {
            temps[i] = 0.0f;
        }
    }

    if (frame == NULL || temps == NULL || frame->dlc < 2u) {
        return 0u;
    }

    mask = frame->data[0];
    if ((mask & 0x01u) != 0u) {
        while (count < UNIVERSAL_BATTERY_TEMP_SENSORS &&
               (mask & (uint8_t)(1u << count)) != 0u &&
               src < frame->dlc) {
            temps[count] = jkbmsCanTempToC(frame->data[src++]);
            count++;
        }
        return count;
    }

    if (frame->dlc >= 6u) {
        for (uint8_t i = 0u; i < UNIVERSAL_BATTERY_TEMP_SENSORS; i++) {
            temps[i] = jkbmsCanTempToC(frame->data[i + 1u]);
        }
        return UNIVERSAL_BATTERY_TEMP_SENSORS;
    }

    return 0u;
}

static uint32_t jkbmsCanCellVoltExtId(uint8_t frameIdx)
{
    return JKBMS_CAN_ID_CELL_VOLT_EXT_BASE + ((uint32_t)frameIdx * 0x00010000u);
}

static void jkbmsFormatCanData(const uint8_t *data, uint8_t dlc, char *out, size_t outSize)
{
    size_t pos = 0u;
    uint8_t n = dlc;

    if (out == NULL || outSize == 0u) {
        return;
    }

    out[0] = '\0';
    if (data == NULL) {
        return;
    }

    if (n > 8u) {
        n = 8u;
    }
    for (uint8_t i = 0u; i < n && pos + 4u < outSize; i++) {
        pos += (size_t)snprintf(&out[pos], outSize - pos, "%02X ", data[i]);
    }
    if (pos > 0u) {
        out[pos - 1u] = '\0';
    }
}

static void appendText(char *out, size_t outSize, const char *text)
{
    size_t len = 0u;

    if (out == NULL || outSize == 0u || text == NULL || text[0] == '\0') {
        return;
    }

    len = strlen(out);
    if (len >= outSize - 1u) {
        return;
    }
    if (len > 0u) {
        int n = snprintf(&out[len], outSize - len, ", ");
        if (n < 0) {
            return;
        }
        len += (size_t)n;
        if (len >= outSize - 1u) {
            return;
        }
    }
    (void)snprintf(&out[len], outSize - len, "%s", text);
}

static void appendAlarmByLevel(bridgeTelemetrySnapshot_t *snap, uint8_t level, const char *name)
{
    char text[80] = {0};

    if (snap == NULL || level == 0u || name == NULL) {
        return;
    }

    (void)snprintf(text, sizeof(text), "%s (L%u)", name, (unsigned)level);
    switch (level) {
        case 1u:
            appendText(snap->protections, sizeof(snap->protections), text);
            break;
        case 2u:
            appendText(snap->alarms, sizeof(snap->alarms), text);
            break;
        case 3u:
        default:
            appendText(snap->warnings, sizeof(snap->warnings), text);
            break;
    }
}

static void jkbmsFormatExtendedTemps(const float temps[UNIVERSAL_BATTERY_TEMP_SENSORS],
                                     uint8_t count,
                                     char *out,
                                     size_t outSize)
{
    size_t pos = 0u;

    if (out == NULL || outSize == 0u) {
        return;
    }
    out[0] = '\0';
    if (temps == NULL || count == 0u) {
        return;
    }

    for (uint8_t i = 0u; i < count && i < UNIVERSAL_BATTERY_TEMP_SENSORS; i++) {
        int n = snprintf(&out[pos],
                         outSize - pos,
                         "%sT%u=%.1fC",
                         (pos > 0u) ? " " : "",
                         (unsigned)(i + 1u),
                         (double)temps[i]);
        if (n < 0) {
            return;
        }
        pos += (size_t)n;
        if (pos >= outSize - 1u) {
            out[outSize - 1u] = '\0';
            return;
        }
    }
}

static void jkbmsRecalculateCellsFromGrid(bridgeTelemetrySnapshot_t *snap)
{
    float sum = 0.0f;
    float minV = 0.0f;
    float maxV = 0.0f;
    uint8_t minIdx = 0u;
    uint8_t maxIdx = 0u;

    if (snap == NULL || snap->cellCount == 0u) {
        return;
    }

    minV = snap->cellVoltagesV[0];
    maxV = snap->cellVoltagesV[0];
    minIdx = 1u;
    maxIdx = 1u;

    for (uint8_t i = 0u; i < snap->cellCount; i++) {
        float v = snap->cellVoltagesV[i];
        sum += v;
        if (v < minV) {
            minV = v;
            minIdx = (uint8_t)(i + 1u);
        }
        if (v > maxV) {
            maxV = v;
            maxIdx = (uint8_t)(i + 1u);
        }
    }

    snap->cellAvgV = sum / (float)snap->cellCount;
    snap->cellDiffV = maxV - minV;
    snap->cellMaxV = maxV;
    snap->cellMinV = minV;
    snap->cellMaxIdx = maxIdx;
    snap->cellMinIdx = minIdx;
    snap->deltaV = snap->cellDiffV;
}

static uint8_t jkbmsDecodeExtendedCellVoltages(const jkbms_can_frame_t *cache,
                                               size_t count,
                                               jkbms_can_decoded_t *out)
{
    uint8_t cellCount = 0u;

    if (out == NULL) {
        return 0u;
    }

    for (uint8_t frameIdx = 0u; frameIdx < JKBMS_CAN_CELL_VOLT_EXT_FRAMES; frameIdx++) {
        uint32_t id = jkbmsCanCellVoltExtId(frameIdx);
        const jkbms_can_frame_t *frame = jkbmsCanFrameById(cache, count, id);

        if (frame != NULL) {
            jkbmsFormatCanData(frame->data, frame->dlc, out->raw18E[frameIdx], sizeof(out->raw18E[frameIdx]));
        }
        if (frame == NULL || frame->dlc < 2u) {
            continue;
        }

        for (uint8_t i = 0u; i < JKBMS_CAN_EXT_CELL_VALUES_PER_FRAME; i++) {
            uint8_t off = (uint8_t)(i * 2u);
            uint8_t cellIdx = (uint8_t)(frameIdx * JKBMS_CAN_EXT_CELL_VALUES_PER_FRAME + i);
            uint16_t mv = 0u;

            if ((uint8_t)(off + 1u) >= frame->dlc || cellIdx >= JKBMS_CAN_EXT_CELL_LIMIT) {
                break;
            }

            mv = jkbmsCanLe16(&frame->data[off]);
            if (mv == 0u || mv > 7000u) {
                continue;
            }

            out->snap.cellVoltagesV[cellIdx] = (float)mv / 1000.0f;
            if ((uint8_t)(cellIdx + 1u) > cellCount) {
                cellCount = (uint8_t)(cellIdx + 1u);
            }
        }
    }

    while (cellCount > 0u && out->snap.cellVoltagesV[cellCount - 1u] <= 0.0f) {
        cellCount--;
    }
    for (uint8_t i = 0u; i < cellCount; i++) {
        if (out->snap.cellVoltagesV[i] <= 0.0f) {
            cellCount = i;
            break;
        }
    }

    if (cellCount > 0u) {
        out->snap.cellCount = cellCount;
        jkbmsRecalculateCellsFromGrid(&out->snap);
        out->model.cellMaxV = out->snap.cellMaxV;
        out->model.cellMinV = out->snap.cellMinV;
        out->model.cellMaxIdx = out->snap.cellMaxIdx;
        out->model.cellMinIdx = out->snap.cellMinIdx;
        out->model.cellDeltaV = out->snap.cellDiffV;
        (void)snprintf(out->extCellText,
                       sizeof(out->extCellText),
                       "count=%u first=%.3fV last=%.3fV",
                       (unsigned)cellCount,
                       (double)out->snap.cellVoltagesV[0],
                       (double)out->snap.cellVoltagesV[cellCount - 1u]);
    }

    return cellCount;
}

static bool jkbmsCanExtract(const char *ifname,
                            const jkbms_can_frame_t *cache,
                            size_t count,
                            jkbms_can_decoded_t *out)
{
    const jkbms_can_frame_t *fBatt = jkbmsCanFrameById(cache, count, JKBMS_CAN_ID_BATT_ST);
    const jkbms_can_frame_t *fCell = jkbmsCanFrameById(cache, count, JKBMS_CAN_ID_CELL_VOLT);
    const jkbms_can_frame_t *fTemp = jkbmsCanFrameById(cache, count, JKBMS_CAN_ID_CELL_TEMP);
    const jkbms_can_frame_t *fAlm = jkbmsCanFrameById(cache, count, JKBMS_CAN_ID_ALM_INFO);
    const jkbms_can_frame_t *fTempExt = jkbmsCanFrameById(cache, count, JKBMS_CAN_ID_CELL_TEMP_EXT);

    if (out == NULL) {
        return false;
    }
    memset(out, 0, sizeof(*out));

    if (fBatt != NULL) {
        jkbmsFormatCanData(fBatt->data, fBatt->dlc, out->raw02F4, sizeof(out->raw02F4));
    }
    if (fCell != NULL) {
        jkbmsFormatCanData(fCell->data, fCell->dlc, out->raw04F4, sizeof(out->raw04F4));
    }
    if (fTemp != NULL) {
        jkbmsFormatCanData(fTemp->data, fTemp->dlc, out->raw05F4, sizeof(out->raw05F4));
    }
    if (fAlm != NULL) {
        jkbmsFormatCanData(fAlm->data, fAlm->dlc, out->raw07F4, sizeof(out->raw07F4));
    }
    if (fTempExt != NULL) {
        jkbmsFormatCanData(fTempExt->data, fTempExt->dlc, out->raw18F228F4, sizeof(out->raw18F228F4));
    }

    if (fBatt != NULL && fBatt->dlc >= 5u) {
        uint16_t voltageDv = jkbmsCanLe16(&fBatt->data[0]);
        uint16_t currentRaw = jkbmsCanLe16(&fBatt->data[2]);
        uint8_t soc = fBatt->data[4];

        out->snap.valid = true;
        snprintf(out->snap.source, sizeof(out->snap.source), "%s", (ifname != NULL) ? ifname : "CAN1");
        snprintf(out->snap.protocol, sizeof(out->snap.protocol), "JKBMS_CAN_250K");
        out->snap.packVoltageV = (float)voltageDv / 10.0f;
        out->snap.currentA = jkbmsCanCurrentToA(currentRaw);
        out->snap.packPowerW = out->snap.packVoltageV * out->snap.currentA;
        out->snap.packPowerValid = true;
        out->snap.socPct = (soc <= 100u) ? soc : 0u;
        out->snap.sohPct = 100u;

        if (fBatt->dlc >= 8u) {
            out->dischargeTimeHours = jkbmsCanLe16(&fBatt->data[6]);
        }

        out->model.valid = true;
        out->model.packVoltageV = out->snap.packVoltageV;
        out->model.packCurrentA = out->snap.currentA;
        out->model.socPct = out->snap.socPct;
        out->model.sohPct = out->snap.sohPct;
    }

    if (fCell != NULL && fCell->dlc >= 6u) {
        uint16_t maxMv = jkbmsCanLe16(&fCell->data[0]);
        uint16_t minMv = jkbmsCanLe16(&fCell->data[3]);
        uint8_t maxIdx = fCell->data[2];
        uint8_t minIdx = fCell->data[5];

        out->haveCell = true;
        out->snap.cellMaxV = (float)maxMv / 1000.0f;
        out->snap.cellMinV = (float)minMv / 1000.0f;
        out->snap.cellMaxIdx = maxIdx;
        out->snap.cellMinIdx = minIdx;
        if (maxMv >= minMv) {
            out->snap.deltaV = (float)(maxMv - minMv) / 1000.0f;
            out->snap.cellDiffV = out->snap.deltaV;
        }
        out->model.cellMaxV = out->snap.cellMaxV;
        out->model.cellMinV = out->snap.cellMinV;
        out->model.cellMaxIdx = maxIdx;
        out->model.cellMinIdx = minIdx;
        out->model.cellDeltaV = out->snap.deltaV;
    }

    (void)jkbmsDecodeExtendedCellVoltages(cache, count, out);

    if (fTemp != NULL && fTemp->dlc >= 5u) {
        float tempMax = jkbmsCanTempToC(fTemp->data[0]);
        float tempMin = jkbmsCanTempToC(fTemp->data[2]);
        float tempAvg = jkbmsCanTempToC(fTemp->data[4]);

        out->haveTemp = true;
        out->tempMaxIdx = fTemp->data[1];
        out->tempMinIdx = fTemp->data[3];
        out->tempSummaryMaxC = tempMax;
        out->tempSummaryMinC = tempMin;
        out->tempSummaryAvgC = tempAvg;
        out->snap.tempMosC = tempAvg;
        out->snap.tempT1C = tempMax;
        out->snap.tempT2C = tempMin;
        out->snap.tempCount = 3u;
        out->model.temperaturesC[0] = tempAvg;
        out->model.temperaturesC[1] = tempMax;
        out->model.temperaturesC[2] = tempMin;
    }

    if (fTempExt != NULL && fTempExt->dlc >= 2u) {
        float extTemps[UNIVERSAL_BATTERY_TEMP_SENSORS] = {0};
        uint8_t extCount = jkbmsCanDecodeExtendedTemps(fTempExt, extTemps);

        if (extCount > 0u) {
            out->haveTemp = true;
            out->haveTempExt = true;
            out->tempExtCount = extCount;
            out->snap.tempCount = extCount;
            out->snap.tempMosC = extTemps[0];
            out->snap.tempT1C = (extCount > 1u) ? extTemps[1] : 0.0f;
            out->snap.tempT2C = (extCount > 2u) ? extTemps[2] : 0.0f;
            out->snap.tempT4C = (extCount > 3u) ? extTemps[3] : 0.0f;
            out->snap.tempT5C = (extCount > 4u) ? extTemps[4] : 0.0f;
            for (uint8_t i = 0u; i < UNIVERSAL_BATTERY_TEMP_SENSORS; i++) {
                out->model.temperaturesC[i] = (i < extCount) ? extTemps[i] : 0.0f;
            }
            jkbmsFormatExtendedTemps(extTemps, extCount, out->extTempText, sizeof(out->extTempText));
        }
    }

    if (fAlm != NULL && fAlm->dlc >= 4u) {
        uint32_t raw = (uint32_t)fAlm->data[0] |
                       ((uint32_t)fAlm->data[1] << 8) |
                       ((uint32_t)fAlm->data[2] << 16) |
                       ((uint32_t)fAlm->data[3] << 24);

        out->haveAlarm = true;
        out->snap.alarmRaw = raw;
        out->model.alarmsMask = raw;
        out->model.warningsMask = raw;
        out->model.protocolState = raw;

        for (size_t i = 0u; i < (sizeof(kJkbmsCanAlarmFields) / sizeof(kJkbmsCanAlarmFields[0])); i++) {
            uint8_t level = (uint8_t)((raw >> (i * 2u)) & 0x03u);
            appendAlarmByLevel(&out->snap, level, kJkbmsCanAlarmFields[i].name);
        }
    }

    return out->snap.valid;
}

bool jkbmsCanAnyValid(const jkbms_can_frame_t *cache, size_t count)
{
    if (cache == NULL) {
        return false;
    }

    for (size_t i = 0u; i < count; i++) {
        if (cache[i].valid) {
            return true;
        }
    }
    return false;
}

bool jkbmsCanTryGetSocPct(const jkbms_can_frame_t *cache, size_t count, uint8_t *socOut)
{
    const jkbms_can_frame_t *fBatt = jkbmsCanFrameById(cache, count, JKBMS_CAN_ID_BATT_ST);

    if (socOut == NULL || fBatt == NULL || fBatt->dlc < 5u) {
        return false;
    }

    if (fBatt->data[4] > 100u) {
        return false;
    }
    *socOut = fBatt->data[4];
    return true;
}

void jkbmsCanUpdateBatteryModel(const char *ifname, const jkbms_can_frame_t *cache, size_t count)
{
    jkbms_can_decoded_t decoded = {0};

    if (jkbmsCanExtract(ifname, cache, count, &decoded)) {
        batteryModelSet(&decoded.model);
    }
}

void jkbmsCanDecodeSnapshot(const char *ifname, const jkbms_can_frame_t *cache, size_t count)
{
    jkbms_can_decoded_t decoded = {0};
    bool valid = jkbmsCanExtract(ifname, cache, count, &decoded);

    if (valid) {
        bridgeSetTelemetrySnapshot(&decoded.snap);
        batteryModelSet(&decoded.model);
    }

    snprintf(s_jkbmsCanLogText,
             sizeof(s_jkbmsCanLogText),
             "JK BMS CAN 250K V2.0\n"
             "  valid : %s\n"
             "  pack  : V=%.2fV  I=%.1fA  SOC=%u%%  SOH=%u%%  discharge_time=%uh\n"
             "  cells : max=%.3fV#%u  min=%.3fV#%u  dV=%.3fV\n"
             "  cellv : %s\n"
             "  temps : summary max=%.1fC(S%u)  min=%.1fC(S%u)  avg=%.1fC  ext=%s\n"
             "  alarm : raw=0x%08" PRIX32 " protections='%.*s' alarms='%.*s' warnings='%.*s'\n"
             "  raw   : 0x02F4=[%s] 0x04F4=[%s] 0x05F4=[%s] 0x07F4=[%s] 0x18F228F4=[%s]\n"
             "          0x18E0=[%s] 0x18E1=[%s] 0x18E2=[%s] 0x18E3=[%s] 0x18E4=[%s] 0x18E5=[%s] 0x18E6=[%s]",
             valid ? "YES" : "NO",
             (double)decoded.snap.packVoltageV,
             (double)decoded.snap.currentA,
             (unsigned)decoded.snap.socPct,
             (unsigned)decoded.snap.sohPct,
             (unsigned)decoded.dischargeTimeHours,
             (double)decoded.snap.cellMaxV,
             (unsigned)decoded.snap.cellMaxIdx,
             (double)decoded.snap.cellMinV,
             (unsigned)decoded.snap.cellMinIdx,
             (double)decoded.snap.deltaV,
             decoded.extCellText[0] ? decoded.extCellText : "-",
             (double)decoded.tempSummaryMaxC,
             (unsigned)decoded.tempMaxIdx,
             (double)decoded.tempSummaryMinC,
             (unsigned)decoded.tempMinIdx,
             (double)decoded.tempSummaryAvgC,
             decoded.haveTempExt ? decoded.extTempText : "-",
             decoded.snap.alarmRaw,
             JKBMS_CAN_LOG_ALERT_LIMIT,
             decoded.snap.protections[0] ? decoded.snap.protections : "None",
             JKBMS_CAN_LOG_ALERT_LIMIT,
             decoded.snap.alarms[0] ? decoded.snap.alarms : "None",
             JKBMS_CAN_LOG_ALERT_LIMIT,
             decoded.snap.warnings[0] ? decoded.snap.warnings : "None",
             decoded.raw02F4[0] ? decoded.raw02F4 : "-",
             decoded.raw04F4[0] ? decoded.raw04F4 : "-",
             decoded.raw05F4[0] ? decoded.raw05F4 : "-",
             decoded.raw07F4[0] ? decoded.raw07F4 : "-",
             decoded.raw18F228F4[0] ? decoded.raw18F228F4 : "-",
             decoded.raw18E[0][0] ? decoded.raw18E[0] : "-",
             decoded.raw18E[1][0] ? decoded.raw18E[1] : "-",
             decoded.raw18E[2][0] ? decoded.raw18E[2] : "-",
             decoded.raw18E[3][0] ? decoded.raw18E[3] : "-",
             decoded.raw18E[4][0] ? decoded.raw18E[4] : "-",
             decoded.raw18E[5][0] ? decoded.raw18E[5] : "-",
             decoded.raw18E[6][0] ? decoded.raw18E[6] : "-");

    bridgeSetDecodedLogSnapshot(s_jkbmsCanLogText);

    ESP_LOGI(TAG, "CAN-%s JK BMS CAN 250K SNAPSHOT", (ifname != NULL) ? ifname : "CAN1");
    ESP_LOGI(TAG,
             "  valid=%s pack=%.2fV %.1fA SOC=%u%% cells max=%.3f#%u min=%.3f#%u temps avg/max/min=%.1f/%.1f/%.1f",
             valid ? "YES" : "NO",
             (double)decoded.snap.packVoltageV,
             (double)decoded.snap.currentA,
             (unsigned)decoded.snap.socPct,
             (double)decoded.snap.cellMaxV,
             (unsigned)decoded.snap.cellMaxIdx,
             (double)decoded.snap.cellMinV,
             (unsigned)decoded.snap.cellMinIdx,
             (double)decoded.snap.tempMosC,
             (double)decoded.snap.tempT1C,
             (double)decoded.snap.tempT2C);
    if (decoded.haveAlarm) {
        ESP_LOGI(TAG,
                 "  alarm raw=0x%08" PRIX32 " protections='%s' alarms='%s' warnings='%s'",
                 decoded.snap.alarmRaw,
                 decoded.snap.protections[0] ? decoded.snap.protections : "None",
                 decoded.snap.alarms[0] ? decoded.snap.alarms : "None",
                 decoded.snap.warnings[0] ? decoded.snap.warnings : "None");
    }
}
