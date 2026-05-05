#include "jkbms_can_protocol.h"

#include "../../Web_interface/web_bridge_api.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "protocols/common/battery_model.h"

#include "esp_log.h"

static const char *TAG = "SNIFFER_BRIDGE";
static char s_jkbmsCanLogText[2048];

typedef struct {
    bridgeTelemetrySnapshot_t snap;
    universal_battery_model_t model;
    uint16_t dischargeTimeHours;
    uint8_t tempMaxIdx;
    uint8_t tempMinIdx;
    bool haveCell;
    bool haveTemp;
    bool haveAlarm;
    char raw02F4[32];
    char raw04F4[32];
    char raw05F4[32];
    char raw07F4[32];
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
        default:
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

static bool jkbmsCanExtract(const char *ifname,
                            const jkbms_can_frame_t *cache,
                            size_t count,
                            jkbms_can_decoded_t *out)
{
    const jkbms_can_frame_t *fBatt = jkbmsCanFrameById(cache, count, JKBMS_CAN_ID_BATT_ST);
    const jkbms_can_frame_t *fCell = jkbmsCanFrameById(cache, count, JKBMS_CAN_ID_CELL_VOLT);
    const jkbms_can_frame_t *fTemp = jkbmsCanFrameById(cache, count, JKBMS_CAN_ID_CELL_TEMP);
    const jkbms_can_frame_t *fAlm = jkbmsCanFrameById(cache, count, JKBMS_CAN_ID_ALM_INFO);

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

    if (fTemp != NULL && fTemp->dlc >= 5u) {
        float tempMax = jkbmsCanTempToC(fTemp->data[0]);
        float tempMin = jkbmsCanTempToC(fTemp->data[2]);
        float tempAvg = jkbmsCanTempToC(fTemp->data[4]);

        out->haveTemp = true;
        out->tempMaxIdx = fTemp->data[1];
        out->tempMinIdx = fTemp->data[3];
        out->snap.tempMosC = tempAvg;
        out->snap.tempT1C = tempMax;
        out->snap.tempT2C = tempMin;
        out->snap.tempCount = 3u;
        out->model.temperaturesC[0] = tempAvg;
        out->model.temperaturesC[1] = tempMax;
        out->model.temperaturesC[2] = tempMin;
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
             "  temps : max=%.1fC(S%u)  min=%.1fC(S%u)  avg=%.1fC\n"
             "  alarm : raw=0x%08" PRIX32 " protections='%s' alarms='%s' warnings='%s'\n"
             "  raw   : 0x02F4=[%s] 0x04F4=[%s] 0x05F4=[%s] 0x07F4=[%s]",
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
             (double)decoded.snap.tempT1C,
             (unsigned)decoded.tempMaxIdx,
             (double)decoded.snap.tempT2C,
             (unsigned)decoded.tempMinIdx,
             (double)decoded.snap.tempMosC,
             decoded.snap.alarmRaw,
             decoded.snap.protections[0] ? decoded.snap.protections : "None",
             decoded.snap.alarms[0] ? decoded.snap.alarms : "None",
             decoded.snap.warnings[0] ? decoded.snap.warnings : "None",
             decoded.raw02F4[0] ? decoded.raw02F4 : "-",
             decoded.raw04F4[0] ? decoded.raw04F4 : "-",
             decoded.raw05F4[0] ? decoded.raw05F4 : "-",
             decoded.raw07F4[0] ? decoded.raw07F4 : "-");

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
