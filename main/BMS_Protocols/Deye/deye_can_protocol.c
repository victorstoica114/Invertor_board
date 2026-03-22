#include "deye_can_protocol.h"

#include "../../bridge.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"

static const char *TAG = "SNIFFER_BRIDGE";
static char s_deyeCanLogText[2048];

static int deyeCanCacheIndex(uint32_t id)
{
    if (id < PYLON_CAN_ID_MIN || id > PYLON_CAN_ID_MAX) {
        return -1;
    }
    return (int)(id - PYLON_CAN_ID_MIN);
}

static const pylon_can_frame_t *deyeCanFrameById(const pylon_can_frame_t *cache, size_t count, uint32_t id)
{
    int idx = deyeCanCacheIndex(id);
    if (cache == NULL || idx < 0 || (size_t)idx >= count) {
        return NULL;
    }
    return cache[idx].valid ? &cache[idx] : NULL;
}

static inline uint16_t deyeCanLe16(const uint8_t *p)
{
    return (uint16_t)((((uint16_t)p[1]) << 8) | (uint16_t)p[0]);
}

static inline int16_t deyeCanLe16s(const uint8_t *p)
{
    return (int16_t)deyeCanLe16(p);
}

static void deyeFormatCanData(const uint8_t *data, uint8_t dlc, char *out, size_t outSize)
{
    size_t pos = 0;
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
    for (uint8_t i = 0; i < n && pos + 4u < outSize; i++) {
        pos += (size_t)snprintf(&out[pos], outSize - pos, "%02X ", data[i]);
    }
    if (pos > 0u) {
        out[pos - 1u] = '\0';
    }
}

static void deyeFormatCanAscii(const uint8_t *data, uint8_t dlc, char *out, size_t outSize)
{
    size_t n = 0u;

    if (out == NULL || outSize == 0u) {
        return;
    }

    out[0] = '\0';
    if (data == NULL) {
        return;
    }

    n = dlc;
    if (n >= outSize) {
        n = outSize - 1u;
    }
    for (size_t i = 0; i < n; i++) {
        uint8_t c = data[i];
        out[i] = (c >= 32u && c <= 126u) ? (char)c : '.';
    }
    out[n] = '\0';
}

static float deyeTempWordToC(uint16_t raw)
{
    if (raw <= 200u) {
        return (float)raw;
    }
    return (float)raw / 10.0f;
}

void deyeCanDecodeSnapshot(const char *ifname, const pylon_can_frame_t *cache, size_t count)
{
    const pylon_can_frame_t *f351 = deyeCanFrameById(cache, count, 0x351u);
    const pylon_can_frame_t *f355 = deyeCanFrameById(cache, count, 0x355u);
    const pylon_can_frame_t *f356 = deyeCanFrameById(cache, count, 0x356u);
    const pylon_can_frame_t *f359 = deyeCanFrameById(cache, count, 0x359u);
    const pylon_can_frame_t *f35C = deyeCanFrameById(cache, count, 0x35Cu);
    const pylon_can_frame_t *f35E = deyeCanFrameById(cache, count, 0x35Eu);
    const pylon_can_frame_t *f370 = deyeCanFrameById(cache, count, 0x370u);
    const pylon_can_frame_t *f371 = deyeCanFrameById(cache, count, 0x371u);
    bridgeTelemetrySnapshot_t snap = {0};
    universal_battery_model_t model = {0};
    char raw359[32] = {0};
    char raw35C[32] = {0};
    char raw370[32] = {0};
    char raw371[32] = {0};
    char ascii35E[16] = {0};
    char tag359[8] = {0};
    float chargeVoltLimit = 0.0f;
    float chargeCurrentLimit = 0.0f;
    float dischargeCurrentLimit = 0.0f;
    float lowerDischargeVoltTentative = 0.0f;
    float packVolt = 0.0f;
    float packCurrent = 0.0f;
    float avgTemp = 0.0f;
    float tempMax = 0.0f;
    float tempMin = 0.0f;
    float cellMax = 0.0f;
    float cellMin = 0.0f;
    uint16_t soc = 0;
    uint16_t soh = 0;
    uint8_t moduleCount = 0u;
    uint8_t status35C = 0u;
    uint16_t tempMaxSensor = 0u;
    uint16_t tempMinSensor = 0u;
    uint16_t cellMaxIdx = 0u;
    uint16_t cellMinIdx = 0u;
    bool chargeEnabled = false;
    bool dischargeEnabled = false;
    bool balanceEnabled = false;

    if (f351 && f351->dlc >= 8u) {
        chargeVoltLimit = (float)deyeCanLe16(&f351->data[0]) / 10.0f;
        chargeCurrentLimit = (float)deyeCanLe16(&f351->data[2]) / 10.0f;
        dischargeCurrentLimit = (float)deyeCanLe16(&f351->data[4]) / 10.0f;
        lowerDischargeVoltTentative = (float)deyeCanLe16(&f351->data[6]) / 10.0f;
    }
    if (f355 && f355->dlc >= 4u) {
        soc = deyeCanLe16(&f355->data[0]);
        soh = deyeCanLe16(&f355->data[2]);
    }
    if (f356 && f356->dlc >= 6u) {
        packVolt = (float)deyeCanLe16(&f356->data[0]) / 100.0f;
        packCurrent = (float)deyeCanLe16s(&f356->data[2]) / 10.0f;
        avgTemp = (float)deyeCanLe16(&f356->data[4]) / 10.0f;
    }
    if (f359) {
        formatCanData(f359->data, f359->dlc, raw359, sizeof(raw359));
        if (f359->dlc >= 5u) {
            moduleCount = f359->data[4];
        }
        if (f359->dlc >= 7u) {
            deyeFormatCanAscii(&f359->data[5], 2u, tag359, sizeof(tag359));
        }
    }
    if (f35C) {
        formatCanData(f35C->data, f35C->dlc, raw35C, sizeof(raw35C));
        if (f35C->dlc >= 1u) {
            status35C = f35C->data[0];
            chargeEnabled = (status35C & 0x80u) != 0u;
            dischargeEnabled = (status35C & 0x40u) != 0u;
            balanceEnabled = (status35C & 0x20u) != 0u;
        }
    }
    if (f35E) {
        deyeFormatCanAscii(f35E->data, f35E->dlc, ascii35E, sizeof(ascii35E));
    }
    if (f370 && f370->dlc >= 8u) {
        formatCanData(f370->data, f370->dlc, raw370, sizeof(raw370));
        tempMax = deyeTempWordToC(deyeCanLe16(&f370->data[0]));
        tempMin = deyeTempWordToC(deyeCanLe16(&f370->data[2]));
        cellMax = (float)deyeCanLe16(&f370->data[4]) / 1000.0f;
        cellMin = (float)deyeCanLe16(&f370->data[6]) / 1000.0f;
    }
    if (f371 && f371->dlc >= 8u) {
        formatCanData(f371->data, f371->dlc, raw371, sizeof(raw371));
        tempMaxSensor = deyeCanLe16(&f371->data[0]);
        tempMinSensor = deyeCanLe16(&f371->data[2]);
        cellMaxIdx = deyeCanLe16(&f371->data[4]);
        cellMinIdx = deyeCanLe16(&f371->data[6]);
    }

    snap.valid = (f351 != NULL) && (f355 != NULL) && (f356 != NULL);
    snprintf(snap.source, sizeof(snap.source), "%s", (ifname != NULL) ? ifname : "CAN1");
    snprintf(snap.protocol, sizeof(snap.protocol), "CAN_DEYE");
    snap.currentA = packCurrent;
    snap.socPct = (soc <= 100u) ? (uint8_t)soc : 0u;
    snap.sohPct = (soh <= 100u) ? (uint8_t)soh : 0u;
    snap.cellMaxV = cellMax;
    snap.cellMinV = cellMin;
    snap.cellMaxIdx = (cellMaxIdx <= 255u) ? (uint8_t)cellMaxIdx : 0u;
    snap.cellMinIdx = (cellMinIdx <= 255u) ? (uint8_t)cellMinIdx : 0u;
    snap.deltaV = (cellMax > 0.0f && cellMin > 0.0f) ? (cellMax - cellMin) : 0.0f;
    snap.tempMosC = avgTemp;
    snap.tempT1C = tempMax;
    snap.tempT2C = tempMin;
    snap.pylonStatus63 = status35C;
    bridgeSetTelemetrySnapshot(&snap);

    model.valid = snap.valid;
    model.packVoltageV = packVolt;
    model.packCurrentA = packCurrent;
    model.socPct = snap.socPct;
    model.sohPct = snap.sohPct;
    model.chargeVoltageLimitV = chargeVoltLimit;
    model.chargeCurrentLimitA = chargeCurrentLimit;
    model.dischargeCurrentLimitA = dischargeCurrentLimit;
    model.cellMaxV = cellMax;
    model.cellMinV = cellMin;
    model.cellMaxIdx = (cellMaxIdx <= 255u) ? (uint8_t)cellMaxIdx : 0u;
    model.cellMinIdx = (cellMinIdx <= 255u) ? (uint8_t)cellMinIdx : 0u;
    model.cellDeltaV = snap.deltaV;
    model.temperaturesC[0] = avgTemp;
    model.temperaturesC[1] = tempMax;
    model.temperaturesC[2] = tempMin;
    model.chargeEnabled = chargeEnabled;
    model.dischargeEnabled = dischargeEnabled;
    model.balanceEnabled = balanceEnabled;
    model.protocolState = status35C;
    bridgeSetUniversalBatteryModel(&model);

    snprintf(s_deyeCanLogText,
             sizeof(s_deyeCanLogText),
             "CAN Deye\n"
             "  valid : %s\n"
             "  name  : %s\n"
             "  pack  : V=%.2fV  I=%.1fA  avgT=%.1fC  SOC=%u%%  SOH=%u%%\n"
             "  limits: chgV=%.1fV  chgI=%.1fA  disI=%.1fA  lowV?=%.1fV\n"
             "  cells : max=%.3fV#%u  min=%.3fV#%u  dV=%.3fV\n"
             "  temps : max=%.1fC(S%u)  min=%.1fC(S%u)  avg=%.1fC\n"
             "  state : 0x35C=[%s] charge=%s discharge=%s balance=%s\n"
             "  info  : modules=%u  tag='%s'  0x359=[%s]  0x370=[%s]  0x371=[%s]\n"
             "  note  : Pylon-like dialect with JK-BMS identity",
             snap.valid ? "YES" : "NO",
             ascii35E[0] ? ascii35E : "(none)",
             (double)packVolt,
             (double)packCurrent,
             (double)avgTemp,
             (unsigned)soc,
             (unsigned)soh,
             (double)chargeVoltLimit,
             (double)chargeCurrentLimit,
             (double)dischargeCurrentLimit,
             (double)lowerDischargeVoltTentative,
             (double)cellMax,
             (unsigned)cellMaxIdx,
             (double)cellMin,
             (unsigned)cellMinIdx,
             (double)snap.deltaV,
             (double)tempMax,
             (unsigned)tempMaxSensor,
             (double)tempMin,
             (unsigned)tempMinSensor,
             (double)avgTemp,
             raw35C[0] ? raw35C : "-",
             chargeEnabled ? "ON" : "OFF",
             dischargeEnabled ? "ON" : "OFF",
             balanceEnabled ? "ON" : "OFF",
             (unsigned)moduleCount,
             tag359[0] ? tag359 : "-",
             raw359[0] ? raw359 : "-",
             raw370[0] ? raw370 : "-",
             raw371[0] ? raw371 : "-");

    bridgeSetDecodedLogSnapshot(s_deyeCanLogText);

    ESP_LOGI(TAG, "CAN-%s DEYE SNAPSHOT", (ifname != NULL) ? ifname : "CAN1");
    ESP_LOGI(TAG, "  valid : %s", snap.valid ? "YES" : "NO");
    ESP_LOGI(TAG, "  name  : %s", ascii35E[0] ? ascii35E : "(none)");
    ESP_LOGI(TAG,
             "  pack  : V=%.2fV I=%.1fA avgT=%.1fC SOC=%u%% SOH=%u%%",
             (double)packVolt,
             (double)packCurrent,
             (double)avgTemp,
             (unsigned)soc,
             (unsigned)soh);
    ESP_LOGI(TAG,
             "  limits: chgV=%.1fV chgI=%.1fA disI=%.1fA lowV?=%.1fV",
             (double)chargeVoltLimit,
             (double)chargeCurrentLimit,
             (double)dischargeCurrentLimit,
             (double)lowerDischargeVoltTentative);
    ESP_LOGI(TAG,
             "  cells : max=%.3fV#%u min=%.3fV#%u dV=%.3fV",
             (double)cellMax,
             (unsigned)cellMaxIdx,
             (double)cellMin,
             (unsigned)cellMinIdx,
             (double)snap.deltaV);
    ESP_LOGI(TAG,
             "  temps : max=%.1fC(S%u) min=%.1fC(S%u) avg=%.1fC",
             (double)tempMax,
             (unsigned)tempMaxSensor,
             (double)tempMin,
             (unsigned)tempMinSensor,
             (double)avgTemp);
    ESP_LOGI(TAG,
             "  state : 0x35C=[%s] charge=%s discharge=%s balance=%s",
             raw35C[0] ? raw35C : "-",
             chargeEnabled ? "ON" : "OFF",
             dischargeEnabled ? "ON" : "OFF",
             balanceEnabled ? "ON" : "OFF");
    ESP_LOGI(TAG,
             "  info  : modules=%u tag='%s' 0x359=[%s] 0x370=[%s] 0x371=[%s]",
             (unsigned)moduleCount,
             tag359[0] ? tag359 : "-",
             raw359[0] ? raw359 : "-",
             raw370[0] ? raw370 : "-",
             raw371[0] ? raw371 : "-");
}
