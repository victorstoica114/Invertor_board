#include "pylon_can_protocol.h"

#include "../../bridge.h"

#include <stdio.h>
#include <string.h>

#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "SNIFFER_BRIDGE";
static char s_pylonCanLogText[2048];
static pylon_can_frame_t s_cache[PYLON_CAN_CACHE_COUNT];

static int pylonCanCacheIndex(uint32_t id)
{
    if (id < PYLON_CAN_ID_MIN || id > PYLON_CAN_ID_MAX) {
        return -1;
    }
    return (int)(id - PYLON_CAN_ID_MIN);
}

static const pylon_can_frame_t *pylonCanFrameById(const pylon_can_frame_t *cache, size_t count, uint32_t id)
{
    int idx = pylonCanCacheIndex(id);
    if (cache == NULL || idx < 0 || (size_t)idx >= count) {
        return NULL;
    }
    return cache[idx].valid ? &cache[idx] : NULL;
}

static inline uint16_t pylonCanLe16(const uint8_t *p)
{
    return (uint16_t)((((uint16_t)p[1]) << 8) | (uint16_t)p[0]);
}

static inline int16_t pylonCanLe16s(const uint8_t *p)
{
    return (int16_t)pylonCanLe16(p);
}

static void formatCanData(const uint8_t *data, uint8_t dlc, char *out, size_t outSize)
{
    size_t pos = 0;
    uint8_t n = dlc;

    if (out == NULL || outSize == 0) {
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
    if (pos > 0) {
        out[pos - 1u] = '\0';
    }
}

static void formatCanAscii(const uint8_t *data, uint8_t dlc, char *out, size_t outSize)
{
    size_t n = 0;

    if (out == NULL || outSize == 0) {
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

bool pylonCanAnyValid(const pylon_can_frame_t *cache, size_t count)
{
    if (cache == NULL) {
        return false;
    }

    for (size_t i = 0; i < count; i++) {
        if (cache[i].valid) {
            return true;
        }
    }
    return false;
}

void pylonCanDecodeSnapshot(const char *ifname, const pylon_can_frame_t *cache, size_t count)
{
    const pylon_can_frame_t *f351 = pylonCanFrameById(cache, count, 0x351u);
    const pylon_can_frame_t *f355 = pylonCanFrameById(cache, count, 0x355u);
    const pylon_can_frame_t *f356 = pylonCanFrameById(cache, count, 0x356u);
    const pylon_can_frame_t *f359 = pylonCanFrameById(cache, count, 0x359u);
    const pylon_can_frame_t *f35A = pylonCanFrameById(cache, count, 0x35Au);
    const pylon_can_frame_t *f35C = pylonCanFrameById(cache, count, 0x35Cu);
    const pylon_can_frame_t *f35E = pylonCanFrameById(cache, count, 0x35Eu);
    const pylon_can_frame_t *f372 = pylonCanFrameById(cache, count, 0x372u);
    const pylon_can_frame_t *f373 = pylonCanFrameById(cache, count, 0x373u);
    const pylon_can_frame_t *f374 = pylonCanFrameById(cache, count, 0x374u);
    const pylon_can_frame_t *f375 = pylonCanFrameById(cache, count, 0x375u);
    const pylon_can_frame_t *f376 = pylonCanFrameById(cache, count, 0x376u);
    const pylon_can_frame_t *f377 = pylonCanFrameById(cache, count, 0x377u);
    const pylon_can_frame_t *f379 = pylonCanFrameById(cache, count, 0x379u);
    bridgeTelemetrySnapshot_t snap = {0};
    char raw359[32] = {0};
    char raw35A[32] = {0};
    char raw35C[32] = {0};
    char raw372[32] = {0};
    char raw373[32] = {0};
    char raw379[32] = {0};
    char ascii35E[16] = {0};
    char ascii374[16] = {0};
    char ascii375[16] = {0};
    char ascii376[16] = {0};
    char ascii377[16] = {0};
    float chargeVoltLimit = 0.0f;
    float chargeCurrentLimit = 0.0f;
    float dischargeCurrentLimit = 0.0f;
    float lowerDischargeVoltTentative = 0.0f;
    float packVolt = 0.0f;
    float packCurrent = 0.0f;
    float avgTemp = 0.0f;
    float cellMinTentative = 0.0f;
    float cellMaxTentative = 0.0f;
    float tempMinTentative = 0.0f;
    float tempMaxTentative = 0.0f;
    uint16_t soc = 0;
    uint16_t soh = 0;
    uint8_t moduleCount = 0;
    uint8_t status35C = 0;
    universal_battery_model_t model = {0};

    if (f351 && f351->dlc >= 8u) {
        chargeVoltLimit = (float)pylonCanLe16(&f351->data[0]) / 10.0f;
        chargeCurrentLimit = (float)pylonCanLe16(&f351->data[2]) / 10.0f;
        dischargeCurrentLimit = (float)pylonCanLe16(&f351->data[4]) / 10.0f;
        lowerDischargeVoltTentative = (float)pylonCanLe16(&f351->data[6]) / 10.0f;
    }
    if (f355 && f355->dlc >= 4u) {
        soc = pylonCanLe16(&f355->data[0]);
        soh = pylonCanLe16(&f355->data[2]);
    }
    if (f356 && f356->dlc >= 6u) {
        packVolt = (float)pylonCanLe16(&f356->data[0]) / 100.0f;
        packCurrent = (float)pylonCanLe16s(&f356->data[2]) / 10.0f;
        avgTemp = (float)pylonCanLe16(&f356->data[4]) / 10.0f;
    }
    if (f359 && f359->dlc >= 5u) {
        moduleCount = f359->data[4];
        formatCanData(f359->data, f359->dlc, raw359, sizeof(raw359));
    }
    if (f35A) {
        formatCanData(f35A->data, f35A->dlc, raw35A, sizeof(raw35A));
    }
    if (f35C) {
        formatCanData(f35C->data, f35C->dlc, raw35C, sizeof(raw35C));
        if (f35C->dlc >= 1u) {
            status35C = f35C->data[0];
        }
    }
    if (f35E) {
        formatCanAscii(f35E->data, f35E->dlc, ascii35E, sizeof(ascii35E));
    }
    if (f372) {
        formatCanData(f372->data, f372->dlc, raw372, sizeof(raw372));
    }
    if (f373 && f373->dlc >= 8u) {
        cellMinTentative = (float)pylonCanLe16(&f373->data[0]) / 1000.0f;
        cellMaxTentative = (float)pylonCanLe16(&f373->data[2]) / 1000.0f;
        tempMinTentative = (float)pylonCanLe16(&f373->data[4]) / 10.0f;
        tempMaxTentative = (float)pylonCanLe16(&f373->data[6]) / 10.0f;
        formatCanData(f373->data, f373->dlc, raw373, sizeof(raw373));
    }
    if (f374) formatCanAscii(f374->data, f374->dlc, ascii374, sizeof(ascii374));
    if (f375) formatCanAscii(f375->data, f375->dlc, ascii375, sizeof(ascii375));
    if (f376) formatCanAscii(f376->data, f376->dlc, ascii376, sizeof(ascii376));
    if (f377) formatCanAscii(f377->data, f377->dlc, ascii377, sizeof(ascii377));
    if (f379) formatCanData(f379->data, f379->dlc, raw379, sizeof(raw379));

    snap.valid = (f355 != NULL) && (f356 != NULL);
    snprintf(snap.source, sizeof(snap.source), "%s", (ifname != NULL) ? ifname : "CAN1");
    snprintf(snap.protocol, sizeof(snap.protocol), "CAN_PYLON");
    snap.currentA = packCurrent;
    snap.socPct = (soc <= 100u) ? (uint8_t)soc : 0u;
    snap.sohPct = (soh <= 100u) ? (uint8_t)soh : 0u;
    if (cellMaxTentative > 0.0f) {
        snap.cellMaxV = cellMaxTentative;
    }
    if (cellMinTentative > 0.0f) {
        snap.cellMinV = cellMinTentative;
    }
    if (cellMaxTentative > 0.0f && cellMinTentative > 0.0f) {
        snap.deltaV = cellMaxTentative - cellMinTentative;
    }
    snap.tempMosC = avgTemp;
    snap.tempT1C = tempMinTentative;
    snap.tempT2C = tempMaxTentative;
    snap.pylonStatus63 = status35C;
    bridgeSetTelemetrySnapshot(&snap);

    model.valid = snap.valid;
    model.packVoltageV = packVolt;
    model.packCurrentA = packCurrent;
    model.socPct = snap.socPct;
    model.sohPct = snap.sohPct;
    model.cycleCount = snap.cycles;
    model.chargeVoltageLimitV = chargeVoltLimit;
    model.chargeCurrentLimitA = chargeCurrentLimit;
    model.dischargeCurrentLimitA = dischargeCurrentLimit;
    model.cellMaxV = cellMaxTentative;
    model.cellMinV = cellMinTentative;
    model.cellMaxIdx = 0u;
    model.cellMinIdx = 0u;
    model.cellDeltaV = (cellMaxTentative > 0.0f && cellMinTentative > 0.0f) ? (cellMaxTentative - cellMinTentative) : 0.0f;
    model.temperaturesC[0] = avgTemp;
    model.temperaturesC[1] = tempMinTentative;
    model.temperaturesC[2] = tempMaxTentative;
    model.temperaturesC[3] = tempMinTentative;
    model.temperaturesC[4] = tempMaxTentative;
    model.chargeEnabled = (status35C & 0x80u) != 0u;
    model.dischargeEnabled = (status35C & 0x40u) != 0u;
    model.balanceEnabled = (status35C & 0x20u) != 0u;
    model.protocolState = status35C;
    bridgeSetUniversalBatteryModel(&model);

    snprintf(s_pylonCanLogText,
             sizeof(s_pylonCanLogText),
             "CAN Pylon\n"
             "  valid : %s\n"
             "  name  : %s\n"
             "  pack  : V=%.2fV  I=%.1fA  avgT=%.1fC  SOC=%u%%  SOH=%u%%\n"
             "  limits: chgV=%.1fV  chgI=%.1fA  disI=%.1fA  lowV?=%.1fV\n"
             "  info? : modules=%u  0x359=[%s]  0x35A=[%s]  0x35C=[%s]\n"
             "  ext?  : 0x372=[%s]  0x373=[%s]\n"
             "  cells?: min=%.3fV  max=%.3fV  dV=%.3fV  tMin?=%.1fC  tMax?=%.1fC\n"
             "  text  : 0x35E='%s'  0x374='%s'  0x375='%s'  0x376='%s'  0x377='%s'\n"
             "  misc? : 0x379=[%s]\n"
             "  undecoded/tentative: 0x359,0x35A,0x35C,0x372,0x373,0x374-0x377,0x379",
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
             (unsigned)moduleCount,
             raw359[0] ? raw359 : "-",
             raw35A[0] ? raw35A : "-",
             raw35C[0] ? raw35C : "-",
             raw372[0] ? raw372 : "-",
             raw373[0] ? raw373 : "-",
             (double)cellMinTentative,
             (double)cellMaxTentative,
             (double)((cellMaxTentative > 0.0f && cellMinTentative > 0.0f) ? (cellMaxTentative - cellMinTentative) : 0.0f),
             (double)tempMinTentative,
             (double)tempMaxTentative,
             ascii35E[0] ? ascii35E : "-",
             ascii374[0] ? ascii374 : "-",
             ascii375[0] ? ascii375 : "-",
             ascii376[0] ? ascii376 : "-",
             ascii377[0] ? ascii377 : "-",
             raw379[0] ? raw379 : "-");

    bridgeSetDecodedLogSnapshot(s_pylonCanLogText);

    ESP_LOGI(TAG, "CAN-%s PYLON SNAPSHOT", (ifname != NULL) ? ifname : "CAN1");
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
             "  cells?: min=%.3fV max=%.3fV dV=%.3fV tMin?=%.1fC tMax?=%.1fC",
             (double)cellMinTentative,
             (double)cellMaxTentative,
             (double)((cellMaxTentative > 0.0f && cellMinTentative > 0.0f) ? (cellMaxTentative - cellMinTentative) : 0.0f),
             (double)tempMinTentative,
             (double)tempMaxTentative);
    ESP_LOGI(TAG, "  info? : modules=%u 0x359=[%s] 0x35A=[%s] 0x35C=[%s]",
             (unsigned)moduleCount,
             raw359[0] ? raw359 : "-",
             raw35A[0] ? raw35A : "-",
             raw35C[0] ? raw35C : "-");
    ESP_LOGI(TAG, "  text  : 0x374='%s' 0x375='%s' 0x376='%s' 0x377='%s'",
             ascii374[0] ? ascii374 : "-",
             ascii375[0] ? ascii375 : "-",
             ascii376[0] ? ascii376 : "-",
             ascii377[0] ? ascii377 : "-");
    ESP_LOGI(TAG, "  undecoded/tentative: 0x359,0x35A,0x35C,0x372,0x373,0x374-0x377,0x379");
}

void pylonCanResetCaches(void)
{
    memset(s_cache, 0, sizeof(s_cache));
}

void pylonCanOnFrame(const char *ifname, const twai_message_t *m)
{
    int idx = 0;
    if (m == NULL) {
        return;
    }
    idx = pylonCanCacheIndex((uint32_t)m->identifier);
    if (idx < 0 || (size_t)idx >= PYLON_CAN_CACHE_COUNT) {
        return;
    }

    pylon_can_frame_t f = {0};
    f.valid = true;
    f.id = (uint32_t)m->identifier;
    f.dlc = (uint8_t)m->data_length_code;
    if (f.dlc > 8u) {
        f.dlc = 8u;
    }
    memcpy(f.data, m->data, f.dlc);
    f.updatedMs = (uint32_t)(esp_timer_get_time() / 1000ULL);

    s_cache[idx] = f;
    pylonCanDecodeSnapshot(ifname, s_cache, PYLON_CAN_CACHE_COUNT);
}
