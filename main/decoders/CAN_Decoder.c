#include "decoders/CAN_Decoder.h"

#include "config.h"
#include "Web_interface/web_bridge_api.h"
#include "protocols/common/battery_model.h"
#include "protocols/growatt/growatt_registers_map.h"
#include "protocols/deye/deye_can_protocol.h"
#include "protocols/deye/deye_registers_map.h"
#include "protocols/jkbms_can/jkbms_can_protocol.h"
#include "protocols/pylon/pylon_can_protocol.h"
#include "runtime_settings.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#define CAN_DECODER_IMMEDIATE_DECODE_LOG_ENABLE 0
#define CAN_BMS_CACHE_ID_MIN GROWATT_CAN_CACHE_ID_MIN
#define CAN_BMS_CACHE_ID_MAX GROWATT_CAN_CACHE_ID_MAX
#define CAN_BMS_CACHE_COUNT (CAN_BMS_CACHE_ID_MAX - CAN_BMS_CACHE_ID_MIN + 1u)

typedef struct {
    bool valid;
    uint32_t id;
    uint32_t updatedMs;
    uint8_t dlc;
    uint8_t data[8];
} canBmsCachedFrame_t;

static portMUX_TYPE g_canBmsCacheMux = portMUX_INITIALIZER_UNLOCKED;
static canBmsCachedFrame_t g_can1BmsCache[CAN_BMS_CACHE_COUNT];
static canBmsCachedFrame_t g_can2BmsCache[CAN_BMS_CACHE_COUNT];
static pylon_can_frame_t g_can1PylonCache[PYLON_CAN_CACHE_COUNT];
static pylon_can_frame_t g_can2PylonCache[PYLON_CAN_CACHE_COUNT];
static jkbms_can_frame_t g_can1JkbmsCache[JKBMS_CAN_CACHE_COUNT];
static jkbms_can_frame_t g_can2JkbmsCache[JKBMS_CAN_CACHE_COUNT];
static char g_canGrowattDecodedLog[2048];

static inline uint16_t can_be16(const uint8_t *p);
static inline int16_t can_be16s(const uint8_t *p);
static inline uint16_t can_le16(const uint8_t *p);
static inline int16_t can_le16s(const uint8_t *p);
static void canUpdateUniversalModelFromGrowattCache(const char *ifname);

static float canCorrectPylonPackVoltage(float rawPackVoltageV, float chargeVoltageLimitV)
{
    if (rawPackVoltageV > 0.0f &&
        chargeVoltageLimitV >= 30.0f &&
        rawPackVoltageV < ((chargeVoltageLimitV * 0.5f) + 2.0f)) {
        return rawPackVoltageV * 2.0f;
    }
    return rawPackVoltageV;
}

static canBmsCachedFrame_t *canBmsCacheForIf(const char *ifname)
{
    if (ifname == NULL) {
        return g_can1BmsCache;
    }
    if (strcmp(ifname, "CAN1") == 0) {
        return g_can1BmsCache;
    }
    if (strcmp(ifname, "CAN2") == 0) {
        return g_can2BmsCache;
    }
    return NULL;
}

static pylon_can_frame_t *canPylonCacheForIf(const char *ifname)
{
    if (ifname == NULL) {
        return g_can1PylonCache;
    }
    if (strcmp(ifname, "CAN1") == 0) {
        return g_can1PylonCache;
    }
    if (strcmp(ifname, "CAN2") == 0) {
        return g_can2PylonCache;
    }
    return NULL;
}

static jkbms_can_frame_t *canJkbmsCacheForIf(const char *ifname)
{
    if (ifname == NULL) {
        return g_can1JkbmsCache;
    }
    if (strcmp(ifname, "CAN1") == 0) {
        return g_can1JkbmsCache;
    }
    if (strcmp(ifname, "CAN2") == 0) {
        return g_can2JkbmsCache;
    }
    return NULL;
}

static int canProtocolForIf(const char *ifname)
{
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    const char *name = (ifname != NULL) ? ifname : "CAN1";

    if ((settings.bms_line == LINE_CAN) &&
        (strcmp(name, (settings.bms_port == 1) ? "CAN1" : "CAN2") == 0)) {
        return settings.bms_protocol;
    }
    if ((settings.inverter_line == LINE_CAN) &&
        (strcmp(name, (settings.inverter_port == 1) ? "CAN1" : "CAN2") == 0)) {
        return settings.inverter_protocol;
    }
    return 0;
}

static bool canPylonGetFrameById(const pylon_can_frame_t *cache,
                                 uint32_t id,
                                 const pylon_can_frame_t **out)
{
    if (out == NULL) {
        return false;
    }
    *out = NULL;
    if (cache == NULL || id < PYLON_CAN_ID_MIN || id > PYLON_CAN_ID_MAX) {
        return false;
    }

    size_t idx = (size_t)(id - PYLON_CAN_ID_MIN);
    if (!cache[idx].valid) {
        return false;
    }

    *out = &cache[idx];
    return true;
}

static void canUpdateUniversalModelFromPylonCache(const char *ifname)
{
    const char *name = (ifname != NULL) ? ifname : "CAN1";
    int protocol = canProtocolForIf(name);
    bool deyeProtocol = protocol == PROTOCOL_CAN_DEYE;
    pylon_can_frame_t local[PYLON_CAN_CACHE_COUNT];
    pylon_can_frame_t *src = NULL;
    const pylon_can_frame_t *f351 = NULL;
    const pylon_can_frame_t *f355 = NULL;
    const pylon_can_frame_t *f356 = NULL;
    const pylon_can_frame_t *f35C = NULL;
    const pylon_can_frame_t *f373 = NULL;
    const pylon_can_frame_t *f370 = NULL;
    const pylon_can_frame_t *f371 = NULL;
    bool haveSoc = false;
    bool havePack = false;
    universal_battery_model_t model = {0};
    uint32_t nowMs = (uint32_t)(esp_timer_get_time() / 1000LL);

    if ((protocol != PROTOCOL_CAN_PYLON) && !deyeProtocol) {
        return;
    }

    src = canPylonCacheForIf(name);
    if (src == NULL) {
        return;
    }

    portENTER_CRITICAL(&g_canBmsCacheMux);
    memcpy(local, src, sizeof(local));
    portEXIT_CRITICAL(&g_canBmsCacheMux);

    batteryModelGetReal(&model);

    if (canPylonGetFrameById(local, 0x355u, &f355) && f355->dlc >= 4u) {
        uint16_t soc = can_le16(&f355->data[0]);
        uint16_t soh = can_le16(&f355->data[2]);
        if (soc <= 100u) {
            model.socPct = (uint8_t)soc;
        }
        if (soh <= 100u) {
            model.sohPct = (uint8_t)soh;
        }
        haveSoc = true;
    }

    if (canPylonGetFrameById(local, 0x351u, &f351) && f351->dlc >= 6u) {
        model.chargeVoltageLimitV = (float)can_le16(&f351->data[0]) / 10.0f;
        model.chargeCurrentLimitA = (float)can_le16(&f351->data[2]) / 10.0f;
        model.dischargeCurrentLimitA = (float)can_le16(&f351->data[4]) / 10.0f;
    }

    if (canPylonGetFrameById(local, 0x356u, &f356) && f356->dlc >= 6u) {
        float rawPackVoltageV = (float)can_le16(&f356->data[0]) / 100.0f;
        model.packVoltageV = canCorrectPylonPackVoltage(rawPackVoltageV, model.chargeVoltageLimitV);
        model.packCurrentA = (float)can_le16s(&f356->data[2]) / 10.0f;
        model.temperaturesC[0] = (float)can_le16(&f356->data[4]) / 10.0f;
        havePack = true;
    }

    if (!deyeProtocol && canPylonGetFrameById(local, 0x373u, &f373) && f373->dlc >= 8u) {
        model.cellMinV = (float)can_le16(&f373->data[0]) / 1000.0f;
        model.cellMaxV = (float)can_le16(&f373->data[2]) / 1000.0f;
        model.cellDeltaV = model.cellMaxV - model.cellMinV;
        model.temperaturesC[1] = (float)can_le16(&f373->data[4]) / 10.0f;
        model.temperaturesC[2] = (float)can_le16(&f373->data[6]) / 10.0f;
    }

    if (deyeProtocol && canPylonGetFrameById(local, DEYE_CAN_ID_TEMP_CELL_370, &f370) && f370->dlc >= 8u) {
        uint16_t tMaxRaw = can_le16(&f370->data[DEYE_CAN_370_OFF_TEMP_MAX_RAW]);
        uint16_t tMinRaw = can_le16(&f370->data[DEYE_CAN_370_OFF_TEMP_MIN_RAW]);
        model.temperaturesC[1] = (tMaxRaw <= 200u) ? (float)tMaxRaw : ((float)tMaxRaw / 10.0f);
        model.temperaturesC[2] = (tMinRaw <= 200u) ? (float)tMinRaw : ((float)tMinRaw / 10.0f);
        model.cellMaxV = (float)can_le16(&f370->data[DEYE_CAN_370_OFF_CELL_MAX_MV]) / 1000.0f;
        model.cellMinV = (float)can_le16(&f370->data[DEYE_CAN_370_OFF_CELL_MIN_MV]) / 1000.0f;
        model.cellDeltaV = model.cellMaxV - model.cellMinV;
    }

    if (deyeProtocol && canPylonGetFrameById(local, DEYE_CAN_ID_SENSOR_INDEX_371, &f371) && f371->dlc >= 8u) {
        uint16_t cellMaxIdx = can_le16(&f371->data[DEYE_CAN_371_OFF_CELL_MAX_IDX]);
        uint16_t cellMinIdx = can_le16(&f371->data[DEYE_CAN_371_OFF_CELL_MIN_IDX]);
        if (cellMaxIdx <= 255u) {
            model.cellMaxIdx = (uint8_t)cellMaxIdx;
        }
        if (cellMinIdx <= 255u) {
            model.cellMinIdx = (uint8_t)cellMinIdx;
        }
    }

    if (canPylonGetFrameById(local, 0x35Cu, &f35C) && f35C->dlc >= 1u) {
        uint8_t status = f35C->data[0];
        model.protocolState = status;
        model.chargeEnabled = (status & 0x80u) != 0u;
        model.dischargeEnabled = (status & 0x40u) != 0u;
        model.balanceEnabled = (status & 0x20u) != 0u;
    }

    model.valid = haveSoc && havePack;
    if (model.valid) {
        model.updatedMs = nowMs;
        batteryModelSet(&model);
    }
}

static int canBmsCacheIndex(uint32_t id)
{
    if (id < CAN_BMS_CACHE_ID_MIN || id > CAN_BMS_CACHE_ID_MAX) {
        return -1;
    }
    return (int)(id - CAN_BMS_CACHE_ID_MIN);
}

static void canBmsCacheUpdate(const char *ifname, const twai_message_t *m)
{
    if (ifname == NULL || m == NULL) return;

    canBmsCachedFrame_t *cache = canBmsCacheForIf(ifname);
    if (cache == NULL) return;

    int idx = canBmsCacheIndex((uint32_t)m->identifier);
    if (idx < 0) return;

    canBmsCachedFrame_t f = {0};
    f.valid = true;
    f.id = (uint32_t)m->identifier;
    f.updatedMs = (uint32_t)(esp_timer_get_time() / 1000LL);
    f.dlc = (uint8_t)m->data_length_code;
    if (f.dlc > 8u) f.dlc = 8u;
    memcpy(f.data, m->data, f.dlc);

    portENTER_CRITICAL(&g_canBmsCacheMux);
    cache[idx] = f;
    portEXIT_CRITICAL(&g_canBmsCacheMux);
}

static void canPylonCacheUpdate(const char *ifname, const twai_message_t *m)
{
    if (ifname == NULL || m == NULL) return;

    pylon_can_frame_t *cache = canPylonCacheForIf(ifname);
    if (cache == NULL) return;

    if ((uint32_t)m->identifier < PYLON_CAN_ID_MIN || (uint32_t)m->identifier > PYLON_CAN_ID_MAX) return;
    int idx = (int)((uint32_t)m->identifier - PYLON_CAN_ID_MIN);

    pylon_can_frame_t f = {0};
    f.valid = true;
    f.id = (uint32_t)m->identifier;
    f.updatedMs = (uint32_t)(esp_timer_get_time() / 1000LL);
    f.dlc = (uint8_t)m->data_length_code;
    if (f.dlc > 8u) f.dlc = 8u;
    memcpy(f.data, m->data, f.dlc);

    portENTER_CRITICAL(&g_canBmsCacheMux);
    cache[idx] = f;
    portEXIT_CRITICAL(&g_canBmsCacheMux);
}

static void canJkbmsCacheUpdate(const char *ifname, const twai_message_t *m)
{
    if (ifname == NULL || m == NULL) return;

    jkbms_can_frame_t *cache = canJkbmsCacheForIf(ifname);
    if (cache == NULL) return;

    int idx = jkbmsCanCacheIndex((uint32_t)m->identifier);
    if (idx < 0 || (size_t)idx >= JKBMS_CAN_CACHE_COUNT) return;

    jkbms_can_frame_t f = {0};
    f.valid = true;
    f.id = (uint32_t)m->identifier;
    f.updatedMs = (uint32_t)(esp_timer_get_time() / 1000LL);
    f.dlc = (uint8_t)m->data_length_code;
    if (f.dlc > 8u) f.dlc = 8u;
    memcpy(f.data, m->data, f.dlc);

    portENTER_CRITICAL(&g_canBmsCacheMux);
    cache[idx] = f;
    portEXIT_CRITICAL(&g_canBmsCacheMux);
}

static void canUpdateUniversalModelFromJkbmsCache(const char *ifname)
{
    const char *name = (ifname != NULL) ? ifname : "CAN1";
    int protocol = canProtocolForIf(name);
    jkbms_can_frame_t local[JKBMS_CAN_CACHE_COUNT];
    jkbms_can_frame_t *src = NULL;

    if (protocol != PROTOCOL_CAN_JKBMS_250K) {
        return;
    }

    src = canJkbmsCacheForIf(name);
    if (src == NULL) {
        return;
    }

    portENTER_CRITICAL(&g_canBmsCacheMux);
    memcpy(local, src, sizeof(local));
    portEXIT_CRITICAL(&g_canBmsCacheMux);

    jkbmsCanUpdateBatteryModel(name, local, JKBMS_CAN_CACHE_COUNT);
}

static const canBmsCachedFrame_t *canGrowattFrameById(const canBmsCachedFrame_t *cache, uint32_t id)
{
    int idx = canBmsCacheIndex(id);
    if (cache == NULL || idx < 0 || (size_t)idx >= CAN_BMS_CACHE_COUNT) {
        return NULL;
    }
    return cache[idx].valid ? &cache[idx] : NULL;
}

static bool canGrowattValidPackVoltageRaw(uint16_t raw, float *voltageOut)
{
    if (raw >= 3000u && raw <= 9000u) {
        if (voltageOut != NULL) {
            *voltageOut = (float)raw / 100.0f;
        }
        return true;
    }

    if (raw >= 300u && raw <= 900u) {
        if (voltageOut != NULL) {
            *voltageOut = (float)raw / 10.0f;
        }
        return true;
    }

    return false;
}

static bool canGrowattDecodePackVoltage(const uint8_t *p, bool *littleEndianOut, float *voltageOut)
{
    float beVoltage = 0.0f;
    float leVoltage = 0.0f;
    bool beOk = false;
    bool leOk = false;

    if (p == NULL) {
        return false;
    }

    beOk = canGrowattValidPackVoltageRaw(can_be16(p), &beVoltage);
    leOk = canGrowattValidPackVoltageRaw(can_le16(p), &leVoltage);

    if (beOk && (!leOk || beVoltage >= leVoltage)) {
        if (littleEndianOut != NULL) {
            *littleEndianOut = false;
        }
        if (voltageOut != NULL) {
            *voltageOut = beVoltage;
        }
        return true;
    }

    if (leOk) {
        if (littleEndianOut != NULL) {
            *littleEndianOut = true;
        }
        if (voltageOut != NULL) {
            *voltageOut = leVoltage;
        }
        return true;
    }

    return false;
}

static uint16_t canGrowattU16(const uint8_t *p, bool littleEndian)
{
    return littleEndian ? can_le16(p) : can_be16(p);
}

static int16_t canGrowattI16(const uint8_t *p, bool littleEndian)
{
    return littleEndian ? can_le16s(p) : can_be16s(p);
}

static bool canGrowattCellCandidateValid(uint16_t maxMv, uint16_t minMv)
{
    return minMv >= 1500u &&
           minMv <= 5000u &&
           maxMv >= 1500u &&
           maxMv <= 5000u &&
           maxMv >= minMv;
}

static uint32_t canGrowattCellCandidateScore(uint16_t maxMv,
                                             uint16_t minMv,
                                             float packVoltageV)
{
    uint32_t score = (uint32_t)(maxMv - minMv);

    if (packVoltageV > 10.0f) {
        uint16_t avgMv = (uint16_t)((packVoltageV * 1000.0f / 16.0f) + 0.5f);
        uint16_t midMv = (uint16_t)(((uint32_t)maxMv + (uint32_t)minMv) / 2u);
        uint16_t diff = (midMv > avgMv) ? (uint16_t)(midMv - avgMv) : (uint16_t)(avgMv - midMv);
        score += (uint32_t)diff * 4u;
    }

    return score;
}

static bool canGrowattTryCellCandidate(uint16_t maxMv,
                                       uint16_t minMv,
                                       uint8_t maxIdx,
                                       uint8_t minIdx,
                                       float packVoltageV,
                                       uint32_t *scoreInOut,
                                       uint16_t *maxOut,
                                       uint16_t *minOut,
                                       uint8_t *maxIdxOut,
                                       uint8_t *minIdxOut)
{
    uint32_t score = 0u;

    if (!canGrowattCellCandidateValid(maxMv, minMv) || scoreInOut == NULL) {
        return false;
    }

    score = canGrowattCellCandidateScore(maxMv, minMv, packVoltageV);
    if (score >= *scoreInOut) {
        return false;
    }

    *scoreInOut = score;
    if (maxOut != NULL) {
        *maxOut = maxMv;
    }
    if (minOut != NULL) {
        *minOut = minMv;
    }
    if (maxIdxOut != NULL) {
        *maxIdxOut = (maxIdx >= 1u && maxIdx <= 32u) ? maxIdx : 0u;
    }
    if (minIdxOut != NULL) {
        *minIdxOut = (minIdx >= 1u && minIdx <= 32u) ? minIdx : 0u;
    }
    return true;
}

static bool canGrowattDecodeCellExtremes(const uint8_t *d,
                                         float packVoltageV,
                                         uint16_t *maxMvOut,
                                         uint16_t *minMvOut,
                                         uint8_t *maxIdxOut,
                                         uint8_t *minIdxOut)
{
    uint32_t bestScore = UINT32_MAX;
    uint16_t bestMax = 0u;
    uint16_t bestMin = 0u;
    uint8_t bestMaxIdx = 0u;
    uint8_t bestMinIdx = 0u;

    if (d == NULL) {
        return false;
    }

    (void)canGrowattTryCellCandidate(can_be16(&d[0]), can_be16(&d[2]), d[5], d[6],
                                     packVoltageV, &bestScore, &bestMax, &bestMin,
                                     &bestMaxIdx, &bestMinIdx);
    (void)canGrowattTryCellCandidate(can_le16(&d[0]), can_le16(&d[2]), d[5], d[6],
                                     packVoltageV, &bestScore, &bestMax, &bestMin,
                                     &bestMaxIdx, &bestMinIdx);
    (void)canGrowattTryCellCandidate(can_be16(&d[1]), can_be16(&d[3]), d[5], d[6],
                                     packVoltageV, &bestScore, &bestMax, &bestMin,
                                     &bestMaxIdx, &bestMinIdx);
    (void)canGrowattTryCellCandidate(can_le16(&d[1]), can_le16(&d[3]), d[5], d[6],
                                     packVoltageV, &bestScore, &bestMax, &bestMin,
                                     &bestMaxIdx, &bestMinIdx);

    if (bestScore == UINT32_MAX) {
        return false;
    }

    if (maxMvOut != NULL) {
        *maxMvOut = bestMax;
    }
    if (minMvOut != NULL) {
        *minMvOut = bestMin;
    }
    if (maxIdxOut != NULL) {
        *maxIdxOut = bestMaxIdx;
    }
    if (minIdxOut != NULL) {
        *minIdxOut = bestMinIdx;
    }
    return true;
}

static bool canGrowattDecodeCellWord(const uint8_t *p, float packVoltageV, uint16_t *mvOut)
{
    uint16_t be = 0u;
    uint16_t le = 0u;
    bool beOk = false;
    bool leOk = false;

    if (p == NULL || mvOut == NULL) {
        return false;
    }

    be = can_be16(p);
    le = can_le16(p);
    beOk = be >= 1500u && be <= 5000u;
    leOk = le >= 1500u && le <= 5000u;
    if (!beOk && !leOk) {
        return false;
    }

    if (beOk && leOk && packVoltageV > 10.0f) {
        uint16_t avgMv = (uint16_t)((packVoltageV * 1000.0f / 16.0f) + 0.5f);
        uint16_t beDiff = (be > avgMv) ? (uint16_t)(be - avgMv) : (uint16_t)(avgMv - be);
        uint16_t leDiff = (le > avgMv) ? (uint16_t)(le - avgMv) : (uint16_t)(avgMv - le);
        *mvOut = (leDiff < beDiff) ? le : be;
        return true;
    }

    *mvOut = beOk ? be : le;
    return true;
}

static void canGrowattFormatCanData(const uint8_t *data, uint8_t dlc, char *out, size_t outSize)
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

static inline uint16_t can_be16(const uint8_t *p)
{
    return (uint16_t)((((uint16_t)p[0]) << 8) | (uint16_t)p[1]);
}

static inline int16_t can_be16s(const uint8_t *p)
{
    return (int16_t)can_be16(p);
}

static inline uint16_t can_le16(const uint8_t *p)
{
    return (uint16_t)((((uint16_t)p[1]) << 8) | (uint16_t)p[0]);
}

static inline int16_t can_le16s(const uint8_t *p)
{
    return (int16_t)can_le16(p);
}

static const char *growattChemStr(uint8_t code)
{
    switch (code & 0x03u) {
    case 0u: return "LFP";
    case 1u: return "Ternary";
    case 2u: return "LTO";
    default: return "Reserved";
    }
}

static const char *growattModeStr(uint16_t status)
{
    switch (status & 0x0003u) {
    case 0u: return "soft_start";
    case 1u: return "standby";
    case 2u: return "charging";
    case 3u: return "discharging";
    default: return "?";
    }
}

static const char *growattOpModeStr(uint16_t status)
{
    switch ((status >> 8) & 0x03u) {
    case 0u: return "standalone";
    case 1u: return "parallel";
    case 2u: return "parallel_prep";
    default: return "reserved";
    }
}

static const char *const k312Prot1Bits[8] = {
    "soft_start_fail",      /* bit0 */
    "module_uv_prot",       /* bit1 */
    "module_ov_prot",       /* bit2 */
    "cell_uv_prot",         /* bit3 */
    "cell_ov_prot",         /* bit4 */
    "scd_prot",             /* bit5 */
    "chg_oc_prot",          /* bit6 */
    "dis_oc_prot",          /* bit7 */
};

static const char *const k312Prot2Bits[8] = {
    NULL,                    /* bit0 */
    NULL,                    /* bit1 */
    "delta_v_fail_prot",    /* bit2 */
    "system_error_prot",    /* bit3 */
    "utc_prot",             /* bit4 */
    "utd_prot",             /* bit5 */
    "otc_prot",             /* bit6 */
    "otd_prot",             /* bit7 */
};

static const char *const k312Alm1Bits[8] = {
    NULL,                    /* bit0 */
    "module_uv_alarm",      /* bit1 */
    "module_ov_alarm",      /* bit2 */
    "cell_uv_alarm",        /* bit3 */
    "cell_ov_alarm",        /* bit4 */
    NULL,                    /* bit5 */
    "chg_oc_alarm",         /* bit6 */
    "dis_oc_alarm",         /* bit7 */
};

static const char *const k312Alm2Bits[8] = {
    "int_comm_fail_alarm",  /* bit0 */
    "pack_turnoff_alarm",   /* bit1 */
    "delta_v_fail_alarm",   /* bit2 */
    NULL,                    /* bit3 */
    "utc_warn",             /* bit4 */
    "utd_warn",             /* bit5 */
    "otc_warn",             /* bit6 */
    "otd_warn",             /* bit7 */
};

static const char *const k312PwrRedHBits[8] = {
    "pwrred_h_bit0",
    "pwrred_h_bit1",
    "pwrred_h_bit2",
    "pwrred_h_bit3",
    "pwrred_h_bit4",
    "pwrred_h_bit5",
    "pwrred_h_bit6",
    "pwrred_h_bit7",
};

static const char *const k312PwrRedLBits[8] = {
    "pwrred_l_bit0",
    "pwrred_l_bit1",
    "pwrred_l_bit2",
    "pwrred_l_bit3",
    "pwrred_l_bit4",
    "pwrred_l_bit5",
    "pwrred_l_bit6",
    "pwrred_l_bit7",
};

static const char *const k323Prot3Bits[8] = {
    "olc_prot",             /* bit0 */
    "old_prot",             /* bit1 */
    "ext_com_fault",        /* bit2 */
    "pre_chg_fail",         /* bit3 */
    "hw_fault",             /* bit4 */
    "afe_com_fault",        /* bit5 */
    "cell_lost_fault",      /* bit6 */
    "pack_i_sample_fault",  /* bit7 */
};

static const char *const k323Prot4Bits[8] = {
    "flt_sp_umain",         /* bit0 */
    "flt_sp_uload",         /* bit1 */
    "flt_eep_param",        /* bit2 */
    "flt_chbus_reverse",    /* bit3 */
    "flt_ovp",              /* bit4 */
    "flt_ocp",              /* bit5 */
    "flt_parallel",         /* bit6 */
    "flt_prll_udiff_over",  /* bit7 */
};

static const char *const k323Prot5Bits[8] = {
    "flt_dis_ocp",          /* bit0 */
    "flt_ch_ilimit_norsp",  /* bit1 */
    "flt_di_ilimit_norsp",  /* bit2 */
    "flt_bus_open",         /* bit3 */
    NULL,                    /* bit4 */
    NULL,                    /* bit5 */
    NULL,                    /* bit6 */
    NULL,                    /* bit7 */
};

static const char *const k323Warn3Bits[8] = {
    "olc_warn",                     /* bit0 */
    "old_warn",                     /* bit1 */
    "prll_i_inch_h2_oc_warn",       /* bit2 */
    "prll_i_indis_h2_oc_warn",      /* bit3 */
    NULL,                            /* bit4 */
    NULL,                            /* bit5 */
    NULL,                            /* bit6 */
    NULL,                            /* bit7 */
};

static void logActiveBitNames(const char *ifname,
                              uint32_t id,
                              const char *field,
                              uint8_t value,
                              const char *const names[8])
{
    if (value == 0u) {
        return;
    }

    char buf[256] = {0};
    int pos = 0;

    for (int bit = 7; bit >= 0; --bit) {
        if ((value & (uint8_t)(1u << bit)) == 0u) {
            continue;
        }

        const char *name = names[bit];
        if (name == NULL) {
            continue;
        }

        pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, "%s%s",
                        (pos > 0) ? "|" : "",
                        name);
        if (pos >= (int)sizeof(buf)) {
            break;
        }
    }

    ESP_LOGI(EXAMPLE_TAG,
             "CAN-%s 0x%03" PRIX32 " %s active: %s",
             ifname,
             id,
             field,
             (buf[0] != '\0') ? buf : "(only reserved bits)");
}

static bool canStrEndsWith(const char *value, const char *suffix)
{
    size_t valueLen = 0;
    size_t suffixLen = 0;

    if (value == NULL || suffix == NULL) {
        return false;
    }

    valueLen = strlen(value);
    suffixLen = strlen(suffix);
    if (suffixLen > valueLen) {
        return false;
    }
    return strcmp(value + valueLen - suffixLen, suffix) == 0;
}

static void appendAlertName(char *out, uint32_t outSize, const char *name)
{
    size_t pos = 0;

    if (out == NULL || outSize == 0u || name == NULL || name[0] == '\0') {
        return;
    }

    pos = strlen(out);
    if (pos >= outSize - 1u) {
        return;
    }
    if (pos > 0u) {
        pos += (size_t)snprintf(out + pos, outSize - pos, ", ");
    }
    if (pos < outSize - 1u) {
        snprintf(out + pos, outSize - pos, "%s", name);
    }
}

static void appendAlertBitsByCategory(char *protectionsOut,
                                      uint32_t protectionsOutSize,
                                      char *alarmsOut,
                                      uint32_t alarmsOutSize,
                                      char *warningsOut,
                                      uint32_t warningsOutSize,
                                      uint8_t bits,
                                      const char *const names[8])
{
    for (int i = 0; i < 8; i++) {
        const char *name = names[i];
        if (((bits >> i) & 0x01u) == 0u || name == NULL) {
            continue;
        }

        if (canStrEndsWith(name, "_warn")) {
            appendAlertName(warningsOut, warningsOutSize, name);
        } else if (canStrEndsWith(name, "_alarm")) {
            appendAlertName(alarmsOut, alarmsOutSize, name);
        } else if (canStrEndsWith(name, "_prot") ||
                   canStrEndsWith(name, "_fault") ||
                   strncmp(name, "flt_", 4u) == 0) {
            appendAlertName(protectionsOut, protectionsOutSize, name);
        }
    }
}

static void logRawCanMsg(const char *ifname, const twai_message_t *m)
{
    char dataHex[3 * 8 + 1] = {0};
    int pos = 0;

    for (int i = 0; i < m->data_length_code && i < 8; i++) {
        pos += snprintf(&dataHex[pos], sizeof(dataHex) - (size_t)pos, "%02X ", m->data[i]);
        if (pos >= (int)sizeof(dataHex)) break;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "RX on %s: ID=0x%03" PRIX32 " DLC=%d DATA=[%s]",
             ifname,
             (uint32_t)m->identifier,
             m->data_length_code,
             dataHex);
}

static void decodeGrowattCanFrame(const char *ifname, const twai_message_t *m)
{
    if (m == NULL || m->data_length_code != 8) return;

    const uint8_t *d = m->data;
    const uint32_t id = (uint32_t)m->identifier;

    switch (id) {
    case GROWATT_CAN_ID_311_STATUS_LIMITS: {
        /* Observed on JK/Growatt traffic: 0..1 status, 2..3 CV, 4..5 IchgLim, 6..7 IdisLim */
        const uint16_t st     = can_be16(&d[0]);
        const int16_t cv_0p1  = can_be16s(&d[2]);
        const int16_t chg_0p1 = can_be16s(&d[4]);
        const int16_t dis_0p1 = can_be16s(&d[6]);

        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s 0x311: status=0x%04X CV=%.1fV IchgLim=%.1fA IdisLim=%.1fA mode=%s errValid=%u bal=%u sleep=%u outDis=%u outChg=%u termOpen=%u opMode=%s",
                 ifname,
                 (unsigned)st,
                 (double)((float)cv_0p1 / 10.0f),
                 (double)((float)chg_0p1 / 10.0f),
                 (double)((float)dis_0p1 / 10.0f),
                 growattModeStr(st),
                 (unsigned)((st >> 2) & 1u),
                 (unsigned)((st >> 3) & 1u),
                 (unsigned)((st >> 4) & 1u),
                 (unsigned)((st >> 5) & 1u),
                 (unsigned)((st >> 6) & 1u),
                 (unsigned)((st >> 7) & 1u),
                 growattOpModeStr(st));
        break;
    }

    case GROWATT_CAN_ID_312_PROT_ALM:
        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s 0x312: Prot1=0x%02X Prot2=0x%02X Alm1=0x%02X Alm2=0x%02X PackNo=%u PwrRed(H/L)=0x%02X%02X Rsv=0x%02X",
                 ifname,
                 (unsigned)d[0], (unsigned)d[1], (unsigned)d[2], (unsigned)d[3],
                 (unsigned)d[4], (unsigned)d[5], (unsigned)d[6], (unsigned)d[7]);
        logActiveBitNames(ifname, id, "Prot1", d[0], k312Prot1Bits);
        logActiveBitNames(ifname, id, "Prot2", d[1], k312Prot2Bits);
        logActiveBitNames(ifname, id, "Alm1", d[2], k312Alm1Bits);
        logActiveBitNames(ifname, id, "Alm2", d[3], k312Alm2Bits);
        logActiveBitNames(ifname, id, "PwrRedH", d[5], k312PwrRedHBits);
        logActiveBitNames(ifname, id, "PwrRedL", d[6], k312PwrRedLBits);
        break;

    case GROWATT_CAN_ID_313_V_I_SOC_SOH: {
        const int16_t v_0p01 = can_be16s(&d[0]);
        const int16_t i_0p1  = can_be16s(&d[2]);
        const int16_t t_0p1  = can_be16s(&d[4]);
        const uint8_t soh     = (uint8_t)(d[7] & 0x7Fu);
        const uint8_t lifeWarn = (uint8_t)((d[7] >> 7) & 0x01u);

        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s 0x313: V=%.2fV I=%+.1fA Tavg=%.1fC SOC=%u%% SOH=%u%% lifeWarn=%u",
                 ifname,
                 (double)((float)v_0p01 / 100.0f),
                 (double)((float)i_0p1 / 10.0f),
                 (double)((float)t_0p1 / 10.0f),
                 (unsigned)d[6], (unsigned)soh, (unsigned)lifeWarn);
        break;
    }

    case GROWATT_CAN_ID_314_RM_FCC_DV_CYCLES: {
        /* PDF Rev_05 confirms RM/FCC in 10mAh, dV in mV, cycle count in bytes 6..7 */
        const uint16_t rm_10mAh  = can_be16(&d[0]);
        const uint16_t fcc_10mAh = can_be16(&d[2]);
        const uint16_t delta_mV  = can_be16(&d[4]);
        const uint16_t cycles    = can_be16(&d[6]);

        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s 0x314: RM=%.2fAh FCC=%.2fAh dV=%umV Cycles=%u",
                 ifname,
                 (double)((float)rm_10mAh / 100.0f),
                 (double)((float)fcc_10mAh / 100.0f),
                 (unsigned)delta_mV,
                 (unsigned)cycles);
        break;
    }

    case GROWATT_CAN_ID_315_CELL_GRP1:
    case GROWATT_CAN_ID_316_CELL_GRP2:
    case GROWATT_CAN_ID_317_CELL_GRP3:
    case GROWATT_CAN_ID_318_CELL_GRP4: {
        /* Optional frame per PDF; some batteries do not send these. */
        const unsigned base = (unsigned)((id - GROWATT_CAN_ID_315_CELL_GRP1) * 4u + 1u);
        uint16_t c[4];

        for (int i = 0; i < 4; i++) {
            const uint16_t be = can_be16(&d[i * 2]);
            const uint16_t le = can_le16(&d[i * 2]);
            const bool beOk = (be >= 2000u && be <= 5000u);
            const bool leOk = (le >= 2000u && le <= 5000u);
            c[i] = beOk ? be : (leOk ? le : be);
        }

        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s 0x%03X CELLS(opt): C%02u=%umV C%02u=%umV C%02u=%umV C%02u=%umV",
                 ifname, (unsigned)id,
                 base + 0u, (unsigned)c[0],
                 base + 1u, (unsigned)c[1],
                 base + 2u, (unsigned)c[2],
                 base + 3u, (unsigned)c[3]);
        break;
    }

    case GROWATT_CAN_ID_319_CELL_REF_FLAGS: {
        /* PDF says max/min-cell related; on JK traffic these behave more like thresholds + indices. */
        const uint16_t vhi = can_le16(&d[0]);
        const uint16_t vlo = can_le16(&d[2]);
        const uint8_t flags = d[4];
        const uint8_t cmaxNo = d[5];
        const uint8_t cminNo = d[6];
        const uint8_t addr = d[7];

        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s 0x319: VhiRef=%umV(idx=%u) VloRef=%umV(idx=%u) dRef=%umV | type?=%s flags=0x%02X chgEn=%u disEn=%u force1=%u force2=%u addr=%u",
                 ifname,
                 (unsigned)vhi, (unsigned)cmaxNo,
                 (unsigned)vlo, (unsigned)cminNo,
                 (unsigned)(vhi >= vlo ? (vhi - vlo) : 0u),
                 growattChemStr(flags),
                 (unsigned)flags,
                 (unsigned)((flags >> 2) & 1u),
                 (unsigned)((flags >> 3) & 1u),
                 (unsigned)((flags >> 4) & 1u),
                 (unsigned)((flags >> 5) & 1u),
                 (unsigned)addr);
        break;
    }

    case GROWATT_CAN_ID_320_MAKER_SW: {
        /* Manufacturer and version compatibility frame */
        const char a = (d[0] >= 32 && d[0] <= 126) ? (char)d[0] : '?';
        const char b = (d[1] >= 32 && d[1] <= 126) ? (char)d[1] : '?';

        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s 0x320: maker='%c%c' hw=0x%02X swL=0x%02X swHext=0x%02X compat=0x%02X ext=0x%02X rsv=0x%02X",
                 ifname, a, b,
                 (unsigned)d[2], (unsigned)d[3], (unsigned)d[4],
                 (unsigned)d[5], (unsigned)d[6], (unsigned)d[7]);
        break;
    }

    case GROWATT_CAN_ID_321_UPGRADE_INFO: {
        bool allZero = true;
        for (int i = 0; i < 8; i++) {
            if (d[i] != 0) {
                allZero = false;
                break;
            }
        }

        if (allZero) {
            ESP_LOGI(EXAMPLE_TAG, "CAN-%s 0x321: remote-upgrade frame unused (all zero)", ifname);
        } else {
            ESP_LOGI(EXAMPLE_TAG,
                     "CAN-%s 0x321: updStatus=0x%02X progress=%u%% progStatus=0x%02X rsv=[%02X %02X %02X %02X %02X]",
                     ifname,
                     (unsigned)d[0], (unsigned)d[1], (unsigned)d[2],
                     (unsigned)d[3], (unsigned)d[4], (unsigned)d[5], (unsigned)d[6], (unsigned)d[7]);
        }
        break;
    }

    case GROWATT_CAN_ID_322_TEMP_SOC_MIN_MAX: {
        /* PDF Rev_05: highest/min temp, sensor numbers, max/min SOC */
        const int16_t tMax_0p1 = can_be16s(&d[0]);
        const int16_t tMin_0p1 = can_be16s(&d[2]);

        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s 0x322: Tmax=%.1fC(U%u) Tmin=%.1fC(U%u) SOCmax=%u%% SOCmin=%u%%",
                 ifname,
                 (double)((float)tMax_0p1 / 10.0f), (unsigned)d[4],
                 (double)((float)tMin_0p1 / 10.0f), (unsigned)d[5],
                 (unsigned)d[6], (unsigned)d[7]);
        break;
    }

    case GROWATT_CAN_ID_323_CELLCOUNT_PROT_WARN:
        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s 0x323: cellCount=%u prot3=0x%02X prot4=0x%02X prot5=0x%02X warn3=0x%02X",
                 ifname,
                 (unsigned)d[0], (unsigned)d[4], (unsigned)d[5], (unsigned)d[6], (unsigned)d[7]);
        logActiveBitNames(ifname, id, "Prot3", d[4], k323Prot3Bits);
        logActiveBitNames(ifname, id, "Prot4", d[5], k323Prot4Bits);
        logActiveBitNames(ifname, id, "Prot5", d[6], k323Prot5Bits);
        logActiveBitNames(ifname, id, "Warn3", d[7], k323Warn3Bits);
        break;

    default:
        break;
    }
}

static void canUpdateUniversalModelFromGrowattCache(const char *ifname)
{
    const char *name = (ifname != NULL) ? ifname : "CAN1";
    int protocol = canProtocolForIf(name);
    canBmsCachedFrame_t local[CAN_BMS_CACHE_COUNT];
    canBmsCachedFrame_t *src = NULL;
    const canBmsCachedFrame_t *f311 = NULL;
    const canBmsCachedFrame_t *f312 = NULL;
    const canBmsCachedFrame_t *f313 = NULL;
    const canBmsCachedFrame_t *f314 = NULL;
    const canBmsCachedFrame_t *f319 = NULL;
    const canBmsCachedFrame_t *f322 = NULL;
    const canBmsCachedFrame_t *f323 = NULL;
    bridgeTelemetrySnapshot_t snap = {0};
    universal_battery_model_t model = {0};
    float packVoltageV = 0.0f;
    float packCurrentA = 0.0f;
    float avgTempC = 0.0f;
    float tempMaxC = 0.0f;
    float tempMinC = 0.0f;
    float chargeVoltageLimitV = 0.0f;
    float chargeCurrentLimitA = 0.0f;
    float dischargeCurrentLimitA = 0.0f;
    uint16_t cycles = 0u;
    uint16_t remainingCapCah = 0u;
    uint16_t fullCapCah = 0u;
    uint16_t cellMaxMv = 0u;
    uint16_t cellMinMv = 0u;
    uint8_t cellMaxIdx = 0u;
    uint8_t cellMinIdx = 0u;
    uint8_t soc = 0u;
    uint8_t soh = 0u;
    uint16_t status = 0u;
    bool havePack = false;
    bool haveSoc = false;
    bool haveSoh = false;
    bool haveAvgTemp = false;
    bool haveTempRange = false;
    bool haveCellExtremes = false;
    bool haveLimits = false;
    bool haveCycles = false;
    bool haveStatus = false;
    bool littleEndian313 = false;
    bool chargeEnabled = false;
    bool dischargeEnabled = false;
    bool balanceEnabled = false;
    char raw311[32] = {0};
    char raw312[32] = {0};
    char raw313[32] = {0};
    char raw314[32] = {0};
    char raw319[32] = {0};
    char raw322[32] = {0};
    char raw323[32] = {0};

    if (protocol != PROTOCOL_CAN_GROWATT) {
        return;
    }

    src = canBmsCacheForIf(name);
    if (src == NULL) {
        return;
    }

    portENTER_CRITICAL(&g_canBmsCacheMux);
    memcpy(local, src, sizeof(local));
    portEXIT_CRITICAL(&g_canBmsCacheMux);

    f311 = canGrowattFrameById(local, GROWATT_CAN_ID_311_STATUS_LIMITS);
    f312 = canGrowattFrameById(local, GROWATT_CAN_ID_312_PROT_ALM);
    f313 = canGrowattFrameById(local, GROWATT_CAN_ID_313_V_I_SOC_SOH);
    f314 = canGrowattFrameById(local, GROWATT_CAN_ID_314_RM_FCC_DV_CYCLES);
    f319 = canGrowattFrameById(local, GROWATT_CAN_ID_319_CELL_REF_FLAGS);
    f322 = canGrowattFrameById(local, GROWATT_CAN_ID_322_TEMP_SOC_MIN_MAX);
    f323 = canGrowattFrameById(local, GROWATT_CAN_ID_323_CELLCOUNT_PROT_WARN);

    if (f313 != NULL) {
        canGrowattFormatCanData(f313->data, f313->dlc, raw313, sizeof(raw313));
    }
    if (f313 != NULL && f313->dlc >= 8u) {
        const uint8_t *d = f313->data;
        havePack = canGrowattDecodePackVoltage(&d[0], &littleEndian313, &packVoltageV);
        if (havePack) {
            packCurrentA = (float)canGrowattI16(&d[2], littleEndian313) / 10.0f;
            avgTempC = (float)canGrowattI16(&d[4], littleEndian313) / 10.0f;
            haveAvgTemp = avgTempC >= -50.0f && avgTempC <= 120.0f;
        }
        if (d[6] <= 100u) {
            soc = d[6];
            haveSoc = true;
        }
        if ((d[7] & 0x7Fu) <= 100u) {
            soh = (uint8_t)(d[7] & 0x7Fu);
            haveSoh = true;
        }
    }

    if (f311 != NULL) {
        canGrowattFormatCanData(f311->data, f311->dlc, raw311, sizeof(raw311));
    }
    if (f311 != NULL && f311->dlc >= 8u) {
        const uint8_t *d = f311->data;
        bool limitsLittle = false;
        float limitV = 0.0f;

        if (canGrowattDecodePackVoltage(&d[0], &limitsLittle, &limitV)) {
            chargeVoltageLimitV = limitV;
            chargeCurrentLimitA = (float)canGrowattU16(&d[2], limitsLittle) / 10.0f;
            dischargeCurrentLimitA = (float)canGrowattU16(&d[4], limitsLittle) / 10.0f;
            status = canGrowattU16(&d[6], limitsLittle);
            haveLimits = true;
            haveStatus = true;
        } else if (canGrowattDecodePackVoltage(&d[2], &limitsLittle, &limitV)) {
            status = canGrowattU16(&d[0], limitsLittle);
            chargeVoltageLimitV = limitV;
            chargeCurrentLimitA = (float)canGrowattU16(&d[4], limitsLittle) / 10.0f;
            dischargeCurrentLimitA = (float)canGrowattU16(&d[6], limitsLittle) / 10.0f;
            haveLimits = true;
            haveStatus = true;
        }

        if (haveStatus) {
            balanceEnabled = false;
            chargeEnabled = ((status & 0x0001u) != 0u) ||
                            ((status & 0x0040u) != 0u) ||
                            ((status & 0x0300u) == 0x0200u);
            dischargeEnabled = ((status & 0x0020u) != 0u) ||
                               ((status & 0x0300u) == 0x0300u);
        }
    }

    if (f314 != NULL) {
        canGrowattFormatCanData(f314->data, f314->dlc, raw314, sizeof(raw314));
    }
    if (f314 != NULL && f314->dlc >= 8u) {
        uint16_t beCycles = can_be16(&f314->data[6]);
        uint16_t leCycles = can_le16(&f314->data[6]);
        remainingCapCah = can_be16(&f314->data[0]);
        fullCapCah = can_be16(&f314->data[2]);
        cycles = (beCycles <= 20000u) ? beCycles : leCycles;
        haveCycles = true;
    }

    if (f319 != NULL) {
        canGrowattFormatCanData(f319->data, f319->dlc, raw319, sizeof(raw319));
    }
    if (f319 != NULL && f319->dlc >= 7u) {
        haveCellExtremes = canGrowattDecodeCellExtremes(f319->data,
                                                        packVoltageV,
                                                        &cellMaxMv,
                                                        &cellMinMv,
                                                        &cellMaxIdx,
                                                        &cellMinIdx);
        if (!haveStatus && f319->dlc >= 1u) {
            uint8_t flags = f319->data[0];
            chargeEnabled = (flags & 0x80u) != 0u;
            dischargeEnabled = (flags & 0x40u) != 0u;
            status = flags;
            haveStatus = (flags & 0xC0u) != 0u;
        }
    }

    if (f322 != NULL) {
        canGrowattFormatCanData(f322->data, f322->dlc, raw322, sizeof(raw322));
    }
    if (f322 != NULL && f322->dlc >= 8u) {
        int16_t tMaxDeci = canGrowattI16(&f322->data[0], littleEndian313);
        int16_t tMinDeci = canGrowattI16(&f322->data[2], littleEndian313);
        float maxC = (float)tMaxDeci / 10.0f;
        float minC = (float)tMinDeci / 10.0f;

        if (maxC < -50.0f || maxC > 120.0f || minC < -50.0f || minC > 120.0f) {
            tMaxDeci = canGrowattI16(&f322->data[0], !littleEndian313);
            tMinDeci = canGrowattI16(&f322->data[2], !littleEndian313);
            maxC = (float)tMaxDeci / 10.0f;
            minC = (float)tMinDeci / 10.0f;
        }

        if (maxC >= -50.0f && maxC <= 120.0f && minC >= -50.0f && minC <= 120.0f) {
            if (maxC >= minC) {
                tempMaxC = maxC;
                tempMinC = minC;
            } else {
                tempMaxC = minC;
                tempMinC = maxC;
            }
            haveTempRange = true;
        }

        if (!haveSoc && f322->data[6] <= 100u) {
            soc = f322->data[6];
            haveSoc = true;
        }
    }

    for (uint32_t id = GROWATT_CAN_ID_315_CELL_GRP1; id <= GROWATT_CAN_ID_318_CELL_GRP4; id++) {
        const canBmsCachedFrame_t *f = canGrowattFrameById(local, id);
        uint8_t base = (uint8_t)(((id - GROWATT_CAN_ID_315_CELL_GRP1) * 4u) + 1u);

        if (f == NULL || f->dlc < 8u) {
            continue;
        }

        for (uint8_t i = 0u; i < 4u; i++) {
            uint16_t mv = 0u;
            if (!canGrowattDecodeCellWord(&f->data[i * 2u], packVoltageV, &mv)) {
                continue;
            }
            if (snap.cellCount < (uint8_t)(base + i)) {
                snap.cellCount = (uint8_t)(base + i);
            }
            if ((uint8_t)(base + i) <= 32u) {
                snap.cellVoltagesV[base + i - 1u] = (float)mv / 1000.0f;
            }
            if (!haveCellExtremes || mv > cellMaxMv) {
                cellMaxMv = mv;
                cellMaxIdx = (uint8_t)(base + i);
            }
            if (!haveCellExtremes || cellMinMv == 0u || mv < cellMinMv) {
                cellMinMv = mv;
                cellMinIdx = (uint8_t)(base + i);
            }
            haveCellExtremes = true;
        }
    }

    if (f312 != NULL) {
        canGrowattFormatCanData(f312->data, f312->dlc, raw312, sizeof(raw312));
    }
    if (f323 != NULL) {
        canGrowattFormatCanData(f323->data, f323->dlc, raw323, sizeof(raw323));
    }

    snap.valid = havePack && haveSoc;
    if (!snap.valid) {
        return;
    }

    snprintf(snap.source, sizeof(snap.source), "%s", name);
    snprintf(snap.protocol, sizeof(snap.protocol), "CAN_GROWATT");
    snap.currentA = packCurrentA;
    snap.packVoltageV = packVoltageV;
    snap.packPowerW = packVoltageV * packCurrentA;
    snap.socPct = soc;
    snap.sohPct = haveSoh ? soh : 100u;
    snap.cycles = haveCycles ? cycles : 0u;
    snap.remainingAh = (float)remainingCapCah / 100.0f;
    snap.fullAh = (float)fullCapCah / 100.0f;
    if (haveCellExtremes) {
        snap.cellMaxV = (float)cellMaxMv / 1000.0f;
        snap.cellMinV = (float)cellMinMv / 1000.0f;
        snap.cellMaxIdx = cellMaxIdx;
        snap.cellMinIdx = cellMinIdx;
        snap.deltaV = snap.cellMaxV - snap.cellMinV;
        if (snap.cellDiffV <= 0.0f) {
            snap.cellDiffV = snap.deltaV;
        }
    }
    snap.tempMosC = haveAvgTemp ? avgTempC : 0.0f;
    snap.tempT1C = haveTempRange ? tempMaxC : (haveAvgTemp ? avgTempC : 0.0f);
    snap.tempT2C = haveTempRange ? tempMinC : (haveAvgTemp ? avgTempC : 0.0f);
    snap.tempCount = haveTempRange ? 3u : (haveAvgTemp ? 1u : 0u);
    if (haveStatus) {
        snap.pylonStatus63 = (uint8_t)(status & 0xFFu);
        snprintf(snap.stateFlags,
                 sizeof(snap.stateFlags),
                 "charge=%s, discharge=%s, balance=%s",
                 chargeEnabled ? "ON" : "OFF",
                 dischargeEnabled ? "ON" : "OFF",
                 balanceEnabled ? "ON" : "OFF");
    }
    (void)canDecoderGetGrowattAlertText(name,
                                        snap.protections,
                                        sizeof(snap.protections),
                                        snap.alarms,
                                        sizeof(snap.alarms),
                                        snap.warnings,
                                        sizeof(snap.warnings));

    model.valid = true;
    model.packVoltageV = packVoltageV;
    model.packCurrentA = packCurrentA;
    model.socPct = snap.socPct;
    model.sohPct = snap.sohPct;
    model.cycleCount = snap.cycles;
    model.chargeVoltageLimitV = haveLimits ? chargeVoltageLimitV : packVoltageV;
    model.chargeCurrentLimitA = haveLimits ? chargeCurrentLimitA : 0.0f;
    model.dischargeCurrentLimitA = haveLimits ? dischargeCurrentLimitA : 0.0f;
    if (haveCellExtremes) {
        model.cellMaxV = snap.cellMaxV;
        model.cellMinV = snap.cellMinV;
        model.cellMaxIdx = snap.cellMaxIdx;
        model.cellMinIdx = snap.cellMinIdx;
        model.cellDeltaV = snap.deltaV;
    }
    model.temperaturesC[0] = snap.tempMosC;
    model.temperaturesC[1] = snap.tempT1C;
    model.temperaturesC[2] = snap.tempT2C;
    model.chargeEnabled = chargeEnabled;
    model.dischargeEnabled = dischargeEnabled;
    model.balanceEnabled = balanceEnabled;
    model.protocolState = haveStatus ? status : 0u;
    batteryModelSet(&model);
    bridgeSetTelemetrySnapshot(&snap);

    {
        char *logText = g_canGrowattDecodedLog;
        logText[0] = '\0';
        snprintf(logText,
                 sizeof(g_canGrowattDecodedLog),
                 "CAN Growatt\n"
                 "  valid : YES\n"
                 "  pack  : V=%.2fV  I=%.1fA  avgT=%.1fC  SOC=%u%%  SOH=%u%%\n"
                 "  limits: chgV=%.1fV  chgI=%.1fA  disI=%.1fA\n"
                 "  cells : max=%.3fV#%u  min=%.3fV#%u  dV=%.3fV  count=%u\n"
                 "  temps : max=%.1fC  min=%.1fC  avg=%.1fC\n"
                 "  state : raw=0x%04X charge=%s discharge=%s balance=%s\n"
                 "  raw   : 0x311=[%s] 0x312=[%s] 0x313=[%s] 0x314=[%s] 0x319=[%s] 0x322=[%s] 0x323=[%s]\n"
                 "  note  : Growatt low-voltage CAN source feeding the universal battery model",
                 (double)packVoltageV,
                 (double)packCurrentA,
                 (double)(haveAvgTemp ? avgTempC : 0.0f),
                 (unsigned)snap.socPct,
                 (unsigned)snap.sohPct,
                 (double)(haveLimits ? chargeVoltageLimitV : 0.0f),
                 (double)(haveLimits ? chargeCurrentLimitA : 0.0f),
                 (double)(haveLimits ? dischargeCurrentLimitA : 0.0f),
                 (double)snap.cellMaxV,
                 (unsigned)snap.cellMaxIdx,
                 (double)snap.cellMinV,
                 (unsigned)snap.cellMinIdx,
                 (double)snap.deltaV,
                 (unsigned)snap.cellCount,
                 (double)(haveTempRange ? tempMaxC : 0.0f),
                 (double)(haveTempRange ? tempMinC : 0.0f),
                 (double)(haveAvgTemp ? avgTempC : 0.0f),
                 (unsigned)status,
                 chargeEnabled ? "ON" : "OFF",
                 dischargeEnabled ? "ON" : "OFF",
                 balanceEnabled ? "ON" : "OFF",
                 raw311,
                 raw312,
                 raw313,
                 raw314,
                 raw319,
                 raw322,
                 raw323);
        bridgeSetDecodedLogSnapshot(logText);
    }
}

void canDecoderPrintCachedSnapshot(const char *ifname)
{
    const char *name = (ifname != NULL) ? ifname : "CAN1";
    canBmsCachedFrame_t local[CAN_BMS_CACHE_COUNT];
    pylon_can_frame_t pylonLocal[PYLON_CAN_CACHE_COUNT];
    jkbms_can_frame_t jkbmsLocal[JKBMS_CAN_CACHE_COUNT];
    bool any = false;
    bool anyPylon = false;
    bool anyJkbms = false;

    canBmsCachedFrame_t *src = canBmsCacheForIf(name);
    pylon_can_frame_t *pylonSrc = canPylonCacheForIf(name);
    jkbms_can_frame_t *jkbmsSrc = canJkbmsCacheForIf(name);
    if (src == NULL) {
        ESP_LOGI(EXAMPLE_TAG, "CAN-%s SNAPSHOT: unsupported BMS cache interface", name);
        return;
    }

    portENTER_CRITICAL(&g_canBmsCacheMux);
    memcpy(local, src, sizeof(local));
    if (pylonSrc != NULL) {
        memcpy(pylonLocal, pylonSrc, sizeof(pylonLocal));
    } else {
        memset(pylonLocal, 0, sizeof(pylonLocal));
    }
    if (jkbmsSrc != NULL) {
        memcpy(jkbmsLocal, jkbmsSrc, sizeof(jkbmsLocal));
    } else {
        memset(jkbmsLocal, 0, sizeof(jkbmsLocal));
    }
    portEXIT_CRITICAL(&g_canBmsCacheMux);

    anyPylon = pylonCanAnyValid(pylonLocal, PYLON_CAN_CACHE_COUNT);
    if (anyPylon) {
        int protocol = canProtocolForIf(name);

        if (protocol == PROTOCOL_CAN_DEYE) {
            deyeCanDecodeSnapshot(name, pylonLocal, PYLON_CAN_CACHE_COUNT);
        } else {
            pylonCanDecodeSnapshot(name, pylonLocal, PYLON_CAN_CACHE_COUNT);
        }
        return;
    }

    anyJkbms = jkbmsCanAnyValid(jkbmsLocal, JKBMS_CAN_CACHE_COUNT);
    if (anyJkbms && canProtocolForIf(name) == PROTOCOL_CAN_JKBMS_250K) {
        jkbmsCanDecodeSnapshot(name, jkbmsLocal, JKBMS_CAN_CACHE_COUNT);
        return;
    }

    for (size_t i = 0; i < CAN_BMS_CACHE_COUNT; i++) {
        if (local[i].valid) {
            any = true;
            break;
        }
    }

    if (!any) {
        ESP_LOGI(EXAMPLE_TAG, "CAN-%s SNAPSHOT: no cached BMS frames yet", name);
        return;
    }

    ESP_LOGI(EXAMPLE_TAG, "CAN-%s SNAPSHOT BEGIN", name);

    for (size_t i = 0; i < CAN_BMS_CACHE_COUNT; i++) {
        if (!local[i].valid) continue;

        twai_message_t m = {0};
        m.identifier = local[i].id;
        m.data_length_code = local[i].dlc;
        memcpy(m.data, local[i].data, local[i].dlc);
        decodeGrowattCanFrame(name, &m);
    }

    ESP_LOGI(EXAMPLE_TAG, "CAN-%s SNAPSHOT END", name);
}


bool canDecoderTryGetSocPct(const char *ifname, uint8_t *socOut)
{
    if (socOut == NULL) {
        return false;
    }

    const char *name = (ifname != NULL) ? ifname : "CAN1";
    canBmsCachedFrame_t local[CAN_BMS_CACHE_COUNT];
    pylon_can_frame_t pylonLocal[PYLON_CAN_CACHE_COUNT];
    jkbms_can_frame_t jkbmsLocal[JKBMS_CAN_CACHE_COUNT];
    canBmsCachedFrame_t *src = canBmsCacheForIf(name);
    pylon_can_frame_t *pylonSrc = canPylonCacheForIf(name);
    jkbms_can_frame_t *jkbmsSrc = canJkbmsCacheForIf(name);
    if (src == NULL) {
        return false;
    }

    portENTER_CRITICAL(&g_canBmsCacheMux);
    memcpy(local, src, sizeof(local));
    if (pylonSrc != NULL) {
        memcpy(pylonLocal, pylonSrc, sizeof(pylonLocal));
    } else {
        memset(pylonLocal, 0, sizeof(pylonLocal));
    }
    if (jkbmsSrc != NULL) {
        memcpy(jkbmsLocal, jkbmsSrc, sizeof(jkbmsLocal));
    } else {
        memset(jkbmsLocal, 0, sizeof(jkbmsLocal));
    }
    portEXIT_CRITICAL(&g_canBmsCacheMux);

    if (canProtocolForIf(name) == PROTOCOL_CAN_JKBMS_250K &&
        jkbmsCanTryGetSocPct(jkbmsLocal, JKBMS_CAN_CACHE_COUNT, socOut)) {
        return true;
    }

    {
        int idx355 = (int)(0x355u - PYLON_CAN_ID_MIN);
        if (idx355 >= 0 && (size_t)idx355 < PYLON_CAN_CACHE_COUNT &&
            pylonLocal[idx355].valid && pylonLocal[idx355].dlc >= 2u) {
            uint16_t soc = can_le16(&pylonLocal[idx355].data[0]);
            if (soc <= 100u) {
                *socOut = (uint8_t)soc;
                return true;
            }
        }
    }

    int idx313 = canBmsCacheIndex(GROWATT_CAN_ID_313_V_I_SOC_SOH);
    if (idx313 >= 0 && local[idx313].valid && local[idx313].dlc >= 7u) {
        uint8_t soc = local[idx313].data[6];
        if (soc <= 100u) {
            *socOut = soc;
            return true;
        }
    }

    int idx322 = canBmsCacheIndex(GROWATT_CAN_ID_322_TEMP_SOC_MIN_MAX);
    if (idx322 >= 0 && local[idx322].valid && local[idx322].dlc >= 8u) {
        uint8_t socMax = local[idx322].data[6];
        uint8_t socMin = local[idx322].data[7];
        uint8_t soc = (socMax <= 100u) ? socMax : socMin;
        if (soc <= 100u) {
            *socOut = soc;
            return true;
        }
    }

    return false;
}

bool canDecoderHasFreshData(const char *ifname, uint32_t maxAgeMs)
{
    const char *name = (ifname != NULL) ? ifname : "CAN1";
    canBmsCachedFrame_t local[CAN_BMS_CACHE_COUNT];
    pylon_can_frame_t pylonLocal[PYLON_CAN_CACHE_COUNT];
    jkbms_can_frame_t jkbmsLocal[JKBMS_CAN_CACHE_COUNT];
    canBmsCachedFrame_t *src = canBmsCacheForIf(name);
    pylon_can_frame_t *pylonSrc = canPylonCacheForIf(name);
    jkbms_can_frame_t *jkbmsSrc = canJkbmsCacheForIf(name);
    uint32_t nowMs = (uint32_t)(esp_timer_get_time() / 1000LL);

    if (src == NULL) {
        return false;
    }

    portENTER_CRITICAL(&g_canBmsCacheMux);
    memcpy(local, src, sizeof(local));
    if (pylonSrc != NULL) {
        memcpy(pylonLocal, pylonSrc, sizeof(pylonLocal));
    } else {
        memset(pylonLocal, 0, sizeof(pylonLocal));
    }
    if (jkbmsSrc != NULL) {
        memcpy(jkbmsLocal, jkbmsSrc, sizeof(jkbmsLocal));
    } else {
        memset(jkbmsLocal, 0, sizeof(jkbmsLocal));
    }
    portEXIT_CRITICAL(&g_canBmsCacheMux);

    for (size_t i = 0; i < JKBMS_CAN_CACHE_COUNT; i++) {
        if (jkbmsLocal[i].valid && (nowMs - jkbmsLocal[i].updatedMs) <= maxAgeMs) {
            return true;
        }
    }

    for (size_t i = 0; i < PYLON_CAN_CACHE_COUNT; i++) {
        if (pylonLocal[i].valid && (nowMs - pylonLocal[i].updatedMs) <= maxAgeMs) {
            return true;
        }
    }

    for (size_t i = 0; i < CAN_BMS_CACHE_COUNT; i++) {
        if (local[i].valid && (nowMs - local[i].updatedMs) <= maxAgeMs) {
            return true;
        }
    }

    return false;
}

bool canDecoderGetGrowattAlertText(const char *ifname,
                                   char *protectionsOut,
                                   uint32_t protectionsOutSize,
                                   char *alarmsOut,
                                   uint32_t alarmsOutSize,
                                   char *warningsOut,
                                   uint32_t warningsOutSize)
{
    const char *name = (ifname != NULL) ? ifname : "CAN1";
    canBmsCachedFrame_t local[CAN_BMS_CACHE_COUNT];
    canBmsCachedFrame_t *src = canBmsCacheForIf(name);
    int idx312 = 0;
    int idx323 = 0;
    bool any = false;

    if (protectionsOut != NULL && protectionsOutSize > 0u) {
        protectionsOut[0] = '\0';
    }
    if (alarmsOut != NULL && alarmsOutSize > 0u) {
        alarmsOut[0] = '\0';
    }
    if (warningsOut != NULL && warningsOutSize > 0u) {
        warningsOut[0] = '\0';
    }

    if (src == NULL) {
        return false;
    }

    portENTER_CRITICAL(&g_canBmsCacheMux);
    memcpy(local, src, sizeof(local));
    portEXIT_CRITICAL(&g_canBmsCacheMux);

    idx312 = canBmsCacheIndex(GROWATT_CAN_ID_312_PROT_ALM);
    if (idx312 >= 0 && local[idx312].valid && local[idx312].dlc >= 4u) {
        appendAlertBitsByCategory(protectionsOut, protectionsOutSize,
                                  alarmsOut, alarmsOutSize,
                                  warningsOut, warningsOutSize,
                                  local[idx312].data[0], k312Prot1Bits);
        appendAlertBitsByCategory(protectionsOut, protectionsOutSize,
                                  alarmsOut, alarmsOutSize,
                                  warningsOut, warningsOutSize,
                                  local[idx312].data[1], k312Prot2Bits);
        appendAlertBitsByCategory(protectionsOut, protectionsOutSize,
                                  alarmsOut, alarmsOutSize,
                                  warningsOut, warningsOutSize,
                                  local[idx312].data[2], k312Alm1Bits);
        appendAlertBitsByCategory(protectionsOut, protectionsOutSize,
                                  alarmsOut, alarmsOutSize,
                                  warningsOut, warningsOutSize,
                                  local[idx312].data[3], k312Alm2Bits);
        any = true;
    }

    idx323 = canBmsCacheIndex(GROWATT_CAN_ID_323_CELLCOUNT_PROT_WARN);
    if (idx323 >= 0 && local[idx323].valid && local[idx323].dlc >= 8u) {
        appendAlertBitsByCategory(protectionsOut, protectionsOutSize,
                                  alarmsOut, alarmsOutSize,
                                  warningsOut, warningsOutSize,
                                  local[idx323].data[4], k323Prot3Bits);
        appendAlertBitsByCategory(protectionsOut, protectionsOutSize,
                                  alarmsOut, alarmsOutSize,
                                  warningsOut, warningsOutSize,
                                  local[idx323].data[5], k323Prot4Bits);
        appendAlertBitsByCategory(protectionsOut, protectionsOutSize,
                                  alarmsOut, alarmsOutSize,
                                  warningsOut, warningsOutSize,
                                  local[idx323].data[6], k323Prot5Bits);
        appendAlertBitsByCategory(protectionsOut, protectionsOutSize,
                                  alarmsOut, alarmsOutSize,
                                  warningsOut, warningsOutSize,
                                  local[idx323].data[7], k323Warn3Bits);
        any = true;
    }

    return any;
}

void canDecoderOnFrame(const char *ifname, const twai_message_t *m)
{
    if (m == NULL) return;

    canBmsCacheUpdate(ifname, m);
    canPylonCacheUpdate(ifname, m);
    canJkbmsCacheUpdate(ifname, m);
    canUpdateUniversalModelFromGrowattCache(ifname);
    canUpdateUniversalModelFromPylonCache(ifname);
    canUpdateUniversalModelFromJkbmsCache(ifname);

    if (CAN_DECODER_SHOW_RAW_FRAMES) {
        logRawCanMsg(ifname, m);
    }

#if CAN_DECODER_IMMEDIATE_DECODE_LOG_ENABLE
    decodeGrowattCanFrame(ifname, m);
#endif
}

void canDecoderResetCaches(void)
{
    portENTER_CRITICAL(&g_canBmsCacheMux);
    memset(g_can1BmsCache, 0, sizeof(g_can1BmsCache));
    memset(g_can2BmsCache, 0, sizeof(g_can2BmsCache));
    memset(g_can1PylonCache, 0, sizeof(g_can1PylonCache));
    memset(g_can2PylonCache, 0, sizeof(g_can2PylonCache));
    memset(g_can1JkbmsCache, 0, sizeof(g_can1JkbmsCache));
    memset(g_can2JkbmsCache, 0, sizeof(g_can2JkbmsCache));
    portEXIT_CRITICAL(&g_canBmsCacheMux);
}

