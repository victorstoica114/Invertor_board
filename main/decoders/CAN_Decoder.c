#include "decoders/CAN_Decoder.h"

#include "config.h"
#include "Web_interface/web_bridge_api.h"
#include "protocols/common/battery_model.h"
#include "protocols/growatt/growatt_registers_map.h"
#include "protocols/deye/deye_can_protocol.h"
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

static inline uint16_t can_le16(const uint8_t *p);
static inline int16_t can_le16s(const uint8_t *p);

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
    pylon_can_frame_t local[PYLON_CAN_CACHE_COUNT];
    pylon_can_frame_t *src = NULL;
    const pylon_can_frame_t *f351 = NULL;
    const pylon_can_frame_t *f355 = NULL;
    const pylon_can_frame_t *f356 = NULL;
    const pylon_can_frame_t *f35C = NULL;
    const pylon_can_frame_t *f373 = NULL;
    bool haveSoc = false;
    bool havePack = false;
    universal_battery_model_t model = {0};
    uint32_t nowMs = (uint32_t)(esp_timer_get_time() / 1000LL);

    if (protocol != PROTOCOL_CAN_PYLON) {
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

    if (canPylonGetFrameById(local, 0x356u, &f356) && f356->dlc >= 6u) {
        model.packVoltageV = (float)can_le16(&f356->data[0]) / 100.0f;
        model.packCurrentA = (float)can_le16s(&f356->data[2]) / 10.0f;
        model.temperaturesC[0] = (float)can_le16(&f356->data[4]) / 10.0f;
        havePack = true;
    }

    if (canPylonGetFrameById(local, 0x373u, &f373) && f373->dlc >= 8u) {
        model.cellMinV = (float)can_le16(&f373->data[0]) / 1000.0f;
        model.cellMaxV = (float)can_le16(&f373->data[2]) / 1000.0f;
        model.cellDeltaV = model.cellMaxV - model.cellMinV;
        model.temperaturesC[1] = (float)can_le16(&f373->data[4]) / 10.0f;
        model.temperaturesC[2] = (float)can_le16(&f373->data[6]) / 10.0f;
    }

    if (canPylonGetFrameById(local, 0x35Cu, &f35C) && f35C->dlc >= 1u) {
        uint8_t status = f35C->data[0];
        model.protocolState = status;
        model.chargeEnabled = (status & 0x80u) != 0u;
        model.dischargeEnabled = (status & 0x40u) != 0u;
        model.balanceEnabled = (status & 0x20u) != 0u;
    }

    if (canPylonGetFrameById(local, 0x351u, &f351) && f351->dlc >= 6u) {
        model.chargeVoltageLimitV = (float)can_le16(&f351->data[0]) / 10.0f;
        model.chargeCurrentLimitA = (float)can_le16(&f351->data[2]) / 10.0f;
        model.dischargeCurrentLimitA = (float)can_le16(&f351->data[4]) / 10.0f;
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

