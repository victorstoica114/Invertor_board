#include "CAN_Decoder.h"

#include "config.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

bool g_canDecoderShowRawFrames = 0;

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
    case 0x311: {
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

    case 0x312:
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

    case 0x313: {
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

    case 0x314: {
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

    case 0x315:
    case 0x316:
    case 0x317:
    case 0x318: {
        /* Optional frame per PDF; some batteries do not send these. */
        const unsigned base = (unsigned)((id - 0x315u) * 4u + 1u);
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

    case 0x319: {
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

    case 0x320: {
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

    case 0x321: {
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

    case 0x322: {
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

    case 0x323:
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

void canDecoderOnFrame(const char *ifname, const twai_message_t *m)
{
    if (m == NULL) return;

    if (g_canDecoderShowRawFrames) {
        logRawCanMsg(ifname, m);
    }

    decodeGrowattCanFrame(ifname, m);
}
