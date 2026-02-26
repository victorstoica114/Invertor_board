// canGrowattDecoder.c
#include "canGrowattDecoder.h"

#include <string.h>
#include <stdio.h>

#include "esp_log.h"

#ifndef EXAMPLE_TAG
#define EXAMPLE_TAG "SNIFFER_BRIDGE"
#endif

/* Prag “plauzibil” pentru celule Li-ion (mV). Ajustează dacă ai LFP etc. */
#define CELL_MV_MIN 4100u
#define CELL_MV_MAX 4500u

static inline uint16_t be16(const uint8_t *p)
{
    return (uint16_t)((((uint16_t)p[0]) << 8) | (uint16_t)p[1]);
}

static inline uint16_t le16(const uint8_t *p)
{
    return (uint16_t)((((uint16_t)p[1]) << 8) | (uint16_t)p[0]);
}

static inline int16_t be16s(const uint8_t *p)
{
    return (int16_t)be16(p);
}

static inline int16_t le16s(const uint8_t *p)
{
    return (int16_t)le16(p);
}

static inline bool mvPlausible(uint16_t mv)
{
    return (mv >= (uint16_t)CELL_MV_MIN) && (mv <= (uint16_t)CELL_MV_MAX);
}

/* Alege BE/LE “smart” per pereche de 2 bytes, pe baza plauzibilității (2000..5000mV). */
static uint16_t decodeCellmVSmart(const uint8_t *p)
{
    const uint16_t mvBe = be16(p);
    const uint16_t mvLe = le16(p);

    const bool okBe = mvPlausible(mvBe);
    const bool okLe = mvPlausible(mvLe);

    if (okBe && !okLe) return mvBe;
    if (okLe && !okBe) return mvLe;

    /* dacă ambele sunt ok (rar), prefer LE (în practica ta ai văzut LE des) */
    if (okLe && okBe) return mvLe;

    /* nimic plauzibil */
    return 0u;
}

static void growattCellsCompute(canGrowattState_t *s)
{
    if (s->haveCellMask != 0xFFFFu) {
        s->haveCells = false;
        return;
    }

    float vmin = 999.0f, vmax = 0.0f, sum = 0.0f;
    uint8_t imin = 0u, imax = 0u;

    for (uint8_t i = 0; i < 16u; i++) {
        const float v = s->cellV[i];
        sum += v;
        if (v < vmin) { vmin = v; imin = i; }
        if (v > vmax) { vmax = v; imax = i; }
    }

    s->cellAvgV     = sum / 16.0f;
    s->cellMinV     = vmin;
    s->cellMaxV     = vmax;
    s->cellMinIndex = imin;
    s->cellMaxIndex = imax;
    s->cellDeltaV   = vmax - vmin; /* dacă ai deja delta din 0x314, asta o suprascrie când ai celule reale */
    s->haveCells    = true;
}

/* Print scurt când vin frame-uri de celule (debug “partial”). */
static void printCellsPartialLine(const canGrowattState_t *s)
{
    /* afișează doar primele 4 celule din “slotul” nou completat, ca să nu spamezi */
    /* (tu deja vezi ceva de genul: C01=... C02=... C04=...) */
    char buf[160];
    int pos = 0;

    pos += snprintf(buf + pos, sizeof(buf) - (size_t)pos, "CAN-%s CELLS:", s->ifName);

    for (uint8_t i = 0; i < 16u; i++) {
        if ((s->haveCellMask & (1u << i)) == 0u) continue;
        pos += snprintf(buf + pos, sizeof(buf) - (size_t)pos,
                        " C%02u=%.3fV", (unsigned)(i + 1u), (double)s->cellV[i]);
        /* limitează la 4 valori per linie */
        if (i >= 3u) break;
    }

    ESP_LOGI(EXAMPLE_TAG, "%s", buf);
}

/* Print “full line” cu toate celulele (doar când sunt toate 16). */
static void printCellsLine(const canGrowattState_t *s)
{
    char buf[320];
    int pos = 0;

    pos += snprintf(buf + pos, sizeof(buf) - (size_t)pos, "CAN-%s CELLS:", s->ifName);

    for (uint8_t i = 0; i < 16u; i++) {
        pos += snprintf(buf + pos, sizeof(buf) - (size_t)pos,
                        " C%02u=%.3fV", (unsigned)(i + 1u), (double)s->cellV[i]);
        if (pos >= (int)sizeof(buf) - 16) break;
    }

    ESP_LOGI(EXAMPLE_TAG, "%s", buf);
}

static void printCompact(const canGrowattState_t *s)
{
    if (!s->have313 || !s->have314) return;

    if (s->haveCells) {
        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s BMS: %.2fV | %+0.1fA | SOC %u%% | T %dC | Cap %.1fAh | Rem %.1fAh | "
                 "Cmin %.3fV(C%u) | Cmax %.3fV(C%u) | Avg %.3fV | \xCE\x94 %.3fV",
                 s->ifName,
                 (double)s->packVoltageV,
                 (double)s->packCurrentA,
                 (unsigned)s->socPct,
                 (int)s->packTempC,
                 (double)s->capacityAh,
                 (double)s->remainAh,
                 (double)s->cellMinV, (unsigned)(s->cellMinIndex + 1u),
                 (double)s->cellMaxV, (unsigned)(s->cellMaxIndex + 1u),
                 (double)s->cellAvgV,
                 (double)s->cellDeltaV);
    } else {
        /* fallback: packV/16 + delta din 0x314 (aprox) */
        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s BMS: %.2fV | %+0.1fA | SOC %u%% | T %dC | Cap %.1fAh | Rem %.1fAh | "
                 "Avg %.3fV | Cmin~ %.3fV | Cmax~ %.3fV | \xCE\x94 %.3fV",
                 s->ifName,
                 (double)s->packVoltageV,
                 (double)s->packCurrentA,
                 (unsigned)s->socPct,
                 (int)s->packTempC,
                 (double)s->capacityAh,
                 (double)s->remainAh,
                 (double)s->avgCellV,
                 (double)(s->avgCellV - s->cellDeltaV * 0.5f),
                 (double)(s->avgCellV + s->cellDeltaV * 0.5f),
                 (double)s->cellDeltaV);
    }
}

static bool pickCellFrameEndian(const canGrowattState_t *s, const uint8_t *d, uint16_t outMv[4]);

/* Decodează 4 celule / frame: 8 bytes => 4x uint16 */
static void decodeCells4Frame(canGrowattState_t *s, uint8_t frameIndex, const uint8_t *d)
{
    const uint8_t start = (uint8_t)(frameIndex * 4u);   /* 0,4,8,12 */
    uint16_t mvFrame[4];

    /* Accept frame-ul doar daca toate cele 4 valori sunt coerente intr-un singur endianness. */
    if (!pickCellFrameEndian(s, d, mvFrame)) {
        return;
    }

    for (uint8_t i = 0; i < 4u; i++) {
        const uint8_t idx = (uint8_t)(start + i);
        s->cellV[idx] = (float)mvFrame[i] / 1000.0f;
        s->haveCellMask |= (1u << idx);
    }

    {
        char buf[200];
        int pos = 0;
        pos += snprintf(buf + pos, sizeof(buf) - (size_t)pos, "CAN-%s CELLS:", s->ifName);
        for (uint8_t i = 0; i < 4u; i++) {
            const uint8_t idx = (uint8_t)(start + i);
            pos += snprintf(buf + pos, sizeof(buf) - (size_t)pos,
                            " C%02u=%.3fV", (unsigned)(idx + 1u), (double)s->cellV[idx]);
        }
        ESP_LOGI(EXAMPLE_TAG, "%s", buf);
    }

    if (s->haveCellMask == 0xFFFFu) {
        growattCellsCompute(s);
        printCellsLine(s);
    }
}

void canGrowattInit(canGrowattState_t *s, const char *ifName)
{
    memset(s, 0, sizeof(*s));
    s->ifName = ifName;
}

static bool all4PlausibleBE(const uint8_t *d, uint16_t outMv[4])
{
    for (int i = 0; i < 4; i++) {
        outMv[i] = be16(&d[i * 2]);
        if (!mvPlausible(outMv[i])) return false;
    }
    return true;
}

static bool all4PlausibleLE(const uint8_t *d, uint16_t outMv[4])
{
    for (int i = 0; i < 4; i++) {
        outMv[i] = le16(&d[i * 2]);
        if (!mvPlausible(outMv[i])) return false;
    }
    return true;
}

static float absf_local(float x)
{
    return (x < 0.0f) ? -x : x;
}

static bool looksLike4CellsFrame(const uint8_t *d)
{
    uint16_t tmp[4];
    return all4PlausibleBE(d, tmp) || all4PlausibleLE(d, tmp);
}

static bool pickCellFrameEndian(const canGrowattState_t *s, const uint8_t *d, uint16_t outMv[4])
{
    uint16_t beMv[4], leMv[4];
    const bool beAll = all4PlausibleBE(d, beMv);
    const bool leAll = all4PlausibleLE(d, leMv);

    if (beAll && !leAll) {
        memcpy(outMv, beMv, sizeof(beMv));
        return true;
    }
    if (leAll && !beAll) {
        memcpy(outMv, leMv, sizeof(leMv));
        return true;
    }
    if (!beAll && !leAll) {
        return false;
    }

    if (s->have313) {
        float beScore = 0.0f;
        float leScore = 0.0f;
        for (int i = 0; i < 4; i++) {
            beScore += absf_local(((float)beMv[i] / 1000.0f) - s->avgCellV);
            leScore += absf_local(((float)leMv[i] / 1000.0f) - s->avgCellV);
        }
        if (leScore <= beScore) memcpy(outMv, leMv, sizeof(leMv));
        else                    memcpy(outMv, beMv, sizeof(beMv));
        return true;
    }

    memcpy(outMv, leMv, sizeof(leMv));
    return true;
}

static void logReverse319320(const canGrowattState_t *s, uint32_t canId, const uint8_t *d)
{
    /* Common 12-bit packed hypotheses (both byte orders) for reverse-engineering. */
    uint16_t p12be0[4] = {0}, p12le0[4] = {0};
    uint16_t p12be2[4] = {0}, p12le2[4] = {0};

    /* offset 0: bytes 0..5 => 4x12-bit */
    p12be0[0] = (uint16_t)(((uint16_t)d[0] << 4) | (uint16_t)(d[1] >> 4));
    p12be0[1] = (uint16_t)((((uint16_t)d[1] & 0x0Fu) << 8) | (uint16_t)d[2]);
    p12be0[2] = (uint16_t)(((uint16_t)d[3] << 4) | (uint16_t)(d[4] >> 4));
    p12be0[3] = (uint16_t)((((uint16_t)d[4] & 0x0Fu) << 8) | (uint16_t)d[5]);

    p12le0[0] = (uint16_t)((uint16_t)d[0] | (((uint16_t)d[1] & 0x0Fu) << 8));
    p12le0[1] = (uint16_t)(((uint16_t)(d[1] >> 4)) | ((uint16_t)d[2] << 4));
    p12le0[2] = (uint16_t)((uint16_t)d[3] | (((uint16_t)d[4] & 0x0Fu) << 8));
    p12le0[3] = (uint16_t)(((uint16_t)(d[4] >> 4)) | ((uint16_t)d[5] << 4));

    /* offset 2: bytes 2..7 => 4x12-bit */
    p12be2[0] = (uint16_t)(((uint16_t)d[2] << 4) | (uint16_t)(d[3] >> 4));
    p12be2[1] = (uint16_t)((((uint16_t)d[3] & 0x0Fu) << 8) | (uint16_t)d[4]);
    p12be2[2] = (uint16_t)(((uint16_t)d[5] << 4) | (uint16_t)(d[6] >> 4));
    p12be2[3] = (uint16_t)((((uint16_t)d[6] & 0x0Fu) << 8) | (uint16_t)d[7]);

    p12le2[0] = (uint16_t)((uint16_t)d[2] | (((uint16_t)d[3] & 0x0Fu) << 8));
    p12le2[1] = (uint16_t)(((uint16_t)(d[3] >> 4)) | ((uint16_t)d[4] << 4));
    p12le2[2] = (uint16_t)((uint16_t)d[5] | (((uint16_t)d[6] & 0x0Fu) << 8));
    p12le2[3] = (uint16_t)(((uint16_t)(d[6] >> 4)) | ((uint16_t)d[7] << 4));

    ESP_LOGI(EXAMPLE_TAG,
             "CAN-%s 0x%03X RE: b=[%02X %02X %02X %02X %02X %02X %02X %02X] nH=[%X %X %X %X %X %X %X %X] nL=[%X %X %X %X %X %X %X %X]",
             s->ifName, (unsigned)canId,
             d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7],
             d[0] >> 4, d[1] >> 4, d[2] >> 4, d[3] >> 4, d[4] >> 4, d[5] >> 4, d[6] >> 4, d[7] >> 4,
             d[0] & 0x0F, d[1] & 0x0F, d[2] & 0x0F, d[3] & 0x0F, d[4] & 0x0F, d[5] & 0x0F, d[6] & 0x0F, d[7] & 0x0F);

    ESP_LOGI(EXAMPLE_TAG,
             "CAN-%s 0x%03X RE12 o0 BE[%u %u %u %u] LE[%u %u %u %u] | o2 BE[%u %u %u %u] LE[%u %u %u %u]",
             s->ifName, (unsigned)canId,
             (unsigned)p12be0[0], (unsigned)p12be0[1], (unsigned)p12be0[2], (unsigned)p12be0[3],
             (unsigned)p12le0[0], (unsigned)p12le0[1], (unsigned)p12le0[2], (unsigned)p12le0[3],
             (unsigned)p12be2[0], (unsigned)p12be2[1], (unsigned)p12be2[2], (unsigned)p12be2[3],
             (unsigned)p12le2[0], (unsigned)p12le2[1], (unsigned)p12le2[2], (unsigned)p12le2[3]);

    if (canId == 0x319u) {
        const uint16_t w0le = le16(&d[0]);
        const uint16_t w1le = le16(&d[2]);
        const uint16_t w2le = le16(&d[4]);
        const uint16_t w3le = le16(&d[6]);
        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s 0x319 RE fields: wLE=[%u %u %u %u] w2+3200=%u w2+3300=%u b4=%u b5=%u b6=%u b7=%u",
                 s->ifName,
                 (unsigned)w0le, (unsigned)w1le, (unsigned)w2le, (unsigned)w3le,
                 (unsigned)(w2le + 3200u), (unsigned)(w2le + 3300u),
                 (unsigned)d[4], (unsigned)d[5], (unsigned)d[6], (unsigned)d[7]);
    } else if (canId == 0x320u) {
        const uint16_t w1be = be16(&d[2]);
        const uint16_t w2be = be16(&d[4]);
        const uint16_t w3be = be16(&d[6]);
        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s 0x320 RE tailBE=[%u %u %u] tailBE+320=[%u %u %u] tailBE+400=[%u %u %u]",
                 s->ifName,
                 (unsigned)w1be, (unsigned)w2be, (unsigned)w3be,
                 (unsigned)(w1be + 320u), (unsigned)(w2be + 320u), (unsigned)(w3be + 320u),
                 (unsigned)(w1be + 400u), (unsigned)(w2be + 400u), (unsigned)(w3be + 400u));
    }
}

static bool metaFrameChanged319_323(canGrowattState_t *s, uint32_t canId, const uint8_t *d)
{
    if (canId < 0x319u || canId > 0x323u) return true;
    const uint8_t slot = (uint8_t)(canId - 0x319u);
    const uint8_t mask = (uint8_t)(1u << slot);
    if ((s->haveMetaFrameMask319_323 & mask) != 0u && memcmp(s->metaFrame319_323[slot], d, 8) == 0) {
        return false;
    }
    memcpy(s->metaFrame319_323[slot], d, 8);
    s->haveMetaFrameMask319_323 |= mask;
    return true;
}

static void logMeta319(canGrowattState_t *s, const uint8_t *d)
{
    const uint16_t w0 = le16(&d[0]);
    const uint16_t w1 = le16(&d[2]);
    const uint16_t w2 = le16(&d[4]);
    const uint16_t w3 = le16(&d[6]);
    ESP_LOGI(EXAMPLE_TAG,
             "CAN-%s 0x319 META?: thHi=%umV thLo=%umV x=%u (x+3200=%u) idx/min?=%u",
             s->ifName, (unsigned)w0, (unsigned)w1, (unsigned)w2, (unsigned)(w2 + 3200u), (unsigned)w3);
}

static void logMeta320(canGrowattState_t *s, const uint8_t *d)
{
    const uint16_t w0 = be16(&d[0]);
    const uint16_t w1 = be16(&d[2]);
    const uint16_t w2 = be16(&d[4]);
    const uint16_t w3 = be16(&d[6]);
    ESP_LOGI(EXAMPLE_TAG,
             "CAN-%s 0x320 META?: BE=[%u %u %u %u] +320=[%u %u %u] +400=[%u %u %u]",
             s->ifName,
             (unsigned)w0, (unsigned)w1, (unsigned)w2, (unsigned)w3,
             (unsigned)(w1 + 320u), (unsigned)(w2 + 320u), (unsigned)(w3 + 320u),
             (unsigned)(w1 + 400u), (unsigned)(w2 + 400u), (unsigned)(w3 + 400u));
}

static void logMeta321(canGrowattState_t *s, const uint8_t *d)
{
    bool allZero = true;
    for (int i = 0; i < 8; i++) { if (d[i] != 0) { allZero = false; break; } }
    if (allZero) {
        ESP_LOGI(EXAMPLE_TAG, "CAN-%s 0x321 META: all-zero/reserved", s->ifName);
    } else {
        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s 0x321 META: raw=[%02X %02X %02X %02X %02X %02X %02X %02X]",
                 s->ifName, d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
    }
}

static void logMeta322(canGrowattState_t *s, const uint8_t *d)
{
    const int16_t t1_0p1 = be16s(&d[0]);
    const uint16_t x0000 = be16(&d[2]);
    const int16_t t2_0p1 = be16s(&d[4]);
    ESP_LOGI(EXAMPLE_TAG,
             "CAN-%s 0x322 META: T1=%.1fC X=0x%04X T2=%.1fC P1=%u P2=%u",
             s->ifName,
             (double)((float)t1_0p1 / 10.0f),
             (unsigned)x0000,
             (double)((float)t2_0p1 / 10.0f),
             (unsigned)d[6], (unsigned)d[7]);
}

static void logMeta323(canGrowattState_t *s, const uint8_t *d)
{
    ESP_LOGI(EXAMPLE_TAG,
             "CAN-%s 0x323 META: cellCount=%u rawTail=[%02X %02X %02X %02X %02X %02X %02X]",
             s->ifName, (unsigned)d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
}

static void logUnknownCellLikeFrame(canGrowattState_t *s, uint32_t canId, const uint8_t *d)
{
    uint16_t beMv[4] = {0}, leMv[4] = {0};
    int beOk = 0, leOk = 0;

    for (int i = 0; i < 4; i++) {
        beMv[i] = be16(&d[i * 2]);
        leMv[i] = le16(&d[i * 2]);
        if (mvPlausible(beMv[i])) beOk++;
        if (mvPlausible(leMv[i])) leOk++;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "CAN-%s 0x%03X RAW cell-like? BE[%u %u %u %u] ok=%d LE[%u %u %u %u] ok=%d",
             s->ifName,
             (unsigned)canId,
             (unsigned)beMv[0], (unsigned)beMv[1], (unsigned)beMv[2], (unsigned)beMv[3], beOk,
             (unsigned)leMv[0], (unsigned)leMv[1], (unsigned)leMv[2], (unsigned)leMv[3], leOk);

    if (canId == 0x319u || canId == 0x320u) {
        logReverse319320(s, canId, d);
    }
}

void canGrowattOnFrame(canGrowattState_t *s, uint32_t canId, const uint8_t *d, int dlc)
{
    
    if (dlc != 8 || d == NULL) return;

    switch (canId) {

    /* 0x311: status + CV + current limits (BE u16, scale 0.1) */
    case 0x311: {
        s->statusBits       = be16(&d[0]);
        s->recChargeCvV     = (float)be16(&d[2]) / 10.0f;
        s->chgCurrentLimitA = (float)be16(&d[4]) / 10.0f;
        s->disCurrentLimitA = (float)be16(&d[6]) / 10.0f;
        s->have311 = true;

        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s 0x311: status=0x%04X CV=%.1fV IchgLim=%.1fA IdisLim=%.1fA",
                 s->ifName,
                 (unsigned)s->statusBits,
                 (double)s->recChargeCvV,
                 (double)s->chgCurrentLimitA,
                 (double)s->disCurrentLimitA);
        break;
    }

    /* 0x313: pack V (0.01V), pack I (0.1A signed), temp (0.1C signed), SOC, SOH */
    case 0x313: {
        const uint16_t v_0p01 = be16(&d[0]);
        const int16_t  i_0p1  = be16s(&d[2]);
        const int16_t  t_0p1  = be16s(&d[4]);

        s->packVoltageV = (float)v_0p01 / 100.0f;
        s->packCurrentA = (float)i_0p1  / 10.0f;

        /* rotunjire către int8_t */
        const float tC = (float)t_0p1 / 10.0f;
        int tRounded = (int)(tC >= 0.0f ? (tC + 0.5f) : (tC - 0.5f));
        if (tRounded > 127) tRounded = 127;
        if (tRounded < -128) tRounded = -128;
        s->packTempC = (int8_t)tRounded;

        s->socPct = d[6];
        s->sohPct = d[7];
        s->have313 = true;

        /* fallback avg celulă (16S) */
        s->avgCellV = s->packVoltageV / 16.0f;

        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s 0x313: V=%.2fV I=%+.1fA T=%dC SOC=%u%% SOH=%u%%",
                 s->ifName,
                 (double)s->packVoltageV,
                 (double)s->packCurrentA,
                 (int)s->packTempC,
                 (unsigned)s->socPct,
                 (unsigned)s->sohPct);

        /* dacă deja avem 0x314, printează compact */
        printCompact(s);
        break;
    }

    /* 0x314: remainAh (0.01Ah), capacityAh (0.01Ah), cellDelta(mV) */
    case 0x314: {
        const uint16_t rem_0p01 = be16(&d[0]);
        const uint16_t cap_0p01 = be16(&d[2]);
        const uint16_t dv_mV    = be16(&d[4]);

        s->remainAh   = (float)rem_0p01 / 100.0f;
        s->capacityAh = (float)cap_0p01 / 100.0f;
        s->cellDeltaV = (float)dv_mV / 1000.0f;
        s->have314 = true;

        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s 0x314: Rem=%.2fAh Cap=%.2fAh dV=%umV",
                 s->ifName,
                 (double)s->remainAh,
                 (double)s->capacityAh,
                 (unsigned)dv_mV);

        printCompact(s);
        break;
    }

        /* Per-cell 16S: standard 0x315..0x318 */
        case 0x315:
        case 0x316:
        case 0x317:
        case 0x318:
            decodeCells4Frame(s, (uint8_t)(canId - 0x315u), d);
            break;

        /* Variant observat: 0x319..0x31C */
        case 0x319:
            if (looksLike4CellsFrame(d)) {
                decodeCells4Frame(s, 0u, d);
            } else {
                if (metaFrameChanged319_323(s, canId, d)) logMeta319(s, d);
                logUnknownCellLikeFrame(s, canId, d);
            }
            break;
        case 0x31A:
        case 0x31B:
        case 0x31C:
            if (looksLike4CellsFrame(d)) decodeCells4Frame(s, (uint8_t)(canId - 0x319u), d);
            else                        logUnknownCellLikeFrame(s, canId, d);
            break;
        /* Altă variantă observată în log-urile tale: 0x319,0x320,0x321,0x322 (4 frame-uri / 16 celule)
        Notă: 0x323 apare separat și NU pare să fie celule (îl ignorăm). */
        case 0x320: {
            if (looksLike4CellsFrame(d)) {
                decodeCells4Frame(s, 1u, d);
            } else {
                if (metaFrameChanged319_323(s, canId, d)) logMeta320(s, d);
                logUnknownCellLikeFrame(s, canId, d);
            }
            break;
        }

        case 0x321:
            if (metaFrameChanged319_323(s, canId, d)) logMeta321(s, d);
            logUnknownCellLikeFrame(s, canId, d);
            break;

        case 0x322:
            if (metaFrameChanged319_323(s, canId, d)) logMeta322(s, d);
            logUnknownCellLikeFrame(s, canId, d);
            break;

        case 0x323:
            if (metaFrameChanged319_323(s, canId, d)) logMeta323(s, d);
            logUnknownCellLikeFrame(s, canId, d);
            break;

        default: {
            /* caută candidați “cells”: 4 cuvinte în 4100..4500mV */
            uint16_t mv[4];

            if (all4PlausibleBE(d, mv)) {
                ESP_LOGI(EXAMPLE_TAG, "CAN-%s CAND 0x%03X BE: %u %u %u %u mV",
                        s->ifName, (unsigned)canId, (unsigned)mv[0], (unsigned)mv[1], (unsigned)mv[2], (unsigned)mv[3]);
            } else if (all4PlausibleLE(d, mv)) {
                ESP_LOGI(EXAMPLE_TAG, "CAN-%s CAND 0x%03X LE: %u %u %u %u mV",
                        s->ifName, (unsigned)canId, (unsigned)mv[0], (unsigned)mv[1], (unsigned)mv[2], (unsigned)mv[3]);
            }
            break;
        }

    }
}
