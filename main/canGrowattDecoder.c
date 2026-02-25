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

static void logUnknownCellLikeFrame(const canGrowattState_t *s, uint32_t canId, const uint8_t *d)
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
        case 0x31A:
        case 0x31B:
        case 0x31C:
            if (looksLike4CellsFrame(d)) decodeCells4Frame(s, (uint8_t)(canId - 0x319u), d);
            else                        logUnknownCellLikeFrame(s, canId, d);
            break;
        /* Altă variantă observată în log-urile tale: 0x319,0x320,0x321,0x322 (4 frame-uri / 16 celule)
        Notă: 0x323 apare separat și NU pare să fie celule (îl ignorăm). */
        case 0x320:
        case 0x321:
        case 0x322: {
            uint8_t fi = 0u;
            if      (canId == 0x320u) fi = 1u;
            else if (canId == 0x321u) fi = 2u;
            else                      fi = 3u;
            if (looksLike4CellsFrame(d)) decodeCells4Frame(s, fi, d);
            else                        logUnknownCellLikeFrame(s, canId, d);
            break;
        }

        case 0x323:
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
