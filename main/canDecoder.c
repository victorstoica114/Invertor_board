// canDecoder.c
#include "canDecoder.h"
#include <string.h>
#include <stdio.h>

static inline uint16_t be16(const uint8_t *p)
{
    return (uint16_t)((((uint16_t)p[0]) << 8) | (uint16_t)p[1]);
}

static inline int16_t be16s(const uint8_t *p)
{
    return (int16_t)be16(p);
}

void canDecoderInit(bmsCanState_t *s)
{
    memset(s, 0, sizeof(*s));
}

void canDecoderFeed(bmsCanState_t *s, uint32_t canId, const uint8_t *d, int dlc)
{
    if (s == NULL || d == NULL || dlc < 8) return;

    switch (canId) {

        case 0x311: {
            s->statusBits       = be16(&d[0]);
            s->recChargeCvV     = (float)be16(&d[2]) / 10.0f;
            s->chgCurrentLimitA = (float)be16(&d[4]) / 10.0f;
            s->disCurrentLimitA = (float)be16(&d[6]) / 10.0f;
            s->have311 = true;
        } break;

        case 0x313: {
            uint16_t vMax_mV = be16(&d[0]);
            uint16_t vMin_mV = be16(&d[2]);
            uint16_t dV_mV   = be16(&d[4]);

            s->maxCellV     = (float)vMax_mV / 1000.0f;
            s->minCellV     = (float)vMin_mV / 1000.0f;
            s->deltaCellV   = (float)dV_mV   / 1000.0f;
            s->maxCellIndex = d[6];
            s->minCellIndex = d[7];

            s->have313 = true;
        } break;

        case 0x314: {
            uint16_t v_0p01 = be16(&d[0]);
            int16_t  i_0p1  = be16s(&d[2]);

            s->packVoltageV = (float)v_0p01 / 100.0f;
            s->packCurrentA = (float)i_0p1  / 10.0f;
            s->socPct       = d[4];
            s->tMaxC        = (int8_t)d[5];

            // RM în 10mAh => Ah
            uint16_t rm10mAh = be16(&d[6]);
            s->remainAh = (float)rm10mAh / 100.0f;

            s->have314 = true;
        } break;

        default:
            break;
    }
}

bool canDecoderCanPrintState(const bmsCanState_t *s)
{
    // minim: V/I/SOC/Tmax (0x314). Dacă există 0x313, includem cell min/max/delta.
    return (s != NULL && s->have314);
}

void canDecoderFormatState(const bmsCanState_t *s, char *out, int outSize)
{
    if (s == NULL || out == NULL || outSize <= 0) return;

    if (s->have313) {
        snprintf(out, outSize,
                 "BMS: %.2fV | %+0.1fA | SOC %u%% | Tmax %dC | RM %.2fAh | Cmin %.3fV(C%u) | Cmax %.3fV(C%u) | \xCE\x94 %.3fV",
                 (double)s->packVoltageV,
                 (double)s->packCurrentA,
                 (unsigned)s->socPct,
                 (int)s->tMaxC,
                 (double)s->remainAh,
                 (double)s->minCellV, (unsigned)s->minCellIndex,
                 (double)s->maxCellV, (unsigned)s->maxCellIndex,
                 (double)s->deltaCellV);
    } else {
        snprintf(out, outSize,
                 "BMS: %.2fV | %+0.1fA | SOC %u%% | Tmax %dC | RM %.2fAh",
                 (double)s->packVoltageV,
                 (double)s->packCurrentA,
                 (unsigned)s->socPct,
                 (int)s->tMaxC,
                 (double)s->remainAh);
    }
}