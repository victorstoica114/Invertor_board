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
    if (dlc != 8 || d == NULL) return;

    switch (canId) {
    case 0x314: {
        /* old/other mapping in your canDecoder (keep as-is if you use it elsewhere) */
        uint16_t cmin_mV = be16(&d[0]);
        uint16_t cmax_mV = be16(&d[2]);
        uint16_t dV_mV   = be16(&d[4]);

        s->minCellV   = (float)cmin_mV / 1000.0f;
        s->maxCellV   = (float)cmax_mV / 1000.0f;
        s->deltaCellV = (float)dV_mV   / 1000.0f;

        s->socPct = d[6];
        s->sohPct = d[7];

        s->have314 = true;
        break;
    }

    case 0x313: {
        uint16_t v_0p01 = be16(&d[0]);
        int16_t  i_0p1  = be16s(&d[2]);
        int16_t  t_0p1  = be16s(&d[4]);

        s->packVoltageV = (float)v_0p01 / 100.0f;
        s->packCurrentA = (float)i_0p1  / 10.0f;
        s->tMaxC        = (float)t_0p1  / 10.0f;

        s->have313 = true;
        break;
    }

    default:
        break;
    }
}

void canDecoderFormatState(const bmsCanState_t *s, char *out, size_t outLen)
{
    if (!(s->have313 && s->have314)) {
        snprintf(out, outLen, "BMS: waiting frames...");
        return;
    }

    snprintf(out, outLen,
             "BMS: %.2fV | %+0.1fA | SOC %u%% | Tmin/Max unknown | Cmin %.3fV | Cmax %.3fV | Δ %.3fV",
             (double)s->packVoltageV,
             (double)s->packCurrentA,
             (unsigned)s->socPct,
             (double)s->minCellV,
             (double)s->maxCellV,
             (double)s->deltaCellV);
}