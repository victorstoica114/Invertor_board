#include "canDecoder.h"

#include <string.h>

/*
 * Lightweight decoder for the subset of BMS CAN frames we see on the bus.
 * IDs handled:
 *   0x311: status + limits
 *   0x313: pack voltage/current + SOC (+ temp)
 *   0x314: capacities + cell voltage diff
 */

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
    if (s == NULL || d == NULL || dlc != 8) return;

    switch (canId) {

    case 0x311: {
        s->statusBits   = be16(&d[0]);
        s->recChargeCvV = (float)be16(&d[2]) / 10.0f;
        s->chgLimitA    = (float)be16(&d[4]) / 10.0f;
        s->disLimitA    = (float)be16(&d[6]) / 10.0f;
        s->have311 = true;
        break;
    }

    case 0x313: {
        /*
         * Mapping matching your raw frames + phone app:
         *   d[0..1] pack voltage in 0.01V
         *   d[2..3] pack current in 0.1A, signed
         *   d[4..5] temperature in 0.1C, signed (MOS / max temp)
         *   d[6]    SOC in %
         *   d[7]    SOH in % (often 100)
         */
        uint16_t v_0p01 = be16(&d[0]);
        int16_t  i_0p1  = be16s(&d[2]);
        int16_t  t_0p1  = be16s(&d[4]);

        s->packVoltageV = (float)v_0p01 / 100.0f;
        s->packCurrentA = (float)i_0p1 / 10.0f;
        s->socPct       = d[6];
        s->sohPct       = d[7];

        /* keep one representative integer temperature */
        s->tMaxC = (int8_t)((t_0p1 >= 0) ? ((t_0p1 + 5) / 10) : ((t_0p1 - 5) / 10));

        s->have313 = true;
        break;
    }

    case 0x314: {
        /*
         * Mapping matching your raw frames + phone app:
         *   d[0..1] remaining capacity in 0.01Ah
         *   d[2..3] nominal capacity in 0.01Ah
         *   d[4..5] cell voltage diff in mV
         *   d[6..7] reserved / balance current (unknown scaling)
         */
        uint16_t rm_0p01Ah  = be16(&d[0]);
        uint16_t cap_0p01Ah = be16(&d[2]);
        uint16_t dV_mV      = be16(&d[4]);

        s->remainAh   = (float)rm_0p01Ah / 100.0f;
        s->capacityAh = (float)cap_0p01Ah / 100.0f;
        s->deltaCellV = (float)dV_mV / 1000.0f;

        /* If we have pack voltage, derive avg/min/max for a 16S pack. */
        if (s->packVoltageV > 0.0f) {
            float avg = s->packVoltageV / 16.0f;
            s->minCellV = avg - (s->deltaCellV * 0.5f);
            s->maxCellV = avg + (s->deltaCellV * 0.5f);
        }

        s->have314 = true;
        break;
    }

    default:
        break;
    }
}