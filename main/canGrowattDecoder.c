#include "canGrowattDecoder.h"

#include <string.h>
#include "esp_log.h"

#ifndef EXAMPLE_TAG
#define EXAMPLE_TAG "SNIFFER_BRIDGE"
#endif

#ifndef CAN_RAW_VALUES
#define CAN_RAW_VALUES 1
#endif



static inline uint16_t be16(const uint8_t *p)
{
    return (uint16_t)((((uint16_t)p[0]) << 8) | (uint16_t)p[1]);
}

static inline int16_t be16s(const uint8_t *p)
{
    return (int16_t)be16(p);
}

static void printCompact(const canGrowattState_t *s)
{
    if (!s->have313) return;

    if (s->have314) {
        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s BMS: %.2fV | %+0.1fA | SOC %u%% | T %dC | Cap %.1fAh | Rem %.1fAh | Avg %.3fV | Cmin~ %.3fV | Cmax~ %.3fV | \xCE\x94 %.3fV",
                 s->ifName,
                 (double)s->packVoltageV,
                 (double)s->packCurrentA,
                 (unsigned)s->socPct,
                 (int)s->tMaxC,
                 (double)s->capacityAh,
                 (double)s->remainAh,
                 (double)s->avgCellV,
                 (double)s->cellMinV,
                 (double)s->cellMaxV,
                 (double)s->cellDeltaV);
    } else {
        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s BMS: %.2fV | %+0.1fA | SOC %u%% | T %dC",
                 s->ifName,
                 (double)s->packVoltageV,
                 (double)s->packCurrentA,
                 (unsigned)s->socPct,
                 (int)s->tMaxC);
    }
}

void canGrowattInit(canGrowattState_t *s, const char *ifName)
{
    memset(s, 0, sizeof(*s));
    s->ifName = ifName;
    s->cellCount = 16;
}

void canGrowattOnFrame(canGrowattState_t *s, uint32_t canId, const uint8_t *d, int dlc)
{
    if (s == NULL || d == NULL || dlc != 8) return;

    switch (canId) {

    case 0x311: {
        s->statusBits        = be16(&d[0]);
        s->recChargeCvV      = (float)be16(&d[2]) / 10.0f;
        s->chgCurrentLimitA  = (float)be16(&d[4]) / 10.0f;
        s->disCurrentLimitA  = (float)be16(&d[6]) / 10.0f;
        s->have311 = true;

#if CAN_RAW_VALUES
        ESP_LOGI(EXAMPLE_TAG, "CAN-%s 0x311: status=0x%04X CV=%.1fV IchgLim=%.1fA IdisLim=%.1fA",
                 s->ifName, (unsigned)s->statusBits,
                 (double)s->recChargeCvV, (double)s->chgCurrentLimitA, (double)s->disCurrentLimitA);
#endif
        break;
    }

    case 0x313: {
        uint16_t v_0p01 = be16(&d[0]);
        int16_t  i_0p1  = be16s(&d[2]);
        int16_t  t_0p1  = be16s(&d[4]);

        s->packVoltageV = (float)v_0p01 / 100.0f;
        s->packCurrentA = (float)i_0p1 / 10.0f;
        s->socPct       = d[6];
        s->sohPct       = d[7];
        s->tMaxC        = (int8_t)((t_0p1 >= 0) ? ((t_0p1 + 5) / 10) : ((t_0p1 - 5) / 10));
        s->have313 = true;

#if CAN_RAW_VALUES
        ESP_LOGI(EXAMPLE_TAG, "CAN-%s 0x313: V=%.2fV I=%+.1fA T=%dC SOC=%u%% SOH=%u%%",
                 s->ifName, (double)s->packVoltageV, (double)s->packCurrentA,
                 (int)s->tMaxC, (unsigned)s->socPct, (unsigned)s->sohPct);
#endif
        break;
    }

    case 0x314: {
        uint16_t rm_0p01Ah  = be16(&d[0]);
        uint16_t cap_0p01Ah = be16(&d[2]);
        uint16_t dV_mV      = be16(&d[4]);

        s->remainAh   = (float)rm_0p01Ah / 100.0f;
        s->capacityAh = (float)cap_0p01Ah / 100.0f;
        s->cellDeltaV = (float)dV_mV / 1000.0f;

        if (s->cellCount == 0) s->cellCount = 16;
        if (s->have313 && s->packVoltageV > 0.0f) {
            s->avgCellV = s->packVoltageV / (float)s->cellCount;
            s->cellMinV = s->avgCellV - (s->cellDeltaV * 0.5f);
            s->cellMaxV = s->avgCellV + (s->cellDeltaV * 0.5f);
        }

        s->have314 = true;

#if CAN_RAW_VALUES
        ESP_LOGI(EXAMPLE_TAG, "CAN-%s 0x314: Rem=%.2fAh Cap=%.2fAh dV=%umV",
                 s->ifName, (double)s->remainAh, (double)s->capacityAh, (unsigned)dV_mV);
#endif

        printCompact(s);
        break;
    }

    default:
        break;
    }
}