// canGrowattDecoder.c
#include "canGrowattDecoder.h"
#include <string.h>
#include <stdio.h>
#include "esp_log.h"

#ifndef EXAMPLE_TAG
#define EXAMPLE_TAG "SNIFFER_BRIDGE"
#endif

#ifndef CAN_RAW_VALUES
#define CAN_RAW_VALUES 0
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
    if (!s->have314) return;

    if (s->have313) {
        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s BMS: %.2fV | %+0.1fA | SOC %u%% | Tmax %dC | Cmin %.3fV(C%u) | Cmax %.3fV(C%u) | \xCE\x94 %.3fV",
                 s->ifName,
                 (double)s->packVoltageV,
                 (double)s->packCurrentA,
                 (unsigned)s->socPct,
                 (int)s->tMaxC,
                 (double)s->cellMinV, (unsigned)s->cellMinIndex,
                 (double)s->cellMaxV, (unsigned)s->cellMaxIndex,
                 (double)s->cellDeltaV);
    } else {
        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s BMS: %.2fV | %+0.1fA | SOC %u%% | Tmax %dC",
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
}

void canGrowattOnFrame(canGrowattState_t *s, uint32_t canId, const uint8_t *d, int dlc)
{
    if (dlc != 8 || d == NULL) return;

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
        uint16_t vMax_mV = be16(&d[0]);
        uint16_t vMin_mV = be16(&d[2]);
        uint16_t dV_mV   = be16(&d[4]);

        s->cellMaxV     = (float)vMax_mV / 1000.0f;
        s->cellMinV     = (float)vMin_mV / 1000.0f;
        s->cellDeltaV   = (float)dV_mV   / 1000.0f;

        s->cellMaxIndex = d[6];
        s->cellMinIndex = d[7];

        s->have313 = true;

#if CAN_RAW_VALUES
        ESP_LOGI(EXAMPLE_TAG, "CAN-%s 0x313: Cmax=%umV(C%u) Cmin=%umV(C%u) dV=%umV",
                 s->ifName, (unsigned)vMax_mV, (unsigned)s->cellMaxIndex,
                 (unsigned)vMin_mV, (unsigned)s->cellMinIndex,
                 (unsigned)dV_mV);
#endif
        break;
    }

    case 0x314: {
        uint16_t v_0p01 = be16(&d[0]);
        int16_t  i_0p1  = be16s(&d[2]);

        s->packVoltageV   = (float)v_0p01 / 100.0f;
        s->packCurrentA   = (float)i_0p1  / 10.0f;
        s->socPct         = d[4];
        s->tMaxC          = (int8_t)d[5];
        s->gaugeRm_10mAh  = be16(&d[6]);

        s->have314 = true;

#if CAN_RAW_VALUES
        ESP_LOGI(EXAMPLE_TAG, "CAN-%s 0x314: V=%.2fV I=%+.1fA SOC=%u%% Tmax=%dC RM=%u(10mAh)",
                 s->ifName, (double)s->packVoltageV, (double)s->packCurrentA,
                 (unsigned)s->socPct, (int)s->tMaxC, (unsigned)s->gaugeRm_10mAh);
#endif

        printCompact(s);
        break;
    }

    default:
        break;
    }
}