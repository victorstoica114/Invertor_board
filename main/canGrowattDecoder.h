#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *ifName;

    uint16_t statusBits;

    float recChargeCvV;
    float chgCurrentLimitA;
    float disCurrentLimitA;

    float packVoltageV;
    float packCurrentA;
    uint8_t socPct;
    uint8_t sohPct;

    float remainAh;
    float capacityAh;

    float cellMinV;
    float cellMaxV;
    float cellDeltaV;

    float avgCellV;
    uint8_t cellCount;

    int8_t tMaxC;

    bool have311;
    bool have313;
    bool have314;

    

} canGrowattState_t;

void canGrowattInit(canGrowattState_t *s, const char *ifName);

void canGrowattOnFrame(canGrowattState_t *s,
                       uint32_t canId,
                       const uint8_t *d,
                       int dlc);

#ifdef __cplusplus
}
#endif