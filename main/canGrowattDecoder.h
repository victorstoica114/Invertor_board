// canGrowattDecoder.h
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    const char *ifName;

    bool have311;
    bool have313;
    bool have314;

    // 0x311
    uint16_t statusBits;      // raw bits 0..11 (vezi doc)
    float recChargeCvV;       // 0.1V
    float chgCurrentLimitA;   // 0.1A
    float disCurrentLimitA;   // 0.1A

    // 0x314
    float packVoltageV;       // 0.01V
    float packCurrentA;       // 0.1A signed
    uint8_t socPct;           // 0..100
    int8_t tMaxC;             // signed °C
    uint16_t gaugeRm_10mAh;   // 10mAh units

    // 0x313
    float cellMinV;           // 1mV -> V
    float cellMaxV;           // 1mV -> V
    float cellDeltaV;         // 1mV -> V
    uint8_t cellMaxIndex;     // 1..N
    uint8_t cellMinIndex;     // 1..N
} canGrowattState_t;

void canGrowattInit(canGrowattState_t *s, const char *ifName);
void canGrowattOnFrame(canGrowattState_t *s, uint32_t canId, const uint8_t *data, int dlc);