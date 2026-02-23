// canDecoder.h
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool have311;
    bool have313;
    bool have314;

    // 0x314
    float packVoltageV;     // 0.01V
    float packCurrentA;     // 0.1A signed
    uint8_t socPct;         // %
    int8_t tMaxC;           // signed °C
    float remainAh;         // din RM (10mAh) => Ah

    // 0x313
    float minCellV;         // V
    uint8_t minCellIndex;
    float maxCellV;         // V
    uint8_t maxCellIndex;
    float deltaCellV;       // V

    // 0x311 (opțional, le păstrăm ca info)
    uint16_t statusBits;
    float recChargeCvV;
    float chgCurrentLimitA;
    float disCurrentLimitA;
} bmsCanState_t;

void canDecoderInit(bmsCanState_t *s);
void canDecoderFeed(bmsCanState_t *s, uint32_t canId, const uint8_t *d, int dlc);
bool canDecoderCanPrintState(const bmsCanState_t *s);
void canDecoderFormatState(const bmsCanState_t *s, char *out, int outSize);