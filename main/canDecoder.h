// canDecoder.h
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* decoded values */
    float    packVoltageV;
    float    packCurrentA;
    uint8_t  socPct;
    int8_t   tMaxC;
    uint16_t gaugeRm_10mAh;

    float    cellMinV;
    float    cellMaxV;
    float    cellDeltaV;
    uint8_t  cellMinIndex;
    uint8_t  cellMaxIndex;

    float    t1C;
    float    t2C;

    /* availability flags */
    bool have313;
    bool have314;
    bool have322;
} bmsCanState_t;

void canDecoderInit(bmsCanState_t *s);
void canDecoderFeed(bmsCanState_t *s, uint32_t canId, const uint8_t *d, int dlc);

bool canDecoderCanPrintState(const bmsCanState_t *s);
void canDecoderFormatState(const bmsCanState_t *s, char *out, size_t outLen);

#ifdef __cplusplus
}
#endif