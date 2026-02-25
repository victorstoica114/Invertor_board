#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* legacy fields used by canDecoder.c */
    float packV;
    float packI;

    float cellMinV;
    float cellMaxV;
    float cellDeltaV;

    float t1C;
    float t2C;

    uint8_t soc;

    bool have311;
    bool have312;
    bool have313;
    bool have314;
    bool have322;
    bool have323;

} bmsCanState_t;

void canDecoderInit(bmsCanState_t *s);
void canDecoderFeed(bmsCanState_t *s, uint32_t canId, const uint8_t *d, int dlc);
int  canDecoderFormatState(char *out, size_t outSize, const bmsCanState_t *s);

#ifdef __cplusplus
}
#endif