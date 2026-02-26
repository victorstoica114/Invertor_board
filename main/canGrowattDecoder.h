#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *ifName;

    /* 0x311 */
    uint16_t statusBits;
    float recChargeCvV;
    float chgCurrentLimitA;
    float disCurrentLimitA;
    bool have311;

    /* 0x313 */
    float packVoltageV;
    float packCurrentA;
    int8_t packTempC;     /* pack temperature (rounded) */
    uint8_t socPct;
    uint8_t sohPct;
    bool have313;

    /* 0x314 */
    float remainAh;
    float capacityAh;
    float cellDeltaV;     /* mV -> V */
    bool have314;

    /* fallback approx */
    float avgCellV;       /* packVoltage/16 */

    /* per-cell (16S) */
    float cellV[16];
    uint32_t haveCellMask;    /* bit i = cellV[i] valid */
    bool haveCells;

    /* real stats from cellV[] when haveCells=true */
    float cellAvgV;
    float cellMinV;
    float cellMaxV;
    uint8_t cellMinIndex;     /* 0..15 */
    uint8_t cellMaxIndex;     /* 0..15 */

    /* meta/config frames 0x319..0x323 (log dedupe) */
    uint8_t metaFrame319_323[5][8];
    uint8_t haveMetaFrameMask319_323;
} canGrowattState_t;

void canGrowattInit(canGrowattState_t *s, const char *ifName);

void canGrowattOnFrame(canGrowattState_t *s,
                       uint32_t canId,
                       const uint8_t *d,
                       int dlc);

#ifdef __cplusplus
}
#endif
