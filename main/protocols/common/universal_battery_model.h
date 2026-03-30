#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UNIVERSAL_BATTERY_TEMP_SENSORS 5

typedef struct {
    bool valid;
    uint32_t updatedMs;
    float packVoltageV;
    float packCurrentA;
    uint8_t socPct;
    uint8_t sohPct;
    uint16_t cycleCount;
    float chargeVoltageLimitV;
    float chargeCurrentLimitA;
    float dischargeCurrentLimitA;
    float cellMaxV;
    float cellMinV;
    uint8_t cellMaxIdx;
    uint8_t cellMinIdx;
    float cellDeltaV;
    float temperaturesC[UNIVERSAL_BATTERY_TEMP_SENSORS];
    bool chargeEnabled;
    bool dischargeEnabled;
    bool balanceEnabled;
    uint32_t alarmsMask;
    uint32_t warningsMask;
    uint32_t protocolState;
} universal_battery_model_t;

#ifdef __cplusplus
}
#endif
