#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool valid;
    char source[24];
    char protocol[24];
    float currentA;
    float packVoltageV;
    float packPowerW;
    float balanceCurrentA;
    float remainingAh;
    float fullAh;
    uint16_t cycles;
    uint8_t socPct;
    uint8_t sohPct;
    float cellMaxV;
    float cellMinV;
    uint8_t cellMaxIdx;
    uint8_t cellMinIdx;
    float deltaV;
    float tempMosC;
    float tempT1C;
    float tempT2C;
    float tempT4C;
    float tempT5C;
    uint8_t pylonStatus63;
    uint8_t deyeStatus35C;
    uint8_t deyeTempMaxSensor;
    uint8_t deyeTempMinSensor;
    uint8_t cellCount;
    float cellVoltagesV[32];
    float cellAvgV;
    float cellDiffV;
    uint32_t alarmRaw;
    uint8_t prechargeState;
    char stateFlags[128];
    char protections[256];
    char alarms[256];
    char warnings[256];
} bridgeTelemetrySnapshot_t;

void rs485BridgeEnable(void);
void canBridgeEnable(void);
void bridgeReloadFromRuntimeSettings(void);
void bridgeGetTelemetrySnapshot(bridgeTelemetrySnapshot_t *out);
void bridgeSetTelemetrySnapshot(const bridgeTelemetrySnapshot_t *in);
void bridgeGetDecodedLogSnapshot(char *out, uint32_t outSize);
void bridgeSetDecodedLogSnapshot(const char *text);

#ifdef __cplusplus
}
#endif
