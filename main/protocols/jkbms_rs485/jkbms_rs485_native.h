#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define JKBMS_RS485_NATIVE_MAX_CELLS 32u
#define JKBMS_RS485_NATIVE_MAX_FRAME_LEN 512u
#define JKBMS_RS485_NATIVE_READ_ALL_REQUEST_LEN 21u

typedef struct {
    bool valid;

    bool hasSoc;
    uint8_t socPct;

    bool hasSoh;
    uint8_t sohPct;

    bool hasTempMosC;
    int16_t tempMosC;
    bool hasTempBat1C;
    int16_t tempBat1C;
    bool hasTempBat2C;
    int16_t tempBat2C;
    bool hasTempSensorCount;
    uint8_t tempSensorCount;

    bool hasPackVoltageMv;
    uint32_t packVoltageMv;
    bool hasPackCurrentMa;
    int32_t packCurrentMa;
    bool hasPackPowerMw;
    int32_t packPowerMw;

    bool hasFullMah;
    uint32_t fullMah;
    bool hasCycles;
    uint32_t cycles;

    bool hasAlarmBits;
    uint16_t alarmBits;

    bool hasStatusFlags;
    uint16_t statusFlags;
    bool chargeEnabled;
    bool dischargeEnabled;
    bool balanceActive;

    uint8_t cellCount;
    uint16_t cellMv[JKBMS_RS485_NATIVE_MAX_CELLS];

    bool hasCellAvgMv;
    uint16_t cellAvgMv;
    bool hasCellDiffMaxMv;
    uint16_t cellDiffMaxMv;
    bool hasCellExtremes;
    uint16_t minCellMv;
    uint16_t maxCellMv;
    uint8_t minCellIndex;
    uint8_t maxCellIndex;
} jkbms_rs485_native_snapshot_t;

#ifdef __cplusplus
extern "C" {
#endif

size_t jkbmsRs485NativeBuildReadAllRequest(uint8_t *out, size_t outSize);
bool jkbmsRs485NativeDecodeFrame(const uint8_t *frame,
                                 size_t frameLen,
                                 jkbms_rs485_native_snapshot_t *out);
void jkbmsRs485NativeFormatAlertFields(uint16_t alarmBits,
                                       char *protections,
                                       size_t protectionsSize,
                                       char *alarms,
                                       size_t alarmsSize,
                                       char *warnings,
                                       size_t warningsSize);

#ifdef __cplusplus
}
#endif
