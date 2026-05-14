#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SEPLOS_RS485_MAX_CELLS 16u
#define SEPLOS_RS485_MAX_TEMPS 6u
#define SEPLOS_RS485_MAX_INFO_BYTES 128u
#define SEPLOS_RS485_MAX_FRAME_LEN 320u

#define SEPLOS_RS485_PROTOCOL_VERSION 0x20u
#define SEPLOS_RS485_DEFAULT_ADDRESS 0x00u
#define SEPLOS_RS485_DEFAULT_REQUEST_INFO 0x00u
#define SEPLOS_RS485_CID1_BMS 0x46u
#define SEPLOS_RS485_CID2_TELEMETRY 0x42u
#define SEPLOS_RS485_CID2_ALARMS 0x44u

typedef enum {
    SEPLOS_RS485_REQUEST_STYLE_LEN_CHECK = 0,
    SEPLOS_RS485_REQUEST_STYLE_SIMPLE_LEN_NIBBLE = 1,
} seplos_rs485_request_style_t;

typedef struct {
    uint8_t version;
    uint8_t address;
    uint8_t cid1;
    uint8_t cid2;
    uint16_t lengthField;
    uint8_t info[SEPLOS_RS485_MAX_INFO_BYTES];
    size_t infoLen;
} seplos_rs485_frame_t;

typedef struct {
    bool valid;
    bool hasTelemetry;
    bool hasAlarms;

    uint8_t dataFlag;
    uint8_t commandGroup;

    uint8_t cellCount;
    uint16_t cellMv[SEPLOS_RS485_MAX_CELLS];
    bool hasCellExtremes;
    uint16_t minCellMv;
    uint16_t maxCellMv;
    uint8_t minCellIndex;
    uint8_t maxCellIndex;
    bool hasCellAvgMv;
    uint16_t cellAvgMv;
    bool hasCellDiffMv;
    uint16_t cellDiffMv;

    uint8_t tempCount;
    int16_t tempDeciC[SEPLOS_RS485_MAX_TEMPS];

    bool hasPackCurrentCa;
    int16_t packCurrentCa;
    bool hasPackVoltageCv;
    uint16_t packVoltageCv;
    bool hasPackPowerW;
    int32_t packPowerW;

    bool hasRemainingCapacityCah;
    uint16_t remainingCapacityCah;
    bool hasFullCapacityCah;
    uint16_t fullCapacityCah;
    bool hasRatedCapacityCah;
    uint16_t ratedCapacityCah;
    bool hasSocDeciPct;
    uint16_t socDeciPct;
    bool hasSohDeciPct;
    uint16_t sohDeciPct;
    bool hasCycles;
    uint16_t cycles;
    bool hasPortVoltageCv;
    uint16_t portVoltageCv;

    uint8_t cellAlarmFlags[SEPLOS_RS485_MAX_CELLS];
    uint8_t tempAlarmFlags[SEPLOS_RS485_MAX_TEMPS];
    uint8_t currentAlarmFlags;
    uint8_t voltageAlarmFlags;
    uint8_t customAlarmFlags;
    uint8_t warningBytes[8];
    uint8_t powerStatus;
    uint16_t balanceFlags;
    uint8_t systemStatus;
    bool chargeEnabled;
    bool dischargeEnabled;
    bool sleepMode;
} seplos_rs485_snapshot_t;

size_t seplosRs485BuildRequest(uint8_t cid2,
                               uint8_t address,
                               uint8_t protocolVersion,
                               uint8_t *out,
                               size_t outSize);

size_t seplosRs485BuildRequestWithStyle(uint8_t cid2,
                                        uint8_t address,
                                        uint8_t requestInfo,
                                        uint8_t protocolVersion,
                                        seplos_rs485_request_style_t style,
                                        uint8_t *out,
                                        size_t outSize);

bool seplosRs485DecodeFrame(const uint8_t *frame,
                            size_t frameLen,
                            seplos_rs485_frame_t *out);

bool seplosRs485DecodeTelemetryInfo(const uint8_t *info,
                                    size_t infoLen,
                                    seplos_rs485_snapshot_t *out);

bool seplosRs485DecodeAlarmInfo(const uint8_t *info,
                                size_t infoLen,
                                seplos_rs485_snapshot_t *inOut);

void seplosRs485FormatAlertFields(const seplos_rs485_snapshot_t *snapshot,
                                  char *protections,
                                  size_t protectionsSize,
                                  char *alarms,
                                  size_t alarmsSize,
                                  char *warnings,
                                  size_t warningsSize);

#ifdef __cplusplus
}
#endif
