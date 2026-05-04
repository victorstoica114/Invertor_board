#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PROTOCOL_ID_GROWATT = 0,
    PROTOCOL_ID_PYLON = 1,
    PROTOCOL_ID_JKBMS = 2,
    PROTOCOL_ID_PACE = 3,
    PROTOCOL_ID_JKBMS_NATIVE = 4,
    PROTOCOL_ID_VOLTRONIC = 5,
    PROTOCOL_ID_CHINA_TOWER = 6,
} protocol_id_t;

#define BMS_DECODED_PACKET_MAX_CELLS 32u
#define BMS_DECODED_PACKET_MAX_TEMPS 6u

typedef struct {
    protocol_id_t sourceProtocol;
    uint32_t sequence;
    int64_t timestampUs;

    bool hasSoc;
    uint8_t socPct;

    bool hasTemperatureC;
    int16_t temperatureC;
    uint8_t tempCount;
    int16_t tempDeciC[BMS_DECODED_PACKET_MAX_TEMPS];

    bool hasPackVoltageCv;
    uint16_t packVoltageCv;

    bool hasCellExtremes;
    uint16_t minCellMv;
    uint16_t maxCellMv;
    uint8_t minCellIndex;
    uint8_t maxCellIndex;

    uint8_t cellCount;
    uint16_t cellMv[BMS_DECODED_PACKET_MAX_CELLS];

    bool hasWarningFlags;
    uint16_t warningFlags;

    bool hasProtectionFlags;
    uint16_t protectionFlags;

    bool hasStatusFlags;
    uint16_t statusFlags;

    bool hasBalanceFlags;
    uint16_t balanceFlags;
} bms_decoded_packet_t;

const char *protocolIdToStr(protocol_id_t id);

#ifdef __cplusplus
}
#endif
