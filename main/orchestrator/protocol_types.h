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
} protocol_id_t;

typedef struct {
    protocol_id_t sourceProtocol;
    uint32_t sequence;
    int64_t timestampUs;

    bool hasSoc;
    uint8_t socPct;

    bool hasTemperatureC;
    int16_t temperatureC;

    bool hasPackVoltageCv;
    uint16_t packVoltageCv;

    bool hasCellExtremes;
    uint16_t minCellMv;
    uint16_t maxCellMv;
    uint8_t minCellIndex;
    uint8_t maxCellIndex;
} bms_decoded_packet_t;

const char *protocolIdToStr(protocol_id_t id);

#ifdef __cplusplus
}
#endif
