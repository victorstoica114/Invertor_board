#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "orchestrator/protocol_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool valid;
    uint32_t sequence;
    int64_t timestampUs;

    bool hasPackVoltageCv;
    uint16_t packVoltageCv;

    bool hasCurrentDeciA;
    int16_t currentDeciA;

    bool hasSocDeciPct;
    uint16_t socDeciPct;

    bool hasCycles;
    uint16_t cycles;

    bool hasCapacity;
    uint32_t ratedCapacityMah;
    uint32_t remainingCapacityMah;

    bool hasCellExtremes;
    uint16_t maxCellMv;
    uint16_t minCellMv;
    uint8_t maxCellIndex;
    uint8_t minCellIndex;

    uint8_t cellCount;
    uint16_t cellMv[BMS_DECODED_PACKET_MAX_CELLS];

    uint8_t tempCount;
    int16_t tempDeciC[BMS_DECODED_PACKET_MAX_TEMPS];

    bool chargeEnabled;
    bool dischargeEnabled;
    bool balanceEnabled;
    uint32_t alarmMask;
    uint32_t warningMask;
    uint32_t protocolState;
} daly_rs485_snapshot_t;

esp_err_t dalyRs485BmsTaskStart(QueueHandle_t outQueue);
esp_err_t dalyRs485BmsTaskStartConfigured(QueueHandle_t outQueue,
                                         uint8_t bmsPort,
                                         bool publishBatteryModel);
esp_err_t dalyRs485BmsTaskStop(void);
bool dalyRs485BmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket);
bool dalyRs485BmsTaskGetLatestSnapshot(daly_rs485_snapshot_t *outSnapshot);
bool dalyRs485BuildDecodedPacket(const daly_rs485_snapshot_t *snapshot,
                                 uint32_t sequence,
                                 bms_decoded_packet_t *outPacket);

#ifdef __cplusplus
}
#endif
