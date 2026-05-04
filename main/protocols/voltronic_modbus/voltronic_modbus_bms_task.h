#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "decoders/modbusDecoder.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "orchestrator/protocol_types.h"
#include "protocols/voltronic_modbus/voltronic_modbus_registers_map.h"

typedef struct {
    bool valid;
    int64_t timestampUs;

    bool hasSoc;
    uint8_t socPct;

    bool hasPackVoltage;
    float packVoltageV;
    bool hasPackCurrent;
    float packCurrentA;
    bool hasPackPower;
    float packPowerW;

    bool hasFullMah;
    uint32_t fullMah;
    bool hasRemainMah;
    uint32_t remainMah;
    bool hasDesignMah;
    uint32_t designMah;

    bool hasChargeLimits;
    float chargeVoltageLimitV;
    float dischargeVoltageLimitV;
    float chargeCurrentLimitA;
    float dischargeCurrentLimitA;

    bool hasStatusFlags;
    uint16_t statusFlags;
    bool chargeEnabled;
    bool dischargeEnabled;
    bool chargeImmediately;
    bool chargeImmediately2;
    bool fullChargeRequested;

    bool hasAlarmRegisters;
    uint16_t chargeAlarm;
    uint16_t dischargeAlarm;
    uint16_t chargeProtect;
    uint16_t chargeProtect2;
    uint16_t dischargeProtect;
    uint16_t dischargeProtect2;
    uint16_t bmsState;

    uint8_t cellCount;
    uint16_t cellMv[VOLTRONIC_MB_MAX_CELLS];
    bool hasCellExtremes;
    uint16_t minCellMv;
    uint16_t maxCellMv;
    uint8_t minCellIndex;
    uint8_t maxCellIndex;

    uint8_t tempCount;
    int16_t tempDeciC[VOLTRONIC_MB_MAX_TEMPS];

    uint8_t cellStateCount;
    uint8_t cellState[VOLTRONIC_MB_MAX_CELLS];
    uint8_t tempStateCount;
    uint8_t tempState[VOLTRONIC_MB_MAX_TEMPS];
    uint8_t moduleState[VOLTRONIC_MB_MODULE_STATE_COUNT];
} voltronic_modbus_snapshot_t;

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t voltronicModbusBmsTaskStart(QueueHandle_t outQueue);
esp_err_t voltronicModbusBmsTaskStop(void);
bool voltronicModbusBmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket);
bool voltronicModbusBmsTaskGetLatestSnapshot(voltronic_modbus_snapshot_t *outSnapshot);
bool voltronicModbusBuildDecodedSnapshot(const modbusDecoder_t *decoder,
                                         int64_t sourceUs,
                                         voltronic_modbus_snapshot_t *outSnapshot);
bool voltronicModbusBuildDecodedPacket(const voltronic_modbus_snapshot_t *snapshot,
                                       uint32_t sequence,
                                       bms_decoded_packet_t *outPacket);

#ifdef __cplusplus
}
#endif
