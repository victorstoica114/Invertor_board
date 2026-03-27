#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "orchestrator/protocol_types.h"

#define JKBMS_MAX_CELLS 32u

typedef struct {
    bool valid;

    bool hasSoc;
    uint8_t socPct;

    bool hasSoh;
    uint8_t sohPct;

    bool hasPrecharge;
    uint8_t prechargeState;

    bool hasTempMosC;
    int16_t tempMosC;
    bool hasTempBat1C;
    int16_t tempBat1C;
    bool hasTempBat2C;
    int16_t tempBat2C;

    bool hasPackVoltageMv;
    uint32_t packVoltageMv;
    bool hasPackCurrentMa;
    int32_t packCurrentMa;
    bool hasPackPowerMw;
    int32_t packPowerMw;
    bool hasBalanceCurrentMa;
    int16_t balanceCurrentMa;

    bool hasRemainMah;
    int32_t remainMah;
    bool hasFullMah;
    uint32_t fullMah;
    bool hasCycles;
    uint32_t cycles;

    bool hasCellAvgMv;
    uint16_t cellAvgMv;
    bool hasCellDiffMaxMv;
    uint16_t cellDiffMaxMv;

    bool hasAlarmBits;
    uint32_t alarmBits;

    uint8_t cellCount;
    uint16_t cellMv[JKBMS_MAX_CELLS];

    bool hasCellExtremes;
    uint16_t minCellMv;
    uint16_t maxCellMv;
    uint8_t minCellIndex;
    uint8_t maxCellIndex;
} jkbms_modbus_snapshot_t;

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t jkbmsModbusBmsTaskStart(QueueHandle_t outQueue);
esp_err_t jkbmsModbusBmsTaskStop(void);
bool jkbmsModbusBmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket);
bool jkbmsModbusBmsTaskGetLatestSnapshot(jkbms_modbus_snapshot_t *outSnapshot);

#ifdef __cplusplus
}
#endif
