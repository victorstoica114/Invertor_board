#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "orchestrator/protocol_types.h"
#include "protocols/jkbms_rs485/jkbms_rs485_native.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t jkbmsRs485BmsTaskStart(QueueHandle_t outQueue);
esp_err_t jkbmsRs485BmsTaskStop(void);
bool jkbmsRs485BmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket);
bool jkbmsRs485BmsTaskGetLatestSnapshot(jkbms_rs485_native_snapshot_t *outSnapshot);

#ifdef __cplusplus
}
#endif
