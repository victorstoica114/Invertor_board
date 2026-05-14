#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "orchestrator/protocol_types.h"
#include "protocols/seplos_rs485/seplos_rs485_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t seplosRs485BmsTaskStart(QueueHandle_t outQueue);
esp_err_t seplosRs485BmsTaskStop(void);
bool seplosRs485BmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket);
bool seplosRs485BmsTaskGetLatestSnapshot(seplos_rs485_snapshot_t *outSnapshot);

#ifdef __cplusplus
}
#endif
