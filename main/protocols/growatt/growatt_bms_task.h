#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "orchestrator/protocol_types.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t growattBmsTaskStart(QueueHandle_t outQueue);
esp_err_t growattBmsTaskStop(void);
bool growattBmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket);

#ifdef __cplusplus
}
#endif
