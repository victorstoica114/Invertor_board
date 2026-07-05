#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "orchestrator/protocol_types.h"
#include "protocols/daly_rs485/daly_rs485_bms_task.h"
#include "runtime_settings.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t dalyCanBmsTaskStart(QueueHandle_t outQueue,
                              const bridge_runtime_settings_t *settings);
esp_err_t dalyCanBmsTaskStop(void);
bool dalyCanBmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket);
bool dalyCanBmsTaskGetLatestSnapshot(daly_rs485_snapshot_t *outSnapshot);

#ifdef __cplusplus
}
#endif
