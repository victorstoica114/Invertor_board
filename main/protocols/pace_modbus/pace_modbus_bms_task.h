#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "decoders/modbusDecoder.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "orchestrator/protocol_types.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t paceModbusBmsTaskStart(QueueHandle_t outQueue);
esp_err_t paceModbusBmsTaskStop(void);
bool paceModbusBmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket);
bool paceModbusBuildDecodedPacket(const modbusDecoder_t *decoder,
                                  uint32_t sequence,
                                  bms_decoded_packet_t *outPacket);

#ifdef __cplusplus
}
#endif
