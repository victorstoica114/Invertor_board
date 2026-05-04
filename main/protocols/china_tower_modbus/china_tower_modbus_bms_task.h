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

esp_err_t chinaTowerModbusBmsTaskStart(QueueHandle_t outQueue);
esp_err_t chinaTowerModbusBmsTaskStop(void);
bool chinaTowerModbusBmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket);
bool chinaTowerModbusBuildDecodedPacket(const modbusDecoder_t *decoder,
                                        uint32_t sequence,
                                        bms_decoded_packet_t *outPacket);

#ifdef __cplusplus
}
#endif
