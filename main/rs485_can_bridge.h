#pragma once

#include "driver/twai.h"
#include "modbusDecoder.h"

#ifdef __cplusplus
extern "C" {
#endif

void rs485Can322BridgeEnable(modbusDecoder_t *srcDecoder, twai_handle_t txBus, const char *txName);

#ifdef __cplusplus
}
#endif
