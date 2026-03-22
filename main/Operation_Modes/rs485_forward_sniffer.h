#pragma once

#include "modbusDecoder.h"
#include "runtime_settings.h"

#ifdef __cplusplus
extern "C" {
#endif

void rs485ForwardSnifferStart(const bridge_runtime_settings_t *settings);
void rs485ForwardSnifferStop(void);
modbusDecoder_t *rs485ForwardSnifferGetDecoder(int port);
void rs485ForwardSnifferResetDecoders(void);

#ifdef __cplusplus
}
#endif
