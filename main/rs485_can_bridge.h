#pragma once

#include "driver/gpio.h"
#include "driver/twai.h"
#include "driver/uart.h"
#include "modbusDecoder.h"

#ifdef __cplusplus
extern "C" {
#endif

void rs485Can322BridgeEnable(modbusDecoder_t *srcDecoder, twai_handle_t txBus, const char *txName);
void canRs485SocBridgeEnable(uart_port_t inverterUart, gpio_num_t inverterDir, const char *ifName);
void rs485Can322BridgeStop(void);
void canRs485SocBridgeStop(void);

#ifdef __cplusplus
}
#endif
