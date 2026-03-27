#pragma once

#include "driver/gpio.h"
#include "driver/twai.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "modbusDecoder.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t canRs485GrowattBridgeEnable(uart_port_t inverterUart,
                                      gpio_num_t inverterDir,
                                      const char *ifName,
                                      twai_handle_t srcCanBus,
                                      const char *srcCanIf);
esp_err_t jkbmsRs485GrowattBridgeEnable(uart_port_t inverterUart,
                                        gpio_num_t inverterDir,
                                        const char *ifName);
void canRs485GrowattBridgeStop(void);

void rs485Can322BridgeEnable(modbusDecoder_t *srcDecoder, twai_handle_t txBus, const char *txName);

#ifdef __cplusplus
}
#endif
