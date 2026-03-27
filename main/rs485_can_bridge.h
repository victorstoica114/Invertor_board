#pragma once

#include "driver/gpio.h"
#include "driver/twai.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "modbusDecoder.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool valid;
    int64_t timestampUs;
    uint8_t socPct;
    uint8_t sohPct;
    int16_t tempC;
    uint16_t packCv;
    uint16_t cycles;
    uint16_t remainingCapCah;
    uint16_t fullCapCah;
    uint16_t cellMaxMv;
    uint16_t cellMinMv;
    uint8_t cellMaxIdx;
    uint8_t cellMinIdx;
    uint8_t cellCount;
    uint16_t cellMv[16];
} can_rs485_growatt_snapshot_t;

esp_err_t canRs485GrowattBridgeEnable(uart_port_t inverterUart,
                                      gpio_num_t inverterDir,
                                      const char *ifName,
                                      twai_handle_t srcCanBus,
                                      const char *srcCanIf);
esp_err_t jkbmsRs485GrowattBridgeEnable(uart_port_t inverterUart,
                                        gpio_num_t inverterDir,
                                        const char *ifName);
void canRs485GrowattBridgeStop(void);
bool canRs485GrowattBridgeGetLatestSnapshot(can_rs485_growatt_snapshot_t *out);

void rs485Can322BridgeEnable(modbusDecoder_t *srcDecoder, twai_handle_t txBus, const char *txName);

#ifdef __cplusplus
}
#endif
