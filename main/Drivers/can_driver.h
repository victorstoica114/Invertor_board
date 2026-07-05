#pragma once

#include "freertos/FreeRTOS.h"
#include "driver/twai.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void canInit(void);
void canReinit(void);
esp_err_t canReinitPort(uint8_t port, uint32_t bitrate);
esp_err_t canReinitPortMode(uint8_t port, uint32_t bitrate, twai_mode_t mode);
twai_handle_t canGetBus0(void);   /* CAN1 */
twai_handle_t canGetBus1(void);   /* CAN2 */

#ifdef __cplusplus
}
#endif
