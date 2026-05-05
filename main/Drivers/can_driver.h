#pragma once

#include "freertos/FreeRTOS.h"
#include "driver/twai.h"

#ifdef __cplusplus
extern "C" {
#endif

void canInit(void);
void canReinit(void);
twai_handle_t canGetBus0(void);   /* CAN1 */
twai_handle_t canGetBus1(void);   /* CAN2 */

#ifdef __cplusplus
}
#endif
