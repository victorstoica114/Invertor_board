#pragma once

#include "driver/twai.h"

#ifdef __cplusplus
extern "C" {
#endif

void canInit(void);
void canReinit(void);
twai_handle_t canGetBus0(void);
twai_handle_t canGetBus1(void);
void canResetBuses(void);

#ifdef __cplusplus
}
#endif
