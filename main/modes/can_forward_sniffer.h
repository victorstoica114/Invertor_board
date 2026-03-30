#pragma once

#include "runtime_settings.h"

#ifdef __cplusplus
extern "C" {
#endif

void canForwardSnifferStart(const bridge_runtime_settings_t *settings);
void canForwardSnifferStop(void);

#ifdef __cplusplus
}
#endif
