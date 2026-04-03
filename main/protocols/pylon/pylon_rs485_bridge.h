#pragma once

#include <stdbool.h>

#include "runtime_settings.h"

#ifdef __cplusplus
extern "C" {
#endif

bool pylonRs485BridgeSupportsRoute(const bridge_runtime_settings_t *settings);
bool pylonRs485BridgeHandlesCurrentConfig(void);
void pylonRs485BridgeEnable(void);
void pylonRs485BridgeStop(void);

#ifdef __cplusplus
}
#endif
