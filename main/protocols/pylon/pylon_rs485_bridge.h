#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "runtime_settings.h"

#ifdef __cplusplus
extern "C" {
#endif

bool pylonRs485BridgeSupportsRoute(const bridge_runtime_settings_t *settings);
bool pylonRs485BridgeHandlesCurrentConfig(void);
void pylonRs485BridgeEnable(void);
void pylonRs485BridgeStop(void);

#ifdef HOST_TEST
bool pylonRs485BridgeBuildSyntheticInfo61ForTest(char *out, size_t outSize);
bool pylonRs485BridgeBuildSyntheticInfo63ForTest(char *out, size_t outSize);
#endif

#ifdef __cplusplus
}
#endif
