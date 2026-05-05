#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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
bool pylonRs485BridgeProbeShouldWaitForQuietForTest(uint8_t mode,
                                                    int64_t nowUs,
                                                    int64_t lastBmsTrafficUs,
                                                    int64_t lastInverterTrafficUs);
#endif

#ifdef __cplusplus
}
#endif
