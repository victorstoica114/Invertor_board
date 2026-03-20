#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool pylonRs485BridgeHandlesCurrentConfig(void);
void pylonRs485BridgeEnable(void);
void pylonRs485BridgeStop(void);

#ifdef __cplusplus
}
#endif
