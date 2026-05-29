#pragma once

#include <stdbool.h>

#include "protocols/common/universal_battery_model.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef universal_battery_model_t battery_model_t;

bool batteryModelIsFresh(const battery_model_t *model);
void batteryModelGet(battery_model_t *out);
void batteryModelGetWithStaleMs(battery_model_t *out, uint32_t staleMs);
void batteryModelGetReal(battery_model_t *out);
void batteryModelSet(const battery_model_t *in);
void batteryModelClear(void);
void batteryModelGetDebugOverride(battery_model_t *out, bool *enabledOut);
void batteryModelSetDebugOverride(const battery_model_t *in);
void batteryModelSetDebugOverrideEnabled(bool enabled);
bool batteryModelIsDebugOverrideEnabled(void);

#ifdef __cplusplus
}
#endif
