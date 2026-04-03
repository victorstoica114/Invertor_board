#pragma once

#include <stdbool.h>

#include "protocols/common/universal_battery_model.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef universal_battery_model_t battery_model_t;

bool batteryModelIsFresh(const battery_model_t *model);
void batteryModelGet(battery_model_t *out);
void batteryModelSet(const battery_model_t *in);
void batteryModelClear(void);

#ifdef __cplusplus
}
#endif
