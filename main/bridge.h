#pragma once

#include "Web_interface/web_bridge_api.h"
#include "Protocols/Common/universal_battery_model.h"

#ifdef __cplusplus
extern "C" {
#endif

void bridgeGetUniversalBatteryModel(universal_battery_model_t *out);
void bridgeSetUniversalBatteryModel(const universal_battery_model_t *in);

#ifdef __cplusplus
}
#endif
