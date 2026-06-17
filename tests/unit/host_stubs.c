#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "runtime_settings.h"
#include "Web_interface/web_bridge_api.h"
#include "protocols/common/battery_model.h"

// Host-only stubs to allow building decoder logic without ESP-IDF.

bridge_runtime_settings_t g_hostRuntimeSettings;
battery_model_t g_hostBatteryModel;
bool g_hostBatteryModelValid;

void hostStubsReset(void)
{
    memset(&g_hostRuntimeSettings, 0, sizeof(g_hostRuntimeSettings));
    memset(&g_hostBatteryModel, 0, sizeof(g_hostBatteryModel));
    g_hostBatteryModelValid = false;
}

bridge_runtime_settings_t runtimeSettingsGet(void)
{
    return g_hostRuntimeSettings;
}

void batteryModelGetReal(battery_model_t *out)
{
    if (out != NULL) {
        *out = g_hostBatteryModel;
    }
}

void batteryModelSet(const battery_model_t *model)
{
    if (model != NULL) {
        g_hostBatteryModel = *model;
        g_hostBatteryModelValid = model->valid;
    } else {
        memset(&g_hostBatteryModel, 0, sizeof(g_hostBatteryModel));
        g_hostBatteryModelValid = false;
    }
}

void batteryModelClear(void)
{
    batteryModelSet(NULL);
}

void bridgeSetTelemetrySnapshot(const bridgeTelemetrySnapshot_t *snapshot)
{
    (void)snapshot;
}

void bridgeSetDecodedLogSnapshot(const char *log)
{
    (void)log;
}
