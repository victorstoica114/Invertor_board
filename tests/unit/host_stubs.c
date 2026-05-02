#include <stdint.h>
#include <string.h>

#include "runtime_settings.h"
#include "Web_interface/web_bridge_api.h"
#include "protocols/common/battery_model.h"

// Host-only stubs to allow building decoder logic without ESP-IDF.

bridge_runtime_settings_t runtimeSettingsGet(void)
{
    bridge_runtime_settings_t s = {0};
    return s;
}

void batteryModelGetReal(battery_model_t *out)
{
    if (out != NULL) {
        memset(out, 0, sizeof(*out));
    }
}

void batteryModelSet(const battery_model_t *model)
{
    (void)model;
}

void bridgeSetTelemetrySnapshot(const bridgeTelemetrySnapshot_t *snapshot)
{
    (void)snapshot;
}

void bridgeSetDecodedLogSnapshot(const char *log)
{
    (void)log;
}
