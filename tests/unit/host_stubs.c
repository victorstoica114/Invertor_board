#include <stdint.h>
#include <string.h>

#include "runtime_settings.h"
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

void bridgeSetTelemetrySnapshot(const char *ifname, const void *snapshot, uint32_t snapshotSize)
{
    (void)ifname;
    (void)snapshot;
    (void)snapshotSize;
}

void bridgeSetDecodedLogSnapshot(const char *ifname, const char *log)
{
    (void)ifname;
    (void)log;
}
