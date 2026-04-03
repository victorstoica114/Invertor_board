#include "protocols/common/battery_model.h"

#include <string.h>

#include "config.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

static battery_model_t g_batteryModel;
static portMUX_TYPE g_batteryModelMux = portMUX_INITIALIZER_UNLOCKED;

static uint32_t batteryModelNowMs(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000LL);
}

bool batteryModelIsFresh(const battery_model_t *model)
{
    const uint32_t nowMs = batteryModelNowMs();

    if (model == NULL || !model->valid || model->updatedMs == 0u) {
        return false;
    }

    return (nowMs - model->updatedMs) <= BRIDGE_SOURCE_STALE_MS;
}

void batteryModelGet(battery_model_t *out)
{
    if (out == NULL) {
        return;
    }

    portENTER_CRITICAL(&g_batteryModelMux);
    *out = g_batteryModel;
    portEXIT_CRITICAL(&g_batteryModelMux);

    if (!batteryModelIsFresh(out)) {
        memset(out, 0, sizeof(*out));
    }
}

void batteryModelSet(const battery_model_t *in)
{
    const uint32_t nowMs = batteryModelNowMs();

    portENTER_CRITICAL(&g_batteryModelMux);
    if (in == NULL) {
        memset(&g_batteryModel, 0, sizeof(g_batteryModel));
    } else {
        g_batteryModel = *in;
        if (g_batteryModel.valid) {
            if (g_batteryModel.updatedMs == 0u) {
                g_batteryModel.updatedMs = nowMs;
            }
        } else {
            g_batteryModel.updatedMs = 0u;
        }
    }
    portEXIT_CRITICAL(&g_batteryModelMux);
}

void batteryModelClear(void)
{
    batteryModelSet(NULL);
}
