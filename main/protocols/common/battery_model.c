#include "protocols/common/battery_model.h"

#include <inttypes.h>
#include <string.h>

#include "config.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static battery_model_t g_batteryModel;
static battery_model_t g_debugBatteryModel;
static bool g_debugBatteryModelEnabled;
static portMUX_TYPE g_batteryModelMux = portMUX_INITIALIZER_UNLOCKED;
static int64_t g_lastBatteryModelTraceUs;

static void batteryModelTraceWrite(const char *action, const battery_model_t *model)
{
    const int64_t nowUs = esp_timer_get_time();
    const char *taskName = pcTaskGetName(NULL);

    if ((nowUs - g_lastBatteryModelTraceUs) < 200000LL) {
        return;
    }
    g_lastBatteryModelTraceUs = nowUs;

    ESP_LOGI("BATTERY_MODEL",
             "%s by task=%s valid=%s V=%.2fV I=%.2fA soc=%u soh=%u cyc=%u chg=%u dis=%u bal=%u state=0x%02X upd=%" PRIu32,
             (action != NULL) ? action : "set",
             (taskName != NULL) ? taskName : "(null)",
             (model != NULL && model->valid) ? "YES" : "NO",
             (double)((model != NULL) ? model->packVoltageV : 0.0f),
             (double)((model != NULL) ? model->packCurrentA : 0.0f),
             (unsigned)((model != NULL) ? model->socPct : 0u),
             (unsigned)((model != NULL) ? model->sohPct : 0u),
             (unsigned)((model != NULL) ? model->cycleCount : 0u),
             (unsigned)((model != NULL && model->chargeEnabled) ? 1u : 0u),
             (unsigned)((model != NULL && model->dischargeEnabled) ? 1u : 0u),
             (unsigned)((model != NULL && model->balanceEnabled) ? 1u : 0u),
             (unsigned)((model != NULL) ? (model->protocolState & 0xFFu) : 0u),
             (uint32_t)((model != NULL) ? model->updatedMs : 0u));
}

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
    const uint32_t nowMs = batteryModelNowMs();

    if (out == NULL) {
        return;
    }

    portENTER_CRITICAL(&g_batteryModelMux);
    if (g_debugBatteryModelEnabled && g_debugBatteryModel.valid) {
        *out = g_debugBatteryModel;
        out->updatedMs = nowMs;
        portEXIT_CRITICAL(&g_batteryModelMux);
        return;
    }
    *out = g_batteryModel;
    portEXIT_CRITICAL(&g_batteryModelMux);

    if (!batteryModelIsFresh(out)) {
        memset(out, 0, sizeof(*out));
    }
}

void batteryModelGetReal(battery_model_t *out)
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
    battery_model_t logged = {0};

    portENTER_CRITICAL(&g_batteryModelMux);
    if (in == NULL) {
        memset(&g_batteryModel, 0, sizeof(g_batteryModel));
        logged = g_batteryModel;
    } else {
        g_batteryModel = *in;
        if (g_batteryModel.valid) {
            if (g_batteryModel.updatedMs == 0u) {
                g_batteryModel.updatedMs = nowMs;
            }
        } else {
            g_batteryModel.updatedMs = 0u;
        }
        logged = g_batteryModel;
    }
    portEXIT_CRITICAL(&g_batteryModelMux);

    batteryModelTraceWrite((in == NULL) ? "clear" : "set", &logged);
}

void batteryModelClear(void)
{
    batteryModelSet(NULL);
}

void batteryModelGetDebugOverride(battery_model_t *out, bool *enabledOut)
{
    if (enabledOut != NULL) {
        *enabledOut = false;
    }
    if (out != NULL) {
        memset(out, 0, sizeof(*out));
    }

    portENTER_CRITICAL(&g_batteryModelMux);
    if (enabledOut != NULL) {
        *enabledOut = g_debugBatteryModelEnabled;
    }
    if (out != NULL) {
        *out = g_debugBatteryModel;
    }
    portEXIT_CRITICAL(&g_batteryModelMux);
}

void batteryModelSetDebugOverride(const battery_model_t *in)
{
    const uint32_t nowMs = batteryModelNowMs();
    battery_model_t logged = {0};

    portENTER_CRITICAL(&g_batteryModelMux);
    if (in == NULL) {
        memset(&g_debugBatteryModel, 0, sizeof(g_debugBatteryModel));
        logged = g_debugBatteryModel;
    } else {
        g_debugBatteryModel = *in;
        if (g_debugBatteryModel.valid) {
            g_debugBatteryModel.updatedMs = nowMs;
        } else {
            g_debugBatteryModel.updatedMs = 0u;
        }
        logged = g_debugBatteryModel;
    }
    portEXIT_CRITICAL(&g_batteryModelMux);

    batteryModelTraceWrite((in == NULL) ? "debug_clear" : "debug_set", &logged);
}

void batteryModelSetDebugOverrideEnabled(bool enabled)
{
    portENTER_CRITICAL(&g_batteryModelMux);
    g_debugBatteryModelEnabled = enabled && g_debugBatteryModel.valid;
    portEXIT_CRITICAL(&g_batteryModelMux);
}

bool batteryModelIsDebugOverrideEnabled(void)
{
    bool enabled = false;

    portENTER_CRITICAL(&g_batteryModelMux);
    enabled = g_debugBatteryModelEnabled && g_debugBatteryModel.valid;
    portEXIT_CRITICAL(&g_batteryModelMux);
    return enabled;
}
