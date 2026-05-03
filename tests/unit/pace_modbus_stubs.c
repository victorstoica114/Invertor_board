#include <stddef.h>
#include <string.h>

#include "Drivers/rs485_driver.h"
#include "Web_interface/web_bridge_api.h"
#include "protocols/common/battery_model.h"
#include "runtime_settings.h"

bridge_runtime_settings_t runtimeSettingsGet(void)
{
    bridge_runtime_settings_t s = {0};
    s.bms_port = 1u;
    return s;
}

void batteryModelSet(const battery_model_t *model)
{
    (void)model;
}

void batteryModelClear(void)
{
}

bool batteryModelIsDebugOverrideEnabled(void)
{
    return false;
}

void batteryModelGet(battery_model_t *out)
{
    if (out != NULL) {
        memset(out, 0, sizeof(*out));
    }
}

uart_port_t rs485GetUart1(void)
{
    return 0;
}

uart_port_t rs485GetUart2(void)
{
    return 1;
}

gpio_num_t rs485GetDir1(void)
{
    return 0;
}

gpio_num_t rs485GetDir2(void)
{
    return 0;
}

esp_err_t rs485WriteBytes(uart_port_t uart,
                          gpio_num_t dirPin,
                          const uint8_t *data,
                          int len,
                          TickType_t txTimeoutTicks)
{
    (void)uart;
    (void)dirPin;
    (void)data;
    (void)len;
    (void)txTimeoutTicks;
    return ESP_OK;
}

void bridgeSetTelemetrySnapshot(const bridgeTelemetrySnapshot_t *snapshot)
{
    (void)snapshot;
}

void bridgeSetDecodedLogSnapshot(const char *log)
{
    (void)log;
}
