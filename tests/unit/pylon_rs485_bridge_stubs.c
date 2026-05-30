#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "Drivers/RS485/rs485_driver.h"
#include "Web_interface/web_bridge_api.h"
#include "protocols/common/battery_model.h"
#include "runtime_settings.h"

bridge_runtime_settings_t g_pylonBridgeTestSettings;
battery_model_t g_pylonBridgeTestModel;
bool g_pylonBridgeTestModelValid;
bridgeTelemetrySnapshot_t g_pylonBridgeLastTelemetry;
char g_pylonBridgeLastDecodedLog[2048];

void pylonBridgeStubReset(void)
{
    memset(&g_pylonBridgeTestSettings, 0, sizeof(g_pylonBridgeTestSettings));
    memset(&g_pylonBridgeTestModel, 0, sizeof(g_pylonBridgeTestModel));
    memset(&g_pylonBridgeLastTelemetry, 0, sizeof(g_pylonBridgeLastTelemetry));
    memset(g_pylonBridgeLastDecodedLog, 0, sizeof(g_pylonBridgeLastDecodedLog));
    g_pylonBridgeTestModelValid = false;
}

bridge_runtime_settings_t runtimeSettingsGet(void)
{
    return g_pylonBridgeTestSettings;
}

void batteryModelGet(battery_model_t *out)
{
    if (out != NULL) {
        *out = g_pylonBridgeTestModel;
        out->valid = g_pylonBridgeTestModelValid;
    }
}

void batteryModelGetWithStaleMs(battery_model_t *out, uint32_t staleMs)
{
    (void)staleMs;
    batteryModelGet(out);
}

void batteryModelGetReal(battery_model_t *out)
{
    batteryModelGet(out);
}

void batteryModelSet(const battery_model_t *model)
{
    if (model != NULL) {
        g_pylonBridgeTestModel = *model;
        g_pylonBridgeTestModelValid = model->valid;
    }
}

void batteryModelClear(void)
{
    memset(&g_pylonBridgeTestModel, 0, sizeof(g_pylonBridgeTestModel));
    g_pylonBridgeTestModelValid = false;
}

bool batteryModelIsDebugOverrideEnabled(void)
{
    return false;
}

void bridgeSetTelemetrySnapshot(const bridgeTelemetrySnapshot_t *snapshot)
{
    if (snapshot != NULL) {
        g_pylonBridgeLastTelemetry = *snapshot;
    } else {
        memset(&g_pylonBridgeLastTelemetry, 0, sizeof(g_pylonBridgeLastTelemetry));
    }
}

void bridgeSetDecodedLogSnapshot(const char *log)
{
    if (log != NULL) {
        snprintf(g_pylonBridgeLastDecodedLog, sizeof(g_pylonBridgeLastDecodedLog), "%s", log);
    } else {
        g_pylonBridgeLastDecodedLog[0] = '\0';
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

uint32_t rs485GetBaudRate(uart_port_t uart)
{
    (void)uart;
    return RS485_DEFAULT_BAUDRATE;
}

gpio_num_t rs485GetDir1(void)
{
    return 0;
}

gpio_num_t rs485GetDir2(void)
{
    return 0;
}

void rs485DriverSetTx(gpio_num_t dirPin, bool txEnable)
{
    (void)dirPin;
    (void)txEnable;
}

void rs485DriverWriteFrame(uart_port_t uart, gpio_num_t dirPin, const uint8_t *frame, int len)
{
    (void)uart;
    (void)dirPin;
    (void)frame;
    (void)len;
}
