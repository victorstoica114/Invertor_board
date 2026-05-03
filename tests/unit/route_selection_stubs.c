#include <stdbool.h>
#include <string.h>

#include "decoders/CAN_Decoder.h"
#include "Drivers/can_driver.h"
#include "Drivers/RS485/rs485_driver.h"
#include "esp_err.h"
#include "modes/can_forward_sniffer.h"
#include "protocols/common/battery_model.h"
#include "protocols/growatt/growatt_bms_task.h"
#include "protocols/growatt/growatt_inverter_task.h"
#include "protocols/jkbms_modbus/jkbms_modbus_bms_task.h"
#include "protocols/pace_modbus/pace_modbus_bms_task.h"
#include "protocols/pylon/pylon_bms_task.h"
#include "protocols/pylon/pylon_inverter_task.h"
#include "protocols/pylon/pylon_rs485_bridge.h"
#include "protocols/rs485_growatt/rs485_growatt_bridge.h"
#include "runtime_settings.h"

bridge_runtime_settings_t runtimeSettingsGet(void)
{
    bridge_runtime_settings_t s = {0};
    return s;
}

void batteryModelGet(battery_model_t *out)
{
    if (out != NULL) {
        memset(out, 0, sizeof(*out));
    }
}

bool batteryModelIsDebugOverrideEnabled(void)
{
    return false;
}

esp_err_t growattBmsTaskStart(QueueHandle_t outQueue)
{
    (void)outQueue;
    return ESP_OK;
}

esp_err_t growattBmsTaskStop(void)
{
    return ESP_OK;
}

esp_err_t pylonBmsTaskStart(QueueHandle_t outQueue)
{
    (void)outQueue;
    return ESP_OK;
}

esp_err_t pylonBmsTaskStop(void)
{
    return ESP_OK;
}

esp_err_t jkbmsModbusBmsTaskStart(QueueHandle_t outQueue)
{
    (void)outQueue;
    return ESP_OK;
}

esp_err_t jkbmsModbusBmsTaskStop(void)
{
    return ESP_OK;
}

esp_err_t paceModbusBmsTaskStart(QueueHandle_t outQueue)
{
    (void)outQueue;
    return ESP_OK;
}

esp_err_t paceModbusBmsTaskStop(void)
{
    return ESP_OK;
}

esp_err_t growattInverterTaskStart(QueueHandle_t inQueue)
{
    (void)inQueue;
    return ESP_OK;
}

esp_err_t growattInverterTaskStop(void)
{
    return ESP_OK;
}

esp_err_t pylonInverterTaskStart(QueueHandle_t inQueue)
{
    (void)inQueue;
    return ESP_OK;
}

esp_err_t pylonInverterTaskStop(void)
{
    return ESP_OK;
}

twai_handle_t canGetBus0(void)
{
    return NULL;
}

twai_handle_t canGetBus1(void)
{
    return NULL;
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

bool pylonRs485BridgeSupportsRoute(const bridge_runtime_settings_t *settings)
{
    (void)settings;
    return false;
}

void pylonRs485BridgeEnable(void)
{
}

void pylonRs485BridgeStop(void)
{
}

esp_err_t canRs485GrowattBridgeEnable(uart_port_t inverterUart,
                                      gpio_num_t inverterDir,
                                      const char *ifName,
                                      twai_handle_t srcCanBus,
                                      const char *srcCanIf)
{
    (void)inverterUart;
    (void)inverterDir;
    (void)ifName;
    (void)srcCanBus;
    (void)srcCanIf;
    return ESP_OK;
}

esp_err_t jkbmsRs485GrowattBridgeEnable(uart_port_t inverterUart,
                                        gpio_num_t inverterDir,
                                        const char *ifName)
{
    (void)inverterUart;
    (void)inverterDir;
    (void)ifName;
    return ESP_OK;
}

void canRs485GrowattBridgeStop(void)
{
}

void canDecoderResetCaches(void)
{
}

void canForwardSnifferStart(const bridge_runtime_settings_t *settings)
{
    (void)settings;
}

void canForwardSnifferStop(void)
{
}
