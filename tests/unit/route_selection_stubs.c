#include <stdbool.h>
#include <string.h>

#include "decoders/CAN_Decoder.h"
#include "Drivers/can_driver.h"
#include "Drivers/RS485/rs485_driver.h"
#include "config.h"
#include "esp_err.h"
#include "modes/can_forward_sniffer.h"
#include "protocols/china_tower_modbus/china_tower_modbus_bms_task.h"
#include "protocols/common/battery_model.h"
#include "protocols/daly_can/daly_can_bms_task.h"
#include "protocols/daly_rs485/daly_rs485_bms_task.h"
#include "protocols/growatt/growatt_bms_task.h"
#include "protocols/growatt/growatt_inverter_task.h"
#include "protocols/jkbms_modbus/jkbms_modbus_bms_task.h"
#include "protocols/jkbms_rs485/jkbms_rs485_bms_task.h"
#include "protocols/pace_modbus/pace_modbus_bms_task.h"
#include "protocols/pylon/pylon_bms_task.h"
#include "protocols/pylon/pylon_inverter_task.h"
#include "protocols/pylon/pylon_rs485_bridge.h"
#include "protocols/rs485_growatt/rs485_growatt_bridge.h"
#include "protocols/seplos_rs485/seplos_rs485_bms_task.h"
#include "protocols/voltronic_modbus/voltronic_modbus_bms_task.h"
#include "protocols/wow_modbus/wow_modbus_bms_task.h"
#include "runtime_settings.h"

int g_routeStubGrowattBmsStartCount;
int g_routeStubJkbmsModbusBmsStartCount;
int g_routeStubVoltronicBmsStartCount;
int g_routeStubChinaTowerBmsStartCount;
int g_routeStubWowBmsStartCount;
int g_routeStubSeplosBmsStartCount;
int g_routeStubDalyRs485BmsStartCount;
int g_routeStubDalyCanBmsStartCount;
int g_routeStubPylonInverterStartCount;
int g_routeStubPylonBridgeEnableCount;
int g_routeStubCanForwardStartCount;
int g_routeStubRs485GrowattResponderStartCount;

void routeSelectionStubReset(void)
{
    g_routeStubGrowattBmsStartCount = 0;
    g_routeStubJkbmsModbusBmsStartCount = 0;
    g_routeStubVoltronicBmsStartCount = 0;
    g_routeStubChinaTowerBmsStartCount = 0;
    g_routeStubWowBmsStartCount = 0;
    g_routeStubSeplosBmsStartCount = 0;
    g_routeStubDalyRs485BmsStartCount = 0;
    g_routeStubDalyCanBmsStartCount = 0;
    g_routeStubPylonInverterStartCount = 0;
    g_routeStubPylonBridgeEnableCount = 0;
    g_routeStubCanForwardStartCount = 0;
    g_routeStubRs485GrowattResponderStartCount = 0;
}

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
    g_routeStubGrowattBmsStartCount++;
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
    g_routeStubJkbmsModbusBmsStartCount++;
    return ESP_OK;
}

esp_err_t jkbmsModbusBmsTaskStop(void)
{
    return ESP_OK;
}

esp_err_t jkbmsRs485BmsTaskStart(QueueHandle_t outQueue)
{
    (void)outQueue;
    return ESP_OK;
}

esp_err_t jkbmsRs485BmsTaskStop(void)
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

esp_err_t voltronicModbusBmsTaskStart(QueueHandle_t outQueue)
{
    (void)outQueue;
    g_routeStubVoltronicBmsStartCount++;
    return ESP_OK;
}

esp_err_t voltronicModbusBmsTaskStop(void)
{
    return ESP_OK;
}

esp_err_t chinaTowerModbusBmsTaskStart(QueueHandle_t outQueue)
{
    (void)outQueue;
    g_routeStubChinaTowerBmsStartCount++;
    return ESP_OK;
}

esp_err_t chinaTowerModbusBmsTaskStop(void)
{
    return ESP_OK;
}

esp_err_t wowModbusBmsTaskStart(QueueHandle_t outQueue)
{
    (void)outQueue;
    g_routeStubWowBmsStartCount++;
    return ESP_OK;
}

esp_err_t wowModbusBmsTaskStop(void)
{
    return ESP_OK;
}

esp_err_t seplosRs485BmsTaskStart(QueueHandle_t outQueue)
{
    (void)outQueue;
    g_routeStubSeplosBmsStartCount++;
    return ESP_OK;
}

esp_err_t seplosRs485BmsTaskStop(void)
{
    return ESP_OK;
}

bool seplosRs485BmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket)
{
    (void)outPacket;
    return false;
}

bool seplosRs485BmsTaskGetLatestSnapshot(seplos_rs485_snapshot_t *outSnapshot)
{
    (void)outSnapshot;
    return false;
}

esp_err_t dalyRs485BmsTaskStart(QueueHandle_t outQueue)
{
    (void)outQueue;
    g_routeStubDalyRs485BmsStartCount++;
    return ESP_OK;
}

esp_err_t dalyRs485BmsTaskStop(void)
{
    return ESP_OK;
}

bool dalyRs485BmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket)
{
    (void)outPacket;
    return false;
}

bool dalyRs485BmsTaskGetLatestSnapshot(daly_rs485_snapshot_t *outSnapshot)
{
    (void)outSnapshot;
    return false;
}

esp_err_t dalyCanBmsTaskStart(QueueHandle_t outQueue)
{
    (void)outQueue;
    g_routeStubDalyCanBmsStartCount++;
    return ESP_OK;
}

esp_err_t dalyCanBmsTaskStop(void)
{
    return ESP_OK;
}

bool dalyCanBmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket)
{
    (void)outPacket;
    return false;
}

bool dalyCanBmsTaskGetLatestSnapshot(daly_rs485_snapshot_t *outSnapshot)
{
    (void)outSnapshot;
    return false;
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
    g_routeStubPylonInverterStartCount++;
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
    if (settings == NULL) {
        return false;
    }

    return ((settings->bms_line == LINE_RS485) &&
            (settings->inverter_line == LINE_RS485) &&
            bridgeProtocolIsRs485Pylon(settings->bms_protocol) &&
            bridgeProtocolIsRs485Pylon(settings->inverter_protocol)) ||
           ((settings->mode == MODE_BRIDGE) &&
            (settings->bms_line == LINE_RS485) &&
            (settings->inverter_line == LINE_RS485) &&
            bridgeProtocolIsRs485Pylon(settings->bms_protocol) &&
            (settings->inverter_protocol == PROTOCOL_RS485_GROWATT)) ||
           ((settings->mode == MODE_BRIDGE) &&
            (settings->bms_line == LINE_RS485) &&
            (settings->inverter_line == LINE_CAN) &&
            bridgeProtocolIsRs485Pylon(settings->bms_protocol) &&
            (settings->inverter_protocol == PROTOCOL_CAN_PYLON)) ||
           ((settings->bms_line == LINE_CAN) &&
            (settings->inverter_line == LINE_RS485) &&
            ((settings->bms_protocol == PROTOCOL_CAN_PYLON) ||
             (settings->bms_protocol == PROTOCOL_CAN_GROWATT) ||
             (settings->bms_protocol == PROTOCOL_CAN_DEYE) ||
             (settings->bms_protocol == PROTOCOL_CAN_DALY) ||
             (settings->bms_protocol == PROTOCOL_CAN_JKBMS_250K)) &&
            bridgeProtocolIsRs485Pylon(settings->inverter_protocol));
}

void pylonRs485BridgeEnable(void)
{
    g_routeStubPylonBridgeEnableCount++;
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
    g_routeStubRs485GrowattResponderStartCount++;
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
    g_routeStubCanForwardStartCount++;
}

void canForwardSnifferStop(void)
{
}
