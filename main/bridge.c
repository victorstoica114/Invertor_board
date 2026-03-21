#include "bridge.h"

#include <stdio.h>
#include <string.h>

#include "config.h"
#include "runtime_settings.h"
#include "Drivers/CAN/can_driver.h"
#include "Drivers/RS485/rs485_driver.h"
#include "Operation_Modes/can_forward_sniffer.h"
#include "Operation_Modes/rs485_forward_sniffer.h"
#include "BMS_Protocols/Pylon/pylon_rs485_bridge.h"
#include "rs485_can_bridge.h"

static bridgeTelemetrySnapshot_t gBridgeTelemetry;
static universal_battery_model_t gUniversalBatteryModel;
static char gBridgeDecodedLog[2048];

static const char *canNameByPort(int port)
{
    return (port == 1) ? "CAN1" : "CAN2";
}

static twai_handle_t canBusByPort(int port)
{
    return (port == 1) ? canGetBus0() : canGetBus1();
}

static const char *rsNameByPort(int port)
{
    return (port == 1) ? "RS485_1" : "RS485_2";
}

static uart_port_t rsUartByPort(int port)
{
    return (port == 1) ? rs485GetUart1() : rs485GetUart2();
}

static gpio_num_t rsDirByPort(int port)
{
    return (port == 1) ? rs485GetDir1() : rs485GetDir2();
}

void bridgeGetTelemetrySnapshot(bridgeTelemetrySnapshot_t *out)
{
    bridge_runtime_settings_t settings = runtimeSettingsGet();

    if (out == NULL) {
        return;
    }

    *out = gBridgeTelemetry;

    if (out->source[0] == '\0') {
        snprintf(out->source, sizeof(out->source), "runtime");
    }
    if (out->protocol[0] == '\0') {
        if (settings.bms_protocol == PROTOCOL_RS485_PYLON || settings.inverter_protocol == PROTOCOL_RS485_PYLON) {
            snprintf(out->protocol, sizeof(out->protocol), "RS485_PYLON");
        } else if (settings.bms_protocol == PROTOCOL_CAN_PYLON || settings.inverter_protocol == PROTOCOL_CAN_PYLON) {
            snprintf(out->protocol, sizeof(out->protocol), "CAN_PYLON");
        } else if (settings.bms_protocol == PROTOCOL_RS485_GROWATT || settings.inverter_protocol == PROTOCOL_RS485_GROWATT) {
            snprintf(out->protocol, sizeof(out->protocol), "RS485_GROWATT");
        } else {
            snprintf(out->protocol, sizeof(out->protocol), "CAN_GROWATT");
        }
    }
}

void bridgeSetTelemetrySnapshot(const bridgeTelemetrySnapshot_t *in)
{
    if (in == NULL) {
        memset(&gBridgeTelemetry, 0, sizeof(gBridgeTelemetry));
        memset(&gUniversalBatteryModel, 0, sizeof(gUniversalBatteryModel));
        return;
    }

    gBridgeTelemetry = *in;
    gUniversalBatteryModel.valid = in->valid;
    gUniversalBatteryModel.packCurrentA = in->currentA;
    gUniversalBatteryModel.socPct = in->socPct;
    gUniversalBatteryModel.sohPct = in->sohPct;
    gUniversalBatteryModel.cycleCount = in->cycles;
    gUniversalBatteryModel.cellMaxV = in->cellMaxV;
    gUniversalBatteryModel.cellMinV = in->cellMinV;
    gUniversalBatteryModel.cellMaxIdx = in->cellMaxIdx;
    gUniversalBatteryModel.cellMinIdx = in->cellMinIdx;
    gUniversalBatteryModel.cellDeltaV = in->deltaV;
    gUniversalBatteryModel.temperaturesC[0] = in->tempMosC;
    gUniversalBatteryModel.temperaturesC[1] = in->tempT1C;
    gUniversalBatteryModel.temperaturesC[2] = in->tempT2C;
    gUniversalBatteryModel.temperaturesC[3] = in->tempT4C;
    gUniversalBatteryModel.temperaturesC[4] = in->tempT5C;
    gUniversalBatteryModel.protocolState = in->pylonStatus63;
}

void bridgeGetUniversalBatteryModel(universal_battery_model_t *out)
{
    if (out == NULL) {
        return;
    }

    *out = gUniversalBatteryModel;
}

void bridgeSetUniversalBatteryModel(const universal_battery_model_t *in)
{
    if (in == NULL) {
        memset(&gUniversalBatteryModel, 0, sizeof(gUniversalBatteryModel));
        return;
    }

    gUniversalBatteryModel = *in;
}

void bridgeGetDecodedLogSnapshot(char *out, uint32_t outSize)
{
    if (out == NULL || outSize == 0) {
        return;
    }

    snprintf(out, outSize, "%s", gBridgeDecodedLog);
}

void bridgeSetDecodedLogSnapshot(const char *text)
{
    if (text == NULL) {
        gBridgeDecodedLog[0] = '\0';
        return;
    }

    snprintf(gBridgeDecodedLog, sizeof(gBridgeDecodedLog), "%s", text);
}

void canBridgeEnable(void)
{
    bridge_runtime_settings_t settings = runtimeSettingsGet();

    canForwardSnifferStart(&settings);
}

void rs485BridgeEnable(void)
{
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    const bool bmsToCanGrowatt =
        (settings.bms_line == LINE_RS485) &&
        (settings.inverter_line == LINE_CAN) &&
        (settings.bms_protocol == PROTOCOL_RS485_GROWATT) &&
        (settings.inverter_protocol == PROTOCOL_CAN_GROWATT);
    const bool canToInvRs485Growatt =
        (settings.bms_line == LINE_CAN) &&
        (settings.inverter_line == LINE_RS485) &&
        (settings.bms_protocol == PROTOCOL_CAN_GROWATT) &&
        (settings.inverter_protocol == PROTOCOL_RS485_GROWATT);

    pylonRs485BridgeStop();
    rs485Can322BridgeStop();
    canRs485SocBridgeStop();
    rs485ForwardSnifferStop();

    if (pylonRs485BridgeHandlesCurrentConfig()) {
        pylonRs485BridgeEnable();
        return;
    }

    rs485ForwardSnifferStart(&settings);

    if (bmsToCanGrowatt) {
        rs485Can322BridgeEnable(rs485ForwardSnifferGetDecoder(settings.bms_port),
                                canBusByPort(settings.inverter_port),
                                canNameByPort(settings.inverter_port));
    } else if (canToInvRs485Growatt) {
        canRs485SocBridgeEnable(rsUartByPort(settings.inverter_port),
                                rsDirByPort(settings.inverter_port),
                                rsNameByPort(settings.inverter_port));
    }
}

void bridgeReloadFromRuntimeSettings(void)
{
    rs485BridgeEnable();
    canBridgeEnable();
}
