#include "bridge.h"

#include <stdio.h>
#include <string.h>

#include "config.h"
#include "CAN_Decoder.h"
#include "runtime_settings.h"
#include "Drivers/CAN/can_driver.h"
#include "Drivers/RS485/rs485_driver.h"
#include "Operation_Modes/can_forward_sniffer.h"
#include "Operation_Modes/rs485_forward_sniffer.h"
#include "BMS_Protocols/Pylon/pylon_rs485_bridge.h"
#include "rs485_can_bridge.h"
#include "esp_timer.h"

static bridgeTelemetrySnapshot_t gBridgeTelemetry;
static universal_battery_model_t gUniversalBatteryModel;
static char gBridgeDecodedLog[2048];

static uint32_t bridgeNowMs(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000LL);
}

static bool bridgeUniversalBatteryModelIsFresh(const universal_battery_model_t *model)
{
    uint32_t nowMs = bridgeNowMs();

    if (model == NULL || !model->valid || model->updatedMs == 0u) {
        return false;
    }
    return (nowMs - model->updatedMs) <= BRIDGE_SOURCE_STALE_MS;
}

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

static bool bridgeTryBuildGrowattRs485Telemetry(const bridge_runtime_settings_t *settings,
                                                bridgeTelemetrySnapshot_t *out)
{
    modbusDecoder_t *dec = NULL;
    uint16_t soc = 0;
    uint16_t packAbsICa = 0;
    uint16_t soh = 0;
    uint16_t cycles = 0;
    uint16_t cmaxMv = 0;
    uint16_t cminMv = 0;
    uint16_t cmaxIdx = 0;
    uint16_t cminIdx = 0;
    uint16_t temp = 0;
    uint8_t cellCount = 0;

    if (settings == NULL || out == NULL) {
        return false;
    }
    if (settings->bms_line != LINE_RS485 || settings->bms_protocol != PROTOCOL_RS485_GROWATT) {
        return false;
    }

    dec = rs485ForwardSnifferGetDecoder(settings->bms_port);
    if (dec == NULL || !modbusDecoderHasFreshData(dec, BRIDGE_SOURCE_STALE_MS)) {
        return false;
    }

    if (!modbusDecoderGetReg(dec, GROWATT_MB_REG_SOC_PCT, &soc) ||
        !modbusDecoderGetReg(dec, GROWATT_MB_REG_CELL_MAX_MV, &cmaxMv) ||
        !modbusDecoderGetReg(dec, GROWATT_MB_REG_CELL_MIN_MV, &cminMv) ||
        !modbusDecoderGetReg(dec, GROWATT_MB_REG_CELL_MAX_IDX, &cmaxIdx) ||
        !modbusDecoderGetReg(dec, GROWATT_MB_REG_CELL_MIN_IDX, &cminIdx)) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->valid = true;
    snprintf(out->source, sizeof(out->source), "%s cache", dec->ifName != NULL ? dec->ifName : rsNameByPort(settings->bms_port));
    snprintf(out->protocol, sizeof(out->protocol), "RS485_GROWATT");
    out->socPct = (uint8_t)soc;
    if (modbusDecoderGetReg(dec, GROWATT_MB_REG_PACK_I_ABS_CA_TENTATIVE, &packAbsICa)) {
        out->currentA = (float)packAbsICa / 100.0f;
    }
    if (modbusDecoderGetReg(dec, GROWATT_MB_REG_SOH_PCT, &soh)) {
        out->sohPct = (uint8_t)soh;
    }
    if (modbusDecoderGetReg(dec, GROWATT_MB_REG_CYCLE_COUNT_TENTATIVE, &cycles)) {
        out->cycles = cycles;
    }
    if (modbusDecoderGetReg(dec, GROWATT_MB_REG_TEMP_C, &temp)) {
        out->tempMosC = (float)(int16_t)temp;
    }
    out->cellMaxV = (float)cmaxMv / 1000.0f;
    out->cellMinV = (float)cminMv / 1000.0f;
    out->cellMaxIdx = (uint8_t)cmaxIdx;
    out->cellMinIdx = (uint8_t)cminIdx;
    out->deltaV = out->cellMaxV - out->cellMinV;

    for (uint8_t i = 0; i < 16u; i++) {
        uint16_t mv = 0;
        if (!modbusDecoderGetReg(dec, (uint16_t)(GROWATT_MB_REG_CELL_BASE + i), &mv)) {
            continue;
        }
        out->cellVoltagesV[i] = (float)mv / 1000.0f;
        cellCount = (uint8_t)(i + 1u);
    }
    out->cellCount = cellCount;
    return true;
}

static bool bridgeBuildGenericDecodedLog(const bridgeTelemetrySnapshot_t *snap,
                                         const char *details,
                                         char *out,
                                         uint32_t outSize)
{
    int pos = 0;

    if (snap == NULL || out == NULL || outSize == 0u) {
        return false;
    }

    out[0] = '\0';

    pos += snprintf(out + pos, outSize - (uint32_t)pos,
                    "Telemetry\n"
                    "  valid   : %s\n"
                    "  source  : %s\n"
                    "  protocol: %s\n"
                    "  pack    : I=%.2fA  SOC=%u%%  SOH=%u%%  cycles=%u\n"
                    "  cells   : max=%.3fV#%u  min=%.3fV#%u  dV=%.3fV  count=%u\n"
                    "  temps   : MOS=%.1fC  T1=%.1fC  T2=%.1fC  T4=%.1fC  T5=%.1fC\n",
                    snap->valid ? "YES" : "NO",
                    snap->source[0] != '\0' ? snap->source : "runtime",
                    snap->protocol[0] != '\0' ? snap->protocol : "unknown",
                    (double)snap->currentA,
                    (unsigned)snap->socPct,
                    (unsigned)snap->sohPct,
                    (unsigned)snap->cycles,
                    (double)snap->cellMaxV,
                    (unsigned)snap->cellMaxIdx,
                    (double)snap->cellMinV,
                    (unsigned)snap->cellMinIdx,
                    (double)snap->deltaV,
                    (unsigned)snap->cellCount,
                    (double)snap->tempMosC,
                    (double)snap->tempT1C,
                    (double)snap->tempT2C,
                    (double)snap->tempT4C,
                    (double)snap->tempT5C);

    if (snap->protections[0] != '\0') {
        pos += snprintf(out + pos, outSize - (uint32_t)pos, "  prot    : %s\n", snap->protections);
    }
    if (snap->alarms[0] != '\0') {
        pos += snprintf(out + pos, outSize - (uint32_t)pos, "  alarms  : %s\n", snap->alarms);
    }
    if (snap->warnings[0] != '\0') {
        pos += snprintf(out + pos, outSize - (uint32_t)pos, "  warns   : %s\n", snap->warnings);
    }
    if (snap->stateFlags[0] != '\0') {
        pos += snprintf(out + pos, outSize - (uint32_t)pos,
                        "  state   : 0x35C=0x%02X  %s\n",
                        (unsigned)snap->deyeStatus35C,
                        snap->stateFlags);
    }

    if (snap->cellCount > 0u) {
        for (uint8_t base = 0u; base < snap->cellCount && pos < (int)outSize; base = (uint8_t)(base + 4u)) {
            pos += snprintf(out + pos, outSize - (uint32_t)pos, "  cell%02u-%02u: ",
                            (unsigned)(base + 1u),
                            (unsigned)((base + 4u <= snap->cellCount) ? (base + 4u) : snap->cellCount));
            for (uint8_t i = base; i < snap->cellCount && i < (uint8_t)(base + 4u) && pos < (int)outSize; i++) {
                pos += snprintf(out + pos, outSize - (uint32_t)pos,
                                "%sC%02u=%.3fV",
                                (i == base) ? "" : "  ",
                                (unsigned)(i + 1u),
                                (double)snap->cellVoltagesV[i]);
            }
            pos += snprintf(out + pos, outSize - (uint32_t)pos, "\n");
        }
    }

    if (details != NULL && details[0] != '\0') {
        pos += snprintf(out + pos, outSize - (uint32_t)pos, "\nDetails\n%s", details);
    }

    return pos > 0;
}

static bool bridgeTryAttachGrowattCanAlerts(const bridge_runtime_settings_t *settings,
                                            bridgeTelemetrySnapshot_t *out)
{
    const char *ifname = NULL;

    if (settings == NULL || out == NULL) {
        return false;
    }
    if (settings->bms_line != LINE_CAN || settings->bms_protocol != PROTOCOL_CAN_GROWATT) {
        return false;
    }

    ifname = canNameByPort(settings->bms_port);
    if (!canDecoderHasFreshData(ifname, BRIDGE_SOURCE_STALE_MS)) {
        return false;
    }

    canDecoderGetGrowattAlertText(ifname,
                                  out->protections, sizeof(out->protections),
                                  out->alarms, sizeof(out->alarms),
                                  out->warnings, sizeof(out->warnings));

    if (out->source[0] == '\0') {
        snprintf(out->source, sizeof(out->source), "%s cache", ifname);
    }
    if (out->protocol[0] == '\0') {
        snprintf(out->protocol, sizeof(out->protocol), "CAN_GROWATT");
    }
    if (out->protections[0] != '\0' || out->alarms[0] != '\0' || out->warnings[0] != '\0') {
        out->valid = true;
    }

    return true;
}

void bridgeGetTelemetrySnapshot(bridgeTelemetrySnapshot_t *out)
{
    bridge_runtime_settings_t settings = runtimeSettingsGet();

    if (out == NULL) {
        return;
    }

    *out = gBridgeTelemetry;

    if (settings.bms_line == LINE_RS485 && settings.bms_protocol == PROTOCOL_RS485_GROWATT) {
        if (bridgeTryBuildGrowattRs485Telemetry(&settings, out)) {
            return;
        }
        memset(out, 0, sizeof(*out));
    }

    bridgeTryAttachGrowattCanAlerts(&settings, out);

    if (out->source[0] == '\0') {
        snprintf(out->source, sizeof(out->source), "runtime");
    }
    if (out->protocol[0] == '\0') {
        if (settings.bms_protocol == PROTOCOL_RS485_PYLON || settings.inverter_protocol == PROTOCOL_RS485_PYLON) {
            snprintf(out->protocol, sizeof(out->protocol), "RS485_PYLON");
        } else if (settings.bms_protocol == PROTOCOL_CAN_PYLON || settings.inverter_protocol == PROTOCOL_CAN_PYLON) {
            snprintf(out->protocol, sizeof(out->protocol), "CAN_PYLON");
        } else if (settings.bms_protocol == PROTOCOL_CAN_DEYE || settings.inverter_protocol == PROTOCOL_CAN_DEYE) {
            snprintf(out->protocol, sizeof(out->protocol), "CAN_DEYE");
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
    gUniversalBatteryModel.updatedMs = in->valid ? bridgeNowMs() : 0u;
}

void bridgeGetUniversalBatteryModel(universal_battery_model_t *out)
{
    if (out == NULL) {
        return;
    }

    *out = gUniversalBatteryModel;
    if (!bridgeUniversalBatteryModelIsFresh(out)) {
        memset(out, 0, sizeof(*out));
    }
}

void bridgeSetUniversalBatteryModel(const universal_battery_model_t *in)
{
    if (in == NULL) {
        memset(&gUniversalBatteryModel, 0, sizeof(gUniversalBatteryModel));
        return;
    }

    gUniversalBatteryModel = *in;
    if (gUniversalBatteryModel.valid) {
        if (gUniversalBatteryModel.updatedMs == 0u) {
            gUniversalBatteryModel.updatedMs = bridgeNowMs();
        }
    } else {
        gUniversalBatteryModel.updatedMs = 0u;
    }
}

void bridgeGetDecodedLogSnapshot(char *out, uint32_t outSize)
{
    bridgeTelemetrySnapshot_t snap = {0};

    if (out == NULL || outSize == 0) {
        return;
    }

    bridgeGetTelemetrySnapshot(&snap);
    if (bridgeBuildGenericDecodedLog(&snap, gBridgeDecodedLog, out, outSize)) {
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
        ((settings.bms_protocol == PROTOCOL_CAN_GROWATT) ||
         (settings.bms_protocol == PROTOCOL_CAN_PYLON)) &&
        (settings.inverter_protocol == PROTOCOL_RS485_GROWATT);

    pylonRs485BridgeStop();
    rs485Can322BridgeStop();
    canRs485GrowattBridgeStop();
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
        canRs485GrowattBridgeEnable(rsUartByPort(settings.inverter_port),
                                    rsDirByPort(settings.inverter_port),
                                    rsNameByPort(settings.inverter_port),
                                    canNameByPort(settings.bms_port));
    }
}

void bridgeReloadFromRuntimeSettings(void)
{
    bridge_runtime_settings_t settings = runtimeSettingsGet();

    ESP_LOGI(EXAMPLE_TAG,
             "Reload bridge: mode=%u bms(line=%u prot=%u port=%u) inv(line=%u prot=%u port=%u)",
             (unsigned)settings.mode,
             (unsigned)settings.bms_line,
             (unsigned)settings.bms_protocol,
             (unsigned)settings.bms_port,
             (unsigned)settings.inverter_line,
             (unsigned)settings.inverter_protocol,
             (unsigned)settings.inverter_port);

    pylonRs485BridgeStop();
    rs485Can322BridgeStop();
    canRs485GrowattBridgeStop();
    rs485ForwardSnifferStop();
    canForwardSnifferStop();

    canDecoderResetCaches();
    rs485ForwardSnifferResetDecoders();
    rs485ResetPorts();
    canResetBuses();
    memset(&gBridgeTelemetry, 0, sizeof(gBridgeTelemetry));
    memset(&gUniversalBatteryModel, 0, sizeof(gUniversalBatteryModel));
    gBridgeDecodedLog[0] = '\0';

    rs485BridgeEnable();
    canBridgeEnable();
}
