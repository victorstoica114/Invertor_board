#include "Web_interface/web_bridge_api.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "modes/mode_manager.h"
#include "config.h"
#include "orchestrator/protocol_types.h"
#include "protocols/china_tower_modbus/china_tower_modbus_bms_task.h"
#include "protocols/growatt/growatt_bms_task.h"
#include "protocols/jkbms_modbus/jkbms_modbus_alerts.h"
#include "protocols/jkbms_modbus/jkbms_modbus_bms_task.h"
#include "protocols/jkbms_rs485/jkbms_rs485_bms_task.h"
#include "protocols/pace_modbus/pace_modbus_bms_task.h"
#include "protocols/rs485_growatt/rs485_growatt_bridge.h"
#include "protocols/voltronic_modbus/voltronic_modbus_bms_task.h"
#include "protocols/wow_modbus/wow_modbus_bms_task.h"
#include "runtime_settings.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#define BRIDGE_TAG "BRIDGE_COMPAT"

static bridgeTelemetrySnapshot_t g_manualTelemetry;
static bool g_haveManualTelemetry;
static uint32_t g_manualTelemetryUpdatedMs;
static char g_decodedLog[2048];
static portMUX_TYPE g_bridgeMux = portMUX_INITIALIZER_UNLOCKED;

static uint32_t bridgeNowMs(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000LL);
}

static const char *modeToStr(uint8_t mode)
{
    switch (mode) {
        case MODE_SNIFFER:
            return "sniffer";
        case MODE_FORWARD:
            return "forward";
        case MODE_BRIDGE:
            return "bridge";
        default:
            return "unknown";
    }
}

static const char *protocolToStr(uint8_t protocol)
{
    switch (protocol) {
        case PROTOCOL_CAN_GROWATT:
            return "CAN_GROWATT";
        case PROTOCOL_RS485_GROWATT:
            return "RS485_GROWATT";
        case PROTOCOL_RS485_PYLON:
            return "RS485_PYLON";
        case PROTOCOL_RS485_PYLON_115200:
            return "RS485_PYLON_115200";
        case PROTOCOL_CAN_PYLON:
            return "CAN_PYLON";
        case PROTOCOL_CAN_DEYE:
            return "CAN_DEYE";
        case PROTOCOL_CAN_GOODWE:
            return "CAN_GOODWE";
        case PROTOCOL_CAN_SOFAR:
            return "CAN_SOFAR";
        case PROTOCOL_CAN_SMA:
            return "CAN_SMA";
        case PROTOCOL_CAN_VICTRON:
            return "CAN_VICTRON";
        case PROTOCOL_CAN_JKBMS_250K:
            return "JKBMS_CAN_250K";
        case PROTOCOL_RS485_JKBMS:
            return "JKBMS_MODBUS";
        case PROTOCOL_RS485_JKBMS_115200:
            return "JKBMS_MODBUS_115200";
        case PROTOCOL_RS485_PACE:
            return "PACE_RS485_MODBUS";
        case PROTOCOL_RS485_JKBMS_NATIVE:
            return "JKBMS_RS485_NATIVE";
        case PROTOCOL_RS485_VOLTRONIC:
            return "VOLTRONIC_MODBUS";
        case PROTOCOL_RS485_CHINA_TOWER:
            return "CHINA_TOWER_MODBUS";
        case PROTOCOL_RS485_WOW:
            return "WOW_MODBUS";
        default:
            return "UNKNOWN";
    }
}

typedef struct {
    uint16_t mask;
    const char *name;
} namedFlag_t;

static const namedFlag_t kPaceWarningFlags[] = {
    {0x0001u, "Cell overvoltage"},
    {0x0002u, "Cell undervoltage"},
    {0x0004u, "Pack overvoltage"},
    {0x0008u, "Pack undervoltage"},
    {0x0010u, "Charging overcurrent"},
    {0x0020u, "Discharging overcurrent"},
    {0x0100u, "Charging overtemperature"},
    {0x0200u, "Discharging overtemperature"},
    {0x0400u, "Charging undertemperature"},
    {0x0800u, "Discharging undertemperature"},
    {0x1000u, "Environment overtemperature"},
    {0x2000u, "Environment undertemperature"},
    {0x4000u, "MOSFET overtemperature"},
    {0x8000u, "Low state of charge"},
};

static const namedFlag_t kPaceProtectionFlags[] = {
    {0x0001u, "Cell overvoltage"},
    {0x0002u, "Cell undervoltage"},
    {0x0004u, "Pack overvoltage"},
    {0x0008u, "Pack undervoltage"},
    {0x0010u, "Charging overcurrent"},
    {0x0020u, "Discharging overcurrent"},
    {0x0040u, "Short circuit"},
    {0x0080u, "Charging overvoltage"},
    {0x0100u, "Charging overtemperature"},
    {0x0200u, "Discharging overtemperature"},
    {0x0400u, "Charging undertemperature"},
    {0x0800u, "Discharging undertemperature"},
    {0x1000u, "MOSFET overtemperature"},
    {0x2000u, "Environment overtemperature"},
    {0x4000u, "Environment undertemperature"},
    {0x8000u, "Battery full"},
};

static const namedFlag_t kPaceFaultFlags[] = {
    {0x0001u, "Charging MOSFET fault"},
    {0x0002u, "Discharging MOSFET fault"},
    {0x0004u, "Temperature sensor fault"},
    {0x0010u, "Battery cell fault"},
    {0x0020u, "AFE communication fault"},
};

static const namedFlag_t kPaceStatusFlags[] = {
    {0x0100u, "CHG"},
    {0x0200u, "DCHG"},
    {0x0400u, "MOSFET_CHG"},
    {0x0800u, "MOSFET_DCHG"},
    {0x1000u, "LIMIT_CHG"},
    {0x4000u, "Charger inversed"},
    {0x8000u, "HEAT"},
};

static const namedFlag_t kGrowattWarningFlags[] = {
    {0x0001u, "Cell overvoltage"},
    {0x0002u, "Cell undervoltage"},
    {0x0004u, "Pack overvoltage"},
    {0x0008u, "Pack undervoltage"},
    {0x0010u, "Discharge overcurrent"},
    {0x0020u, "Charge overcurrent"},
    {0x0040u, "Discharge overtemperature"},
    {0x0080u, "Discharge undertemperature"},
    {0x0100u, "Charge overtemperature"},
    {0x0200u, "Charge undertemperature"},
    {0x0400u, "MOS overtemperature"},
    {0x0800u, "Ambient overtemperature"},
    {0x1000u, "Ambient undertemperature"},
    {0x2000u, "System low voltage"},
};

static const namedFlag_t kGrowattProtectionFlags[] = {
    {0x0001u, "Discharge overcurrent"},
    {0x0002u, "Discharge short circuit"},
    {0x0004u, "Pack overvoltage"},
    {0x0008u, "Pack undervoltage"},
    {0x0010u, "Discharge overtemperature"},
    {0x0020u, "Charge overtemperature"},
    {0x0040u, "Discharge undertemperature"},
    {0x0080u, "Charge undertemperature"},
    {0x0100u, "Soft-start failure"},
    {0x0200u, "Permanent fault"},
    {0x0400u, "Cell voltage delta"},
    {0x0800u, "Charge overcurrent"},
    {0x1000u, "MOS overtemperature"},
    {0x2000u, "Ambient overtemperature"},
    {0x4000u, "Ambient undertemperature"},
};

static const namedFlag_t kVoltronicChargeAlarmFlags[] = {
    {0x0001u, "Charge overtemperature alarm"},
    {0x0002u, "Cell overvoltage alarm"},
    {0x0004u, "Charge low-temperature alarm"},
    {0x0008u, "Charge overcurrent alarm"},
};

static const namedFlag_t kVoltronicDischargeAlarmFlags[] = {
    {0x0001u, "Discharge overtemperature alarm"},
    {0x0002u, "Discharge low-temperature alarm"},
    {0x0004u, "MOSFET overtemperature alarm"},
    {0x0008u, "Cell undervoltage alarm"},
};

static const namedFlag_t kVoltronicChargeProtectFlags[] = {
    {0x0001u, "In-system signal missing"},
    {0x0002u, "MOSFET driver not ready"},
    {0x0004u, "AFE communication fail"},
    {0x0008u, "AFE charge overcurrent"},
    {0x0010u, "AFE discharge overcurrent"},
    {0x0020u, "AFE discharge short-circuit"},
    {0x0040u, "MOSFET overtemperature"},
    {0x0080u, "Safety undervoltage"},
    {0x0100u, "Charge overtemperature"},
    {0x0200u, "Charge low-temperature"},
    {0x0400u, "Cell overvoltage"},
    {0x0800u, "Charge overcurrent"},
    {0x1000u, "Second-level cell overvoltage"},
    {0x2000u, "CAN ID distribution incomplete"},
    {0x8000u, "Second-level charge overcurrent"},
};

static const namedFlag_t kVoltronicChargeProtect2Flags[] = {
    {0x0001u, "Shutdown by command"},
    {0x0002u, "Low-voltage shutdown"},
    {0x0004u, "Charge short-circuit detected"},
    {0x0008u, "CAN ID error"},
};

static const namedFlag_t kVoltronicDischargeProtectFlags[] = {
    {0x0001u, "In-system signal missing"},
    {0x0002u, "MOSFET driver not ready"},
    {0x0004u, "AFE charge overcurrent"},
    {0x0008u, "AFE discharge overcurrent"},
    {0x0010u, "AFE discharge short-circuit"},
    {0x0040u, "CAN ID error"},
    {0x0400u, "MOSFET overtemperature"},
    {0x0800u, "Cell undervoltage"},
    {0x1000u, "Discharge overcurrent"},
    {0x2000u, "Discharge low-temperature"},
    {0x4000u, "Discharge overtemperature"},
    {0x8000u, "Low-voltage shutdown"},
};

static void appendListItem(char *out, size_t outSize, const char *item)
{
    size_t used = 0u;
    int w = 0;

    if (out == NULL || outSize == 0u || item == NULL || item[0] == '\0') {
        return;
    }

    used = strlen(out);
    if (used >= (outSize - 1u)) {
        return;
    }

    w = snprintf(out + used, outSize - used, "%s%s", (used == 0u) ? "" : ", ", item);
    if (w <= 0) {
        return;
    }
    if ((size_t)w >= (outSize - used)) {
        out[outSize - 1u] = '\0';
    }
}

static void formatNamedFlagList(uint16_t bits,
                                const namedFlag_t *map,
                                size_t mapCount,
                                char *out,
                                size_t outSize)
{
    uint16_t known = 0u;

    if (out == NULL || outSize == 0u) {
        return;
    }

    out[0] = '\0';
    if (bits == 0u) {
        return;
    }

    for (size_t i = 0u; i < mapCount; i++) {
        known = (uint16_t)(known | map[i].mask);
        if ((bits & map[i].mask) != 0u) {
            appendListItem(out, outSize, map[i].name);
        }
    }

    uint16_t unknown = (uint16_t)(bits & (uint16_t)(~known));
    for (uint8_t i = 0u; i < 16u; i++) {
        uint16_t mask = (uint16_t)(1u << i);
        if ((unknown & mask) != 0u) {
            char item[24];
            snprintf(item, sizeof(item), "Unknown 0x%04X", (unsigned)mask);
            appendListItem(out, outSize, item);
        }
    }
}

static void appendPrefixedNamedFlagList(char *out,
                                        size_t outSize,
                                        const char *prefix,
                                        uint16_t bits,
                                        const namedFlag_t *map,
                                        size_t mapCount)
{
    char flags[256];
    char item[320];

    if (out == NULL || outSize == 0u || bits == 0u) {
        return;
    }

    formatNamedFlagList(bits, map, mapCount, flags, sizeof(flags));
    if (flags[0] == '\0') {
        return;
    }

    snprintf(item, sizeof(item), "%s%s", (prefix != NULL) ? prefix : "", flags);
    appendListItem(out, outSize, item);
}

static void fillPaceAlertFields(const bms_decoded_packet_t *packet, bridgeTelemetrySnapshot_t *out)
{
    uint16_t warningFlags = 0u;
    uint16_t protectionFlags = 0u;
    uint16_t statusFlags = 0u;
    uint16_t balanceFlags = 0u;
    char statusText[48] = {0};

    if (packet == NULL || out == NULL ||
        (packet->sourceProtocol != PROTOCOL_ID_PACE &&
         packet->sourceProtocol != PROTOCOL_ID_WOW)) {
        return;
    }

    if (packet->hasWarningFlags) {
        warningFlags = packet->warningFlags;
        formatNamedFlagList(warningFlags,
                            kPaceWarningFlags,
                            sizeof(kPaceWarningFlags) / sizeof(kPaceWarningFlags[0]),
                            out->warnings,
                            sizeof(out->warnings));
    }
    if (packet->hasProtectionFlags) {
        protectionFlags = packet->protectionFlags;
        formatNamedFlagList(protectionFlags,
                            kPaceProtectionFlags,
                            sizeof(kPaceProtectionFlags) / sizeof(kPaceProtectionFlags[0]),
                            out->protections,
                            sizeof(out->protections));
    }
    if (packet->hasStatusFlags) {
        statusFlags = packet->statusFlags;
        formatNamedFlagList((uint16_t)(statusFlags & 0x00FFu),
                            kPaceFaultFlags,
                            sizeof(kPaceFaultFlags) / sizeof(kPaceFaultFlags[0]),
                            out->alarms,
                            sizeof(out->alarms));
        formatNamedFlagList((uint16_t)(statusFlags & 0xFF00u),
                            kPaceStatusFlags,
                            sizeof(kPaceStatusFlags) / sizeof(kPaceStatusFlags[0]),
                            statusText,
                            sizeof(statusText));
    }
    if (packet->hasBalanceFlags) {
        balanceFlags = packet->balanceFlags;
    }

    out->alarmRaw = ((uint32_t)warningFlags << 16) | (uint32_t)protectionFlags;
    snprintf(out->stateFlags,
             sizeof(out->stateFlags),
             "Warn=0x%04X, Prot=0x%04X, Fault=0x%02X, Status=0x%02X%s%s, Balance=0x%04X",
             (unsigned)warningFlags,
             (unsigned)protectionFlags,
             (unsigned)(statusFlags & 0x00FFu),
             (unsigned)((statusFlags & 0xFF00u) >> 8),
             (statusText[0] != '\0') ? " " : "",
             statusText,
             (unsigned)balanceFlags);
}

static void fillChinaTowerAlertFields(const bms_decoded_packet_t *packet,
                                      bridgeTelemetrySnapshot_t *out)
{
    uint16_t warningFlags = 0u;
    uint16_t protectionFlags = 0u;
    uint16_t statusFlags = 0u;
    uint16_t balanceFlags = 0u;

    if (packet == NULL || out == NULL || packet->sourceProtocol != PROTOCOL_ID_CHINA_TOWER) {
        return;
    }

    if (packet->hasWarningFlags) {
        warningFlags = packet->warningFlags;
        formatNamedFlagList(warningFlags, NULL, 0u, out->warnings, sizeof(out->warnings));
    }
    if (packet->hasProtectionFlags) {
        protectionFlags = packet->protectionFlags;
        formatNamedFlagList(protectionFlags, NULL, 0u, out->protections, sizeof(out->protections));
    }
    if (packet->hasStatusFlags) {
        statusFlags = packet->statusFlags;
    }
    if (packet->hasBalanceFlags) {
        balanceFlags = packet->balanceFlags;
    }

    out->alarmRaw = ((uint32_t)warningFlags << 16) | (uint32_t)protectionFlags;
    snprintf(out->stateFlags,
             sizeof(out->stateFlags),
             "Warn=0x%04X, Prot=0x%04X, Status=0x%04X, Balance=0x%04X",
             (unsigned)warningFlags,
             (unsigned)protectionFlags,
             (unsigned)statusFlags,
             (unsigned)balanceFlags);
}

static const char *growattRunMode(uint16_t statusFlags)
{
    switch (statusFlags & 0x0003u) {
        case 0u:
            return "soft_starting";
        case 1u:
            return "standby";
        case 2u:
            return "charging";
        case 3u:
            return "discharging";
        default:
            return "unknown";
    }
}

static const char *growattMasterMode(uint16_t statusFlags)
{
    switch ((statusFlags >> 8) & 0x0003u) {
        case 0u:
            return "standalone";
        case 1u:
            return "parallel";
        case 2u:
            return "parallel_ready";
        default:
            return "reserved";
    }
}

static const char *growattSpStatus(uint16_t statusFlags)
{
    switch ((statusFlags >> 10) & 0x0003u) {
        case 0u:
            return "none";
        case 1u:
            return "standby";
        case 2u:
            return "charging";
        case 3u:
            return "discharging";
        default:
            return "unknown";
    }
}

static void fillGrowattAlertFields(const bms_decoded_packet_t *packet, bridgeTelemetrySnapshot_t *out)
{
    uint16_t warningFlags = 0u;
    uint16_t protectionFlags = 0u;
    uint16_t statusFlags = 0u;

    if (packet == NULL || out == NULL || packet->sourceProtocol != PROTOCOL_ID_GROWATT) {
        return;
    }

    if (packet->hasWarningFlags) {
        warningFlags = packet->warningFlags;
        formatNamedFlagList(warningFlags,
                            kGrowattWarningFlags,
                            sizeof(kGrowattWarningFlags) / sizeof(kGrowattWarningFlags[0]),
                            out->warnings,
                            sizeof(out->warnings));
    }
    if (packet->hasProtectionFlags) {
        protectionFlags = packet->protectionFlags;
        formatNamedFlagList(protectionFlags,
                            kGrowattProtectionFlags,
                            sizeof(kGrowattProtectionFlags) / sizeof(kGrowattProtectionFlags[0]),
                            out->protections,
                            sizeof(out->protections));
    }
    if (packet->hasStatusFlags) {
        statusFlags = packet->statusFlags;
    }

    out->alarmRaw = ((uint32_t)warningFlags << 16) | (uint32_t)protectionFlags;
    snprintf(out->stateFlags,
             sizeof(out->stateFlags),
             "Status=0x%04X mode=%s err=%s bal=%s master=%s sp=%s Warn=0x%04X Prot=0x%04X",
             (unsigned)statusFlags,
             growattRunMode(statusFlags),
             (statusFlags & 0x0004u) ? "YES" : "NO",
             (statusFlags & 0x0008u) ? "ON" : "OFF",
             growattMasterMode(statusFlags),
             growattSpStatus(statusFlags),
             (unsigned)warningFlags,
             (unsigned)protectionFlags);
}

static const char *voltronicWarningStateText(uint8_t state)
{
    switch (state) {
        case 0x00u:
            return "";
        case 0x01u:
            return "below lower limit";
        case 0x02u:
            return "above higher limit";
        case 0xF0u:
            return "other error";
        default:
            return "unknown state";
    }
}

static void appendVoltronicWarningState(char *out,
                                        size_t outSize,
                                        const char *label,
                                        uint8_t index,
                                        uint8_t state)
{
    char item[80];
    const char *stateText = voltronicWarningStateText(state);

    if (out == NULL || outSize == 0u || label == NULL || stateText[0] == '\0') {
        return;
    }

    if (index > 0u) {
        snprintf(item, sizeof(item), "%s %u %s", label, (unsigned)index, stateText);
    } else {
        snprintf(item, sizeof(item), "%s %s", label, stateText);
    }
    appendListItem(out, outSize, item);
}

static void fillVoltronicAlertFields(const voltronic_modbus_snapshot_t *snapshot,
                                     bridgeTelemetrySnapshot_t *out)
{
    static const char *const moduleLabels[VOLTRONIC_MB_MODULE_STATE_COUNT] = {
        "Module charge voltage",
        "Module discharge voltage",
        "Cell charge voltage",
        "Cell discharge voltage",
        "Module charge current",
        "Module discharge current",
        "Module charge temperature",
        "Module discharge temperature",
        "Cell charge temperature",
        "Cell discharge temperature",
    };

    if (snapshot == NULL || out == NULL || !snapshot->valid) {
        return;
    }

    if (snapshot->hasAlarmRegisters) {
        appendPrefixedNamedFlagList(out->alarms,
                                    sizeof(out->alarms),
                                    "Charge: ",
                                    snapshot->chargeAlarm,
                                    kVoltronicChargeAlarmFlags,
                                    sizeof(kVoltronicChargeAlarmFlags) / sizeof(kVoltronicChargeAlarmFlags[0]));
        appendPrefixedNamedFlagList(out->alarms,
                                    sizeof(out->alarms),
                                    "Discharge: ",
                                    snapshot->dischargeAlarm,
                                    kVoltronicDischargeAlarmFlags,
                                    sizeof(kVoltronicDischargeAlarmFlags) / sizeof(kVoltronicDischargeAlarmFlags[0]));
        appendPrefixedNamedFlagList(out->protections,
                                    sizeof(out->protections),
                                    "Charge: ",
                                    snapshot->chargeProtect,
                                    kVoltronicChargeProtectFlags,
                                    sizeof(kVoltronicChargeProtectFlags) / sizeof(kVoltronicChargeProtectFlags[0]));
        appendPrefixedNamedFlagList(out->protections,
                                    sizeof(out->protections),
                                    "Charge2: ",
                                    snapshot->chargeProtect2,
                                    kVoltronicChargeProtect2Flags,
                                    sizeof(kVoltronicChargeProtect2Flags) / sizeof(kVoltronicChargeProtect2Flags[0]));
        appendPrefixedNamedFlagList(out->protections,
                                    sizeof(out->protections),
                                    "Discharge: ",
                                    snapshot->dischargeProtect,
                                    kVoltronicDischargeProtectFlags,
                                    sizeof(kVoltronicDischargeProtectFlags) / sizeof(kVoltronicDischargeProtectFlags[0]));
    }

    for (uint8_t i = 0u; i < snapshot->cellStateCount && i < VOLTRONIC_MB_MAX_CELLS; i++) {
        appendVoltronicWarningState(out->warnings,
                                    sizeof(out->warnings),
                                    "Cell",
                                    (uint8_t)(i + 1u),
                                    snapshot->cellState[i]);
    }
    for (uint8_t i = 0u; i < snapshot->tempStateCount && i < VOLTRONIC_MB_MAX_TEMPS; i++) {
        appendVoltronicWarningState(out->warnings,
                                    sizeof(out->warnings),
                                    "Temperature",
                                    (uint8_t)(i + 1u),
                                    snapshot->tempState[i]);
    }
    for (uint8_t i = 0u; i < VOLTRONIC_MB_MODULE_STATE_COUNT; i++) {
        appendVoltronicWarningState(out->warnings,
                                    sizeof(out->warnings),
                                    moduleLabels[i],
                                    0u,
                                    snapshot->moduleState[i]);
    }

    out->alarmRaw = ((uint32_t)snapshot->chargeAlarm << 16) |
                    (uint32_t)snapshot->dischargeAlarm;
    snprintf(out->stateFlags,
             sizeof(out->stateFlags),
             "Status=0x%04X CHG=%s DSG=%s Now=%s/%s FullReq=%s CA=0x%04X DA=0x%04X CP=0x%04X CP2=0x%04X DP=0x%04X DP2=0x%04X BMS=0x%04X",
             snapshot->hasStatusFlags ? (unsigned)snapshot->statusFlags : 0u,
             snapshot->chargeEnabled ? "ON" : "OFF",
             snapshot->dischargeEnabled ? "ON" : "OFF",
             snapshot->chargeImmediately ? "YES" : "NO",
             snapshot->chargeImmediately2 ? "YES" : "NO",
             snapshot->fullChargeRequested ? "YES" : "NO",
             (unsigned)snapshot->chargeAlarm,
             (unsigned)snapshot->dischargeAlarm,
             (unsigned)snapshot->chargeProtect,
             (unsigned)snapshot->chargeProtect2,
             (unsigned)snapshot->dischargeProtect,
             (unsigned)snapshot->dischargeProtect2,
             (unsigned)snapshot->bmsState);
}

static void fillTelemetryFromVoltronicSnapshot(const voltronic_modbus_snapshot_t *snapshot,
                                               bridgeTelemetrySnapshot_t *out)
{
    uint32_t sumMv = 0u;
    uint8_t counted = 0u;

    if (snapshot == NULL || out == NULL || !snapshot->valid) {
        return;
    }

    out->valid = true;
    snprintf(out->source, sizeof(out->source), "%s", "VOLTRONIC_BMS_TASK");
    snprintf(out->protocol, sizeof(out->protocol), "%s", "VOLTRONIC_MODBUS");

    if (snapshot->hasSoc) {
        out->socPct = snapshot->socPct;
    }
    out->sohPct = 100u;
    if (snapshot->hasPackVoltage) {
        out->packVoltageV = snapshot->packVoltageV;
    }
    if (snapshot->hasPackCurrent) {
        out->currentA = snapshot->packCurrentA;
    }
    if (snapshot->hasPackPower) {
        out->packPowerW = snapshot->packPowerW;
    }
    if (snapshot->hasFullMah) {
        out->fullAh = (float)snapshot->fullMah / 1000.0f;
    }
    if (snapshot->hasRemainMah) {
        out->remainingAh = (float)snapshot->remainMah / 1000.0f;
    }
    if (snapshot->hasStatusFlags) {
        out->pylonStatus63 = (uint8_t)((snapshot->chargeEnabled ? 0x80u : 0u) |
                                       (snapshot->dischargeEnabled ? 0x40u : 0u));
    }

    out->tempMosC = 0.0f;
    out->tempT1C = 0.0f;
    out->tempT2C = 0.0f;
    out->tempT4C = 0.0f;
    out->tempT5C = 0.0f;
    out->tempCount = snapshot->tempCount;
    if (snapshot->tempCount == 3u) {
        out->tempMosC = (float)snapshot->tempDeciC[0] / 10.0f;
        out->tempT1C = (float)snapshot->tempDeciC[1] / 10.0f;
        out->tempT2C = (float)snapshot->tempDeciC[2] / 10.0f;
    } else {
        if (snapshot->tempCount > 5u) {
            out->tempMosC = (float)snapshot->tempDeciC[5] / 10.0f;
        }
        if (snapshot->tempCount > 0u) {
            out->tempT1C = (float)snapshot->tempDeciC[0] / 10.0f;
        }
        if (snapshot->tempCount > 1u) {
            out->tempT2C = (float)snapshot->tempDeciC[1] / 10.0f;
        }
        if (snapshot->tempCount > 3u) {
            out->tempT4C = (float)snapshot->tempDeciC[3] / 10.0f;
        }
        if (snapshot->tempCount > 4u) {
            out->tempT5C = (float)snapshot->tempDeciC[4] / 10.0f;
        }
    }

    out->cellCount = snapshot->cellCount;
    for (uint8_t i = 0u; i < snapshot->cellCount && i < 32u; i++) {
        out->cellVoltagesV[i] = (float)snapshot->cellMv[i] / 1000.0f;
        if (snapshot->cellMv[i] > 0u) {
            sumMv += snapshot->cellMv[i];
            counted++;
        }
    }
    if (counted > 0u) {
        out->cellAvgV = ((float)sumMv / (float)counted) / 1000.0f;
    }
    if (snapshot->hasCellExtremes) {
        out->cellMaxV = (float)snapshot->maxCellMv / 1000.0f;
        out->cellMinV = (float)snapshot->minCellMv / 1000.0f;
        out->cellMaxIdx = snapshot->maxCellIndex;
        out->cellMinIdx = snapshot->minCellIndex;
        out->deltaV = out->cellMaxV - out->cellMinV;
        out->cellDiffV = out->deltaV;
    }

    fillVoltronicAlertFields(snapshot, out);
}

static bool pacePacketTempC(const bms_decoded_packet_t *packet, uint8_t index, float *outC)
{
    if (packet == NULL || outC == NULL || index >= packet->tempCount ||
        index >= BMS_DECODED_PACKET_MAX_TEMPS) {
        return false;
    }

    *outC = (float)packet->tempDeciC[index] / 10.0f;
    return true;
}

static void fillPaceTemperatureFields(const bms_decoded_packet_t *packet, bridgeTelemetrySnapshot_t *out)
{
    float tempC = 0.0f;

    if (packet == NULL || out == NULL ||
        (packet->sourceProtocol != PROTOCOL_ID_PACE &&
         packet->sourceProtocol != PROTOCOL_ID_CHINA_TOWER &&
         packet->sourceProtocol != PROTOCOL_ID_WOW)) {
        return;
    }

    if (packet->sourceProtocol == PROTOCOL_ID_CHINA_TOWER) {
        if (packet->tempCount > 0u) {
            out->tempCount = packet->tempCount;
        }
        if (pacePacketTempC(packet, 0u, &tempC)) {
            out->tempT1C = tempC;
        }
        if (pacePacketTempC(packet, 1u, &tempC)) {
            out->tempT2C = tempC;
        }
        if (pacePacketTempC(packet, 2u, &tempC)) {
            out->tempMosC = tempC;
        }
        out->tempT4C = 0.0f;
        out->tempT5C = 0.0f;
        return;
    }

    if (packet->tempCount > 0u) {
        out->tempCount = packet->tempCount;
    }
    if (pacePacketTempC(packet, 0u, &tempC)) {
        out->tempT1C = tempC;
    }
    if (pacePacketTempC(packet, 1u, &tempC)) {
        out->tempT2C = tempC;
    }
    if (pacePacketTempC(packet, 2u, &tempC)) {
        out->tempT4C = tempC;
    }
    if (pacePacketTempC(packet, 3u, &tempC)) {
        out->tempT5C = tempC;
    }
    if (pacePacketTempC(packet, 4u, &tempC)) {
        out->tempMosC = tempC;
    }
}

static void inferFixedTemperatureCount(const bms_decoded_packet_t *packet,
                                       bridgeTelemetrySnapshot_t *out)
{
    if (packet == NULL || out == NULL || out->tempCount != 0u ||
        (packet->sourceProtocol != PROTOCOL_ID_PACE &&
         packet->sourceProtocol != PROTOCOL_ID_WOW)) {
        return;
    }

    if (out->tempMosC != 0.0f || out->tempT1C != 0.0f || out->tempT2C != 0.0f ||
        out->tempT4C != 0.0f || out->tempT5C != 0.0f) {
        out->tempCount = 5u;
    }
}

static void fillTelemetryFromJkbmsSnapshot(const jkbms_modbus_snapshot_t *snapshot,
                                           bridgeTelemetrySnapshot_t *out)
{
    if (snapshot == NULL || out == NULL || !snapshot->valid) {
        return;
    }

    out->valid = true;
    snprintf(out->source, sizeof(out->source), "%s", "JKBMS_BMS_TASK");
    snprintf(out->protocol, sizeof(out->protocol), "%s", "JKBMS_MODBUS");

    if (snapshot->hasSoc) {
        out->socPct = snapshot->socPct;
    }
    if (snapshot->hasSoh) {
        out->sohPct = snapshot->sohPct;
    }
    if (snapshot->hasCycles) {
        out->cycles = (uint16_t)((snapshot->cycles > 65535u) ? 65535u : snapshot->cycles);
    }

    if (snapshot->hasPackVoltageMv) {
        out->packVoltageV = (float)snapshot->packVoltageMv / 1000.0f;
    }
    if (snapshot->hasPackCurrentMa) {
        out->currentA = (float)snapshot->packCurrentMa / 1000.0f;
    }
    if (snapshot->hasPackPowerMw) {
        out->packPowerW = (float)snapshot->packPowerMw / 1000.0f;
    }
    if (snapshot->hasBalanceCurrentMa) {
        out->balanceCurrentA = (float)snapshot->balanceCurrentMa / 1000.0f;
    }

    if (snapshot->hasRemainMah) {
        out->remainingAh = (float)snapshot->remainMah / 1000.0f;
    }
    if (snapshot->hasFullMah) {
        out->fullAh = (float)snapshot->fullMah / 1000.0f;
    }

    if (snapshot->hasTempMosC) {
        out->tempMosC = (float)snapshot->tempMosC;
    }
    if (snapshot->hasTempBat1C) {
        out->tempT1C = (float)snapshot->tempBat1C;
    }
    if (snapshot->hasTempBat2C) {
        out->tempT2C = (float)snapshot->tempBat2C;
    }

    if (snapshot->hasCellAvgMv) {
        out->cellAvgV = (float)snapshot->cellAvgMv / 1000.0f;
    }
    if (snapshot->hasCellDiffMaxMv) {
        out->cellDiffV = (float)snapshot->cellDiffMaxMv / 1000.0f;
    }

    out->cellCount = snapshot->cellCount;
    for (uint8_t i = 0; i < snapshot->cellCount && i < 32u; i++) {
        out->cellVoltagesV[i] = (float)snapshot->cellMv[i] / 1000.0f;
    }

    uint32_t decodedAlarmBits = 0u;
    if (snapshot->hasCellExtremes) {
        out->cellMaxV = (float)snapshot->maxCellMv / 1000.0f;
        out->cellMinV = (float)snapshot->minCellMv / 1000.0f;
        out->cellMaxIdx = snapshot->maxCellIndex;
        out->cellMinIdx = snapshot->minCellIndex;
        out->deltaV = out->cellMaxV - out->cellMinV;
    }

    if (snapshot->hasAlarmBits) {
        out->alarmRaw = snapshot->alarmBits;
        decodedAlarmBits = jkbmsModbusNormalizeAlarmBits(snapshot->alarmBits);
        jkbmsModbusFormatAlertFields(snapshot->alarmBits,
                                     out->protections,
                                     sizeof(out->protections),
                                     out->alarms,
                                     sizeof(out->alarms),
                                     out->warnings,
                                     sizeof(out->warnings));
    }

    if (snapshot->hasPrecharge) {
        out->prechargeState = snapshot->prechargeState;
    }

    snprintf(out->stateFlags,
             sizeof(out->stateFlags),
             "AlarmRaw=0x%08" PRIX32 ", AlarmDecoded=0x%08" PRIX32 ", Precharge=%u, Cells=%u",
             snapshot->hasAlarmBits ? snapshot->alarmBits : 0u,
             decodedAlarmBits,
             snapshot->hasPrecharge ? snapshot->prechargeState : 0u,
             (unsigned)snapshot->cellCount);
}

static void fillTelemetryFromJkbmsNativeSnapshot(const jkbms_rs485_native_snapshot_t *snapshot,
                                                 bridgeTelemetrySnapshot_t *out)
{
    if (snapshot == NULL || out == NULL || !snapshot->valid) {
        return;
    }

    out->valid = true;
    snprintf(out->source, sizeof(out->source), "%s", "JKBMS_NATIVE_TASK");
    snprintf(out->protocol, sizeof(out->protocol), "%s", "JKBMS_RS485_NATIVE");

    if (snapshot->hasSoc) {
        out->socPct = snapshot->socPct;
    }
    if (snapshot->hasSoh) {
        out->sohPct = snapshot->sohPct;
    }
    if (snapshot->hasCycles) {
        out->cycles = (uint16_t)((snapshot->cycles > 65535u) ? 65535u : snapshot->cycles);
    }
    if (snapshot->hasPackVoltageMv) {
        out->packVoltageV = (float)snapshot->packVoltageMv / 1000.0f;
    }
    if (snapshot->hasPackCurrentMa) {
        out->currentA = (float)snapshot->packCurrentMa / 1000.0f;
    }
    if (snapshot->hasPackPowerMw) {
        out->packPowerW = (float)snapshot->packPowerMw / 1000.0f;
    }
    if (snapshot->hasFullMah) {
        out->fullAh = (float)snapshot->fullMah / 1000.0f;
    }

    if (snapshot->hasTempMosC) {
        out->tempMosC = (float)snapshot->tempMosC;
    }
    if (snapshot->hasTempBat1C) {
        out->tempT1C = (float)snapshot->tempBat1C;
    }
    if (snapshot->hasTempBat2C) {
        out->tempT2C = (float)snapshot->tempBat2C;
    }

    if (snapshot->hasCellAvgMv) {
        out->cellAvgV = (float)snapshot->cellAvgMv / 1000.0f;
    }
    if (snapshot->hasCellDiffMaxMv) {
        out->cellDiffV = (float)snapshot->cellDiffMaxMv / 1000.0f;
    }

    out->cellCount = snapshot->cellCount;
    for (uint8_t i = 0; i < snapshot->cellCount && i < 32u; i++) {
        out->cellVoltagesV[i] = (float)snapshot->cellMv[i] / 1000.0f;
    }

    if (snapshot->hasCellExtremes) {
        out->cellMaxV = (float)snapshot->maxCellMv / 1000.0f;
        out->cellMinV = (float)snapshot->minCellMv / 1000.0f;
        out->cellMaxIdx = snapshot->maxCellIndex;
        out->cellMinIdx = snapshot->minCellIndex;
        out->deltaV = out->cellMaxV - out->cellMinV;
    }

    if (snapshot->hasAlarmBits) {
        out->alarmRaw = snapshot->alarmBits;
        jkbmsRs485NativeFormatAlertFields(snapshot->alarmBits,
                                          out->protections,
                                          sizeof(out->protections),
                                          out->alarms,
                                          sizeof(out->alarms),
                                          out->warnings,
                                          sizeof(out->warnings));
    }

    snprintf(out->stateFlags,
             sizeof(out->stateFlags),
             "AlarmRaw=0x%04X, Status=0x%04X, CHG=%s, DSG=%s, BAL=%s, Cells=%u",
             snapshot->hasAlarmBits ? snapshot->alarmBits : 0u,
             snapshot->hasStatusFlags ? snapshot->statusFlags : 0u,
             snapshot->chargeEnabled ? "ON" : "OFF",
             snapshot->dischargeEnabled ? "ON" : "OFF",
             snapshot->balanceActive ? "ON" : "OFF",
             (unsigned)snapshot->cellCount);
}

static void bridgeFormatBmsInterface(char *out, size_t outSize, const bridge_runtime_settings_t *settings)
{
    uint8_t port = 1u;

    if (out == NULL || outSize == 0u || settings == NULL) {
        return;
    }

    if (settings->bms_port >= 1u && settings->bms_port <= 2u) {
        port = settings->bms_port;
    }

    if (settings->bms_line == LINE_RS485) {
        snprintf(out, outSize, "RS485_%u", (unsigned)port);
        return;
    }

    snprintf(out, outSize, "CAN%u", (unsigned)port);
}

static const char *bridgeBmsTaskDebugName(uint8_t protocol)
{
    switch (protocol) {
        case PROTOCOL_RS485_JKBMS:
        case PROTOCOL_RS485_JKBMS_115200:
            return "JKBMS";
        case PROTOCOL_RS485_JKBMS_NATIVE:
            return "JKBMS_NATIVE";
        case PROTOCOL_RS485_PACE:
            return "PACE";
        case PROTOCOL_RS485_VOLTRONIC:
            return "VOLTRONIC";
        case PROTOCOL_RS485_CHINA_TOWER:
            return "CHINA_TOWER";
        case PROTOCOL_RS485_WOW:
            return "WOW";
        default:
            return "GROWATT";
    }
}

static const char *bridgeBmsTaskSourceName(uint8_t protocol)
{
    switch (protocol) {
        case PROTOCOL_RS485_JKBMS:
        case PROTOCOL_RS485_JKBMS_115200:
            return "JKBMS_BMS_TASK";
        case PROTOCOL_RS485_JKBMS_NATIVE:
            return "JKBMS_NATIVE_TASK";
        case PROTOCOL_RS485_PACE:
            return "PACE_BMS_TASK";
        case PROTOCOL_RS485_VOLTRONIC:
            return "VOLTRONIC_BMS_TASK";
        case PROTOCOL_RS485_CHINA_TOWER:
            return "CHINA_TOWER_BMS_TASK";
        case PROTOCOL_RS485_WOW:
            return "WOW_BMS_TASK";
        default:
            return "GROWATT_BMS_TASK";
    }
}

static void fillTelemetryFromLatestPacket(bridgeTelemetrySnapshot_t *out, uint32_t *updatedMsOut)
{
    bms_decoded_packet_t packet = {0};
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    can_rs485_growatt_snapshot_t canRsSnap = {0};

    bool hasPacket = false;
    uint32_t srcUpdatedMs = 0u;
    const bool canToRsGrowattRoute =
        (settings.bms_line == LINE_CAN) &&
        (settings.inverter_line == LINE_RS485) &&
        ((settings.bms_protocol == PROTOCOL_CAN_GROWATT) ||
         (settings.bms_protocol == PROTOCOL_CAN_PYLON) ||
         (settings.bms_protocol == PROTOCOL_CAN_GOODWE) ||
         (settings.bms_protocol == PROTOCOL_CAN_SOFAR) ||
         (settings.bms_protocol == PROTOCOL_CAN_SMA) ||
         (settings.bms_protocol == PROTOCOL_CAN_VICTRON)) &&
        (settings.inverter_protocol == PROTOCOL_RS485_GROWATT);
    const bool pylonRs485Route =
        (settings.inverter_line == LINE_RS485) &&
        (((settings.bms_line == LINE_RS485) &&
          bridgeProtocolIsRs485Pylon(settings.bms_protocol) &&
          bridgeProtocolIsRs485Pylon(settings.inverter_protocol)) ||
         ((settings.bms_line == LINE_CAN) &&
          ((settings.bms_protocol == PROTOCOL_CAN_PYLON) ||
           (settings.bms_protocol == PROTOCOL_CAN_DEYE) ||
           (settings.bms_protocol == PROTOCOL_CAN_JKBMS_250K)) &&
          bridgeProtocolIsRs485Pylon(settings.inverter_protocol)));

    if (pylonRs485Route) {
        ESP_LOGD(BRIDGE_TAG, "[FILL] Pylon RS485 route - skipping fill");
        return;
    }

    if (canToRsGrowattRoute && canRs485GrowattBridgeGetLatestSnapshot(&canRsSnap)) {
        ESP_LOGD(BRIDGE_TAG, "[FILL] Using CAN->RS485 translator snapshot: soc=%u%%, v=%.2fV",
                 canRsSnap.socPct, (double)canRsSnap.packCv / 100.0);
        const float tempAvgC = (float)canRsSnap.tempDeciC / 10.0f;
        float tempMinC = tempAvgC;
        float tempMaxC = tempAvgC;
        srcUpdatedMs = (uint32_t)(canRsSnap.timestampUs / 1000ULL);
        if (canRsSnap.tempMinDeciC != 0 || canRsSnap.tempMaxDeciC != 0) {
            tempMinC = (float)canRsSnap.tempMinDeciC / 10.0f;
            tempMaxC = (float)canRsSnap.tempMaxDeciC / 10.0f;
        }

        out->valid = true;
        snprintf(out->source, sizeof(out->source), "%s", "CAN_RS485_TRANSLATOR");
        snprintf(out->protocol, sizeof(out->protocol), "%s", protocolToStr(settings.bms_protocol));

        out->socPct = canRsSnap.socPct;
        out->sohPct = canRsSnap.sohPct;
        out->tempMosC = tempAvgC;
        out->tempT1C = tempMinC;
        out->tempT2C = tempMaxC;
        out->tempT4C = tempAvgC;
        out->tempT5C = tempAvgC;
        out->packVoltageV = (float)canRsSnap.packCv / 100.0f;
        out->cycles = canRsSnap.cycles;
        out->remainingAh = (float)canRsSnap.remainingCapCah / 100.0f;
        out->fullAh = (float)canRsSnap.fullCapCah / 100.0f;

        out->cellMaxV = (float)canRsSnap.cellMaxMv / 1000.0f;
        out->cellMinV = (float)canRsSnap.cellMinMv / 1000.0f;
        out->cellMaxIdx = canRsSnap.cellMaxIdx;
        out->cellMinIdx = canRsSnap.cellMinIdx;
        out->deltaV = out->cellMaxV - out->cellMinV;

        out->cellCount = canRsSnap.cellCount;
        for (uint8_t i = 0; i < canRsSnap.cellCount && i < 32u; i++) {
            out->cellVoltagesV[i] = (float)canRsSnap.cellMv[i] / 1000.0f;
        }
        if (srcUpdatedMs != 0u) {
            out->updatedMs = srcUpdatedMs;
            if (updatedMsOut != NULL) {
                *updatedMsOut = srcUpdatedMs;
            }
        }
        return;
    }

    if (bridgeProtocolIsRs485JkbmsModbus(settings.bms_protocol)) {
        jkbms_modbus_snapshot_t snapshot = {0};
        bool haveSnapshot = jkbmsModbusBmsTaskGetLatestSnapshot(&snapshot);
        hasPacket = jkbmsModbusBmsTaskGetLatestPacket(&packet);
        if (hasPacket) {
            srcUpdatedMs = (uint32_t)(packet.timestampUs / 1000ULL);
        }
        if (haveSnapshot) {
            ESP_LOGD(BRIDGE_TAG, "[FILL] Using JKBMS snapshot: soc=%u%%, v=%.2fV",
                     snapshot.socPct, (double)snapshot.packVoltageMv / 1000.0);
            fillTelemetryFromJkbmsSnapshot(&snapshot, out);
            if (srcUpdatedMs != 0u) {
                out->updatedMs = srcUpdatedMs;
                if (updatedMsOut != NULL) {
                    *updatedMsOut = srcUpdatedMs;
                }
            }
            return;
        }
    } else if (settings.bms_protocol == PROTOCOL_RS485_JKBMS_NATIVE) {
        jkbms_rs485_native_snapshot_t snapshot = {0};
        bool haveSnapshot = jkbmsRs485BmsTaskGetLatestSnapshot(&snapshot);
        hasPacket = jkbmsRs485BmsTaskGetLatestPacket(&packet);
        if (hasPacket) {
            srcUpdatedMs = (uint32_t)(packet.timestampUs / 1000ULL);
        }
        if (haveSnapshot) {
            ESP_LOGD(BRIDGE_TAG, "[FILL] Using JKBMS native snapshot: soc=%u%%, v=%.2fV",
                     snapshot.socPct, (double)snapshot.packVoltageMv / 1000.0);
            fillTelemetryFromJkbmsNativeSnapshot(&snapshot, out);
            if (srcUpdatedMs != 0u) {
                out->updatedMs = srcUpdatedMs;
                if (updatedMsOut != NULL) {
                    *updatedMsOut = srcUpdatedMs;
                }
            }
            return;
        }
    } else if (settings.bms_protocol == PROTOCOL_RS485_PACE) {
        hasPacket = paceModbusBmsTaskGetLatestPacket(&packet);
        if (hasPacket) {
            srcUpdatedMs = (uint32_t)(packet.timestampUs / 1000ULL);
        }
    } else if (settings.bms_protocol == PROTOCOL_RS485_VOLTRONIC) {
        voltronic_modbus_snapshot_t snapshot = {0};
        bool haveSnapshot = voltronicModbusBmsTaskGetLatestSnapshot(&snapshot);
        hasPacket = voltronicModbusBmsTaskGetLatestPacket(&packet);
        if (hasPacket) {
            srcUpdatedMs = (uint32_t)(packet.timestampUs / 1000ULL);
        }
        if (haveSnapshot) {
            ESP_LOGD(BRIDGE_TAG, "[FILL] Using Voltronic snapshot: soc=%u%%, v=%.2fV",
                     snapshot.socPct, (double)snapshot.packVoltageV);
            fillTelemetryFromVoltronicSnapshot(&snapshot, out);
            if (srcUpdatedMs == 0u && snapshot.timestampUs > 0) {
                srcUpdatedMs = (uint32_t)(snapshot.timestampUs / 1000ULL);
            }
            if (srcUpdatedMs != 0u) {
                out->updatedMs = srcUpdatedMs;
                if (updatedMsOut != NULL) {
                    *updatedMsOut = srcUpdatedMs;
                }
            }
            return;
        }
    } else if (settings.bms_protocol == PROTOCOL_RS485_CHINA_TOWER) {
        hasPacket = chinaTowerModbusBmsTaskGetLatestPacket(&packet);
        if (hasPacket) {
            srcUpdatedMs = (uint32_t)(packet.timestampUs / 1000ULL);
        }
    } else if (settings.bms_protocol == PROTOCOL_RS485_WOW) {
        hasPacket = wowModbusBmsTaskGetLatestPacket(&packet);
        if (hasPacket) {
            srcUpdatedMs = (uint32_t)(packet.timestampUs / 1000ULL);
        }
    } else {
        hasPacket = growattBmsTaskGetLatestPacket(&packet);
        if (hasPacket) {
            srcUpdatedMs = (uint32_t)(packet.timestampUs / 1000ULL);
        }
    }

    if (!hasPacket) {
        ESP_LOGD(BRIDGE_TAG, "[FILL] No packet available from BMS task");
        return;
    }

    ESP_LOGD(BRIDGE_TAG, "[FILL] Using BMS task packet: protocol=%s, soc=%u%%",
             bridgeBmsTaskDebugName(settings.bms_protocol),
             packet.hasSoc ? packet.socPct : 0);

    out->valid = true;
    snprintf(out->source,
             sizeof(out->source),
             "%s",
             bridgeBmsTaskSourceName(settings.bms_protocol));
    snprintf(out->protocol, sizeof(out->protocol), "%s", protocolToStr(settings.bms_protocol));

    if (packet.hasSoc) {
        out->socPct = packet.socPct;
    }
    if (packet.hasTemperatureC) {
        out->tempMosC = (float)packet.temperatureC;
        if (packet.sourceProtocol != PROTOCOL_ID_GROWATT) {
            out->tempT1C = (float)packet.temperatureC;
            out->tempT2C = (float)packet.temperatureC;
            out->tempT4C = (float)packet.temperatureC;
            out->tempT5C = (float)packet.temperatureC;
        }
    }
    fillPaceTemperatureFields(&packet, out);
    inferFixedTemperatureCount(&packet, out);
    if (packet.hasCellExtremes) {
        out->cellMaxV = (float)packet.maxCellMv / 1000.0f;
        out->cellMinV = (float)packet.minCellMv / 1000.0f;
        out->cellMaxIdx = packet.maxCellIndex;
        out->cellMinIdx = packet.minCellIndex;
        out->deltaV = out->cellMaxV - out->cellMinV;
    }
    if (packet.cellCount > 0u) {
        uint32_t sumMv = 0u;
        uint8_t counted = 0u;
        uint8_t limit = (packet.cellCount > BMS_DECODED_PACKET_MAX_CELLS)
                            ? BMS_DECODED_PACKET_MAX_CELLS
                            : packet.cellCount;
        out->cellCount = limit;
        for (uint8_t i = 0u; i < limit && i < 32u; i++) {
            out->cellVoltagesV[i] = (float)packet.cellMv[i] / 1000.0f;
            if (packet.cellMv[i] > 0u) {
                sumMv += packet.cellMv[i];
                counted++;
            }
        }
        if (counted > 0u) {
            out->cellAvgV = ((float)sumMv / (float)counted) / 1000.0f;
        }
        if (out->deltaV <= 0.0f && out->cellMaxV >= out->cellMinV) {
            out->cellDiffV = out->cellMaxV - out->cellMinV;
        } else {
            out->cellDiffV = out->deltaV;
        }
    }
    if (packet.hasPackVoltageCv) {
        out->packVoltageV = (float)packet.packVoltageCv / 100.0f;
        out->currentA = 0.0f;
    }
    fillPaceAlertFields(&packet, out);
    fillChinaTowerAlertFields(&packet, out);
    fillGrowattAlertFields(&packet, out);

    if (srcUpdatedMs != 0u) {
        out->updatedMs = srcUpdatedMs;
        if (updatedMsOut != NULL) {
            *updatedMsOut = srcUpdatedMs;
        }
    }
}

static void buildFallbackLog(char *out, uint32_t outSize)
{
    bridgeTelemetrySnapshot_t snap = {0};
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    jkbms_modbus_snapshot_t jkbms = {0};
    bool haveJkbmsSnapshot = false;

    if (bridgeProtocolIsRs485JkbmsModbus(settings.bms_protocol)) {
        haveJkbmsSnapshot = jkbmsModbusBmsTaskGetLatestSnapshot(&jkbms);
    }

    bridgeGetTelemetrySnapshot(&snap);
    if (haveJkbmsSnapshot) {
        snprintf(out,
                 outSize,
                 "Runtime\n"
                 "  mode         : %s\n"
                 "  bms prot     : %s\n"
                 "  inv prot     : %s\n"
                 "JKBMS Telemetry\n"
                 "  valid        : %s\n"
                 "  SOC / SOH    : %u %% / %u %%\n"
                 "  Pack         : %.3f V | %.3f A | %.1f W\n"
                 "  Capacity     : %.3f Ah / %.3f Ah\n"
                 "  Temperatures : MOS %.1f C | T1 %.1f C | T2 %.1f C\n"
                 "  Cells        : count=%u max=%.3fV(#%u) min=%.3fV(#%u) dV=%.3fV avg=%.3fV\n"
                 "  AlarmRaw     : 0x%08" PRIX32 "\n"
                 "  Alarm bits   : %s\n",
                 modeToStr(settings.mode),
                 protocolToStr(settings.bms_protocol),
                 protocolToStr(settings.inverter_protocol),
                 snap.valid ? "YES" : "NO",
                 (unsigned)snap.socPct,
                 (unsigned)snap.sohPct,
                 (double)snap.packVoltageV,
                 (double)snap.currentA,
                 (double)snap.packPowerW,
                 (double)snap.remainingAh,
                 (double)snap.fullAh,
                 (double)snap.tempMosC,
                 (double)snap.tempT1C,
                 (double)snap.tempT2C,
                 (unsigned)snap.cellCount,
                 (double)snap.cellMaxV,
                 (unsigned)snap.cellMaxIdx,
                 (double)snap.cellMinV,
                 (unsigned)snap.cellMinIdx,
                 (double)snap.deltaV,
                 (double)snap.cellAvgV,
                 snap.alarmRaw,
                 (snap.alarms[0] != '\0') ? snap.alarms : "None");
        return;
    }

    snprintf(out,
             outSize,
             "Runtime\n"
             "  mode     : %s\n"
             "  bms prot : %s\n"
             "  inv prot : %s\n"
             "Telemetry\n"
             "  valid    : %s\n"
             "  source   : %s\n"
             "  protocol : %s\n"
             "  SOC      : %u %%\n"
             "  Temp MOS : %.1f C\n"
             "  Cell max : %.3f V (#%u)\n"
             "  Cell min : %.3f V (#%u)\n"
             "  Delta    : %.3f V\n",
             modeToStr(settings.mode),
             protocolToStr(settings.bms_protocol),
             protocolToStr(settings.inverter_protocol),
             snap.valid ? "YES" : "NO",
             snap.source[0] != '\0' ? snap.source : "-",
             snap.protocol[0] != '\0' ? snap.protocol : "-",
             (unsigned)snap.socPct,
             (double)snap.tempMosC,
             (double)snap.cellMaxV,
             (unsigned)snap.cellMaxIdx,
             (double)snap.cellMinV,
             (unsigned)snap.cellMinIdx,
             (double)snap.deltaV);
}

void bridgeReloadFromRuntimeSettings(void)
{
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    esp_err_t err = workingModesApplyRuntimeSettings();
    ESP_LOGW(BRIDGE_TAG,
             "Runtime settings updated (mode=%s, bms=%s, inverter=%s, apply=0x%x).",
             modeToStr(settings.mode),
             protocolToStr(settings.bms_protocol),
             protocolToStr(settings.inverter_protocol),
             (unsigned)err);
}

void bridgeGetTelemetrySnapshot(bridgeTelemetrySnapshot_t *out)
{
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    uint32_t nowMs = bridgeNowMs();
    uint32_t updatedMs = 0u;
    bool manualStale = false;

    if (out == NULL) {
        return;
    }

    memset(out, 0, sizeof(*out));
    snprintf(out->source, sizeof(out->source), "runtime");
    snprintf(out->protocol, sizeof(out->protocol), "%s", protocolToStr(settings.bms_protocol));

    bool usedManualCache = false;
    bool manualCacheStale = false;
    bool noManualCache = false;
    uint32_t manualAge = 0u;
    char manualSource[32] = {0};
    uint8_t manualSoc = 0;

    portENTER_CRITICAL(&g_bridgeMux);
    if (g_haveManualTelemetry) {
        updatedMs = g_manualTelemetryUpdatedMs;
        manualAge = (nowMs >= g_manualTelemetryUpdatedMs) ? (nowMs - g_manualTelemetryUpdatedMs) : 0u;
        if ((g_manualTelemetryUpdatedMs != 0u) &&
            ((nowMs - g_manualTelemetryUpdatedMs) <= WEB_TELEMETRY_STALE_MS)) {
            *out = g_manualTelemetry;
            usedManualCache = true;
            snprintf(manualSource, sizeof(manualSource), "%s", g_manualTelemetry.source);
            manualSoc = g_manualTelemetry.socPct;
        } else if (g_manualTelemetryUpdatedMs != 0u) {
            manualStale = true;
            manualCacheStale = true;
            g_haveManualTelemetry = false;
        }
    } else {
        noManualCache = true;
    }
    portEXIT_CRITICAL(&g_bridgeMux);

    /* Log after exiting critical section to avoid potential deadlock */
    if (usedManualCache) {
        ESP_LOGD(BRIDGE_TAG, "[TELEM] Using manual cache: age=%u ms, source=%s, valid=%s, soc=%u%%",
                 manualAge, manualSource, out->valid ? "YES" : "NO", manualSoc);
    } else if (manualCacheStale) {
        ESP_LOGD(BRIDGE_TAG, "[TELEM] Manual cache STALE: age=%u ms > threshold=%d ms",
                 manualAge, WEB_TELEMETRY_STALE_MS);
    } else if (noManualCache) {
        ESP_LOGD(BRIDGE_TAG, "[TELEM] No manual cache available");
    }

    ESP_LOGD(BRIDGE_TAG, "[TELEM] Before fill: valid=%s, source=%s",
             out->valid ? "YES" : "NO", out->source);

    fillTelemetryFromLatestPacket(out, &updatedMs);

    ESP_LOGD(BRIDGE_TAG, "[TELEM] After fill: valid=%s, source=%s, updatedMs=%u",
             out->valid ? "YES" : "NO", out->source, updatedMs);

    if (updatedMs != 0u) {
        out->updatedMs = updatedMs;
        out->ageMs = (nowMs >= updatedMs) ? (nowMs - updatedMs) : 0u;
    } else {
        out->ageMs = 0u;
    }

    /* Override source with user-friendly interface name (CAN1, RS485_1, etc.) before stale check */
    if (out->valid) {
        char iface[16] = {0};
        bridgeFormatBmsInterface(iface, sizeof(iface), &settings);
        if (iface[0] != '\0') {
            snprintf(out->source, sizeof(out->source), "%s", iface);
        }
    }

    {
        bool ageStale = (out->updatedMs != 0u) && (out->ageMs > WEB_TELEMETRY_STALE_MS);
        out->stale = ageStale || (manualStale && (updatedMs == 0u));
        ESP_LOGD(BRIDGE_TAG, "[TELEM] Stale check: ageStale=%s (age=%u ms, threshold=%d ms), "
                 "manualStale=%s, final_stale=%s",
                 ageStale ? "YES" : "NO", out->ageMs, WEB_TELEMETRY_STALE_MS,
                 manualStale ? "YES" : "NO", out->stale ? "YES" : "NO");
    }

    if (out->stale) {
        ESP_LOGI(BRIDGE_TAG, "[TELEM] Data is STALE - clearing telemetry snapshot");
        char protocol[sizeof(out->protocol)] = {0};
        snprintf(protocol, sizeof(protocol), "%s", out->protocol);
        memset(out, 0, sizeof(*out));
        snprintf(out->protocol, sizeof(out->protocol), "%s", protocol);
        out->stale = true;
        out->updatedMs = updatedMs;
        out->ageMs = (updatedMs != 0u && nowMs >= updatedMs) ? (nowMs - updatedMs) : 0u;
        return;
    }

    ESP_LOGI(BRIDGE_TAG, "[TELEM] Final snapshot: valid=%s, source=%s, soc=%u%%, v=%.2fV, age=%u ms",
             out->valid ? "YES" : "NO", out->source, out->socPct,
             (double)out->packVoltageV, out->ageMs);
}

void bridgeSetTelemetrySnapshot(const bridgeTelemetrySnapshot_t *in)
{
    uint32_t nowMs = bridgeNowMs();
    bool didClear = false;
    bool didUpdate = false;
    char logSource[32] = {0};
    uint8_t logSoc = 0;
    float logVoltage = 0.0f;

    portENTER_CRITICAL(&g_bridgeMux);
    if (in == NULL) {
        didClear = true;
        memset(&g_manualTelemetry, 0, sizeof(g_manualTelemetry));
        g_haveManualTelemetry = false;
        g_manualTelemetryUpdatedMs = 0u;
    } else {
        if (!in->valid && g_haveManualTelemetry && g_manualTelemetry.valid) {
            portEXIT_CRITICAL(&g_bridgeMux);
            ESP_LOGD(BRIDGE_TAG, "[TELEM_SET] Ignoring invalid update (existing cache is valid)");
            return;
        }
        didUpdate = true;
        snprintf(logSource, sizeof(logSource), "%s", in->source);
        logSoc = in->socPct;
        logVoltage = in->packVoltageV;
        g_manualTelemetry = *in;
        g_manualTelemetry.updatedMs = in->valid ? nowMs : 0u;
        g_manualTelemetry.ageMs = 0u;
        g_manualTelemetry.stale = false;
        g_manualTelemetryUpdatedMs = g_manualTelemetry.updatedMs;
        g_haveManualTelemetry = true;
    }
    portEXIT_CRITICAL(&g_bridgeMux);

    /* Log after exiting critical section to avoid potential deadlock */
    if (didClear) {
        ESP_LOGI(BRIDGE_TAG, "[TELEM_SET] Clearing manual cache (NULL input)");
    } else if (didUpdate) {
        ESP_LOGI(BRIDGE_TAG, "[TELEM_SET] Updating manual cache: source=%s, valid=%s, soc=%u%%, v=%.2fV",
                 logSource, in->valid ? "YES" : "NO", logSoc, (double)logVoltage);
    }
}

void bridgeGetDecodedLogSnapshot(char *out, uint32_t outSize)
{
    char cached[sizeof(g_decodedLog)] = {0};

    if (out == NULL || outSize == 0u) {
        return;
    }

    portENTER_CRITICAL(&g_bridgeMux);
    snprintf(cached, sizeof(cached), "%s", g_decodedLog);
    portEXIT_CRITICAL(&g_bridgeMux);

    if (cached[0] != '\0') {
        snprintf(out, outSize, "%s", cached);
        return;
    }

    buildFallbackLog(out, outSize);
}

void bridgeSetDecodedLogSnapshot(const char *text)
{
    portENTER_CRITICAL(&g_bridgeMux);
    if (text == NULL) {
        g_decodedLog[0] = '\0';
    } else {
        snprintf(g_decodedLog, sizeof(g_decodedLog), "%s", text);
    }
    portEXIT_CRITICAL(&g_bridgeMux);
}
