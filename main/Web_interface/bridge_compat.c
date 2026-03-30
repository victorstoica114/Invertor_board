#include "bridge.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "modes/mode_manager.h"
#include "config.h"
#include "orchestrator/protocol_types.h"
#include "protocols/growatt/growatt_bms_task.h"
#include "protocols/jkbms_modbus/jkbms_modbus_bms_task.h"
#include "rs485_can_bridge.h"
#include "runtime_settings.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#define BRIDGE_TAG "BRIDGE_COMPAT"

static bridgeTelemetrySnapshot_t g_manualTelemetry;
static universal_battery_model_t g_universalBatteryModel;
static bool g_haveManualTelemetry;
static uint32_t g_manualTelemetryUpdatedMs;
static char g_decodedLog[2048];
static portMUX_TYPE g_bridgeMux = portMUX_INITIALIZER_UNLOCKED;

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
        case PROTOCOL_RS485_JKBMS:
            return "JKBMS_MODBUS";
        default:
            return "UNKNOWN";
    }
}

static void formatBitList(uint32_t bits, const char *prefix, char *out, size_t outSize)
{
    size_t used = 0u;
    if (out == NULL || outSize == 0u) {
        return;
    }

    out[0] = '\0';
    if (bits == 0u) {
        return;
    }

    for (uint8_t i = 0; i < 32u; i++) {
        if ((bits & (1u << i)) == 0u) {
            continue;
        }

        int w = snprintf(out + used,
                         outSize - used,
                         "%s%s%u",
                         (used == 0u) ? "" : ", ",
                         (prefix != NULL) ? prefix : "B",
                         (unsigned)i);
        if (w <= 0) {
            break;
        }
        if ((size_t)w >= (outSize - used)) {
            used = outSize - 1u;
            out[used] = '\0';
            break;
        }
        used += (size_t)w;
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

    if (snapshot->hasCellExtremes) {
        out->cellMaxV = (float)snapshot->maxCellMv / 1000.0f;
        out->cellMinV = (float)snapshot->minCellMv / 1000.0f;
        out->cellMaxIdx = snapshot->maxCellIndex;
        out->cellMinIdx = snapshot->minCellIndex;
        out->deltaV = out->cellMaxV - out->cellMinV;
    }

    if (snapshot->hasAlarmBits) {
        out->alarmRaw = snapshot->alarmBits;
        formatBitList(snapshot->alarmBits, "A", out->alarms, sizeof(out->alarms));
        formatBitList(snapshot->alarmBits & 0x0000FFFFu, "P", out->protections, sizeof(out->protections));
        formatBitList((snapshot->alarmBits >> 16), "W", out->warnings, sizeof(out->warnings));
    }

    if (snapshot->hasPrecharge) {
        out->prechargeState = snapshot->prechargeState;
    }

    snprintf(out->stateFlags,
             sizeof(out->stateFlags),
             "AlarmRaw=0x%08" PRIX32 ", Precharge=%u, Cells=%u",
             snapshot->hasAlarmBits ? snapshot->alarmBits : 0u,
             snapshot->hasPrecharge ? snapshot->prechargeState : 0u,
             (unsigned)snapshot->cellCount);
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
          (settings.bms_protocol == PROTOCOL_RS485_PYLON) &&
          (settings.inverter_protocol == PROTOCOL_RS485_PYLON)) ||
         ((settings.bms_line == LINE_CAN) &&
          (settings.bms_protocol == PROTOCOL_CAN_PYLON) &&
          (settings.inverter_protocol == PROTOCOL_RS485_PYLON)));

    if (pylonRs485Route) {
        return;
    }

    if (canToRsGrowattRoute && canRs485GrowattBridgeGetLatestSnapshot(&canRsSnap)) {
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

    if (settings.bms_protocol == PROTOCOL_RS485_JKBMS) {
        jkbms_modbus_snapshot_t snapshot = {0};
        bool haveSnapshot = jkbmsModbusBmsTaskGetLatestSnapshot(&snapshot);
        hasPacket = jkbmsModbusBmsTaskGetLatestPacket(&packet);
        if (hasPacket) {
            srcUpdatedMs = (uint32_t)(packet.timestampUs / 1000ULL);
        }
        if (haveSnapshot) {
            fillTelemetryFromJkbmsSnapshot(&snapshot, out);
            if (srcUpdatedMs != 0u) {
                out->updatedMs = srcUpdatedMs;
                if (updatedMsOut != NULL) {
                    *updatedMsOut = srcUpdatedMs;
                }
            }
            return;
        }
    } else {
        hasPacket = growattBmsTaskGetLatestPacket(&packet);
        if (hasPacket) {
            srcUpdatedMs = (uint32_t)(packet.timestampUs / 1000ULL);
        }
    }

    if (!hasPacket) {
        return;
    }

    out->valid = true;
    snprintf(out->source,
             sizeof(out->source),
             "%s",
             (settings.bms_protocol == PROTOCOL_RS485_JKBMS) ? "JKBMS_BMS_TASK" : "GROWATT_BMS_TASK");
    snprintf(out->protocol, sizeof(out->protocol), "%s", protocolToStr(settings.bms_protocol));

    if (packet.hasSoc) {
        out->socPct = packet.socPct;
    }
    if (packet.hasTemperatureC) {
        out->tempMosC = (float)packet.temperatureC;
        out->tempT1C = (float)packet.temperatureC;
        out->tempT2C = (float)packet.temperatureC;
        out->tempT4C = (float)packet.temperatureC;
        out->tempT5C = (float)packet.temperatureC;
    }
    if (packet.hasCellExtremes) {
        out->cellMaxV = (float)packet.maxCellMv / 1000.0f;
        out->cellMinV = (float)packet.minCellMv / 1000.0f;
        out->cellMaxIdx = packet.maxCellIndex;
        out->cellMinIdx = packet.minCellIndex;
        out->deltaV = out->cellMaxV - out->cellMinV;
    }
    if (packet.hasPackVoltageCv) {
        out->packVoltageV = (float)packet.packVoltageCv / 100.0f;
        out->currentA = 0.0f;
    }

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

    if (settings.bms_protocol == PROTOCOL_RS485_JKBMS) {
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

void rs485BridgeEnable(void)
{
    /* Kept for backwards compatibility with older web/runtime integration. */
}

void canBridgeEnable(void)
{
    /* Kept for backwards compatibility with older web/runtime integration. */
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

    portENTER_CRITICAL(&g_bridgeMux);
    if (g_haveManualTelemetry) {
        updatedMs = g_manualTelemetryUpdatedMs;
        if ((g_manualTelemetryUpdatedMs != 0u) &&
            ((nowMs - g_manualTelemetryUpdatedMs) <= WEB_TELEMETRY_STALE_MS)) {
            *out = g_manualTelemetry;
        } else if (g_manualTelemetryUpdatedMs != 0u) {
            manualStale = true;
            g_haveManualTelemetry = false;
        }
    }
    portEXIT_CRITICAL(&g_bridgeMux);

    fillTelemetryFromLatestPacket(out, &updatedMs);

    if (updatedMs != 0u) {
        out->updatedMs = updatedMs;
        out->ageMs = (nowMs >= updatedMs) ? (nowMs - updatedMs) : 0u;
    } else {
        out->ageMs = 0u;
    }

    {
        bool ageStale = (out->updatedMs != 0u) && (out->ageMs > WEB_TELEMETRY_STALE_MS);
        out->stale = ageStale || (manualStale && (updatedMs == 0u));
    }

    if (out->stale) {
        char protocol[sizeof(out->protocol)] = {0};
        snprintf(protocol, sizeof(protocol), "%s", out->protocol);
        memset(out, 0, sizeof(*out));
        snprintf(out->protocol, sizeof(out->protocol), "%s", protocol);
        out->stale = true;
        out->updatedMs = updatedMs;
        out->ageMs = (updatedMs != 0u && nowMs >= updatedMs) ? (nowMs - updatedMs) : 0u;
    }
}

void bridgeSetTelemetrySnapshot(const bridgeTelemetrySnapshot_t *in)
{
    uint32_t nowMs = bridgeNowMs();

    portENTER_CRITICAL(&g_bridgeMux);
    if (in == NULL) {
        memset(&g_manualTelemetry, 0, sizeof(g_manualTelemetry));
        memset(&g_universalBatteryModel, 0, sizeof(g_universalBatteryModel));
        g_haveManualTelemetry = false;
        g_manualTelemetryUpdatedMs = 0u;
    } else {
        g_manualTelemetry = *in;
        g_manualTelemetry.updatedMs = in->valid ? nowMs : 0u;
        g_manualTelemetry.ageMs = 0u;
        g_manualTelemetry.stale = false;
        g_manualTelemetryUpdatedMs = g_manualTelemetry.updatedMs;
        g_haveManualTelemetry = true;
        g_universalBatteryModel.valid = in->valid;
        g_universalBatteryModel.packVoltageV = in->packVoltageV;
        g_universalBatteryModel.packCurrentA = in->currentA;
        g_universalBatteryModel.socPct = in->socPct;
        g_universalBatteryModel.sohPct = in->sohPct;
        g_universalBatteryModel.cycleCount = in->cycles;
        g_universalBatteryModel.cellMaxV = in->cellMaxV;
        g_universalBatteryModel.cellMinV = in->cellMinV;
        g_universalBatteryModel.cellMaxIdx = in->cellMaxIdx;
        g_universalBatteryModel.cellMinIdx = in->cellMinIdx;
        g_universalBatteryModel.cellDeltaV = in->deltaV;
        g_universalBatteryModel.temperaturesC[0] = in->tempMosC;
        g_universalBatteryModel.temperaturesC[1] = in->tempT1C;
        g_universalBatteryModel.temperaturesC[2] = in->tempT2C;
        g_universalBatteryModel.temperaturesC[3] = in->tempT4C;
        g_universalBatteryModel.temperaturesC[4] = in->tempT5C;
        g_universalBatteryModel.protocolState = in->pylonStatus63;
        g_universalBatteryModel.updatedMs = in->valid ? nowMs : 0u;
    }
    portEXIT_CRITICAL(&g_bridgeMux);
}

void bridgeGetUniversalBatteryModel(universal_battery_model_t *out)
{
    if (out == NULL) {
        return;
    }

    portENTER_CRITICAL(&g_bridgeMux);
    *out = g_universalBatteryModel;
    portEXIT_CRITICAL(&g_bridgeMux);

    if (!bridgeUniversalBatteryModelIsFresh(out)) {
        memset(out, 0, sizeof(*out));
    }
}

void bridgeSetUniversalBatteryModel(const universal_battery_model_t *in)
{
    uint32_t nowMs = bridgeNowMs();

    portENTER_CRITICAL(&g_bridgeMux);
    if (in == NULL) {
        memset(&g_universalBatteryModel, 0, sizeof(g_universalBatteryModel));
    } else {
        g_universalBatteryModel = *in;
        if (g_universalBatteryModel.valid) {
            if (g_universalBatteryModel.updatedMs == 0u) {
                g_universalBatteryModel.updatedMs = nowMs;
            }
        } else {
            g_universalBatteryModel.updatedMs = 0u;
        }
    }
    portEXIT_CRITICAL(&g_bridgeMux);
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
