#include "bridge.h"

#include <stdio.h>
#include <string.h>

#include "config.h"
#include "orchestrator/protocol_types.h"
#include "protocols/growatt/growatt_bms_task.h"
#include "runtime_settings.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define BRIDGE_TAG "BRIDGE_COMPAT"

static bridgeTelemetrySnapshot_t g_manualTelemetry;
static bool g_haveManualTelemetry;
static char g_decodedLog[2048];
static portMUX_TYPE g_bridgeMux = portMUX_INITIALIZER_UNLOCKED;

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
        default:
            return "UNKNOWN";
    }
}

static void fillTelemetryFromLatestPacket(bridgeTelemetrySnapshot_t *out)
{
    bms_decoded_packet_t packet = {0};
    bridge_runtime_settings_t settings = runtimeSettingsGet();

    if (!growattBmsTaskGetLatestPacket(&packet)) {
        return;
    }

    out->valid = true;
    snprintf(out->source, sizeof(out->source), "GROWATT_BMS_TASK");
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
        out->currentA = 0.0f;
    }
}

static void buildFallbackLog(char *out, uint32_t outSize)
{
    bridgeTelemetrySnapshot_t snap = {0};
    bridge_runtime_settings_t settings = runtimeSettingsGet();

    bridgeGetTelemetrySnapshot(&snap);
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
    ESP_LOGW(BRIDGE_TAG,
             "Runtime settings updated (mode=%s, bms=%s, inverter=%s).",
             modeToStr(settings.mode),
             protocolToStr(settings.bms_protocol),
             protocolToStr(settings.inverter_protocol));
}

void bridgeGetTelemetrySnapshot(bridgeTelemetrySnapshot_t *out)
{
    bridge_runtime_settings_t settings = runtimeSettingsGet();

    if (out == NULL) {
        return;
    }

    memset(out, 0, sizeof(*out));
    snprintf(out->source, sizeof(out->source), "runtime");
    snprintf(out->protocol, sizeof(out->protocol), "%s", protocolToStr(settings.bms_protocol));

    portENTER_CRITICAL(&g_bridgeMux);
    if (g_haveManualTelemetry) {
        *out = g_manualTelemetry;
    }
    portEXIT_CRITICAL(&g_bridgeMux);

    fillTelemetryFromLatestPacket(out);
}

void bridgeSetTelemetrySnapshot(const bridgeTelemetrySnapshot_t *in)
{
    portENTER_CRITICAL(&g_bridgeMux);
    if (in == NULL) {
        memset(&g_manualTelemetry, 0, sizeof(g_manualTelemetry));
        g_haveManualTelemetry = false;
    } else {
        g_manualTelemetry = *in;
        g_haveManualTelemetry = true;
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
