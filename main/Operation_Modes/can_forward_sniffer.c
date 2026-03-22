#include "Operation_Modes/can_forward_sniffer.h"

#include <inttypes.h>
#include <stdio.h>

#include "config.h"
#include "CAN_Decoder.h"
#include "Drivers/CAN/can_driver.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    const char   *rxName;
    const char   *txName;
    twai_handle_t rxBus;
    twai_handle_t txBus;
    bool          applyExcludeList;
    bool          forwardEnabled;
    bool          rawLogEnabled;
    const char   *rawLogLabel;
} canBridgeCtx_t;

static TaskHandle_t s_canTaskA = NULL;
static TaskHandle_t s_canTaskB = NULL;
static TaskHandle_t s_canSnapshotTask = NULL;

static void deleteTaskIfRunning(TaskHandle_t *handle)
{
    if (handle != NULL && *handle != NULL) {
        vTaskDelete(*handle);
        *handle = NULL;
    }
}

static bool canIdExcludedToInverter(uint32_t id)
{
    for (size_t i = 0; i < g_can1ToCan2ExcludeIdsCount; i++) {
        if (g_can1ToCan2ExcludeIds[i] == id) {
            return true;
        }
    }
    return false;
}

static const char *canNameByPort(int port)
{
    return (port == 1) ? "CAN1" : "CAN2";
}

static twai_handle_t canBusByPort(int port)
{
    return (port == 1) ? canGetBus0() : canGetBus1();
}

static const char *canProtocolLabel(int protocol)
{
    switch (protocol) {
        case PROTOCOL_CAN_PYLON: return "PYLON";
        case PROTOCOL_CAN_DEYE: return "DEYE";
        case PROTOCOL_CAN_GROWATT: return "GROWATT";
        default: return "CAN";
    }
}

static void logRawCanFrameLabeled(const char *label, const char *ifname, const twai_message_t *m)
{
    char dataHex[3 * 8 + 1] = {0};
    int pos = 0;
    int dlc = 0;

    if (ifname == NULL || m == NULL) {
        return;
    }

    dlc = m->data_length_code;
    if (dlc > 8) {
        dlc = 8;
    }

    for (int i = 0; i < dlc; i++) {
        pos += snprintf(&dataHex[pos], sizeof(dataHex) - (size_t)pos, "%02X ", m->data[i]);
        if (pos >= (int)sizeof(dataHex)) {
            break;
        }
    }

    ESP_LOGI(EXAMPLE_TAG,
             "CAN %s RAW on %s: ID=0x%03" PRIX32 " DLC=%d DATA=[%s]",
             (label != NULL && label[0] != '\0') ? label : "RAW",
             ifname,
             (uint32_t)m->identifier,
             dlc,
             dataHex);
}

static void canPeriodicSnapshotTask(void *pv)
{
    const char *ifname = (const char *)pv;

    vTaskDelay(pdMS_TO_TICKS(CAN_DECODER_SNAPSHOT_PRINT_PERIOD_MS));
    while (1) {
        canDecoderPrintCachedSnapshot(ifname);
        vTaskDelay(pdMS_TO_TICKS(CAN_DECODER_SNAPSHOT_PRINT_PERIOD_MS));
    }
}

static void canBridgeTask(void *pv)
{
    canBridgeCtx_t *ctx = (canBridgeCtx_t *)pv;
    twai_message_t rx;

    while (1) {
        if (twai_receive_v2(ctx->rxBus, &rx, portMAX_DELAY) != ESP_OK) {
            continue;
        }
#ifdef TWAI_MSG_FLAG_SELF
        if (rx.flags & TWAI_MSG_FLAG_SELF) {
            continue;
        }
#endif
        canDecoderOnFrame(ctx->rxName, &rx);
        if (ctx->rawLogEnabled) {
            logRawCanFrameLabeled(ctx->rawLogLabel, ctx->rxName, &rx);
        }

        if (!ctx->forwardEnabled) {
            continue;
        }
        if (ctx->applyExcludeList && canIdExcludedToInverter((uint32_t)rx.identifier)) {
            continue;
        }

        esp_err_t e = twai_transmit_v2(ctx->txBus, &rx, pdMS_TO_TICKS(50));
        if (e != ESP_OK) {
            ESP_LOGW(EXAMPLE_TAG,
                     "CAN forward %s -> %s failed (err=0x%x)",
                     ctx->rxName,
                     ctx->txName,
                     (unsigned)e);
        }
    }
}

void canForwardSnifferStop(void)
{
    deleteTaskIfRunning(&s_canTaskA);
    deleteTaskIfRunning(&s_canTaskB);
    deleteTaskIfRunning(&s_canSnapshotTask);
}

void canForwardSnifferStart(const bridge_runtime_settings_t *settings)
{
    static canBridgeCtx_t bmsToInv;
    static canBridgeCtx_t invToBms;
    static canBridgeCtx_t bmsSniff;
    static canBridgeCtx_t invSniff;
    const bool bmsOnCan = (settings != NULL) && (settings->bms_line == LINE_CAN);
    const bool invOnCan = (settings != NULL) && (settings->inverter_line == LINE_CAN);
    const bool bmsCanPylon = (settings != NULL) &&
                             (settings->bms_line == LINE_CAN) &&
                             (settings->bms_protocol == PROTOCOL_CAN_PYLON);
    const bool invCanPylon = (settings != NULL) &&
                             (settings->inverter_line == LINE_CAN) &&
                             (settings->inverter_protocol == PROTOCOL_CAN_PYLON);
    const bool bmsCanDeye = (settings != NULL) &&
                            (settings->bms_line == LINE_CAN) &&
                            (settings->bms_protocol == PROTOCOL_CAN_DEYE);
    const bool invCanDeye = (settings != NULL) &&
                            (settings->inverter_line == LINE_CAN) &&
                            (settings->inverter_protocol == PROTOCOL_CAN_DEYE);
    const bool sameCanProtocol = (settings != NULL) &&
                                 bmsOnCan &&
                                 invOnCan &&
                                 (settings->bms_protocol == settings->inverter_protocol);
    const bool canForwardEnabled = (settings != NULL) &&
                                   ((settings->mode == MODE_FORWARD) ||
                                    ((settings->mode == MODE_BRIDGE) && sameCanProtocol));

    canForwardSnifferStop();

    if (settings == NULL) {
        return;
    }

    if (bmsOnCan || invOnCan) {
        const char *snapIf = bmsOnCan ? canNameByPort(settings->bms_port) : canNameByPort(settings->inverter_port);
        xTaskCreate(canPeriodicSnapshotTask, "can_snapshot", 6144, (void *)snapIf, 7, &s_canSnapshotTask);
        ESP_LOGI(EXAMPLE_TAG, "CAN periodic snapshot enabled (%d ms)", CAN_DECODER_SNAPSHOT_PRINT_PERIOD_MS);
    }

    if (bmsOnCan && invOnCan) {
        bmsToInv.rxName = canNameByPort(settings->bms_port);
        bmsToInv.txName = canNameByPort(settings->inverter_port);
        bmsToInv.rxBus = canBusByPort(settings->bms_port);
        bmsToInv.txBus = canBusByPort(settings->inverter_port);
        bmsToInv.applyExcludeList = CAN_EXCLUDE_LIST_ENABLE;
        bmsToInv.forwardEnabled = canForwardEnabled;
        bmsToInv.rawLogEnabled = bmsCanPylon || bmsCanDeye;
        bmsToInv.rawLogLabel = canProtocolLabel(settings->bms_protocol);

        invToBms.rxName = canNameByPort(settings->inverter_port);
        invToBms.txName = canNameByPort(settings->bms_port);
        invToBms.rxBus = canBusByPort(settings->inverter_port);
        invToBms.txBus = canBusByPort(settings->bms_port);
        invToBms.applyExcludeList = false;
        invToBms.forwardEnabled = canForwardEnabled;
        invToBms.rawLogEnabled = invCanPylon || invCanDeye;
        invToBms.rawLogLabel = canProtocolLabel(settings->inverter_protocol);

        xTaskCreate(canBridgeTask, "can_bms_to_inv", 4096, &bmsToInv, 10, &s_canTaskA);
        xTaskCreate(canBridgeTask, "can_inv_to_bms", 4096, &invToBms, 10, &s_canTaskB);

        ESP_LOGI(EXAMPLE_TAG,
                 "CAN bridge enabled (%s[P%d] <-> %s[P%d]), forward=%s, exclude=%s",
                 bmsToInv.rxName,
                 settings->bms_port,
                 bmsToInv.txName,
                 settings->inverter_port,
                 canForwardEnabled ? "ON" : "OFF",
                 CAN_EXCLUDE_LIST_ENABLE ? "ON" : "OFF");
        if ((settings->mode == MODE_BRIDGE) && sameCanProtocol) {
            ESP_LOGI(EXAMPLE_TAG,
                     "CAN bridge mode uses direct pass-through because both sides are %s",
                     (settings->bms_protocol == PROTOCOL_CAN_PYLON) ? "CAN_PYLON" :
                     ((settings->bms_protocol == PROTOCOL_CAN_DEYE) ? "CAN_DEYE" : "CAN_GROWATT"));
        }
        return;
    }

    if (bmsOnCan) {
        bmsSniff.rxName = canNameByPort(settings->bms_port);
        bmsSniff.txName = canNameByPort(settings->bms_port);
        bmsSniff.rxBus = canBusByPort(settings->bms_port);
        bmsSniff.txBus = canBusByPort(settings->bms_port);
        bmsSniff.applyExcludeList = false;
        bmsSniff.forwardEnabled = false;
        bmsSniff.rawLogEnabled = bmsCanPylon || bmsCanDeye;
        bmsSniff.rawLogLabel = canProtocolLabel(settings->bms_protocol);
        xTaskCreate(canBridgeTask, "can_bms_sniff", 4096, &bmsSniff, 10, &s_canTaskA);
        ESP_LOGI(EXAMPLE_TAG, "CAN sniffer enabled on BMS side (%s[P%d])", bmsSniff.rxName, settings->bms_port);
        if (bmsCanPylon) {
            ESP_LOGI(EXAMPLE_TAG, "CAN Pylon diagnostic logging enabled on BMS side (%s[P%d])",
                     bmsSniff.rxName,
                     settings->bms_port);
        } else if (bmsCanDeye) {
            ESP_LOGI(EXAMPLE_TAG, "CAN Deye diagnostic logging enabled on BMS side (%s[P%d])",
                     bmsSniff.rxName,
                     settings->bms_port);
        }
    }

    if (invOnCan) {
        invSniff.rxName = canNameByPort(settings->inverter_port);
        invSniff.txName = canNameByPort(settings->inverter_port);
        invSniff.rxBus = canBusByPort(settings->inverter_port);
        invSniff.txBus = canBusByPort(settings->inverter_port);
        invSniff.applyExcludeList = false;
        invSniff.forwardEnabled = false;
        invSniff.rawLogEnabled = invCanPylon || invCanDeye;
        invSniff.rawLogLabel = canProtocolLabel(settings->inverter_protocol);
        xTaskCreate(canBridgeTask, "can_inv_sniff", 4096, &invSniff, 10, &s_canTaskB);
        ESP_LOGI(EXAMPLE_TAG, "CAN sniffer enabled on inverter side (%s[P%d])", invSniff.rxName, settings->inverter_port);
        if (invCanPylon) {
            ESP_LOGI(EXAMPLE_TAG, "CAN Pylon diagnostic logging enabled on inverter side (%s[P%d])",
                     invSniff.rxName,
                     settings->inverter_port);
        } else if (invCanDeye) {
            ESP_LOGI(EXAMPLE_TAG, "CAN Deye diagnostic logging enabled on inverter side (%s[P%d])",
                     invSniff.rxName,
                     settings->inverter_port);
        }
    }

    if (!bmsOnCan && !invOnCan) {
        ESP_LOGI(EXAMPLE_TAG, "CAN bridge/sniffer not used by current topology");
    }
}
