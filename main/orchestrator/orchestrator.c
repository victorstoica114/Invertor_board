#include "orchestrator/orchestrator.h"

#include <string.h>

#include "Drivers/can_driver.h"
#include "Drivers/rs485_driver.h"
#include "protocols/pylon/pylon_rs485_bridge.h"
#include "decoders/CAN_Decoder.h"
#include "modes/can_forward_sniffer.h"
#include "config.h"
#include "protocols/china_tower_modbus/china_tower_modbus_bms_task.h"
#include "protocols/daly_rs485/daly_rs485_bms_task.h"
#include "protocols/growatt/growatt_bms_task.h"
#include "protocols/growatt/growatt_inverter_task.h"
#include "protocols/jkbms_modbus/jkbms_modbus_bms_task.h"
#include "protocols/jkbms_rs485/jkbms_rs485_bms_task.h"
#include "protocols/pace_modbus/pace_modbus_bms_task.h"
#include "protocols/pylon/pylon_bms_task.h"
#include "protocols/pylon/pylon_inverter_task.h"
#include "protocols/rs485_growatt/rs485_growatt_bridge.h"
#include "protocols/seplos_rs485/seplos_rs485_bms_task.h"
#include "protocols/voltronic_modbus/voltronic_modbus_bms_task.h"
#include "protocols/wow_modbus/wow_modbus_bms_task.h"
#include "protocols/common/battery_model.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#if ORCHESTRATOR_BMS_QUEUE_LEN != 1
#error "ORCHESTRATOR_BMS_QUEUE_LEN must be 1 when using xQueueOverwrite"
#endif

#if ORCHESTRATOR_INVERTER_QUEUE_LEN != 1
#error "ORCHESTRATOR_INVERTER_QUEUE_LEN must be 1 when using xQueueOverwrite"
#endif

typedef struct {
    QueueHandle_t bmsQueue;
    QueueHandle_t inverterQueue;
    protocol_id_t bmsProtocol;
    protocol_id_t inverterProtocol;
    bool canRs485TranslatorActive;

    bool haveLastForwarded;
    bms_decoded_packet_t lastForwarded;
    int64_t lastForwardUs;
} orchestratorCtx_t;

static orchestratorCtx_t g_orchestratorCtx;
static TaskHandle_t g_orchestratorTaskHandle;

const char *protocolIdToStr(protocol_id_t id)
{
    switch (id) {
        case PROTOCOL_ID_GROWATT:
            return "GROWATT";
        case PROTOCOL_ID_PYLON:
            return "PYLON";
        case PROTOCOL_ID_JKBMS:
            return "JKBMS_MODBUS";
        case PROTOCOL_ID_PACE:
            return "PACE_RS485_MODBUS";
        case PROTOCOL_ID_JKBMS_NATIVE:
            return "JKBMS_RS485_NATIVE";
        case PROTOCOL_ID_VOLTRONIC:
            return "VOLTRONIC_MODBUS";
        case PROTOCOL_ID_CHINA_TOWER:
            return "CHINA_TOWER_MODBUS";
        case PROTOCOL_ID_WOW:
            return "WOW_MODBUS";
        case PROTOCOL_ID_SEPLOS:
            return "SEPLOS_RS485";
        case PROTOCOL_ID_DALY:
            return "DALY_RS485";
        default:
            return "UNKNOWN";
    }
}

static bool packetEquivalent(const bms_decoded_packet_t *a, const bms_decoded_packet_t *b)
{
    if (a == NULL || b == NULL) {
        return false;
    }

    if ((a->hasSoc != b->hasSoc) ||
        (a->hasSoc && (a->socPct != b->socPct)) ||
        (a->hasTemperatureC != b->hasTemperatureC) ||
        (a->hasTemperatureC && (a->temperatureC != b->temperatureC)) ||
        (a->hasPackVoltageCv != b->hasPackVoltageCv) ||
        (a->hasPackVoltageCv && (a->packVoltageCv != b->packVoltageCv)) ||
        (a->hasCellExtremes != b->hasCellExtremes) ||
        (a->hasWarningFlags != b->hasWarningFlags) ||
        (a->hasWarningFlags && (a->warningFlags != b->warningFlags)) ||
        (a->hasProtectionFlags != b->hasProtectionFlags) ||
        (a->hasProtectionFlags && (a->protectionFlags != b->protectionFlags)) ||
        (a->hasStatusFlags != b->hasStatusFlags) ||
        (a->hasStatusFlags && (a->statusFlags != b->statusFlags)) ||
        (a->hasBalanceFlags != b->hasBalanceFlags) ||
        (a->hasBalanceFlags && (a->balanceFlags != b->balanceFlags))) {
        return false;
    }

    if (a->tempCount != b->tempCount) {
        return false;
    }
    if (a->tempCount > 0u) {
        uint8_t limit = (a->tempCount > BMS_DECODED_PACKET_MAX_TEMPS)
                            ? BMS_DECODED_PACKET_MAX_TEMPS
                            : a->tempCount;
        if (memcmp(a->tempDeciC, b->tempDeciC, (size_t)limit * sizeof(a->tempDeciC[0])) != 0) {
            return false;
        }
    }

    if (a->hasCellExtremes &&
        ((a->minCellMv != b->minCellMv) ||
         (a->maxCellMv != b->maxCellMv) ||
         (a->minCellIndex != b->minCellIndex) ||
         (a->maxCellIndex != b->maxCellIndex))) {
        return false;
    }

    if (a->cellCount != b->cellCount) {
        return false;
    }
    if (a->cellCount > 0u) {
        uint8_t limit = (a->cellCount > BMS_DECODED_PACKET_MAX_CELLS)
                            ? BMS_DECODED_PACKET_MAX_CELLS
                            : a->cellCount;
        if (memcmp(a->cellMv, b->cellMv, (size_t)limit * sizeof(a->cellMv[0])) != 0) {
            return false;
        }
    }

    return true;
}

static esp_err_t startBmsTask(protocol_id_t protocol, QueueHandle_t outQueue)
{
    switch (protocol) {
        case PROTOCOL_ID_GROWATT:
            return growattBmsTaskStart(outQueue);
        case PROTOCOL_ID_PYLON:
            return pylonBmsTaskStart(outQueue);
        case PROTOCOL_ID_JKBMS:
            return jkbmsModbusBmsTaskStart(outQueue);
        case PROTOCOL_ID_PACE:
            return paceModbusBmsTaskStart(outQueue);
        case PROTOCOL_ID_JKBMS_NATIVE:
            return jkbmsRs485BmsTaskStart(outQueue);
        case PROTOCOL_ID_VOLTRONIC:
            return voltronicModbusBmsTaskStart(outQueue);
        case PROTOCOL_ID_CHINA_TOWER:
            return chinaTowerModbusBmsTaskStart(outQueue);
        case PROTOCOL_ID_WOW:
            return wowModbusBmsTaskStart(outQueue);
        case PROTOCOL_ID_SEPLOS:
            return seplosRs485BmsTaskStart(outQueue);
        case PROTOCOL_ID_DALY:
            return dalyRs485BmsTaskStart(outQueue);
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}

static esp_err_t startInverterTask(protocol_id_t protocol, QueueHandle_t inQueue)
{
    switch (protocol) {
        case PROTOCOL_ID_GROWATT:
            return growattInverterTaskStart(inQueue);
        case PROTOCOL_ID_PYLON:
            return pylonInverterTaskStart(inQueue);
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}

static protocol_id_t protocolIdFromUiProtocol(uint8_t protocol)
{
    switch (protocol) {
        case PROTOCOL_CAN_GROWATT:
        case PROTOCOL_RS485_GROWATT:
        case PROTOCOL_CAN_GOODWE:
        case PROTOCOL_CAN_SOFAR:
        case PROTOCOL_CAN_SMA:
        case PROTOCOL_CAN_VICTRON:
            return PROTOCOL_ID_GROWATT;
        case PROTOCOL_CAN_PYLON:
        case PROTOCOL_RS485_PYLON:
        case PROTOCOL_RS485_PYLON_115200:
            return PROTOCOL_ID_PYLON;
        case PROTOCOL_RS485_JKBMS:
        case PROTOCOL_RS485_JKBMS_115200:
            return PROTOCOL_ID_JKBMS;
        case PROTOCOL_RS485_PACE:
            return PROTOCOL_ID_PACE;
        case PROTOCOL_RS485_JKBMS_NATIVE:
            return PROTOCOL_ID_JKBMS_NATIVE;
        case PROTOCOL_RS485_VOLTRONIC:
            return PROTOCOL_ID_VOLTRONIC;
        case PROTOCOL_RS485_CHINA_TOWER:
            return PROTOCOL_ID_CHINA_TOWER;
        case PROTOCOL_RS485_WOW:
            return PROTOCOL_ID_WOW;
        case PROTOCOL_RS485_SEPLOS:
        case PROTOCOL_RS485_SEPLOS_19200:
            return PROTOCOL_ID_SEPLOS;
        case PROTOCOL_RS485_DALY:
            return PROTOCOL_ID_DALY;
        default:
            return PROTOCOL_ID_GROWATT;
    }
}

static bool isJkbmsRs485Protocol(uint8_t protocol)
{
    return bridgeProtocolIsRs485JkbmsModbus(protocol) ||
           (protocol == PROTOCOL_RS485_JKBMS_NATIVE);
}

static esp_err_t startJkbmsRs485TaskForUiProtocol(uint8_t protocol, QueueHandle_t outQueue)
{
    if (protocol == PROTOCOL_RS485_JKBMS_NATIVE) {
        return jkbmsRs485BmsTaskStart(outQueue);
    }
    return jkbmsModbusBmsTaskStart(outQueue);
}

static void stopJkbmsRs485Tasks(void)
{
    (void)jkbmsModbusBmsTaskStop();
    (void)jkbmsRs485BmsTaskStop();
}

static const char *canNameByPort(uint8_t port)
{
    return (port == 2u) ? "CAN2" : "CAN1";
}

static twai_handle_t canBusByPort(uint8_t port)
{
    return (port == 2u) ? canGetBus1() : canGetBus0();
}

static const char *rsNameByPort(uint8_t port)
{
    return (port == 2u) ? "RS485_2" : "RS485_1";
}

static uart_port_t rsUartByPort(uint8_t port)
{
    return (port == 2u) ? rs485GetUart2() : rs485GetUart1();
}

static gpio_num_t rsDirByPort(uint8_t port)
{
    return (port == 2u) ? rs485GetDir2() : rs485GetDir1();
}

static bool isCanToRsGrowattRoute(const bridge_runtime_settings_t *settings)
{
    if (settings == NULL) {
        return false;
    }

    const bool canBmsSupported =
        (settings->bms_protocol == PROTOCOL_CAN_GROWATT) ||
        (settings->bms_protocol == PROTOCOL_CAN_PYLON) ||
        (settings->bms_protocol == PROTOCOL_CAN_GOODWE) ||
        (settings->bms_protocol == PROTOCOL_CAN_SOFAR) ||
        (settings->bms_protocol == PROTOCOL_CAN_SMA) ||
        (settings->bms_protocol == PROTOCOL_CAN_VICTRON);

    return (settings->bms_line == LINE_CAN) &&
           (settings->inverter_line == LINE_RS485) &&
           canBmsSupported &&
           (settings->inverter_protocol == PROTOCOL_RS485_GROWATT);
}

static bool isRsJkbmsToRsGrowattRoute(const bridge_runtime_settings_t *settings)
{
    if (settings == NULL) {
        return false;
    }

    return (settings->bms_line == LINE_RS485) &&
           (settings->inverter_line == LINE_RS485) &&
           isJkbmsRs485Protocol(settings->bms_protocol) &&
           (settings->inverter_protocol == PROTOCOL_RS485_GROWATT);
}

static bool buildPacketFromBatteryModel(bms_decoded_packet_t *out)
{
    battery_model_t model = {0};

    if (out == NULL) {
        return false;
    }

    batteryModelGet(&model);
    if (!model.valid || model.updatedMs == 0u) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->sourceProtocol = PROTOCOL_ID_JKBMS;
    out->timestampUs = esp_timer_get_time();

    out->hasSoc = true;
    out->socPct = model.socPct;

    if (model.temperaturesC[0] > -100.0f) {
        out->hasTemperatureC = true;
        out->temperatureC = (int16_t)model.temperaturesC[0];
    }

    if (model.packVoltageV > 0.0f) {
        uint32_t packCv = (uint32_t)(model.packVoltageV * 100.0f + 0.5f);
        out->hasPackVoltageCv = true;
        out->packVoltageCv = (uint16_t)((packCv > UINT16_MAX) ? UINT16_MAX : packCv);
    }

    if (model.cellMaxV > 0.0f && model.cellMinV > 0.0f) {
        uint32_t maxMv = (uint32_t)(model.cellMaxV * 1000.0f + 0.5f);
        uint32_t minMv = (uint32_t)(model.cellMinV * 1000.0f + 0.5f);
        out->hasCellExtremes = true;
        out->maxCellMv = (uint16_t)((maxMv > UINT16_MAX) ? UINT16_MAX : maxMv);
        out->minCellMv = (uint16_t)((minMv > UINT16_MAX) ? UINT16_MAX : minMv);
        out->maxCellIndex = model.cellMaxIdx;
        out->minCellIndex = model.cellMinIdx;
    }

    return out->hasSoc || out->hasTemperatureC || out->hasPackVoltageCv || out->hasCellExtremes;
}

static bool isRsJkbmsToRsPylonRoute(const bridge_runtime_settings_t *settings)
{
    if (settings == NULL) {
        return false;
    }

    return (settings->bms_line == LINE_RS485) &&
           (settings->inverter_line == LINE_RS485) &&
           isJkbmsRs485Protocol(settings->bms_protocol) &&
           bridgeProtocolIsRs485Pylon(settings->inverter_protocol);
}

static bool isRsGrowattToRsPylonRoute(const bridge_runtime_settings_t *settings)
{
    if (settings == NULL) {
        return false;
    }

    return (settings->bms_line == LINE_RS485) &&
           (settings->inverter_line == LINE_RS485) &&
           (settings->bms_protocol == PROTOCOL_RS485_GROWATT) &&
           bridgeProtocolIsRs485Pylon(settings->inverter_protocol);
}

static bool isRsPaceToRsPylonRoute(const bridge_runtime_settings_t *settings)
{
    if (settings == NULL) {
        return false;
    }

    return (settings->bms_line == LINE_RS485) &&
           (settings->inverter_line == LINE_RS485) &&
           (settings->bms_protocol == PROTOCOL_RS485_PACE) &&
           bridgeProtocolIsRs485Pylon(settings->inverter_protocol);
}

static bool isRsVoltronicToRsPylonRoute(const bridge_runtime_settings_t *settings)
{
    if (settings == NULL) {
        return false;
    }

    return (settings->bms_line == LINE_RS485) &&
           (settings->inverter_line == LINE_RS485) &&
           (settings->bms_protocol == PROTOCOL_RS485_VOLTRONIC) &&
           bridgeProtocolIsRs485Pylon(settings->inverter_protocol);
}

static bool isRsChinaTowerToRsPylonRoute(const bridge_runtime_settings_t *settings)
{
    if (settings == NULL) {
        return false;
    }

    return (settings->bms_line == LINE_RS485) &&
           (settings->inverter_line == LINE_RS485) &&
           (settings->bms_protocol == PROTOCOL_RS485_CHINA_TOWER) &&
           bridgeProtocolIsRs485Pylon(settings->inverter_protocol);
}

static bool isRsWowToRsPylonRoute(const bridge_runtime_settings_t *settings)
{
    if (settings == NULL) {
        return false;
    }

    return (settings->bms_line == LINE_RS485) &&
           (settings->inverter_line == LINE_RS485) &&
           (settings->bms_protocol == PROTOCOL_RS485_WOW) &&
           bridgeProtocolIsRs485Pylon(settings->inverter_protocol);
}

static bool isRsSeplosToRsPylonRoute(const bridge_runtime_settings_t *settings)
{
    if (settings == NULL) {
        return false;
    }

    return (settings->bms_line == LINE_RS485) &&
           (settings->inverter_line == LINE_RS485) &&
           ((settings->bms_protocol == PROTOCOL_RS485_SEPLOS) ||
            (settings->bms_protocol == PROTOCOL_RS485_SEPLOS_19200)) &&
           bridgeProtocolIsRs485Pylon(settings->inverter_protocol);
}

static bool isRsDalyToRsPylonRoute(const bridge_runtime_settings_t *settings)
{
    if (settings == NULL) {
        return false;
    }

    return (settings->bms_line == LINE_RS485) &&
           (settings->inverter_line == LINE_RS485) &&
           (settings->bms_protocol == PROTOCOL_RS485_DALY) &&
           bridgeProtocolIsRs485Pylon(settings->inverter_protocol);
}

static void clearTransportBuffers(void)
{
    uint8_t sink[64];
    uart_port_t uarts[2] = { rs485GetUart1(), rs485GetUart2() };
    for (size_t i = 0; i < 2; i++) {
        while (1) {
            int got = uart_read_bytes(uarts[i], sink, sizeof(sink), 0);
            if (got <= 0) {
                break;
            }
        }
    }

    twai_handle_t can1 = canGetBus0();
    twai_handle_t can2 = canGetBus1();
    if (can1 != NULL) {
        (void)twai_clear_receive_queue_v2(can1);
    }
    if (can2 != NULL) {
        (void)twai_clear_receive_queue_v2(can2);
    }
}

static void orchestratorTask(void *pv)
{
    orchestratorCtx_t *ctx = (orchestratorCtx_t *)pv;

    while (1) {
        bms_decoded_packet_t packet = {0};
        if (xQueueReceive(ctx->bmsQueue, &packet, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (batteryModelIsDebugOverrideEnabled()) {
            bms_decoded_packet_t fakePacket = {0};
            if (buildPacketFromBatteryModel(&fakePacket)) {
                packet = fakePacket;
            }
        }

        const int64_t nowUs = esp_timer_get_time();
        const bool changed = !ctx->haveLastForwarded || !packetEquivalent(&ctx->lastForwarded, &packet);
        const bool forceRefresh = (ctx->lastForwardUs == 0) ||
                                  ((nowUs - ctx->lastForwardUs) >=
                                   ((int64_t)ORCHESTRATOR_FORCE_FORWARD_MS * 1000LL));
        if (!changed && !forceRefresh) {
            continue;
        }

        if (xQueueOverwrite(ctx->inverterQueue, &packet) != pdPASS) {
            ESP_LOGW(EXAMPLE_TAG, "Orchestrator queue overwrite failed");
            continue;
        }

        ctx->lastForwarded = packet;
        ctx->haveLastForwarded = true;
        ctx->lastForwardUs = nowUs;
    }
}

static void orchestratorReset(orchestratorCtx_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    if (ctx->bmsQueue != NULL) {
        vQueueDelete(ctx->bmsQueue);
        ctx->bmsQueue = NULL;
    }
    if (ctx->inverterQueue != NULL) {
        vQueueDelete(ctx->inverterQueue);
        ctx->inverterQueue = NULL;
    }
}

esp_err_t orchestratorStart(protocol_id_t bmsProtocol, protocol_id_t inverterProtocol)
{
    if (g_orchestratorTaskHandle != NULL || g_orchestratorCtx.canRs485TranslatorActive) {
        ESP_LOGW(EXAMPLE_TAG, "Orchestrator already running");
        return ESP_ERR_INVALID_STATE;
    }

    memset(&g_orchestratorCtx, 0, sizeof(g_orchestratorCtx));
    g_orchestratorCtx.bmsProtocol = bmsProtocol;
    g_orchestratorCtx.inverterProtocol = inverterProtocol;

    g_orchestratorCtx.bmsQueue =
        xQueueCreate(ORCHESTRATOR_BMS_QUEUE_LEN, sizeof(bms_decoded_packet_t));
    g_orchestratorCtx.inverterQueue =
        xQueueCreate(ORCHESTRATOR_INVERTER_QUEUE_LEN, sizeof(bms_decoded_packet_t));
    if (g_orchestratorCtx.bmsQueue == NULL || g_orchestratorCtx.inverterQueue == NULL) {
        orchestratorReset(&g_orchestratorCtx);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = startBmsTask(g_orchestratorCtx.bmsProtocol, g_orchestratorCtx.bmsQueue);
    if (err != ESP_OK) {
        ESP_LOGE(EXAMPLE_TAG,
                 "Failed to start BMS protocol task (%s), err=0x%x",
                 protocolIdToStr(g_orchestratorCtx.bmsProtocol),
                 (unsigned)err);
        orchestratorReset(&g_orchestratorCtx);
        return err;
    }

    err = startInverterTask(g_orchestratorCtx.inverterProtocol, g_orchestratorCtx.inverterQueue);
    if (err != ESP_OK) {
        ESP_LOGE(EXAMPLE_TAG,
                 "Failed to start inverter protocol task (%s), err=0x%x",
                 protocolIdToStr(g_orchestratorCtx.inverterProtocol),
                 (unsigned)err);
        ESP_LOGE(EXAMPLE_TAG,
                 "BMS task may already be running; keeping queues alive to avoid invalid handles");
        return err;
    }

    BaseType_t taskOk =
        xTaskCreate(orchestratorTask,
                    "orchestrator",
                    ORCHESTRATOR_TASK_STACK,
                    &g_orchestratorCtx,
                    ORCHESTRATOR_TASK_PRIORITY,
                    &g_orchestratorTaskHandle);
    if (taskOk != pdPASS) {
        g_orchestratorTaskHandle = NULL;
        ESP_LOGE(EXAMPLE_TAG,
                 "Failed to create orchestrator task; protocol tasks remain running");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "Orchestrator started: BMS=%s -> Inverter=%s",
             protocolIdToStr(g_orchestratorCtx.bmsProtocol),
             protocolIdToStr(g_orchestratorCtx.inverterProtocol));
    return ESP_OK;
}

esp_err_t orchestratorStartFromRuntime(const bridge_runtime_settings_t *settings)
{
    if (settings == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const bool canToRsGrowatt = isCanToRsGrowattRoute(settings);
    const bool rsJkbmsToRsGrowatt = isRsJkbmsToRsGrowattRoute(settings);
    const bool rsJkbmsToRsPylon = isRsJkbmsToRsPylonRoute(settings);
    const bool rsGrowattToRsPylon = isRsGrowattToRsPylonRoute(settings);
    const bool rsPaceToRsPylon = isRsPaceToRsPylonRoute(settings);
    const bool rsVoltronicToRsPylon = isRsVoltronicToRsPylonRoute(settings);
    const bool rsChinaTowerToRsPylon = isRsChinaTowerToRsPylonRoute(settings);
    const bool rsWowToRsPylon = isRsWowToRsPylonRoute(settings);
    const bool rsSeplosToRsPylon = isRsSeplosToRsPylonRoute(settings);
    const bool rsDalyToRsPylon = isRsDalyToRsPylonRoute(settings);
    const bool pylonRs485Route = pylonRs485BridgeSupportsRoute(settings);
    ESP_LOGI(EXAMPLE_TAG,
             "Orchestrator runtime start: bms(line=%u prot=%u port=%u) inv(line=%u prot=%u port=%u) canToRsGrowatt=%s rsJkbmsToRsGrowatt=%s rsJkbmsToRsPylon=%s rsGrowattToRsPylon=%s rsPaceToRsPylon=%s rsVoltronicToRsPylon=%s rsChinaTowerToRsPylon=%s rsWowToRsPylon=%s rsSeplosToRsPylon=%s rsDalyToRsPylon=%s pylonRs485=%s",
             (unsigned)settings->bms_line,
             (unsigned)settings->bms_protocol,
             (unsigned)settings->bms_port,
             (unsigned)settings->inverter_line,
             (unsigned)settings->inverter_protocol,
             (unsigned)settings->inverter_port,
             canToRsGrowatt ? "YES" : "NO",
             rsJkbmsToRsGrowatt ? "YES" : "NO",
             rsJkbmsToRsPylon ? "YES" : "NO",
             rsGrowattToRsPylon ? "YES" : "NO",
             rsPaceToRsPylon ? "YES" : "NO",
             rsVoltronicToRsPylon ? "YES" : "NO",
             rsChinaTowerToRsPylon ? "YES" : "NO",
             rsWowToRsPylon ? "YES" : "NO",
             rsSeplosToRsPylon ? "YES" : "NO",
             rsDalyToRsPylon ? "YES" : "NO",
             pylonRs485Route ? "YES" : "NO");

    if (canToRsGrowatt) {
        if (g_orchestratorTaskHandle != NULL || g_orchestratorCtx.canRs485TranslatorActive) {
            ESP_LOGW(EXAMPLE_TAG, "Orchestrator already running");
            return ESP_ERR_INVALID_STATE;
        }
#if !CAN_RS485_SOC_TRANSLATOR_ENABLE
        ESP_LOGW(EXAMPLE_TAG, "CAN->RS485 Growatt route requested, but translator is disabled");
        return ESP_ERR_NOT_SUPPORTED;
#else
        esp_err_t startErr = canRs485GrowattBridgeEnable(rsUartByPort(settings->inverter_port),
                                                         rsDirByPort(settings->inverter_port),
                                                         rsNameByPort(settings->inverter_port),
                                                         canBusByPort(settings->bms_port),
                                                         canNameByPort(settings->bms_port));
        if (startErr != ESP_OK) {
            ESP_LOGW(EXAMPLE_TAG,
                     "CAN->RS485 Growatt route failed to start translator (err=0x%x)",
                     (unsigned)startErr);
            return startErr;
        }
        g_orchestratorCtx.canRs485TranslatorActive = true;
        g_orchestratorCtx.bmsProtocol = protocolIdFromUiProtocol(settings->bms_protocol);
        g_orchestratorCtx.inverterProtocol = PROTOCOL_ID_GROWATT;

        ESP_LOGI(EXAMPLE_TAG,
                 "Orchestrator started CAN->RS485 Growatt route: CAN(%s:%u) -> RS485(%s:%u)",
                 canNameByPort(settings->bms_port),
                 (unsigned)settings->bms_port,
                 rsNameByPort(settings->inverter_port),
                 (unsigned)settings->inverter_port);
        return ESP_OK;
#endif
    }

    if (rsJkbmsToRsGrowatt) {
        if (g_orchestratorTaskHandle != NULL || g_orchestratorCtx.canRs485TranslatorActive) {
            ESP_LOGW(EXAMPLE_TAG, "Orchestrator already running");
            return ESP_ERR_INVALID_STATE;
        }

        memset(&g_orchestratorCtx, 0, sizeof(g_orchestratorCtx));
        g_orchestratorCtx.bmsProtocol = protocolIdFromUiProtocol(settings->bms_protocol);
        g_orchestratorCtx.inverterProtocol = PROTOCOL_ID_GROWATT;

        g_orchestratorCtx.bmsQueue =
            xQueueCreate(ORCHESTRATOR_BMS_QUEUE_LEN, sizeof(bms_decoded_packet_t));
        if (g_orchestratorCtx.bmsQueue == NULL) {
            orchestratorReset(&g_orchestratorCtx);
            return ESP_ERR_NO_MEM;
        }

        esp_err_t err = startJkbmsRs485TaskForUiProtocol(settings->bms_protocol,
                                                         g_orchestratorCtx.bmsQueue);
        if (err != ESP_OK) {
            ESP_LOGW(EXAMPLE_TAG,
                     "JKBMS RS485 BMS task failed for RS485->RS485 Growatt route (err=0x%x)",
                     (unsigned)err);
            orchestratorReset(&g_orchestratorCtx);
            return err;
        }

        err = jkbmsRs485GrowattBridgeEnable(rsUartByPort(settings->inverter_port),
                                            rsDirByPort(settings->inverter_port),
                                            rsNameByPort(settings->inverter_port));
        if (err != ESP_OK) {
            ESP_LOGW(EXAMPLE_TAG,
                     "JKBMS->RS485 Growatt route failed to start translator (err=0x%x)",
                     (unsigned)err);
            stopJkbmsRs485Tasks();
            orchestratorReset(&g_orchestratorCtx);
            return err;
        }

        g_orchestratorCtx.canRs485TranslatorActive = true;
        ESP_LOGI(EXAMPLE_TAG,
                 "Orchestrator started JKBMS RS485->RS485 Growatt route: BMS(RS485_%u) -> Inverter(%s:%u)",
                 (unsigned)settings->bms_port,
                 rsNameByPort(settings->inverter_port),
                 (unsigned)settings->inverter_port);
        return ESP_OK;
    }

    if (rsJkbmsToRsPylon) {
        if (g_orchestratorTaskHandle != NULL || g_orchestratorCtx.canRs485TranslatorActive) {
            ESP_LOGW(EXAMPLE_TAG, "Orchestrator already running");
            return ESP_ERR_INVALID_STATE;
        }

        memset(&g_orchestratorCtx, 0, sizeof(g_orchestratorCtx));
        g_orchestratorCtx.bmsProtocol = protocolIdFromUiProtocol(settings->bms_protocol);
        g_orchestratorCtx.inverterProtocol = PROTOCOL_ID_PYLON;

        g_orchestratorCtx.bmsQueue =
            xQueueCreate(ORCHESTRATOR_BMS_QUEUE_LEN, sizeof(bms_decoded_packet_t));
        if (g_orchestratorCtx.bmsQueue == NULL) {
            orchestratorReset(&g_orchestratorCtx);
            return ESP_ERR_NO_MEM;
        }

        esp_err_t err = startJkbmsRs485TaskForUiProtocol(settings->bms_protocol,
                                                         g_orchestratorCtx.bmsQueue);
        if (err != ESP_OK) {
            ESP_LOGW(EXAMPLE_TAG,
                     "JKBMS RS485 BMS task failed for RS485->RS485 Pylon route (err=0x%x)",
                     (unsigned)err);
            orchestratorReset(&g_orchestratorCtx);
            return err;
        }

        pylonRs485BridgeEnable();
        g_orchestratorCtx.canRs485TranslatorActive = true;
        ESP_LOGI(EXAMPLE_TAG,
                 "Orchestrator started JKBMS RS485->RS485 Pylon route: BMS(RS485_%u) -> Inverter(%s:%u)",
                 (unsigned)settings->bms_port,
                 rsNameByPort(settings->inverter_port),
                 (unsigned)settings->inverter_port);
        return ESP_OK;
    }

    if (rsGrowattToRsPylon) {
        if (g_orchestratorTaskHandle != NULL || g_orchestratorCtx.canRs485TranslatorActive) {
            ESP_LOGW(EXAMPLE_TAG, "Orchestrator already running");
            return ESP_ERR_INVALID_STATE;
        }

        memset(&g_orchestratorCtx, 0, sizeof(g_orchestratorCtx));
        g_orchestratorCtx.bmsProtocol = PROTOCOL_ID_GROWATT;
        g_orchestratorCtx.inverterProtocol = PROTOCOL_ID_PYLON;

        g_orchestratorCtx.bmsQueue =
            xQueueCreate(ORCHESTRATOR_BMS_QUEUE_LEN, sizeof(bms_decoded_packet_t));
        if (g_orchestratorCtx.bmsQueue == NULL) {
            orchestratorReset(&g_orchestratorCtx);
            return ESP_ERR_NO_MEM;
        }

        esp_err_t err = growattBmsTaskStart(g_orchestratorCtx.bmsQueue);
        if (err != ESP_OK) {
            ESP_LOGW(EXAMPLE_TAG,
                     "Growatt RS485 BMS task failed for RS485->RS485 Pylon route (err=0x%x)",
                     (unsigned)err);
            orchestratorReset(&g_orchestratorCtx);
            return err;
        }

        pylonRs485BridgeEnable();
        g_orchestratorCtx.canRs485TranslatorActive = true;
        ESP_LOGI(EXAMPLE_TAG,
                 "Orchestrator started Growatt RS485->RS485 Pylon route: BMS(RS485_%u) -> Inverter(%s:%u)",
                 (unsigned)settings->bms_port,
                 rsNameByPort(settings->inverter_port),
                 (unsigned)settings->inverter_port);
        return ESP_OK;
    }

    if (rsPaceToRsPylon) {
        if (g_orchestratorTaskHandle != NULL || g_orchestratorCtx.canRs485TranslatorActive) {
            ESP_LOGW(EXAMPLE_TAG, "Orchestrator already running");
            return ESP_ERR_INVALID_STATE;
        }

        memset(&g_orchestratorCtx, 0, sizeof(g_orchestratorCtx));
        g_orchestratorCtx.bmsProtocol = PROTOCOL_ID_PACE;
        g_orchestratorCtx.inverterProtocol = PROTOCOL_ID_PYLON;

        g_orchestratorCtx.bmsQueue =
            xQueueCreate(ORCHESTRATOR_BMS_QUEUE_LEN, sizeof(bms_decoded_packet_t));
        if (g_orchestratorCtx.bmsQueue == NULL) {
            orchestratorReset(&g_orchestratorCtx);
            return ESP_ERR_NO_MEM;
        }

        esp_err_t err = paceModbusBmsTaskStart(g_orchestratorCtx.bmsQueue);
        if (err != ESP_OK) {
            ESP_LOGW(EXAMPLE_TAG,
                     "PACE BMS task failed for RS485->RS485 Pylon route (err=0x%x)",
                     (unsigned)err);
            orchestratorReset(&g_orchestratorCtx);
            return err;
        }

        pylonRs485BridgeEnable();
        g_orchestratorCtx.canRs485TranslatorActive = true;
        ESP_LOGI(EXAMPLE_TAG,
                 "Orchestrator started PACE RS485->RS485 Pylon route: BMS(RS485_%u) -> Inverter(%s:%u)",
                 (unsigned)settings->bms_port,
                 rsNameByPort(settings->inverter_port),
                 (unsigned)settings->inverter_port);
        return ESP_OK;
    }

    if (rsVoltronicToRsPylon) {
        if (g_orchestratorTaskHandle != NULL || g_orchestratorCtx.canRs485TranslatorActive) {
            ESP_LOGW(EXAMPLE_TAG, "Orchestrator already running");
            return ESP_ERR_INVALID_STATE;
        }

        memset(&g_orchestratorCtx, 0, sizeof(g_orchestratorCtx));
        g_orchestratorCtx.bmsProtocol = PROTOCOL_ID_VOLTRONIC;
        g_orchestratorCtx.inverterProtocol = PROTOCOL_ID_PYLON;

        g_orchestratorCtx.bmsQueue =
            xQueueCreate(ORCHESTRATOR_BMS_QUEUE_LEN, sizeof(bms_decoded_packet_t));
        if (g_orchestratorCtx.bmsQueue == NULL) {
            orchestratorReset(&g_orchestratorCtx);
            return ESP_ERR_NO_MEM;
        }

        esp_err_t err = voltronicModbusBmsTaskStart(g_orchestratorCtx.bmsQueue);
        if (err != ESP_OK) {
            ESP_LOGW(EXAMPLE_TAG,
                     "Voltronic BMS task failed for RS485->RS485 Pylon route (err=0x%x)",
                     (unsigned)err);
            orchestratorReset(&g_orchestratorCtx);
            return err;
        }

        pylonRs485BridgeEnable();
        g_orchestratorCtx.canRs485TranslatorActive = true;
        ESP_LOGI(EXAMPLE_TAG,
                 "Orchestrator started Voltronic RS485->RS485 Pylon route: BMS(RS485_%u) -> Inverter(%s:%u)",
                 (unsigned)settings->bms_port,
                 rsNameByPort(settings->inverter_port),
                 (unsigned)settings->inverter_port);
        return ESP_OK;
    }

    if (rsChinaTowerToRsPylon) {
        if (g_orchestratorTaskHandle != NULL || g_orchestratorCtx.canRs485TranslatorActive) {
            ESP_LOGW(EXAMPLE_TAG, "Orchestrator already running");
            return ESP_ERR_INVALID_STATE;
        }

        memset(&g_orchestratorCtx, 0, sizeof(g_orchestratorCtx));
        g_orchestratorCtx.bmsProtocol = PROTOCOL_ID_CHINA_TOWER;
        g_orchestratorCtx.inverterProtocol = PROTOCOL_ID_PYLON;

        g_orchestratorCtx.bmsQueue =
            xQueueCreate(ORCHESTRATOR_BMS_QUEUE_LEN, sizeof(bms_decoded_packet_t));
        if (g_orchestratorCtx.bmsQueue == NULL) {
            orchestratorReset(&g_orchestratorCtx);
            return ESP_ERR_NO_MEM;
        }

        esp_err_t err = chinaTowerModbusBmsTaskStart(g_orchestratorCtx.bmsQueue);
        if (err != ESP_OK) {
            ESP_LOGW(EXAMPLE_TAG,
                     "China Tower BMS task failed for RS485->RS485 Pylon route (err=0x%x)",
                     (unsigned)err);
            orchestratorReset(&g_orchestratorCtx);
            return err;
        }

        pylonRs485BridgeEnable();
        g_orchestratorCtx.canRs485TranslatorActive = true;
        ESP_LOGI(EXAMPLE_TAG,
                 "Orchestrator started China Tower RS485->RS485 Pylon route: BMS(RS485_%u) -> Inverter(%s:%u)",
                 (unsigned)settings->bms_port,
                 rsNameByPort(settings->inverter_port),
                 (unsigned)settings->inverter_port);
        return ESP_OK;
    }

    if (rsWowToRsPylon) {
        if (g_orchestratorTaskHandle != NULL || g_orchestratorCtx.canRs485TranslatorActive) {
            ESP_LOGW(EXAMPLE_TAG, "Orchestrator already running");
            return ESP_ERR_INVALID_STATE;
        }

        memset(&g_orchestratorCtx, 0, sizeof(g_orchestratorCtx));
        g_orchestratorCtx.bmsProtocol = PROTOCOL_ID_WOW;
        g_orchestratorCtx.inverterProtocol = PROTOCOL_ID_PYLON;

        g_orchestratorCtx.bmsQueue =
            xQueueCreate(ORCHESTRATOR_BMS_QUEUE_LEN, sizeof(bms_decoded_packet_t));
        if (g_orchestratorCtx.bmsQueue == NULL) {
            orchestratorReset(&g_orchestratorCtx);
            return ESP_ERR_NO_MEM;
        }

        esp_err_t err = wowModbusBmsTaskStart(g_orchestratorCtx.bmsQueue);
        if (err != ESP_OK) {
            ESP_LOGW(EXAMPLE_TAG,
                     "WOW BMS task failed for RS485->RS485 Pylon route (err=0x%x)",
                     (unsigned)err);
            orchestratorReset(&g_orchestratorCtx);
            return err;
        }

        pylonRs485BridgeEnable();
        g_orchestratorCtx.canRs485TranslatorActive = true;
        ESP_LOGI(EXAMPLE_TAG,
                 "Orchestrator started WOW RS485->RS485 Pylon route: BMS(RS485_%u) -> Inverter(%s:%u)",
                 (unsigned)settings->bms_port,
                 rsNameByPort(settings->inverter_port),
                 (unsigned)settings->inverter_port);
        return ESP_OK;
    }

    if (rsSeplosToRsPylon) {
        if (g_orchestratorTaskHandle != NULL || g_orchestratorCtx.canRs485TranslatorActive) {
            ESP_LOGW(EXAMPLE_TAG, "Orchestrator already running");
            return ESP_ERR_INVALID_STATE;
        }

        memset(&g_orchestratorCtx, 0, sizeof(g_orchestratorCtx));
        g_orchestratorCtx.bmsProtocol = PROTOCOL_ID_SEPLOS;
        g_orchestratorCtx.inverterProtocol = PROTOCOL_ID_PYLON;

        g_orchestratorCtx.bmsQueue =
            xQueueCreate(ORCHESTRATOR_BMS_QUEUE_LEN, sizeof(bms_decoded_packet_t));
        if (g_orchestratorCtx.bmsQueue == NULL) {
            orchestratorReset(&g_orchestratorCtx);
            return ESP_ERR_NO_MEM;
        }

        esp_err_t err = seplosRs485BmsTaskStart(g_orchestratorCtx.bmsQueue);
        if (err != ESP_OK) {
            ESP_LOGW(EXAMPLE_TAG,
                     "Seplos RS485 BMS task failed for RS485->RS485 Pylon route (err=0x%x)",
                     (unsigned)err);
            orchestratorReset(&g_orchestratorCtx);
            return err;
        }

        pylonRs485BridgeEnable();
        g_orchestratorCtx.canRs485TranslatorActive = true;
        ESP_LOGI(EXAMPLE_TAG,
                 "Orchestrator started Seplos RS485->RS485 Pylon route: BMS(RS485_%u) -> Inverter(%s:%u)",
                 (unsigned)settings->bms_port,
                 rsNameByPort(settings->inverter_port),
                 (unsigned)settings->inverter_port);
        return ESP_OK;
    }

    if (rsDalyToRsPylon) {
        if (g_orchestratorTaskHandle != NULL || g_orchestratorCtx.canRs485TranslatorActive) {
            ESP_LOGW(EXAMPLE_TAG, "Orchestrator already running");
            return ESP_ERR_INVALID_STATE;
        }

        memset(&g_orchestratorCtx, 0, sizeof(g_orchestratorCtx));
        g_orchestratorCtx.bmsProtocol = PROTOCOL_ID_DALY;
        g_orchestratorCtx.inverterProtocol = PROTOCOL_ID_PYLON;

        g_orchestratorCtx.bmsQueue =
            xQueueCreate(ORCHESTRATOR_BMS_QUEUE_LEN, sizeof(bms_decoded_packet_t));
        if (g_orchestratorCtx.bmsQueue == NULL) {
            orchestratorReset(&g_orchestratorCtx);
            return ESP_ERR_NO_MEM;
        }

        esp_err_t err = dalyRs485BmsTaskStart(g_orchestratorCtx.bmsQueue);
        if (err != ESP_OK) {
            ESP_LOGW(EXAMPLE_TAG,
                     "Daly RS485 BMS task failed for RS485->RS485 Pylon route (err=0x%x)",
                     (unsigned)err);
            orchestratorReset(&g_orchestratorCtx);
            return err;
        }

        pylonRs485BridgeEnable();
        g_orchestratorCtx.canRs485TranslatorActive = true;
        ESP_LOGI(EXAMPLE_TAG,
                 "Orchestrator started Daly RS485->RS485 Pylon route: BMS(RS485_%u) -> Inverter(%s:%u)",
                 (unsigned)settings->bms_port,
                 rsNameByPort(settings->inverter_port),
                 (unsigned)settings->inverter_port);
        return ESP_OK;
    }

    if (pylonRs485Route) {
        if (g_orchestratorTaskHandle != NULL || g_orchestratorCtx.canRs485TranslatorActive) {
            ESP_LOGW(EXAMPLE_TAG, "Orchestrator already running");
            return ESP_ERR_INVALID_STATE;
        }

        canDecoderResetCaches();
        pylonRs485BridgeEnable();
        canForwardSnifferStart(settings);
        g_orchestratorCtx.canRs485TranslatorActive = true;
        g_orchestratorCtx.bmsProtocol = protocolIdFromUiProtocol(settings->bms_protocol);
        g_orchestratorCtx.inverterProtocol = protocolIdFromUiProtocol(settings->inverter_protocol);
        ESP_LOGI(EXAMPLE_TAG,
                 "Orchestrator started Pylon RS485 bridge route: BMS(line=%u prot=%u port=%u) -> Inverter(line=%u prot=%u port=%u)",
                 (unsigned)settings->bms_line,
                 (unsigned)settings->bms_protocol,
                 (unsigned)settings->bms_port,
                 (unsigned)settings->inverter_line,
                 (unsigned)settings->inverter_protocol,
                 (unsigned)settings->inverter_port);
        return ESP_OK;
    }

    return orchestratorStart(protocolIdFromUiProtocol(settings->bms_protocol),
                             protocolIdFromUiProtocol(settings->inverter_protocol));
}

esp_err_t orchestratorStop(void)
{
    if (g_orchestratorTaskHandle != NULL) {
        vTaskDelete(g_orchestratorTaskHandle);
        g_orchestratorTaskHandle = NULL;
    }

    canRs485GrowattBridgeStop();
    pylonRs485BridgeStop();
    canForwardSnifferStop();
    canDecoderResetCaches();
    g_orchestratorCtx.canRs485TranslatorActive = false;

    (void)growattBmsTaskStop();
    (void)growattInverterTaskStop();
    (void)jkbmsModbusBmsTaskStop();
    (void)jkbmsRs485BmsTaskStop();
    (void)paceModbusBmsTaskStop();
    (void)voltronicModbusBmsTaskStop();
    (void)chinaTowerModbusBmsTaskStop();
    (void)wowModbusBmsTaskStop();
    (void)seplosRs485BmsTaskStop();
    (void)dalyRs485BmsTaskStop();
    (void)pylonBmsTaskStop();
    (void)pylonInverterTaskStop();
    clearTransportBuffers();

    orchestratorReset(&g_orchestratorCtx);
    memset(&g_orchestratorCtx, 0, sizeof(g_orchestratorCtx));
    return ESP_OK;
}
