#include "orchestrator/orchestrator.h"

#include <string.h>

#include "Drivers/can_driver.h"
#include "Drivers/rs485_driver.h"
#include "config.h"
#include "protocols/growatt/growatt_bms_task.h"
#include "protocols/growatt/growatt_inverter_task.h"
#include "protocols/jkbms_modbus/jkbms_modbus_bms_task.h"
#include "protocols/pylon/pylon_bms_task.h"
#include "protocols/pylon/pylon_inverter_task.h"
#include "BMS_Protocols/Pylon/pylon_can_protocol.h"
#include "BMS_Protocols/Pylon/pylon_rs485_bridge.h"
#include "rs485_can_bridge.h"

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
static TaskHandle_t g_pylonCanSniffTask;

const char *protocolIdToStr(protocol_id_t id)
{
    switch (id) {
        case PROTOCOL_ID_GROWATT:
            return "GROWATT";
        case PROTOCOL_ID_PYLON:
            return "PYLON";
        case PROTOCOL_ID_JKBMS:
            return "JKBMS_MODBUS";
        default:
            return "UNKNOWN";
    }
}

static bool packetEquivalent(const bms_decoded_packet_t *a, const bms_decoded_packet_t *b)
{
    if (a == NULL || b == NULL) {
        return false;
    }

    return (a->hasSoc == b->hasSoc) &&
           (!a->hasSoc || (a->socPct == b->socPct)) &&
           (a->hasTemperatureC == b->hasTemperatureC) &&
           (!a->hasTemperatureC || (a->temperatureC == b->temperatureC)) &&
           (a->hasPackVoltageCv == b->hasPackVoltageCv) &&
           (!a->hasPackVoltageCv || (a->packVoltageCv == b->packVoltageCv)) &&
           (a->hasCellExtremes == b->hasCellExtremes) &&
           (!a->hasCellExtremes ||
            ((a->minCellMv == b->minCellMv) &&
             (a->maxCellMv == b->maxCellMv) &&
             (a->minCellIndex == b->minCellIndex) &&
             (a->maxCellIndex == b->maxCellIndex)));
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
            return PROTOCOL_ID_GROWATT;
        case PROTOCOL_CAN_PYLON:
        case PROTOCOL_RS485_PYLON:
        case PROTOCOL_CAN_VICTRON:
            return PROTOCOL_ID_PYLON;
        case PROTOCOL_RS485_JKBMS:
            return PROTOCOL_ID_JKBMS;
        default:
            return PROTOCOL_ID_GROWATT;
    }
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
        (settings->bms_protocol == PROTOCOL_CAN_PYLON);

    return (settings->bms_line == LINE_CAN) &&
           (settings->inverter_line == LINE_RS485) &&
           canBmsSupported &&
           (settings->inverter_protocol == PROTOCOL_RS485_GROWATT);
}

static bool isCanToRsPylonRoute(const bridge_runtime_settings_t *settings)
{
    if (settings == NULL) {
        return false;
    }

    const bool canBmsSupported =
        (settings->bms_protocol == PROTOCOL_CAN_PYLON) ||
        (settings->bms_protocol == PROTOCOL_CAN_VICTRON);

    return (settings->bms_line == LINE_CAN) &&
           (settings->inverter_line == LINE_RS485) &&
           canBmsSupported &&
           (settings->inverter_protocol == PROTOCOL_RS485_PYLON);
}

static bool isRsJkbmsToRsGrowattRoute(const bridge_runtime_settings_t *settings)
{
    if (settings == NULL) {
        return false;
    }

    return (settings->bms_line == LINE_RS485) &&
           (settings->inverter_line == LINE_RS485) &&
           (settings->bms_protocol == PROTOCOL_RS485_JKBMS) &&
           (settings->inverter_protocol == PROTOCOL_RS485_GROWATT);
}

static void pylonCanSnifferTask(void *pv)
{
    const char *ifname = (const char *)pv;
    twai_handle_t bus = (strcmp(ifname, "CAN2") == 0) ? canGetBus1() : canGetBus0();
    twai_message_t rx = {0};

    pylonCanResetCaches();

    while (1) {
        if (twai_receive_v2(bus, &rx, portMAX_DELAY) != ESP_OK) {
            continue;
        }
#ifdef TWAI_MSG_FLAG_SELF
        if (rx.flags & TWAI_MSG_FLAG_SELF) {
            continue;
        }
#endif
        pylonCanOnFrame(ifname, &rx);
    }
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
    const bool canToRsPylon = isCanToRsPylonRoute(settings);
    const bool rsJkbmsToRsGrowatt = isRsJkbmsToRsGrowattRoute(settings);
    ESP_LOGI(EXAMPLE_TAG,
             "Orchestrator runtime start: bms(line=%u prot=%u port=%u) inv(line=%u prot=%u port=%u) canToRsGrowatt=%s canToRsPylon=%s rsJkbmsToRsGrowatt=%s",
             (unsigned)settings->bms_line,
             (unsigned)settings->bms_protocol,
             (unsigned)settings->bms_port,
             (unsigned)settings->inverter_line,
             (unsigned)settings->inverter_protocol,
             (unsigned)settings->inverter_port,
             canToRsGrowatt ? "YES" : "NO",
             canToRsPylon ? "YES" : "NO",
             rsJkbmsToRsGrowatt ? "YES" : "NO");

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

    if (canToRsPylon) {
        if (g_orchestratorTaskHandle != NULL || g_orchestratorCtx.canRs485TranslatorActive) {
            ESP_LOGW(EXAMPLE_TAG, "Orchestrator already running");
            return ESP_ERR_INVALID_STATE;
        }

        if (!pylonRs485BridgeHandlesCurrentConfig()) {
            ESP_LOGW(EXAMPLE_TAG, "CAN->RS485 Pylon route requested but translator rejected current config");
            return ESP_ERR_NOT_SUPPORTED;
        }

        pylonRs485BridgeEnable();
        g_orchestratorCtx.canRs485TranslatorActive = true;
        g_orchestratorCtx.bmsProtocol = protocolIdFromUiProtocol(settings->bms_protocol);
        g_orchestratorCtx.inverterProtocol = PROTOCOL_ID_PYLON;
        if (g_pylonCanSniffTask == NULL) {
            const char *ifname = canNameByPort(settings->bms_port);
            (void)xTaskCreate(pylonCanSnifferTask,
                              "pylon_can_sniff",
                              4096,
                              (void *)ifname,
                              9,
                              &g_pylonCanSniffTask);
        }
        ESP_LOGI(EXAMPLE_TAG,
                 "Orchestrator started CAN->RS485 Pylon route: CAN(%s:%u prot=%u) -> RS485(%s:%u)",
                 canNameByPort(settings->bms_port),
                 (unsigned)settings->bms_port,
                 (unsigned)settings->bms_protocol,
                 rsNameByPort(settings->inverter_port),
                 (unsigned)settings->inverter_port);
        return ESP_OK;
    }

    if (rsJkbmsToRsGrowatt) {
        if (g_orchestratorTaskHandle != NULL || g_orchestratorCtx.canRs485TranslatorActive) {
            ESP_LOGW(EXAMPLE_TAG, "Orchestrator already running");
            return ESP_ERR_INVALID_STATE;
        }

        memset(&g_orchestratorCtx, 0, sizeof(g_orchestratorCtx));
        g_orchestratorCtx.bmsProtocol = PROTOCOL_ID_JKBMS;
        g_orchestratorCtx.inverterProtocol = PROTOCOL_ID_GROWATT;

        g_orchestratorCtx.bmsQueue =
            xQueueCreate(ORCHESTRATOR_BMS_QUEUE_LEN, sizeof(bms_decoded_packet_t));
        if (g_orchestratorCtx.bmsQueue == NULL) {
            orchestratorReset(&g_orchestratorCtx);
            return ESP_ERR_NO_MEM;
        }

        esp_err_t err = jkbmsModbusBmsTaskStart(g_orchestratorCtx.bmsQueue);
        if (err != ESP_OK) {
            ESP_LOGW(EXAMPLE_TAG,
                     "JKBMS BMS task failed for RS485->RS485 Growatt route (err=0x%x)",
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
            (void)jkbmsModbusBmsTaskStop();
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

    return orchestratorStart(protocolIdFromUiProtocol(settings->bms_protocol),
                             protocolIdFromUiProtocol(settings->inverter_protocol));
}

esp_err_t orchestratorStop(void)
{
    if (g_orchestratorTaskHandle != NULL) {
        vTaskDelete(g_orchestratorTaskHandle);
        g_orchestratorTaskHandle = NULL;
    }
    if (g_pylonCanSniffTask != NULL) {
        vTaskDelete(g_pylonCanSniffTask);
        g_pylonCanSniffTask = NULL;
    }

    canRs485GrowattBridgeStop();
    pylonRs485BridgeStop();
    g_orchestratorCtx.canRs485TranslatorActive = false;

    (void)growattBmsTaskStop();
    (void)growattInverterTaskStop();
    (void)jkbmsModbusBmsTaskStop();
    (void)pylonBmsTaskStop();
    (void)pylonInverterTaskStop();
    clearTransportBuffers();

    orchestratorReset(&g_orchestratorCtx);
    memset(&g_orchestratorCtx, 0, sizeof(g_orchestratorCtx));
    return ESP_OK;
}
