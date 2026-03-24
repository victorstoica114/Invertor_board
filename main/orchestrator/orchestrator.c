#include "orchestrator/orchestrator.h"

#include <string.h>

#include "config.h"
#include "protocols/growatt/growatt_bms_task.h"
#include "protocols/growatt/growatt_inverter_task.h"
#include "protocols/pylon/pylon_bms_task.h"
#include "protocols/pylon/pylon_inverter_task.h"

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
    if (g_orchestratorTaskHandle != NULL) {
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

esp_err_t orchestratorStop(void)
{
    if (g_orchestratorTaskHandle != NULL) {
        vTaskDelete(g_orchestratorTaskHandle);
        g_orchestratorTaskHandle = NULL;
    }

    (void)growattBmsTaskStop();
    (void)growattInverterTaskStop();
    (void)pylonBmsTaskStop();
    (void)pylonInverterTaskStop();

    orchestratorReset(&g_orchestratorCtx);
    memset(&g_orchestratorCtx, 0, sizeof(g_orchestratorCtx));

    return ESP_OK;
}
