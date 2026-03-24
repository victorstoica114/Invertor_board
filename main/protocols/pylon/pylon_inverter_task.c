#include "protocols/pylon/pylon_inverter_task.h"

#include <string.h>

#include "config.h"
#include "orchestrator/protocol_types.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    QueueHandle_t inQueue;
} pylonInverterTaskCtx_t;

static pylonInverterTaskCtx_t g_pylonInverterCtx;
static TaskHandle_t g_pylonInverterTaskHandle;

static void pylonInverterTask(void *pv)
{
    pylonInverterTaskCtx_t *ctx = (pylonInverterTaskCtx_t *)pv;

    int64_t lastLogUs = 0;
    while (1) {
        bms_decoded_packet_t sink = {0};
        (void)xQueueReceive(ctx->inQueue, &sink, pdMS_TO_TICKS(PYLON_PLACEHOLDER_TASK_PERIOD_MS));

        int64_t nowUs = esp_timer_get_time();
        if ((nowUs - lastLogUs) >= 10000000LL) {
            ESP_LOGW(EXAMPLE_TAG,
                     "Pylon inverter task running in scaffold mode. Add protocol encoder/response handling.");
            lastLogUs = nowUs;
        }
    }
}

esp_err_t pylonInverterTaskStart(QueueHandle_t inQueue)
{
    if (inQueue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_pylonInverterTaskHandle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&g_pylonInverterCtx, 0, sizeof(g_pylonInverterCtx));
    g_pylonInverterCtx.inQueue = inQueue;

    BaseType_t taskOk =
        xTaskCreate(pylonInverterTask,
                    "pylon_inv",
                    PYLON_INVERTER_TASK_STACK,
                    &g_pylonInverterCtx,
                    PYLON_INVERTER_TASK_PRIORITY,
                    &g_pylonInverterTaskHandle);
    if (taskOk != pdPASS) {
        g_pylonInverterTaskHandle = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(EXAMPLE_TAG, "Pylon inverter task started (scaffold mode)");
    return ESP_OK;
}
