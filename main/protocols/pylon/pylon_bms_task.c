#include "protocols/pylon/pylon_bms_task.h"

#include <string.h>

#include "config.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    QueueHandle_t outQueue;
} pylonBmsTaskCtx_t;

static pylonBmsTaskCtx_t g_pylonBmsCtx;
static TaskHandle_t g_pylonBmsTaskHandle;

static void pylonBmsTask(void *pv)
{
    pylonBmsTaskCtx_t *ctx = (pylonBmsTaskCtx_t *)pv;
    (void)ctx;

    int64_t lastLogUs = 0;
    while (1) {
        int64_t nowUs = esp_timer_get_time();
        if ((nowUs - lastLogUs) >= 10000000LL) {
            ESP_LOGW(EXAMPLE_TAG,
                     "Pylon BMS task running in scaffold mode. Add protocol details to enable polling/decoding.");
            lastLogUs = nowUs;
        }
        vTaskDelay(pdMS_TO_TICKS(PYLON_PLACEHOLDER_TASK_PERIOD_MS));
    }
}

esp_err_t pylonBmsTaskStart(QueueHandle_t outQueue)
{
    if (outQueue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_pylonBmsTaskHandle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&g_pylonBmsCtx, 0, sizeof(g_pylonBmsCtx));
    g_pylonBmsCtx.outQueue = outQueue;

    BaseType_t taskOk =
        xTaskCreate(pylonBmsTask,
                    "pylon_bms",
                    PYLON_BMS_TASK_STACK,
                    &g_pylonBmsCtx,
                    PYLON_BMS_TASK_PRIORITY,
                    &g_pylonBmsTaskHandle);
    if (taskOk != pdPASS) {
        g_pylonBmsTaskHandle = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(EXAMPLE_TAG, "Pylon BMS task started (scaffold mode)");
    return ESP_OK;
}

esp_err_t pylonBmsTaskStop(void)
{
    if (g_pylonBmsTaskHandle == NULL) {
        return ESP_OK;
    }

    vTaskDelete(g_pylonBmsTaskHandle);
    g_pylonBmsTaskHandle = NULL;
    memset(&g_pylonBmsCtx, 0, sizeof(g_pylonBmsCtx));
    return ESP_OK;
}
