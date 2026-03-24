#include "protocols/growatt/growatt_inverter_task.h"

#include <limits.h>
#include <string.h>

#include "Drivers/can_driver.h"
#include "config.h"
#include "orchestrator/protocol_types.h"
#include "protocols/growatt/growatt_register_map.h"

#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    QueueHandle_t inQueue;
    bool haveLatest;
    bms_decoded_packet_t latest;
    int64_t lastTxUs;
} growattInverterTaskCtx_t;

static growattInverterTaskCtx_t g_growattInverterCtx;
static TaskHandle_t g_growattInverterTaskHandle;

static inline void putBe16(uint8_t *dst, uint16_t v)
{
    dst[0] = (uint8_t)((v >> 8) & 0xFFu);
    dst[1] = (uint8_t)(v & 0xFFu);
}

static int16_t clampTempToDeciC(int16_t tempC)
{
    int32_t deci = (int32_t)tempC * 10;
    if (deci > INT16_MAX) {
        deci = INT16_MAX;
    } else if (deci < INT16_MIN) {
        deci = INT16_MIN;
    }
    return (int16_t)deci;
}

static esp_err_t sendCan322(const bms_decoded_packet_t *packet)
{
    if (packet == NULL || !packet->hasSoc || !packet->hasTemperatureC) {
        return ESP_ERR_INVALID_ARG;
    }

    twai_handle_t txBus = canGetBus1();
    if (txBus == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    twai_message_t tx = {0};
    tx.identifier = GROWATT_CAN_ID_322_TEMP_SOC_MIN_MAX;
    tx.data_length_code = 8;

    const uint8_t soc = (packet->socPct > 100u) ? 100u : packet->socPct;
    const int16_t tDeci = clampTempToDeciC(packet->temperatureC);

    putBe16(&tx.data[0], (uint16_t)tDeci);
    putBe16(&tx.data[2], (uint16_t)tDeci);
    tx.data[4] = 1u;
    tx.data[5] = 1u;
    tx.data[6] = soc;
    tx.data[7] = soc;

    return twai_transmit_v2(txBus, &tx, pdMS_TO_TICKS(20));
}

static void growattInverterTask(void *pv)
{
    growattInverterTaskCtx_t *ctx = (growattInverterTaskCtx_t *)pv;
    const int64_t periodUs = (int64_t)GROWATT_INVERTER_TX_PERIOD_MS * 1000LL;
    const TickType_t waitTicks = pdMS_TO_TICKS(GROWATT_INVERTER_TX_PERIOD_MS);

    while (1) {
        bms_decoded_packet_t incoming = {0};
        BaseType_t got = xQueueReceive(ctx->inQueue, &incoming, waitTicks);
        if (got == pdTRUE) {
            ctx->latest = incoming;
            ctx->haveLatest = true;
        }

        if (!ctx->haveLatest || !ctx->latest.hasSoc || !ctx->latest.hasTemperatureC) {
            continue;
        }

        const int64_t nowUs = esp_timer_get_time();
        const bool due = (ctx->lastTxUs == 0) || ((nowUs - ctx->lastTxUs) >= periodUs);
        if (!due && got != pdTRUE) {
            continue;
        }

        esp_err_t err = sendCan322(&ctx->latest);
        if (err != ESP_OK) {
            ESP_LOGW(EXAMPLE_TAG,
                     "Growatt inverter TX failed (err=0x%x)",
                     (unsigned)err);
            continue;
        }
        ctx->lastTxUs = nowUs;
    }
}

esp_err_t growattInverterTaskStart(QueueHandle_t inQueue)
{
    if (inQueue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_growattInverterTaskHandle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&g_growattInverterCtx, 0, sizeof(g_growattInverterCtx));
    g_growattInverterCtx.inQueue = inQueue;

    BaseType_t taskOk =
        xTaskCreate(growattInverterTask,
                    "growatt_inv",
                    GROWATT_INVERTER_TASK_STACK,
                    &g_growattInverterCtx,
                    GROWATT_INVERTER_TASK_PRIORITY,
                    &g_growattInverterTaskHandle);
    if (taskOk != pdPASS) {
        g_growattInverterTaskHandle = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "Growatt inverter task started (period=%dms)",
             GROWATT_INVERTER_TX_PERIOD_MS);
    return ESP_OK;
}
