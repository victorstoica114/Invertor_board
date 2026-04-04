#include "protocols/growatt/growatt_inverter_task.h"

#include <limits.h>
#include <string.h>

#include "Drivers/can_driver.h"
#include "config.h"
#include "orchestrator/protocol_types.h"
#include "protocols/common/battery_model.h"
#include "protocols/growatt/growatt_registers_map.h"

#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    QueueHandle_t inQueue;
    bool haveLatest;
    bool latestFromDebugOverride;
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

static bool buildPacketFromBatteryModel(bms_decoded_packet_t *packet)
{
    battery_model_t model = {0};

    if (packet == NULL) {
        return false;
    }

    batteryModelGet(&model);
    if (!model.valid || model.updatedMs == 0u) {
        return false;
    }

    memset(packet, 0, sizeof(*packet));
    packet->sourceProtocol = PROTOCOL_ID_JKBMS;
    packet->timestampUs = esp_timer_get_time();
    packet->hasSoc = true;
    packet->socPct = model.socPct;

    if (model.temperaturesC[0] > -100.0f) {
        packet->hasTemperatureC = true;
        packet->temperatureC = (int16_t)model.temperaturesC[0];
    }

    if (model.packVoltageV > 0.0f) {
        uint32_t packCv = (uint32_t)(model.packVoltageV * 100.0f + 0.5f);
        packet->hasPackVoltageCv = true;
        packet->packVoltageCv = (uint16_t)((packCv > UINT16_MAX) ? UINT16_MAX : packCv);
    }

    if (model.cellMaxV > 0.0f && model.cellMinV > 0.0f) {
        uint32_t maxMv = (uint32_t)(model.cellMaxV * 1000.0f + 0.5f);
        uint32_t minMv = (uint32_t)(model.cellMinV * 1000.0f + 0.5f);
        packet->hasCellExtremes = true;
        packet->maxCellMv = (uint16_t)((maxMv > UINT16_MAX) ? UINT16_MAX : maxMv);
        packet->minCellMv = (uint16_t)((minMv > UINT16_MAX) ? UINT16_MAX : minMv);
        packet->maxCellIndex = model.cellMaxIdx;
        packet->minCellIndex = model.cellMinIdx;
    }

    return packet->hasSoc || packet->hasTemperatureC;
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
            ctx->latestFromDebugOverride = false;
        } else if (batteryModelIsDebugOverrideEnabled()) {
            bms_decoded_packet_t fakePacket = {0};
            if (buildPacketFromBatteryModel(&fakePacket)) {
                ctx->latest = fakePacket;
                ctx->haveLatest = true;
                ctx->latestFromDebugOverride = true;
            }
        } else if (ctx->latestFromDebugOverride) {
            memset(&ctx->latest, 0, sizeof(ctx->latest));
            ctx->haveLatest = false;
            ctx->latestFromDebugOverride = false;
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

esp_err_t growattInverterTaskStop(void)
{
    if (g_growattInverterTaskHandle == NULL) {
        return ESP_OK;
    }

    vTaskDelete(g_growattInverterTaskHandle);
    g_growattInverterTaskHandle = NULL;
    memset(&g_growattInverterCtx, 0, sizeof(g_growattInverterCtx));
    return ESP_OK;
}
