#include "protocols/pylon/pylon_inverter_task.h"

#include <limits.h>
#include <stdint.h>
#include <string.h>

#include "Drivers/can_driver.h"
#include "config.h"
#include "orchestrator/protocol_types.h"
#include "protocols/common/battery_model.h"
#include "protocols/pylon/pylon_registers_map.h"
#include "runtime_settings.h"

#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef PYLON_CAN_INVERTER_TX_PERIOD_MS
#define PYLON_CAN_INVERTER_TX_PERIOD_MS 1000u
#endif

typedef struct {
    QueueHandle_t inQueue;
    bool canPylonMode;
    twai_handle_t txBus;
    const char *txName;
    bms_decoded_packet_t latestPacket;
    bool haveLatestPacket;
    int64_t lastTxUs;
    int64_t lastLogUs;
} pylonInverterTaskCtx_t;

static pylonInverterTaskCtx_t g_pylonInverterCtx;
static TaskHandle_t g_pylonInverterTaskHandle;

static twai_handle_t canBusByPort(uint8_t port)
{
    return (port == 2u) ? canGetBus1() : canGetBus0();
}

static const char *canNameByPort(uint8_t port)
{
    return (port == 2u) ? "CAN2" : "CAN1";
}

static uint32_t sourceStaleMsForSettings(const bridge_runtime_settings_t *settings)
{
    if (settings != NULL) {
        if (settings->bms_protocol == PROTOCOL_RS485_DALY) {
            return DALY_RS485_SOURCE_STALE_MS;
        }
        if (bridgeProtocolIsCanDaly(settings->bms_protocol)) {
            return DALY_CAN_SOURCE_STALE_MS;
        }
        if ((settings->bms_line == LINE_RS485) &&
            bridgeProtocolIsRs485Pylon(settings->bms_protocol)) {
            return PYLON_RS485_SOURCE_STALE_MS;
        }
    }
    return BRIDGE_SOURCE_STALE_MS;
}

static void putLe16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static uint16_t floatToU16Scaled(float value, float scale, uint16_t fallback)
{
    if (value < 0.0f || scale <= 0.0f) {
        return fallback;
    }

    const float scaled = (value * scale) + 0.5f;
    if (scaled > (float)UINT16_MAX) {
        return UINT16_MAX;
    }
    return (uint16_t)scaled;
}

static int16_t floatToI16Scaled(float value, float scale)
{
    const float scaled = value * scale;
    if (scaled > (float)INT16_MAX) {
        return INT16_MAX;
    }
    if (scaled < (float)INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

static float firstValidTemperatureC(const battery_model_t *model)
{
    if (model != NULL) {
        for (uint8_t i = 0u; i < UNIVERSAL_BATTERY_TEMP_SENSORS; i++) {
            if (model->temperaturesC[i] > -80.0f && model->temperaturesC[i] < 150.0f) {
                return model->temperaturesC[i];
            }
        }
    }
    return 25.0f;
}

static float averageCellVoltageV(const battery_model_t *model)
{
    if (model == NULL) {
        return 0.0f;
    }

    if (model->cellMinV > 1.5f && model->cellMaxV > 1.5f) {
        return (model->cellMinV + model->cellMaxV) * 0.5f;
    }
    return 0.0f;
}

static uint8_t estimateSeriesCells(const battery_model_t *model)
{
    if (model == NULL || model->packVoltageV <= 0.0f) {
        return 8u;
    }

    const float avgCell = averageCellVoltageV(model);
    if (avgCell > 1.5f) {
        int cells = (int)((model->packVoltageV / avgCell) + 0.5f);
        if (cells < 4) {
            cells = 4;
        } else if (cells > 32) {
            cells = 32;
        }
        return (uint8_t)cells;
    }

    return (model->packVoltageV < 35.0f) ? 8u : 16u;
}

static bool packetToFallbackModel(const bms_decoded_packet_t *packet, battery_model_t *model)
{
    if (packet == NULL || model == NULL) {
        return false;
    }

    memset(model, 0, sizeof(*model));
    model->valid = true;
    model->updatedMs = (uint32_t)(esp_timer_get_time() / 1000LL);
    model->sohPct = 100u;
    model->chargeEnabled = true;
    model->dischargeEnabled = true;
    model->protocolState = 0xC0u;

    if (packet->hasSoc) {
        model->socPct = (packet->socPct > 100u) ? 100u : packet->socPct;
    }
    if (packet->hasPackVoltageCv) {
        model->packVoltageV = (float)packet->packVoltageCv / 100.0f;
    }
    if (packet->hasTemperatureC) {
        model->temperaturesC[0] = (float)packet->temperatureC;
    } else if (packet->tempCount > 0u) {
        model->temperaturesC[0] = (float)packet->tempDeciC[0] / 10.0f;
    } else {
        model->temperaturesC[0] = 25.0f;
    }
    if (packet->hasCellExtremes) {
        model->cellMinV = (float)packet->minCellMv / 1000.0f;
        model->cellMaxV = (float)packet->maxCellMv / 1000.0f;
        model->cellMinIdx = packet->minCellIndex;
        model->cellMaxIdx = packet->maxCellIndex;
        model->cellDeltaV = model->cellMaxV - model->cellMinV;
    }

    return packet->hasSoc || packet->hasPackVoltageCv || packet->hasTemperatureC ||
           packet->hasCellExtremes;
}

static bool packetIsFreshForMs(const bms_decoded_packet_t *packet, uint32_t staleMs)
{
    if (packet == NULL || packet->timestampUs <= 0) {
        return false;
    }

    const int64_t ageUs = esp_timer_get_time() - packet->timestampUs;
    return ageUs >= 0 && ageUs <= ((int64_t)staleMs * 1000LL);
}

static esp_err_t sendCanFrame(twai_handle_t bus, uint32_t id, const uint8_t data[8])
{
    if (bus == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    twai_message_t tx = {0};
    tx.identifier = id;
    tx.data_length_code = 8u;
    memcpy(tx.data, data, 8u);
#ifdef TWAI_MSG_FLAG_SS
    tx.flags |= TWAI_MSG_FLAG_SS;
#endif

    return twai_transmit_v2(bus, &tx, pdMS_TO_TICKS(50));
}

static esp_err_t sendPylonCanSet(const pylonInverterTaskCtx_t *ctx, const battery_model_t *model)
{
    if (ctx == NULL || model == NULL || !model->valid) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t f351[8] = {0};
    uint8_t f355[8] = {0};
    uint8_t f356[8] = {0};
    const uint8_t f359[8] = {0x00u, 0x00u, 0x00u, 0x00u, 0x01u, 0x22u, 0x01u, 0x00u};
    uint8_t f35c[8] = {0};
    const uint8_t f35e[8] = {'P', 'Y', 'L', 'O', 'N', ' ', ' ', ' '};
    uint8_t f373[8] = {0};

    const uint8_t cells = estimateSeriesCells(model);
    const float defaultChargeVoltage = 3.65f * (float)cells;
    const float defaultDischargeVoltage = 2.50f * (float)cells;
    const float chargeVoltage = (model->chargeVoltageLimitV > 0.0f) ?
                                model->chargeVoltageLimitV : defaultChargeVoltage;
    const float chargeCurrent = (model->chargeCurrentLimitA > 0.0f) ?
                                model->chargeCurrentLimitA : 100.0f;
    const float dischargeCurrent = (model->dischargeCurrentLimitA > 0.0f) ?
                                   model->dischargeCurrentLimitA : 100.0f;
    const float avgCell = averageCellVoltageV(model);
    const float packVoltage = (model->packVoltageV > 0.0f) ?
                              model->packVoltageV :
                              ((avgCell > 0.0f) ? avgCell * (float)cells : 0.0f);
    const uint8_t soc = (model->socPct > 100u) ? 100u : model->socPct;
    const uint8_t soh = (model->sohPct > 0u && model->sohPct <= 100u) ? model->sohPct : 100u;
    const float tempC = firstValidTemperatureC(model);
    const float cellMinV = (model->cellMinV > 0.0f) ?
                           model->cellMinV :
                           ((avgCell > 0.0f) ? avgCell : 3.30f);
    const float cellMaxV = (model->cellMaxV > 0.0f) ?
                           model->cellMaxV :
                           ((avgCell > 0.0f) ? avgCell : 3.30f);

    uint8_t status = (uint8_t)(model->protocolState & 0xFFu);
    if (status == 0u) {
        status = 0xC0u;
    }
    status = (uint8_t)(status & (uint8_t)~(0x80u | 0x40u | 0x20u));
    if (model->chargeEnabled || model->protocolState == 0u) {
        status = (uint8_t)(status | 0x80u);
    }
    if (model->dischargeEnabled || model->protocolState == 0u) {
        status = (uint8_t)(status | 0x40u);
    }
    if (model->balanceEnabled) {
        status = (uint8_t)(status | 0x20u);
    }

    putLe16(&f351[PYLON_CAN_351_OFF_CHG_VLIM_DV],
            floatToU16Scaled(chargeVoltage, 10.0f, 0u));
    putLe16(&f351[PYLON_CAN_351_OFF_CHG_ILIM_DA],
            floatToU16Scaled(chargeCurrent, 10.0f, 0u));
    putLe16(&f351[PYLON_CAN_351_OFF_DIS_ILIM_DA],
            floatToU16Scaled(dischargeCurrent, 10.0f, 0u));
    putLe16(&f351[PYLON_CAN_351_OFF_DIS_VLIM_DV],
            floatToU16Scaled(defaultDischargeVoltage, 10.0f, 0u));

    putLe16(&f355[PYLON_CAN_355_OFF_SOC_PCT], soc);
    putLe16(&f355[PYLON_CAN_355_OFF_SOH_PCT], soh);

    putLe16(&f356[PYLON_CAN_356_OFF_PACK_V_CV],
            floatToU16Scaled(packVoltage, 100.0f, 0u));
    putLe16(&f356[PYLON_CAN_356_OFF_PACK_I_DA],
            (uint16_t)floatToI16Scaled(model->packCurrentA, 10.0f));
    putLe16(&f356[PYLON_CAN_356_OFF_TEMP_DECIC],
            (uint16_t)floatToI16Scaled(tempC, 10.0f));

    f35c[0] = status;

    putLe16(&f373[PYLON_CAN_373_OFF_CELL_MIN_MV],
            floatToU16Scaled(cellMinV, 1000.0f, 0u));
    putLe16(&f373[PYLON_CAN_373_OFF_CELL_MAX_MV],
            floatToU16Scaled(cellMaxV, 1000.0f, 0u));
    putLe16(&f373[PYLON_CAN_373_OFF_TEMP1_DECIC],
            (uint16_t)floatToI16Scaled(tempC, 10.0f));
    putLe16(&f373[PYLON_CAN_373_OFF_TEMP2_DECIC],
            (uint16_t)floatToI16Scaled(tempC, 10.0f));

    esp_err_t err = sendCanFrame(ctx->txBus, PYLON_CAN_ID_LIMITS_351, f351);
    if (err == ESP_OK) {
        err = sendCanFrame(ctx->txBus, PYLON_CAN_ID_SOC_SOH_355, f355);
    }
    if (err == ESP_OK) {
        err = sendCanFrame(ctx->txBus, PYLON_CAN_ID_PACK_356, f356);
    }
    if (err == ESP_OK) {
        err = sendCanFrame(ctx->txBus, PYLON_CAN_ID_MODULE_INFO_359, f359);
    }
    if (err == ESP_OK) {
        err = sendCanFrame(ctx->txBus, PYLON_CAN_ID_STATUS_35C, f35c);
    }
    if (err == ESP_OK) {
        err = sendCanFrame(ctx->txBus, PYLON_CAN_ID_ASCII_ID_35E, f35e);
    }
    if (err == ESP_OK) {
        err = sendCanFrame(ctx->txBus, PYLON_CAN_ID_CELL_TEMP_373, f373);
    }

    return err;
}

static void pylonInverterTask(void *pv)
{
    pylonInverterTaskCtx_t *ctx = (pylonInverterTaskCtx_t *)pv;

    ESP_LOGI(EXAMPLE_TAG,
             "Pylon inverter task active in %s mode%s%s",
             ctx->canPylonMode ? "CAN sender" : "scaffold",
             ctx->canPylonMode ? " on " : "",
             ctx->canPylonMode ? ctx->txName : "");

    while (1) {
        bms_decoded_packet_t incoming = {0};
        const TickType_t waitTicks = pdMS_TO_TICKS(ctx->canPylonMode ? 100u :
                                                   PYLON_PLACEHOLDER_TASK_PERIOD_MS);
        if (xQueueReceive(ctx->inQueue, &incoming, waitTicks) == pdTRUE) {
            ctx->latestPacket = incoming;
            ctx->haveLatestPacket = true;
        }

        int64_t nowUs = esp_timer_get_time();

        if (!ctx->canPylonMode) {
            if ((nowUs - ctx->lastLogUs) >= 10000000LL) {
                ESP_LOGW(EXAMPLE_TAG,
                         "Pylon inverter task running in scaffold mode. Add protocol encoder/response handling.");
                ctx->lastLogUs = nowUs;
            }
            continue;
        }

        if (ctx->lastTxUs != 0 &&
            (nowUs - ctx->lastTxUs) < ((int64_t)PYLON_CAN_INVERTER_TX_PERIOD_MS * 1000LL)) {
            continue;
        }

        bridge_runtime_settings_t settings = runtimeSettingsGet();
        const uint32_t staleMs = sourceStaleMsForSettings(&settings);
        battery_model_t model = {0};
        batteryModelGetWithStaleMs(&model, staleMs);
        if (!model.valid && ctx->haveLatestPacket &&
            packetIsFreshForMs(&ctx->latestPacket, staleMs)) {
            (void)packetToFallbackModel(&ctx->latestPacket, &model);
        } else if (ctx->haveLatestPacket &&
                   !packetIsFreshForMs(&ctx->latestPacket, staleMs)) {
            memset(&ctx->latestPacket, 0, sizeof(ctx->latestPacket));
            ctx->haveLatestPacket = false;
        }

        if (!model.valid) {
            if ((nowUs - ctx->lastLogUs) >= 5000000LL) {
                ESP_LOGW(EXAMPLE_TAG,
                         "Pylon CAN inverter sender has no fresh BMS model yet");
                ctx->lastLogUs = nowUs;
            }
            continue;
        }

        esp_err_t err = sendPylonCanSet(ctx, &model);
        if (err != ESP_OK) {
            ESP_LOGW(EXAMPLE_TAG,
                     "Pylon CAN inverter TX on %s failed (err=0x%x)",
                     ctx->txName,
                     (unsigned)err);
            continue;
        }

        ctx->lastTxUs = nowUs;
        if ((nowUs - ctx->lastLogUs) >= 5000000LL) {
            ESP_LOGI(EXAMPLE_TAG,
                     "Pylon CAN inverter TX on %s: SOC=%u%% pack=%.2fV I=%.1fA status=0x%02X",
                     ctx->txName,
                     (unsigned)((model.socPct > 100u) ? 100u : model.socPct),
                     (double)model.packVoltageV,
                     (double)model.packCurrentA,
                     (unsigned)(model.protocolState & 0xFFu));
            ctx->lastLogUs = nowUs;
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
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    g_pylonInverterCtx.canPylonMode =
        (settings.inverter_line == LINE_CAN) &&
        (settings.inverter_protocol == PROTOCOL_CAN_PYLON);
    if (g_pylonInverterCtx.canPylonMode) {
        g_pylonInverterCtx.txBus = canBusByPort(settings.inverter_port);
        g_pylonInverterCtx.txName = canNameByPort(settings.inverter_port);
    }

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

    ESP_LOGI(EXAMPLE_TAG,
             "Pylon inverter task started (%s)",
             g_pylonInverterCtx.canPylonMode ? "CAN_PYLON sender" : "scaffold mode");
    return ESP_OK;
}

esp_err_t pylonInverterTaskStop(void)
{
    if (g_pylonInverterTaskHandle == NULL) {
        return ESP_OK;
    }

    vTaskDelete(g_pylonInverterTaskHandle);
    g_pylonInverterTaskHandle = NULL;
    memset(&g_pylonInverterCtx, 0, sizeof(g_pylonInverterCtx));
    return ESP_OK;
}
