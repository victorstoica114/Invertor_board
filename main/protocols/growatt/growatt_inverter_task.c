#include "protocols/growatt/growatt_inverter_task.h"

#include <limits.h>
#include <string.h>

#include "Drivers/can_driver.h"
#include "config.h"
#include "orchestrator/protocol_types.h"
#include "protocols/common/battery_model.h"
#include "protocols/growatt/growatt_registers_map.h"
#include "runtime_settings.h"

#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    QueueHandle_t inQueue;
    twai_handle_t txBus;
    const char *txName;
    bool haveLatest;
    bool latestFromDebugOverride;
    bms_decoded_packet_t latest;
    int64_t lastTxUs;
    int64_t lastLogUs;
} growattInverterTaskCtx_t;

static growattInverterTaskCtx_t g_growattInverterCtx;
static TaskHandle_t g_growattInverterTaskHandle;

static inline void putBe16(uint8_t *dst, uint16_t v)
{
    dst[0] = (uint8_t)((v >> 8) & 0xFFu);
    dst[1] = (uint8_t)(v & 0xFFu);
}

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
    if (settings != NULL &&
        settings->bms_line == LINE_RS485 &&
        bridgeProtocolIsRs485Pylon(settings->bms_protocol)) {
        return PYLON_RS485_SOURCE_STALE_MS;
    }
    return BRIDGE_SOURCE_STALE_MS;
}

static uint16_t clampU16Scaled(float value, float scale, uint16_t fallback)
{
    if (value <= 0.0f || scale <= 0.0f) {
        return fallback;
    }
    float scaled = value * scale + 0.5f;
    if (scaled <= 0.0f) {
        return fallback;
    }
    if (scaled > 65535.0f) {
        return UINT16_MAX;
    }
    return (uint16_t)scaled;
}

static int16_t clampI16Scaled(float value, float scale)
{
    float scaled = value * scale;
    if (scaled > 32767.0f) {
        return INT16_MAX;
    }
    if (scaled < -32768.0f) {
        return INT16_MIN;
    }
    return (int16_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
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

static esp_err_t sendGrowattCanFrame(twai_handle_t txBus, uint32_t id, const uint8_t data[8])
{
    if (txBus == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    twai_message_t tx = {0};
    tx.identifier = id;
    tx.data_length_code = 8;
    memcpy(tx.data, data, 8u);

    return twai_transmit_v2(txBus, &tx, pdMS_TO_TICKS(20));
}

static esp_err_t sendCan322(twai_handle_t txBus, const bms_decoded_packet_t *packet)
{
    if (packet == NULL || !packet->hasSoc || !packet->hasTemperatureC) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[8] = {0};
    const uint8_t soc = (packet->socPct > 100u) ? 100u : packet->socPct;
    const int16_t tDeci = clampTempToDeciC(packet->temperatureC);

    putBe16(&data[0], (uint16_t)tDeci);
    putBe16(&data[2], (uint16_t)tDeci);
    data[4] = 1u;
    data[5] = 1u;
    data[6] = soc;
    data[7] = soc;

    return sendGrowattCanFrame(txBus, GROWATT_CAN_ID_322_TEMP_SOC_MIN_MAX, data);
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

static bool packetIsFreshForMs(const bms_decoded_packet_t *packet, uint32_t staleMs)
{
    if (packet == NULL || packet->timestampUs <= 0) {
        return false;
    }
    int64_t ageUs = esp_timer_get_time() - packet->timestampUs;
    return ageUs >= 0 && ageUs <= ((int64_t)staleMs * 1000LL);
}

static bool packetToFallbackModel(const bms_decoded_packet_t *packet, battery_model_t *model)
{
    if (packet == NULL || model == NULL) {
        return false;
    }

    memset(model, 0, sizeof(*model));
    if (packet->hasPackVoltageCv) {
        model->packVoltageV = (float)packet->packVoltageCv / 100.0f;
    }
    if (packet->hasSoc) {
        model->socPct = packet->socPct;
    }
    model->sohPct = 100u;
    if (packet->hasCellExtremes) {
        model->cellMaxV = (float)packet->maxCellMv / 1000.0f;
        model->cellMinV = (float)packet->minCellMv / 1000.0f;
        model->cellMaxIdx = packet->maxCellIndex;
        model->cellMinIdx = packet->minCellIndex;
        model->cellDeltaV = model->cellMaxV - model->cellMinV;
    }
    if (packet->hasTemperatureC) {
        model->temperaturesC[0] = (float)packet->temperatureC;
    } else if (packet->tempCount > 0u) {
        model->temperaturesC[0] = (float)packet->tempDeciC[0] / 10.0f;
    }
    model->valid = packet->hasSoc || packet->hasPackVoltageCv || packet->hasTemperatureC;
    model->updatedMs = (uint32_t)(esp_timer_get_time() / 1000LL);
    return model->valid;
}

static uint8_t growattStatusByte0(const battery_model_t *model)
{
    uint8_t status = 0u;

    if (model == NULL) {
        return status;
    }
    if (model->chargeEnabled) {
        status |= (1u << 0);
    }
    if (model->balanceEnabled) {
        status |= (1u << 3);
    }
    if (model->dischargeEnabled) {
        status |= (1u << 5);
    }
    return status;
}

static uint8_t growattChargeState(const battery_model_t *model)
{
    uint8_t state = 0u;

    if (model == NULL) {
        return state;
    }
    if (model->chargeEnabled) {
        state |= (1u << 7);
    }
    if (model->dischargeEnabled) {
        state |= (1u << 6);
    }
    return state;
}

static esp_err_t sendGrowattCanSet(const growattInverterTaskCtx_t *ctx, const battery_model_t *model)
{
    if (ctx == NULL || model == NULL || ctx->txBus == NULL || !model->valid) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t soc = (model->socPct > 100u) ? 100u : model->socPct;
    const uint8_t soh = (model->sohPct > 100u || model->sohPct == 0u) ? 100u : model->sohPct;
    const uint16_t packCv = clampU16Scaled(model->packVoltageV, 100.0f, 0u);
    const int16_t packCurrentDca = clampI16Scaled(model->packCurrentA, 10.0f);
    const int16_t tempDci = clampI16Scaled(model->temperaturesC[0], 10.0f);
    const uint16_t chargeVoltageDv = clampU16Scaled(model->chargeVoltageLimitV, 10.0f, 576u);
    const int16_t chargeCurrentDca = clampI16Scaled(model->chargeCurrentLimitA > 0.0f
                                                       ? model->chargeCurrentLimitA
                                                       : 100.0f,
                                                   10.0f);
    const int16_t dischargeCurrentDca = clampI16Scaled(model->dischargeCurrentLimitA > 0.0f
                                                          ? model->dischargeCurrentLimitA
                                                          : 100.0f,
                                                      10.0f);
    const uint16_t cellMaxMv = clampU16Scaled(model->cellMaxV, 1000.0f, 0u);
    const uint16_t cellMinMv = clampU16Scaled(model->cellMinV, 1000.0f, 0u);
    const uint16_t cellDiffMv = (cellMaxMv >= cellMinMv) ? (uint16_t)(cellMaxMv - cellMinMv) : 0u;
    const uint8_t cellMaxIdx = model->cellMaxIdx;
    const uint8_t cellMinIdx = model->cellMinIdx;
    esp_err_t err = ESP_OK;
    uint8_t data[8] = {0};

    putBe16(&data[0], chargeVoltageDv);
    putBe16(&data[2], (uint16_t)chargeCurrentDca);
    putBe16(&data[4], (uint16_t)dischargeCurrentDca);
    data[6] = growattStatusByte0(model);
    data[7] = model->dischargeEnabled ? 0x0Cu : 0x04u;
    err = sendGrowattCanFrame(ctx->txBus, GROWATT_CAN_ID_311_STATUS_LIMITS, data);

    if (err == ESP_OK) {
        memset(data, 0, sizeof(data));
        data[4] = 16u;
        data[7] = 16u;
        err = sendGrowattCanFrame(ctx->txBus, GROWATT_CAN_ID_312_PROT_ALM, data);
    }

    if (err == ESP_OK) {
        memset(data, 0, sizeof(data));
        putBe16(&data[0], packCv);
        putBe16(&data[2], (uint16_t)packCurrentDca);
        putBe16(&data[4], (uint16_t)tempDci);
        data[6] = soc;
        data[7] = soh;
        err = sendGrowattCanFrame(ctx->txBus, GROWATT_CAN_ID_313_V_I_SOC_SOH, data);
    }

    if (err == ESP_OK) {
        memset(data, 0, sizeof(data));
        putBe16(&data[4], cellDiffMv);
        err = sendGrowattCanFrame(ctx->txBus, GROWATT_CAN_ID_314_RM_FCC_DV_CYCLES, data);
    }

    if (err == ESP_OK) {
        memset(data, 0, sizeof(data));
        data[0] = growattChargeState(model);
        putBe16(&data[1], cellMaxMv);
        putBe16(&data[3], cellMinMv);
        data[5] = cellMaxIdx;
        data[6] = cellMinIdx;
        err = sendGrowattCanFrame(ctx->txBus, GROWATT_CAN_ID_319_CELL_REF_FLAGS, data);
    }

    if (err == ESP_OK) {
        memset(data, 0, sizeof(data));
        data[1] = 1u;
        data[2] = 1u;
        data[3] = 1u;
        err = sendGrowattCanFrame(ctx->txBus, GROWATT_CAN_ID_320_MAKER_SW, data);
    }

    if (err == ESP_OK) {
        bms_decoded_packet_t packet = {0};
        if (buildPacketFromBatteryModel(&packet)) {
            err = sendCan322(ctx->txBus, &packet);
        }
    }

    return err;
}

static void growattInverterTask(void *pv)
{
    growattInverterTaskCtx_t *ctx = (growattInverterTaskCtx_t *)pv;
    const int64_t periodUs = (int64_t)GROWATT_INVERTER_TX_PERIOD_MS * 1000LL;
    const TickType_t waitTicks = pdMS_TO_TICKS(GROWATT_INVERTER_TX_PERIOD_MS);

    while (1) {
        bms_decoded_packet_t incoming = {0};
        BaseType_t got = xQueueReceive(ctx->inQueue, &incoming, waitTicks);
        bridge_runtime_settings_t settings = runtimeSettingsGet();
        const uint32_t staleMs = sourceStaleMsForSettings(&settings);
        battery_model_t model = {0};

        if (got == pdTRUE) {
            ctx->latest = incoming;
            ctx->haveLatest = true;
            ctx->latestFromDebugOverride = false;
        }

        batteryModelGetWithStaleMs(&model, staleMs);
        if (!model.valid && ctx->haveLatest && packetIsFreshForMs(&ctx->latest, staleMs)) {
            (void)packetToFallbackModel(&ctx->latest, &model);
        } else if (ctx->haveLatest && !packetIsFreshForMs(&ctx->latest, staleMs)) {
            memset(&ctx->latest, 0, sizeof(ctx->latest));
            ctx->haveLatest = false;
            ctx->latestFromDebugOverride = false;
        }

        if (!model.valid && batteryModelIsDebugOverrideEnabled()) {
            bms_decoded_packet_t fakePacket = {0};
            if (buildPacketFromBatteryModel(&fakePacket)) {
                ctx->latest = fakePacket;
                ctx->haveLatest = true;
                ctx->latestFromDebugOverride = true;
                (void)packetToFallbackModel(&ctx->latest, &model);
            }
        } else if (ctx->latestFromDebugOverride) {
            memset(&ctx->latest, 0, sizeof(ctx->latest));
            ctx->haveLatest = false;
            ctx->latestFromDebugOverride = false;
        }

        if (!model.valid) {
            int64_t nowUs = esp_timer_get_time();
            if ((nowUs - ctx->lastLogUs) >= 5000000LL) {
                ESP_LOGW(EXAMPLE_TAG,
                         "Growatt CAN inverter sender has no fresh BMS model yet");
                ctx->lastLogUs = nowUs;
            }
            continue;
        }

        const int64_t nowUs = esp_timer_get_time();
        const bool due = (ctx->lastTxUs == 0) || ((nowUs - ctx->lastTxUs) >= periodUs);
        if (!due && got != pdTRUE) {
            continue;
        }

        esp_err_t err = sendGrowattCanSet(ctx, &model);
        if (err != ESP_OK) {
            ESP_LOGW(EXAMPLE_TAG,
                     "Growatt CAN inverter TX on %s failed (err=0x%x)",
                     ctx->txName,
                     (unsigned)err);
            continue;
        }
        ctx->lastTxUs = nowUs;
        if ((nowUs - ctx->lastLogUs) >= 5000000LL) {
            ESP_LOGI(EXAMPLE_TAG,
                     "Growatt CAN inverter TX on %s: SOC=%u%% pack=%.2fV I=%.1fA",
                     ctx->txName,
                     (unsigned)((model.socPct > 100u) ? 100u : model.socPct),
                     (double)model.packVoltageV,
                     (double)model.packCurrentA);
            ctx->lastLogUs = nowUs;
        }
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
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    g_growattInverterCtx.txBus = canBusByPort(settings.inverter_port);
    g_growattInverterCtx.txName = canNameByPort(settings.inverter_port);

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
             "Growatt CAN inverter task started on %s (period=%dms)",
             g_growattInverterCtx.txName,
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
