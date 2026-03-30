#include "protocols/rs485_growatt/rs485_growatt_bms_task.h"

#include <limits.h>
#include <string.h>

#include "Drivers/rs485_driver.h"
#include "config.h"
#include "modbusDecoder.h"
#include "orchestrator/protocol_types.h"
#include "protocols/rs485_growatt/rs485_growatt_modbus_poller.h"
#include "protocols/rs485_growatt/rs485_growatt_registers_map.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    QueueHandle_t outQueue;
    modbusDecoder_t decoder;
    rs485_growatt_modbus_poller_t poller;
    uint32_t sequence;
    int64_t lastPublishUs;
} rs485GrowattBmsTaskCtx_t;

static rs485GrowattBmsTaskCtx_t g_rs485GrowattBmsCtx;
static TaskHandle_t g_rs485GrowattBmsTaskHandle;
static portMUX_TYPE g_latestPacketMux = portMUX_INITIALIZER_UNLOCKED;
static bool g_haveLatestPacket;
static bms_decoded_packet_t g_latestPacket;

static void rs485GrowattStoreLatestPacket(const bms_decoded_packet_t *packet)
{
    if (packet == NULL) {
        return;
    }

    portENTER_CRITICAL(&g_latestPacketMux);
    g_latestPacket = *packet;
    g_haveLatestPacket = true;
    portEXIT_CRITICAL(&g_latestPacketMux);
}

static uint8_t clampU8(uint16_t v, uint8_t vmax)
{
    return (uint8_t)((v > vmax) ? vmax : v);
}

static bool rs485GrowattBuildDecodedPacket(const modbusDecoder_t *decoder,
                                           uint32_t sequence,
                                           bms_decoded_packet_t *outPacket)
{
    if (decoder == NULL || outPacket == NULL) {
        return false;
    }

    memset(outPacket, 0, sizeof(*outPacket));
    outPacket->sourceProtocol = PROTOCOL_ID_GROWATT;
    outPacket->sequence = sequence;
    outPacket->timestampUs = esp_timer_get_time();

    uint16_t regVal = 0;
    if (modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_SOC_PCT, &regVal)) {
        outPacket->hasSoc = true;
        outPacket->socPct = clampU8(regVal, 100u);
    }
    if (modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_TEMP_C, &regVal)) {
        outPacket->hasTemperatureC = true;
        outPacket->temperatureC = (int16_t)regVal;
    }
    if (modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_PACK_V_CV, &regVal)) {
        outPacket->hasPackVoltageCv = true;
        outPacket->packVoltageCv = regVal;
    }

    uint16_t minCell = 0;
    uint16_t maxCell = 0;
    uint16_t minIdx = 0;
    uint16_t maxIdx = 0;
    bool hasMinCell = modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_CELL_MIN_MV, &minCell);
    bool hasMaxCell = modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_CELL_MAX_MV, &maxCell);
    bool hasMinIdx = modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_CELL_MIN_IDX, &minIdx);
    bool hasMaxIdx = modbusDecoderGetCachedReg(decoder, RS485_GROWATT_MB_REG_CELL_MAX_IDX, &maxIdx);
    if (hasMinCell && hasMaxCell && hasMinIdx && hasMaxIdx) {
        outPacket->hasCellExtremes = true;
        outPacket->minCellMv = minCell;
        outPacket->maxCellMv = maxCell;
        outPacket->minCellIndex = clampU8(minIdx, UINT8_MAX);
        outPacket->maxCellIndex = clampU8(maxIdx, UINT8_MAX);
    }

    return outPacket->hasSoc ||
           outPacket->hasTemperatureC ||
           outPacket->hasPackVoltageCv ||
           outPacket->hasCellExtremes;
}

static void rs485GrowattBmsTask(void *pv)
{
    rs485GrowattBmsTaskCtx_t *ctx = (rs485GrowattBmsTaskCtx_t *)pv;
    uint8_t rxChunk[RS485_BUF_SIZE];
    const uart_port_t rxUart = rs485GetUart1();
    int64_t lastRecordedReqUs = 0;

    modbusDecoderInit(&ctx->decoder, "GROWATT_BMS_RS485", GROWATT_BMS_MODBUS_GAP_US);
    rs485GrowattModbusPollerInit(&ctx->poller,
                                 rs485GetUart1(),
                                 rs485GetDir1(),
                                 (uint8_t)GROWATT_BMS_MODBUS_SLAVE_ADDR);
    uart_flush_input(rxUart);

    while (1) {
        int len = uart_read_bytes(rxUart, rxChunk, sizeof(rxChunk), pdMS_TO_TICKS(10));
        int64_t nowUs = esp_timer_get_time();

        if (len > 0) {
            modbusDecoderFeed(&ctx->decoder, rxChunk, len, nowUs);
        } else if (ctx->decoder.haveLastByte &&
                   ((nowUs - ctx->decoder.lastByteUs) > (int64_t)ctx->decoder.gapUs)) {
            modbusDecoderFlush(&ctx->decoder);
        }

        esp_err_t pollErr = rs485GrowattModbusPollerTick(&ctx->poller,
                                                         nowUs,
                                                         GROWATT_BMS_QUERY_PERIOD_MS);
        if (pollErr != ESP_OK && pollErr != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(EXAMPLE_TAG, "RS485 Growatt poll TX failed (err=0x%x)", (unsigned)pollErr);
        } else if (ctx->poller.lastReqValid && ctx->poller.lastReqUs != lastRecordedReqUs) {
            modbusDecoderRecordRequest(&ctx->decoder,
                                       ctx->poller.lastReqSlave,
                                       ctx->poller.lastReqFunc,
                                       ctx->poller.lastReqStart,
                                       ctx->poller.lastReqCount,
                                       ctx->poller.lastReqUs);
            lastRecordedReqUs = ctx->poller.lastReqUs;
        }

        if ((nowUs - ctx->lastPublishUs) >= ((int64_t)GROWATT_BMS_PUBLISH_PERIOD_MS * 1000LL)) {
            bms_decoded_packet_t packet = {0};
            if (rs485GrowattBuildDecodedPacket(&ctx->decoder, ++ctx->sequence, &packet)) {
                rs485GrowattStoreLatestPacket(&packet);
                if (xQueueOverwrite(ctx->outQueue, &packet) != pdPASS) {
                    ESP_LOGW(EXAMPLE_TAG, "RS485 Growatt output queue overwrite failed");
                }
            }
            ctx->lastPublishUs = nowUs;
        }
    }
}

esp_err_t rs485GrowattBmsTaskStart(QueueHandle_t outQueue)
{
    if (outQueue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_rs485GrowattBmsTaskHandle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&g_rs485GrowattBmsCtx, 0, sizeof(g_rs485GrowattBmsCtx));
    g_rs485GrowattBmsCtx.outQueue = outQueue;

    BaseType_t taskOk =
        xTaskCreate(rs485GrowattBmsTask,
                    "rs485_gw_bms",
                    GROWATT_BMS_TASK_STACK,
                    &g_rs485GrowattBmsCtx,
                    GROWATT_BMS_TASK_PRIORITY,
                    &g_rs485GrowattBmsTaskHandle);
    if (taskOk != pdPASS) {
        g_rs485GrowattBmsTaskHandle = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "RS485 Growatt BMS task started (poll=%dms, publish=%dms)",
             GROWATT_BMS_QUERY_PERIOD_MS,
             GROWATT_BMS_PUBLISH_PERIOD_MS);
    return ESP_OK;
}

bool rs485GrowattBmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket)
{
    bool hasPacket = false;

    if (outPacket == NULL) {
        return false;
    }

    portENTER_CRITICAL(&g_latestPacketMux);
    hasPacket = g_haveLatestPacket;
    if (hasPacket) {
        *outPacket = g_latestPacket;
    }
    portEXIT_CRITICAL(&g_latestPacketMux);

    return hasPacket;
}

esp_err_t rs485GrowattBmsTaskStop(void)
{
    if (g_rs485GrowattBmsTaskHandle == NULL) {
        return ESP_OK;
    }

    vTaskDelete(g_rs485GrowattBmsTaskHandle);
    g_rs485GrowattBmsTaskHandle = NULL;
    memset(&g_rs485GrowattBmsCtx, 0, sizeof(g_rs485GrowattBmsCtx));

    portENTER_CRITICAL(&g_latestPacketMux);
    g_haveLatestPacket = false;
    memset(&g_latestPacket, 0, sizeof(g_latestPacket));
    portEXIT_CRITICAL(&g_latestPacketMux);

    return ESP_OK;
}
