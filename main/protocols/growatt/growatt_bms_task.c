#include "protocols/growatt/growatt_bms_task.h"

#include <limits.h>
#include <string.h>

#include "config.h"
#include "Drivers/rs485_driver.h"
#include "modbusDecoder.h"
#include "orchestrator/protocol_types.h"
#include "protocols/growatt/growatt_register_map.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    QueueHandle_t outQueue;
    modbusDecoder_t decoder;
    size_t pollIndex;
    uint32_t sequence;
    int64_t lastPollUs;
    int64_t lastPublishUs;
} growattBmsTaskCtx_t;

static growattBmsTaskCtx_t g_growattBmsCtx;
static TaskHandle_t g_growattBmsTaskHandle;

static uint16_t modbusCrc16(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFFu;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}

static int buildReadHoldingRegsReq(uint8_t slave,
                                   uint16_t start,
                                   uint16_t count,
                                   uint8_t *out,
                                   size_t outCap)
{
    if (out == NULL || outCap < 8u || count == 0u || count > 125u) {
        return 0;
    }

    out[0] = slave;
    out[1] = 0x03u;
    out[2] = (uint8_t)((start >> 8) & 0xFFu);
    out[3] = (uint8_t)(start & 0xFFu);
    out[4] = (uint8_t)((count >> 8) & 0xFFu);
    out[5] = (uint8_t)(count & 0xFFu);

    const uint16_t crc = modbusCrc16(out, 6);
    out[6] = (uint8_t)(crc & 0xFFu);
    out[7] = (uint8_t)((crc >> 8) & 0xFFu);
    return 8;
}

static void sendPollRequest(const growatt_modbus_poll_block_t *block)
{
    if (block == NULL) {
        return;
    }

    uint8_t req[8];
    int reqLen = buildReadHoldingRegsReq((uint8_t)GROWATT_BMS_MODBUS_SLAVE_ADDR,
                                         block->start,
                                         block->count,
                                         req,
                                         sizeof(req));
    if (reqLen <= 0) {
        return;
    }

    esp_err_t err = rs485WriteBytes(rs485GetUart1(),
                                    rs485GetDir1(),
                                    req,
                                    reqLen,
                                    pdMS_TO_TICKS(30));
    if (err != ESP_OK) {
        ESP_LOGW(EXAMPLE_TAG, "Growatt BMS Modbus request TX failed (err=0x%x)", (unsigned)err);
    }
}

static uint8_t clampU8(uint16_t v, uint8_t vmax)
{
    return (uint8_t)((v > vmax) ? vmax : v);
}

static bool buildDecodedPacket(const modbusDecoder_t *decoder,
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
    if (modbusDecoderGetCachedReg(decoder, GROWATT_MB_REG_SOC_PCT, &regVal)) {
        outPacket->hasSoc = true;
        outPacket->socPct = clampU8(regVal, 100u);
    }
    if (modbusDecoderGetCachedReg(decoder, GROWATT_MB_REG_TEMP_C, &regVal)) {
        outPacket->hasTemperatureC = true;
        outPacket->temperatureC = (int16_t)regVal;
    }
    if (modbusDecoderGetCachedReg(decoder, GROWATT_MB_REG_PACK_V_CV, &regVal)) {
        outPacket->hasPackVoltageCv = true;
        outPacket->packVoltageCv = regVal;
    }

    uint16_t minCell = 0;
    uint16_t maxCell = 0;
    uint16_t minIdx = 0;
    uint16_t maxIdx = 0;
    bool hasMinCell = modbusDecoderGetCachedReg(decoder, GROWATT_MB_REG_CELL_MIN_MV, &minCell);
    bool hasMaxCell = modbusDecoderGetCachedReg(decoder, GROWATT_MB_REG_CELL_MAX_MV, &maxCell);
    bool hasMinIdx = modbusDecoderGetCachedReg(decoder, GROWATT_MB_REG_CELL_MIN_IDX, &minIdx);
    bool hasMaxIdx = modbusDecoderGetCachedReg(decoder, GROWATT_MB_REG_CELL_MAX_IDX, &maxIdx);
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

static void growattBmsTask(void *pv)
{
    growattBmsTaskCtx_t *ctx = (growattBmsTaskCtx_t *)pv;
    uint8_t rxChunk[RS485_BUF_SIZE];
    const uart_port_t rxUart = rs485GetUart1();

    modbusDecoderInit(&ctx->decoder, "GROWATT_BMS_RS485", GROWATT_BMS_MODBUS_GAP_US);
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

        if ((nowUs - ctx->lastPollUs) >= ((int64_t)GROWATT_BMS_QUERY_PERIOD_MS * 1000LL)) {
            const growatt_modbus_poll_block_t *block =
                &g_growattModbusPollBlocks[ctx->pollIndex];
            sendPollRequest(block);

            ctx->pollIndex = (ctx->pollIndex + 1u) % g_growattModbusPollBlocksCount;
            ctx->lastPollUs = nowUs;
        }

        if ((nowUs - ctx->lastPublishUs) >= ((int64_t)GROWATT_BMS_PUBLISH_PERIOD_MS * 1000LL)) {
            bms_decoded_packet_t packet = {0};
            if (buildDecodedPacket(&ctx->decoder, ++ctx->sequence, &packet)) {
                if (xQueueOverwrite(ctx->outQueue, &packet) != pdPASS) {
                    ESP_LOGW(EXAMPLE_TAG, "Growatt BMS output queue overwrite failed");
                }
            }
            ctx->lastPublishUs = nowUs;
        }
    }
}

esp_err_t growattBmsTaskStart(QueueHandle_t outQueue)
{
    if (outQueue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_growattBmsTaskHandle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&g_growattBmsCtx, 0, sizeof(g_growattBmsCtx));
    g_growattBmsCtx.outQueue = outQueue;

    BaseType_t taskOk =
        xTaskCreate(growattBmsTask,
                    "growatt_bms",
                    GROWATT_BMS_TASK_STACK,
                    &g_growattBmsCtx,
                    GROWATT_BMS_TASK_PRIORITY,
                    &g_growattBmsTaskHandle);
    if (taskOk != pdPASS) {
        g_growattBmsTaskHandle = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "Growatt BMS task started (poll=%dms, publish=%dms)",
             GROWATT_BMS_QUERY_PERIOD_MS,
             GROWATT_BMS_PUBLISH_PERIOD_MS);
    return ESP_OK;
}
