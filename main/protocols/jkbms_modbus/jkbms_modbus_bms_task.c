#include "protocols/jkbms_modbus/jkbms_modbus_bms_task.h"

#include <limits.h>
#include <string.h>

#include "Drivers/rs485_driver.h"
#include "config.h"
#include "modbusDecoder.h"
#include "orchestrator/protocol_types.h"
#include "protocols/jkbms_modbus/jkbms_modbus_poller.h"
#include "protocols/jkbms_modbus/jkbms_modbus_register_map.h"
#include "runtime_settings.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    QueueHandle_t outQueue;
    modbusDecoder_t decoder;
    jkbms_modbus_poller_t poller;
    uint32_t sequence;
    int64_t lastPublishUs;
} jkbmsModbusBmsTaskCtx_t;

static jkbmsModbusBmsTaskCtx_t g_jkbmsModbusBmsCtx;
static TaskHandle_t g_jkbmsModbusBmsTaskHandle;
static portMUX_TYPE g_latestPacketMux = portMUX_INITIALIZER_UNLOCKED;
static bool g_haveLatestPacket;
static bms_decoded_packet_t g_latestPacket;

static void jkbmsStoreLatestPacket(const bms_decoded_packet_t *packet)
{
    if (packet == NULL) {
        return;
    }

    portENTER_CRITICAL(&g_latestPacketMux);
    g_latestPacket = *packet;
    g_haveLatestPacket = true;
    portEXIT_CRITICAL(&g_latestPacketMux);
}

static bool decoderGetU16(const modbusDecoder_t *decoder, uint16_t reg, uint16_t *out)
{
    return modbusDecoderGetCachedReg(decoder, reg, out);
}

static bool decoderGetS16(const modbusDecoder_t *decoder, uint16_t reg, int16_t *out)
{
    uint16_t raw = 0;
    if (!modbusDecoderGetCachedReg(decoder, reg, &raw)) {
        return false;
    }
    if (out != NULL) {
        *out = (int16_t)raw;
    }
    return true;
}

static bool decoderGetU32(const modbusDecoder_t *decoder, uint16_t reg, uint32_t *out)
{
    uint16_t hi = 0;
    uint16_t lo = 0;
    if (!modbusDecoderGetCachedReg(decoder, reg, &hi)) {
        return false;
    }
    if (!modbusDecoderGetCachedReg(decoder, (uint16_t)(reg + 1u), &lo)) {
        return false;
    }

    if (out != NULL) {
        *out = (((uint32_t)hi) << 16) | (uint32_t)lo;
    }
    return true;
}

static bool decodeSocPct(const modbusDecoder_t *decoder, uint8_t *socOut)
{
    uint16_t raw = 0;

    if (decoderGetU16(decoder, JKBMS_RT_REG_BALAN_SOC_U8X2, &raw)) {
        const uint8_t hi = (uint8_t)((raw >> 8) & 0xFFu);
        const uint8_t lo = (uint8_t)(raw & 0xFFu);
        if (lo <= 100u) {
            *socOut = lo;
            return true;
        }
        if (hi <= 100u) {
            *socOut = hi;
            return true;
        }
    }

    if (decoderGetU16(decoder, (uint16_t)(JKBMS_RT_REG_BALAN_SOC_U8X2 + 1u), &raw)) {
        const uint8_t hi = (uint8_t)((raw >> 8) & 0xFFu);
        const uint8_t lo = (uint8_t)(raw & 0xFFu);
        if (lo <= 100u) {
            *socOut = lo;
            return true;
        }
        if (hi <= 100u) {
            *socOut = hi;
            return true;
        }
    }

    return false;
}

static bool decodeCellExtremes(const modbusDecoder_t *decoder, bms_decoded_packet_t *packet)
{
    uint16_t minMv = UINT16_MAX;
    uint16_t maxMv = 0u;
    uint8_t minIdx = 0u;
    uint8_t maxIdx = 0u;
    uint8_t validCount = 0u;

    for (uint8_t i = 0; i < JKBMS_RT_MAX_CELLS; i++) {
        const uint16_t reg = (uint16_t)(JKBMS_RT_REG_CELL0_MV + ((uint16_t)i * JKBMS_RT_CELL_STEP));
        uint16_t mv = 0u;
        if (!decoderGetU16(decoder, reg, &mv)) {
            continue;
        }

        if (mv < 1000u || mv > 5000u) {
            continue;
        }

        validCount++;
        if (mv < minMv) {
            minMv = mv;
            minIdx = (uint8_t)(i + 1u);
        }
        if (mv > maxMv) {
            maxMv = mv;
            maxIdx = (uint8_t)(i + 1u);
        }
    }

    if (validCount == 0u) {
        return false;
    }

    packet->hasCellExtremes = true;
    packet->minCellMv = minMv;
    packet->maxCellMv = maxMv;
    packet->minCellIndex = minIdx;
    packet->maxCellIndex = maxIdx;
    return true;
}

static bool buildDecodedPacket(const modbusDecoder_t *decoder,
                               uint32_t sequence,
                               bms_decoded_packet_t *outPacket)
{
    if (decoder == NULL || outPacket == NULL) {
        return false;
    }

    memset(outPacket, 0, sizeof(*outPacket));
    outPacket->sourceProtocol = PROTOCOL_ID_JKBMS;
    outPacket->sequence = sequence;
    outPacket->timestampUs = esp_timer_get_time();

    uint8_t soc = 0u;
    if (decodeSocPct(decoder, &soc)) {
        outPacket->hasSoc = true;
        outPacket->socPct = soc;
    }

    int16_t tempDeciC = 0;
    if (decoderGetS16(decoder, JKBMS_RT_REG_TEMP_BAT1_DECIC, &tempDeciC) ||
        decoderGetS16(decoder, JKBMS_RT_REG_TEMP_MOS_DECIC, &tempDeciC)) {
        outPacket->hasTemperatureC = true;
        outPacket->temperatureC = (int16_t)(tempDeciC / 10);
    }

    uint32_t packMv = 0u;
    if (decoderGetU32(decoder, JKBMS_RT_REG_PACK_VOLT_MV_U32, &packMv)) {
        const uint32_t packCv = (packMv + 5u) / 10u;
        outPacket->hasPackVoltageCv = true;
        outPacket->packVoltageCv = (uint16_t)((packCv > UINT16_MAX) ? UINT16_MAX : packCv);
    }

    (void)decodeCellExtremes(decoder, outPacket);

    return outPacket->hasSoc ||
           outPacket->hasTemperatureC ||
           outPacket->hasPackVoltageCv ||
           outPacket->hasCellExtremes;
}

static void jkbmsModbusBmsTask(void *pv)
{
    jkbmsModbusBmsTaskCtx_t *ctx = (jkbmsModbusBmsTaskCtx_t *)pv;
    uint8_t rxChunk[RS485_BUF_SIZE];
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    const uint8_t bmsPort = (settings.bms_port == 2u) ? 2u : 1u;
    const uart_port_t rxUart = (bmsPort == 2u) ? rs485GetUart2() : rs485GetUart1();
    const gpio_num_t dirPin = (bmsPort == 2u) ? rs485GetDir2() : rs485GetDir1();
    const char *ifName = (bmsPort == 2u) ? "JKBMS_RS485_2" : "JKBMS_RS485_1";

    modbusDecoderInit(&ctx->decoder, ifName, JKBMS_BMS_MODBUS_GAP_US);
    jkbmsModbusPollerInit(&ctx->poller,
                          rxUart,
                          dirPin,
                          (uint8_t)JKBMS_BMS_MODBUS_SLAVE_ADDR);
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

        esp_err_t pollErr = jkbmsModbusPollerTick(&ctx->poller,
                                                  nowUs,
                                                  JKBMS_BMS_QUERY_PERIOD_MS);
        if (pollErr != ESP_OK && pollErr != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(EXAMPLE_TAG, "JKBMS Modbus poll TX failed (err=0x%x)", (unsigned)pollErr);
        }

        if ((nowUs - ctx->lastPublishUs) >= ((int64_t)JKBMS_BMS_PUBLISH_PERIOD_MS * 1000LL)) {
            bms_decoded_packet_t packet = {0};
            if (buildDecodedPacket(&ctx->decoder, ++ctx->sequence, &packet)) {
                jkbmsStoreLatestPacket(&packet);
                if (xQueueOverwrite(ctx->outQueue, &packet) != pdPASS) {
                    ESP_LOGW(EXAMPLE_TAG, "JKBMS output queue overwrite failed");
                }
            }
            ctx->lastPublishUs = nowUs;
        }
    }
}

esp_err_t jkbmsModbusBmsTaskStart(QueueHandle_t outQueue)
{
    if (outQueue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_jkbmsModbusBmsTaskHandle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&g_jkbmsModbusBmsCtx, 0, sizeof(g_jkbmsModbusBmsCtx));
    g_jkbmsModbusBmsCtx.outQueue = outQueue;

    BaseType_t taskOk =
        xTaskCreate(jkbmsModbusBmsTask,
                    "jkbms_modbus",
                    JKBMS_BMS_TASK_STACK,
                    &g_jkbmsModbusBmsCtx,
                    JKBMS_BMS_TASK_PRIORITY,
                    &g_jkbmsModbusBmsTaskHandle);
    if (taskOk != pdPASS) {
        g_jkbmsModbusBmsTaskHandle = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "JKBMS Modbus BMS task started (poll=%dms, publish=%dms)",
             JKBMS_BMS_QUERY_PERIOD_MS,
             JKBMS_BMS_PUBLISH_PERIOD_MS);
    return ESP_OK;
}

bool jkbmsModbusBmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket)
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

esp_err_t jkbmsModbusBmsTaskStop(void)
{
    if (g_jkbmsModbusBmsTaskHandle == NULL) {
        return ESP_OK;
    }

    vTaskDelete(g_jkbmsModbusBmsTaskHandle);
    g_jkbmsModbusBmsTaskHandle = NULL;
    memset(&g_jkbmsModbusBmsCtx, 0, sizeof(g_jkbmsModbusBmsCtx));

    portENTER_CRITICAL(&g_latestPacketMux);
    g_haveLatestPacket = false;
    memset(&g_latestPacket, 0, sizeof(g_latestPacket));
    portEXIT_CRITICAL(&g_latestPacketMux);

    return ESP_OK;
}
