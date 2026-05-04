#include "protocols/wow_modbus/wow_modbus_bms_task.h"

#include <inttypes.h>
#include <string.h>

#include "Drivers/rs485_driver.h"
#include "config.h"
#include "protocols/common/battery_model.h"
#include "protocols/pace_modbus/pace_modbus_bms_task.h"
#include "protocols/pace_modbus/pace_modbus_poller.h"
#include "protocols/pace_modbus/pace_modbus_registers_map.h"
#include "runtime_settings.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    QueueHandle_t outQueue;
    modbusDecoder_t decoder;
    pace_modbus_poller_t poller;
    uint32_t sequence;
    int64_t lastPublishUs;
} wowModbusBmsTaskCtx_t;

static wowModbusBmsTaskCtx_t g_wowModbusBmsCtx;
static TaskHandle_t g_wowModbusBmsTaskHandle;
static portMUX_TYPE g_latestPacketMux = portMUX_INITIALIZER_UNLOCKED;
static bool g_haveLatestPacket;
static bms_decoded_packet_t g_latestPacket;
static int64_t g_lastSourceStaleLogUs;
static int64_t g_lastDecodeLogUs;

static bool decoderCacheFresh(const modbusDecoder_t *decoder, int64_t nowUs, int64_t *newestCacheUsOut)
{
    const int64_t newestCacheUs = modbusDecoderGetNewestCacheTsUs(decoder);
    const int64_t maxAgeUs = (int64_t)BRIDGE_SOURCE_STALE_MS * 1000LL;
    const int64_t ageUs = (nowUs >= newestCacheUs) ? (nowUs - newestCacheUs) : 0;

    if (newestCacheUsOut != NULL) {
        *newestCacheUsOut = newestCacheUs;
    }
    return (newestCacheUs > 0) && (ageUs <= maxAgeUs);
}

static void wowStoreLatestPacket(const bms_decoded_packet_t *packet)
{
    if (packet == NULL) {
        return;
    }

    portENTER_CRITICAL(&g_latestPacketMux);
    g_latestPacket = *packet;
    g_haveLatestPacket = true;
    portEXIT_CRITICAL(&g_latestPacketMux);
}

static void wowClearLatestPacket(void)
{
    portENTER_CRITICAL(&g_latestPacketMux);
    g_haveLatestPacket = false;
    memset(&g_latestPacket, 0, sizeof(g_latestPacket));
    portEXIT_CRITICAL(&g_latestPacketMux);
}

static void wowPublishBatteryModel(const bms_decoded_packet_t *packet)
{
    battery_model_t model = {0};

    if (packet == NULL) {
        return;
    }

    model.valid = true;
    model.updatedMs = (uint32_t)(packet->timestampUs / 1000LL);
    model.sohPct = 100u;

    if (packet->hasSoc) {
        model.socPct = packet->socPct;
    }
    if (packet->hasPackVoltageCv) {
        model.packVoltageV = (float)packet->packVoltageCv / 100.0f;
    }
    if (packet->hasCellExtremes) {
        model.cellMinV = (float)packet->minCellMv / 1000.0f;
        model.cellMaxV = (float)packet->maxCellMv / 1000.0f;
        model.cellMinIdx = packet->minCellIndex;
        model.cellMaxIdx = packet->maxCellIndex;
        model.cellDeltaV = (float)(packet->maxCellMv - packet->minCellMv) / 1000.0f;
    }
    for (uint8_t i = 0u; i < packet->tempCount && i < UNIVERSAL_BATTERY_TEMP_SENSORS; i++) {
        model.temperaturesC[i] = (float)packet->tempDeciC[i] / 10.0f;
    }
    if (packet->hasStatusFlags) {
        model.protocolState = packet->statusFlags;
        model.chargeEnabled = (packet->statusFlags & PACE_MB_STATUS_MOSFET_CHG) != 0u;
        model.dischargeEnabled = (packet->statusFlags & PACE_MB_STATUS_MOSFET_DCHG) != 0u;
    }
    if (packet->hasBalanceFlags) {
        model.balanceEnabled = packet->balanceFlags != 0u;
    }
    if (packet->hasWarningFlags) {
        model.warningsMask = packet->warningFlags;
    }
    if (packet->hasProtectionFlags) {
        model.alarmsMask = packet->protectionFlags;
    }

    batteryModelSet(&model);
}

bool wowModbusBuildDecodedPacket(const modbusDecoder_t *decoder,
                                 uint32_t sequence,
                                 bms_decoded_packet_t *outPacket)
{
    if (!paceModbusBuildDecodedPacket(decoder, sequence, outPacket)) {
        return false;
    }

    outPacket->sourceProtocol = PROTOCOL_ID_WOW;
    return true;
}

static void wowLogDecodedPacket(const bms_decoded_packet_t *packet, int64_t nowUs)
{
    if (packet == NULL) {
        return;
    }
    if ((nowUs - g_lastDecodeLogUs) < 5000000LL) {
        return;
    }
    g_lastDecodeLogUs = nowUs;

    ESP_LOGI(EXAMPLE_TAG,
             "WOW decoded via PACE-compatible map: seq=%" PRIu32 " soc=%u%% pack=%.2fV temp=%dC cell_min=%.3fV#%u cell_max=%.3fV#%u warn=0x%04X prot=0x%04X status=0x%04X",
             packet->sequence,
             packet->hasSoc ? (unsigned)packet->socPct : 0u,
             packet->hasPackVoltageCv ? ((double)packet->packVoltageCv / 100.0) : 0.0,
             packet->hasTemperatureC ? (int)packet->temperatureC : 0,
             packet->hasCellExtremes ? ((double)packet->minCellMv / 1000.0) : 0.0,
             packet->hasCellExtremes ? (unsigned)packet->minCellIndex : 0u,
             packet->hasCellExtremes ? ((double)packet->maxCellMv / 1000.0) : 0.0,
             packet->hasCellExtremes ? (unsigned)packet->maxCellIndex : 0u,
             packet->hasWarningFlags ? (unsigned)packet->warningFlags : 0u,
             packet->hasProtectionFlags ? (unsigned)packet->protectionFlags : 0u,
             packet->hasStatusFlags ? (unsigned)packet->statusFlags : 0u);
}

static void wowModbusBmsTask(void *pv)
{
    wowModbusBmsTaskCtx_t *ctx = (wowModbusBmsTaskCtx_t *)pv;
    uint8_t rxChunk[RS485_BUF_SIZE];
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    const uint8_t bmsPort = (settings.bms_port == 2u) ? 2u : 1u;
    const uart_port_t rxUart = (bmsPort == 2u) ? rs485GetUart2() : rs485GetUart1();
    const gpio_num_t dirPin = (bmsPort == 2u) ? rs485GetDir2() : rs485GetDir1();
    const char *ifName = (bmsPort == 2u) ? "WOW_RS485_2" : "WOW_RS485_1";
    int64_t lastRecordedReqUs = 0;

    modbusDecoderInit(&ctx->decoder, ifName, WOW_BMS_MODBUS_GAP_US);
    paceModbusPollerInit(&ctx->poller,
                         rxUart,
                         dirPin,
                         (uint8_t)WOW_BMS_MODBUS_SLAVE_ADDR);
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

        esp_err_t pollErr = paceModbusPollerTick(&ctx->poller,
                                                 nowUs,
                                                 WOW_BMS_QUERY_PERIOD_MS);
        if (pollErr != ESP_OK && pollErr != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(EXAMPLE_TAG, "WOW Modbus poll TX failed (err=0x%x)", (unsigned)pollErr);
        } else if (ctx->poller.lastReqValid && ctx->poller.lastReqUs != lastRecordedReqUs) {
            modbusDecoderRecordRequest(&ctx->decoder,
                                       ctx->poller.lastReqSlave,
                                       ctx->poller.lastReqFunc,
                                       ctx->poller.lastReqStart,
                                       ctx->poller.lastReqCount,
                                       ctx->poller.lastReqUs);
            lastRecordedReqUs = ctx->poller.lastReqUs;
        }

        if ((nowUs - ctx->lastPublishUs) >= ((int64_t)WOW_BMS_PUBLISH_PERIOD_MS * 1000LL)) {
            int64_t newestCacheUs = 0;
            if (!decoderCacheFresh(&ctx->decoder, nowUs, &newestCacheUs)) {
                batteryModelClear();
                wowClearLatestPacket();
                if ((nowUs - g_lastSourceStaleLogUs) >= 1000000LL) {
                    if (newestCacheUs <= 0) {
                        ESP_LOGW(EXAMPLE_TAG,
                                 "WOW Modbus source stale: clearing published data (no valid BMS response yet)");
                    } else {
                        const uint32_t ageMs = (uint32_t)((nowUs - newestCacheUs) / 1000LL);
                        ESP_LOGW(EXAMPLE_TAG,
                                 "WOW Modbus source stale: clearing published data (last_rx_age=%u ms)",
                                 (unsigned)ageMs);
                    }
                    g_lastSourceStaleLogUs = nowUs;
                }
            } else {
                bms_decoded_packet_t packet = {0};
                if (wowModbusBuildDecodedPacket(&ctx->decoder, ++ctx->sequence, &packet)) {
                    wowLogDecodedPacket(&packet, nowUs);
                    wowStoreLatestPacket(&packet);
                    wowPublishBatteryModel(&packet);
                    if (xQueueOverwrite(ctx->outQueue, &packet) != pdPASS) {
                        ESP_LOGW(EXAMPLE_TAG, "WOW output queue overwrite failed");
                    }
                }
            }
            ctx->lastPublishUs = nowUs;
        }
    }
}

esp_err_t wowModbusBmsTaskStart(QueueHandle_t outQueue)
{
    if (outQueue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_wowModbusBmsTaskHandle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&g_wowModbusBmsCtx, 0, sizeof(g_wowModbusBmsCtx));
    g_wowModbusBmsCtx.outQueue = outQueue;
    g_lastSourceStaleLogUs = 0;
    g_lastDecodeLogUs = 0;
    batteryModelClear();
    wowClearLatestPacket();

    BaseType_t taskOk =
        xTaskCreate(wowModbusBmsTask,
                    "wow_bms",
                    WOW_BMS_TASK_STACK,
                    &g_wowModbusBmsCtx,
                    WOW_BMS_TASK_PRIORITY,
                    &g_wowModbusBmsTaskHandle);
    if (taskOk != pdPASS) {
        g_wowModbusBmsTaskHandle = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "WOW Modbus BMS task started (slave=%u poll=%dms, publish=%dms)",
             (unsigned)WOW_BMS_MODBUS_SLAVE_ADDR,
             WOW_BMS_QUERY_PERIOD_MS,
             WOW_BMS_PUBLISH_PERIOD_MS);
    return ESP_OK;
}

bool wowModbusBmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket)
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

esp_err_t wowModbusBmsTaskStop(void)
{
    if (g_wowModbusBmsTaskHandle == NULL) {
        return ESP_OK;
    }

    vTaskDelete(g_wowModbusBmsTaskHandle);
    g_wowModbusBmsTaskHandle = NULL;
    memset(&g_wowModbusBmsCtx, 0, sizeof(g_wowModbusBmsCtx));
    g_lastDecodeLogUs = 0;
    batteryModelClear();
    wowClearLatestPacket();

    return ESP_OK;
}
