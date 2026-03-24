#include "Working_modes.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "CAN_Decoder.h"
#include "Drivers/can_driver.h"
#include "Drivers/rs485_driver.h"
#include "config.h"
#include "modbusDecoder.h"
#include "orchestrator/orchestrator.h"
#include "rs485_can_bridge.h"
#include "runtime_settings.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    const char *rxName;
    const char *txName;
    twai_handle_t rxBus;
    twai_handle_t txBus;
    bool decode;
} canForwardCtx_t;

typedef struct {
    const char *rxName;
    const char *txName;
    uart_port_t rxUart;
    uart_port_t txUart;
    gpio_num_t txDirPin;
    bool decode;
    modbusDecoder_t decoder;
} rs485ForwardCtx_t;

typedef struct {
    const char *ifName;
    twai_handle_t bus;
    bool decode;
} canSnifferCtx_t;

typedef struct {
    const char *ifName;
    uart_port_t uart;
    bool decode;
    modbusDecoder_t decoder;
} rs485SnifferCtx_t;

static TaskHandle_t s_modeTaskHandles[8];
static size_t s_modeTaskCount;
static bool s_modeStarted;

static canForwardCtx_t s_canForward12;
static rs485ForwardCtx_t s_rsForward12;

static canSnifferCtx_t s_canSniffer1;
static canSnifferCtx_t s_canSniffer2;
static rs485SnifferCtx_t s_rsSniffer1;
static rs485SnifferCtx_t s_rsSniffer2;

static const char *canNameByPort(uint8_t port)
{
    return (port == 2u) ? "CAN2" : "CAN1";
}

static twai_handle_t canBusByPort(uint8_t port)
{
    return (port == 2u) ? canGetBus1() : canGetBus0();
}

static const char *rsNameByPort(uint8_t port)
{
    return (port == 2u) ? "RS485_2" : "RS485_1";
}

static uart_port_t rsUartByPort(uint8_t port)
{
    return (port == 2u) ? rs485GetUart2() : rs485GetUart1();
}

static gpio_num_t rsDirByPort(uint8_t port)
{
    return (port == 2u) ? rs485GetDir2() : rs485GetDir1();
}

static protocol_id_t protocolIdFromUiProtocol(uint8_t protocol)
{
    switch (protocol) {
        case PROTOCOL_CAN_GROWATT:
        case PROTOCOL_RS485_GROWATT:
            return PROTOCOL_ID_GROWATT;
        case PROTOCOL_CAN_PYLON:
        case PROTOCOL_RS485_PYLON:
            return PROTOCOL_ID_PYLON;
        default:
            return PROTOCOL_ID_GROWATT;
    }
}

static void pushTaskHandle(TaskHandle_t handle)
{
    if (handle == NULL) {
        return;
    }
    if (s_modeTaskCount < (sizeof(s_modeTaskHandles) / sizeof(s_modeTaskHandles[0]))) {
        s_modeTaskHandles[s_modeTaskCount++] = handle;
    }
}

static void bytesToHex(const uint8_t *data, int len, char *out, size_t outCap)
{
    if (out == NULL || outCap == 0u) {
        return;
    }
    out[0] = '\0';

    if (data == NULL || len <= 0) {
        return;
    }

    int pos = 0;
    int printLen = len;
    if (printLen > WORKING_MODE_HEX_PRINT_LIMIT) {
        printLen = WORKING_MODE_HEX_PRINT_LIMIT;
    }

    for (int i = 0; i < printLen; i++) {
        pos += snprintf(&out[pos], outCap - (size_t)pos, "%02X ", data[i]);
        if (pos >= (int)outCap) {
            break;
        }
    }
    if (pos > 0 && pos < (int)outCap) {
        out[pos - 1] = '\0';
    }
}

static void maybeFlushDecoder(modbusDecoder_t *decoder, int64_t nowUs)
{
    if (decoder == NULL || !decoder->haveLastByte) {
        return;
    }
    if ((nowUs - decoder->lastByteUs) > (int64_t)decoder->gapUs) {
        modbusDecoderFlush(decoder);
    }
}

static void canForwardTask(void *pv)
{
    canForwardCtx_t *ctx = (canForwardCtx_t *)pv;
    twai_message_t rx = {0};

    while (1) {
        if (twai_receive_v2(ctx->rxBus, &rx, portMAX_DELAY) != ESP_OK) {
            continue;
        }

#ifdef TWAI_MSG_FLAG_SELF
        if (rx.flags & TWAI_MSG_FLAG_SELF) {
            continue;
        }
#endif

        if (ctx->decode) {
            canDecoderOnFrame(ctx->rxName, &rx);
        }

        esp_err_t err = twai_transmit_v2(ctx->txBus, &rx, pdMS_TO_TICKS(50));
        if (err != ESP_OK) {
            ESP_LOGW(EXAMPLE_TAG,
                     "FORWARD CAN %s->%s TX failed (err=0x%x)",
                     ctx->rxName,
                     ctx->txName,
                     (unsigned)err);
        }
    }
}

static void rs485ForwardTask(void *pv)
{
    rs485ForwardCtx_t *ctx = (rs485ForwardCtx_t *)pv;
    uint8_t chunk[RS485_BUF_SIZE];

    if (ctx->decode) {
        modbusDecoderInit(&ctx->decoder, ctx->rxName, FORWARD_RS485_GAP_US);
    }

    while (1) {
        int len = uart_read_bytes(ctx->rxUart, chunk, sizeof(chunk), pdMS_TO_TICKS(5));
        int64_t nowUs = esp_timer_get_time();

        if (len > 0) {
            if (ctx->decode) {
                modbusDecoderFeed(&ctx->decoder, chunk, len, nowUs);
            }

            esp_err_t err = rs485WriteBytes(ctx->txUart,
                                            ctx->txDirPin,
                                            chunk,
                                            len,
                                            pdMS_TO_TICKS(50));
            if (err != ESP_OK) {
                ESP_LOGW(EXAMPLE_TAG,
                         "FORWARD RS485 %s->%s TX failed (err=0x%x)",
                         ctx->rxName,
                         ctx->txName,
                         (unsigned)err);
            }
        }

        if (ctx->decode) {
            maybeFlushDecoder(&ctx->decoder, nowUs);
        }
    }
}

static void canSnifferTask(void *pv)
{
    canSnifferCtx_t *ctx = (canSnifferCtx_t *)pv;
    twai_message_t rx = {0};
    char hex[3 * WORKING_MODE_HEX_PRINT_LIMIT + 1];

    while (1) {
        if (twai_receive_v2(ctx->bus, &rx, portMAX_DELAY) != ESP_OK) {
            continue;
        }

#ifdef TWAI_MSG_FLAG_SELF
        if (rx.flags & TWAI_MSG_FLAG_SELF) {
            continue;
        }
#endif

        bytesToHex(rx.data, rx.data_length_code, hex, sizeof(hex));
        ESP_LOGI(EXAMPLE_TAG,
                 "SNIFFER %s: ID=0x%03" PRIX32 " DLC=%u DATA=[%s]%s",
                 ctx->ifName,
                 (uint32_t)rx.identifier,
                 (unsigned)rx.data_length_code,
                 hex,
                 (rx.data_length_code > WORKING_MODE_HEX_PRINT_LIMIT) ? " ..." : "");

        if (ctx->decode) {
            canDecoderOnFrame(ctx->ifName, &rx);
        }
    }
}

static void logRs485Frame(const char *ifName, const uint8_t *frame, int len)
{
    char hex[3 * WORKING_MODE_HEX_PRINT_LIMIT + 1];
    bytesToHex(frame, len, hex, sizeof(hex));

    ESP_LOGI(EXAMPLE_TAG,
             "SNIFFER %s: LEN=%d DATA=[%s]%s",
             ifName,
             len,
             hex,
             (len > WORKING_MODE_HEX_PRINT_LIMIT) ? " ..." : "");
}

static void rs485SnifferTask(void *pv)
{
    rs485SnifferCtx_t *ctx = (rs485SnifferCtx_t *)pv;
    uint8_t chunk[RS485_BUF_SIZE];
    uint8_t frame[256];
    uint16_t frameLen = 0;
    int64_t lastByteUs = 0;
    bool haveLastByte = false;

    if (ctx->decode) {
        modbusDecoderInit(&ctx->decoder, ctx->ifName, SNIFFER_RS485_GAP_US);
    }

    while (1) {
        int len = uart_read_bytes(ctx->uart, chunk, sizeof(chunk), pdMS_TO_TICKS(5));
        int64_t nowUs = esp_timer_get_time();

        if (len > 0) {
            if (ctx->decode) {
                modbusDecoderFeed(&ctx->decoder, chunk, len, nowUs);
            }

            if (haveLastByte && (nowUs - lastByteUs) > (int64_t)SNIFFER_RS485_GAP_US) {
                if (frameLen > 0) {
                    logRs485Frame(ctx->ifName, frame, frameLen);
                    frameLen = 0;
                }
                haveLastByte = false;
            }

            if ((size_t)frameLen + (size_t)len > sizeof(frame)) {
                if (frameLen > 0) {
                    logRs485Frame(ctx->ifName, frame, frameLen);
                }
                frameLen = 0;
            }

            if ((size_t)frameLen + (size_t)len <= sizeof(frame)) {
                memcpy(&frame[frameLen], chunk, (size_t)len);
                frameLen = (uint16_t)(frameLen + len);
                lastByteUs = nowUs;
                haveLastByte = true;
            }
        }

        if (ctx->decode) {
            maybeFlushDecoder(&ctx->decoder, nowUs);
        }

        if (haveLastByte && (nowUs - lastByteUs) > (int64_t)SNIFFER_RS485_GAP_US) {
            if (frameLen > 0) {
                logRs485Frame(ctx->ifName, frame, frameLen);
                frameLen = 0;
            }
            haveLastByte = false;
        }
    }
}

static void canSnapshotTask(void *pv)
{
    const char *ifName = (const char *)pv;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(WORKING_MODE_SNAPSHOT_PERIOD_MS));
        canDecoderPrintCachedSnapshot(ifName);
    }
}

static void rs485SnapshotTask(void *pv)
{
    modbusDecoder_t *decoder = (modbusDecoder_t *)pv;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(WORKING_MODE_SNAPSHOT_PERIOD_MS));
        modbusDecoderPrintSnapshot(decoder);
    }
}

static esp_err_t startBridgeMode(void)
{
    bridge_runtime_settings_t settings = runtimeSettingsGet();

    const bool canToRsGrowatt =
        (settings.bms_line == LINE_CAN) &&
        (settings.inverter_line == LINE_RS485) &&
        (settings.bms_protocol == PROTOCOL_CAN_GROWATT) &&
        (settings.inverter_protocol == PROTOCOL_RS485_GROWATT);

    if (canToRsGrowatt) {
        canRs485GrowattBridgeEnable(rsUartByPort(settings.inverter_port),
                                    rsDirByPort(settings.inverter_port),
                                    rsNameByPort(settings.inverter_port),
                                    canBusByPort(settings.bms_port),
                                    canNameByPort(settings.bms_port));
        ESP_LOGI(EXAMPLE_TAG,
                 "Working mode BRIDGE active: CAN(%s:%u) -> RS485(%s:%u) Growatt translator",
                 canNameByPort(settings.bms_port),
                 (unsigned)settings.bms_port,
                 rsNameByPort(settings.inverter_port),
                 (unsigned)settings.inverter_port);
        return ESP_OK;
    }

    return orchestratorStart(protocolIdFromUiProtocol(settings.bms_protocol),
                             protocolIdFromUiProtocol(settings.inverter_protocol));
}

static esp_err_t startForwardMode(void)
{
    memset(&s_canForward12, 0, sizeof(s_canForward12));
    s_canForward12.rxName = "CAN1";
    s_canForward12.txName = "CAN2";
    s_canForward12.rxBus = canGetBus0();
    s_canForward12.txBus = canGetBus1();
    s_canForward12.decode = FORWARD_CAN_DECODE_ENABLE;

    memset(&s_rsForward12, 0, sizeof(s_rsForward12));
    s_rsForward12.rxName = "RS485_1";
    s_rsForward12.txName = "RS485_2";
    s_rsForward12.rxUart = rs485GetUart1();
    s_rsForward12.txUart = rs485GetUart2();
    s_rsForward12.txDirPin = rs485GetDir2();
    s_rsForward12.decode = FORWARD_RS485_DECODE_ENABLE;

    uart_flush_input(s_rsForward12.rxUart);

    TaskHandle_t task = NULL;
    if (xTaskCreate(canForwardTask,
                    "fw_can1_to_2",
                    FORWARD_CAN_TASK_STACK,
                    &s_canForward12,
                    FORWARD_CAN_TASK_PRIORITY,
                    &task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    pushTaskHandle(task);

    task = NULL;
    if (xTaskCreate(rs485ForwardTask,
                    "fw_rs1_to_2",
                    FORWARD_RS485_TASK_STACK,
                    &s_rsForward12,
                    FORWARD_RS485_TASK_PRIORITY,
                    &task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    pushTaskHandle(task);

    if (FORWARD_CAN_DECODE_ENABLE) {
        task = NULL;
        if (xTaskCreate(canSnapshotTask,
                        "fw_can_snap",
                        WORKING_MODE_SNAPSHOT_TASK_STACK,
                        (void *)"CAN1",
                        WORKING_MODE_SNAPSHOT_TASK_PRIORITY,
                        &task) == pdPASS) {
            pushTaskHandle(task);
        }
    }

    if (FORWARD_RS485_DECODE_ENABLE) {
        task = NULL;
        if (xTaskCreate(rs485SnapshotTask,
                        "fw_rs_snap",
                        WORKING_MODE_SNAPSHOT_TASK_STACK,
                        &s_rsForward12.decoder,
                        WORKING_MODE_SNAPSHOT_TASK_PRIORITY,
                        &task) == pdPASS) {
            pushTaskHandle(task);
        }
    }

    ESP_LOGI(EXAMPLE_TAG,
             "Working mode FORWARD active: CAN1->CAN2, RS485_1->RS485_2 (decode CAN=%s RS485=%s)",
             FORWARD_CAN_DECODE_ENABLE ? "ON" : "OFF",
             FORWARD_RS485_DECODE_ENABLE ? "ON" : "OFF");
    return ESP_OK;
}

static esp_err_t startSnifferMode(void)
{
    memset(&s_canSniffer1, 0, sizeof(s_canSniffer1));
    s_canSniffer1.ifName = "CAN1";
    s_canSniffer1.bus = canGetBus0();
    s_canSniffer1.decode = SNIFFER_CAN_DECODE_ENABLE;

    memset(&s_canSniffer2, 0, sizeof(s_canSniffer2));
    s_canSniffer2.ifName = "CAN2";
    s_canSniffer2.bus = canGetBus1();
    s_canSniffer2.decode = SNIFFER_CAN_DECODE_ENABLE;

    memset(&s_rsSniffer1, 0, sizeof(s_rsSniffer1));
    s_rsSniffer1.ifName = "RS485_1";
    s_rsSniffer1.uart = rs485GetUart1();
    s_rsSniffer1.decode = SNIFFER_RS485_DECODE_ENABLE;

    memset(&s_rsSniffer2, 0, sizeof(s_rsSniffer2));
    s_rsSniffer2.ifName = "RS485_2";
    s_rsSniffer2.uart = rs485GetUart2();
    s_rsSniffer2.decode = SNIFFER_RS485_DECODE_ENABLE;

    uart_flush_input(s_rsSniffer1.uart);
    uart_flush_input(s_rsSniffer2.uart);

    TaskHandle_t task = NULL;
    if (xTaskCreate(canSnifferTask,
                    "snif_can1",
                    SNIFFER_CAN_TASK_STACK,
                    &s_canSniffer1,
                    SNIFFER_CAN_TASK_PRIORITY,
                    &task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    pushTaskHandle(task);

    task = NULL;
    if (xTaskCreate(canSnifferTask,
                    "snif_can2",
                    SNIFFER_CAN_TASK_STACK,
                    &s_canSniffer2,
                    SNIFFER_CAN_TASK_PRIORITY,
                    &task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    pushTaskHandle(task);

    task = NULL;
    if (xTaskCreate(rs485SnifferTask,
                    "snif_rs1",
                    SNIFFER_RS485_TASK_STACK,
                    &s_rsSniffer1,
                    SNIFFER_RS485_TASK_PRIORITY,
                    &task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    pushTaskHandle(task);

    task = NULL;
    if (xTaskCreate(rs485SnifferTask,
                    "snif_rs2",
                    SNIFFER_RS485_TASK_STACK,
                    &s_rsSniffer2,
                    SNIFFER_RS485_TASK_PRIORITY,
                    &task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    pushTaskHandle(task);

    if (SNIFFER_CAN_DECODE_ENABLE) {
        task = NULL;
        if (xTaskCreate(canSnapshotTask,
                        "snif_can_snap",
                        WORKING_MODE_SNAPSHOT_TASK_STACK,
                        (void *)"CAN1",
                        WORKING_MODE_SNAPSHOT_TASK_PRIORITY,
                        &task) == pdPASS) {
            pushTaskHandle(task);
        }
    }

    if (SNIFFER_RS485_DECODE_ENABLE) {
        task = NULL;
        if (xTaskCreate(rs485SnapshotTask,
                        "snif_rs1_snap",
                        WORKING_MODE_SNAPSHOT_TASK_STACK,
                        &s_rsSniffer1.decoder,
                        WORKING_MODE_SNAPSHOT_TASK_PRIORITY,
                        &task) == pdPASS) {
            pushTaskHandle(task);
        }

        task = NULL;
        if (xTaskCreate(rs485SnapshotTask,
                        "snif_rs2_snap",
                        WORKING_MODE_SNAPSHOT_TASK_STACK,
                        &s_rsSniffer2.decoder,
                        WORKING_MODE_SNAPSHOT_TASK_PRIORITY,
                        &task) == pdPASS) {
            pushTaskHandle(task);
        }
    }

    ESP_LOGI(EXAMPLE_TAG,
             "Working mode SNIFFER active on CAN1/CAN2/RS485_1/RS485_2");
    return ESP_OK;
}

const char *workingModeToStr(working_mode_t mode)
{
    switch (mode) {
        case WORKING_MODE_BRIDGE:
            return "bridge";
        case WORKING_MODE_FORWARD:
            return "forward";
        case WORKING_MODE_SNIFFER:
            return "sniffer";
        default:
            return "unknown";
    }
}

esp_err_t workingModesStart(working_mode_t mode)
{
    if (s_modeStarted) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(s_modeTaskHandles, 0, sizeof(s_modeTaskHandles));
    s_modeTaskCount = 0;

    esp_err_t err = ESP_ERR_NOT_SUPPORTED;
    switch (mode) {
        case WORKING_MODE_BRIDGE:
            err = startBridgeMode();
            break;
        case WORKING_MODE_FORWARD:
            err = startForwardMode();
            break;
        case WORKING_MODE_SNIFFER:
            err = startSnifferMode();
            break;
        default:
            err = ESP_ERR_INVALID_ARG;
            break;
    }

    if (err == ESP_OK) {
        s_modeStarted = true;
    }
    return err;
}
