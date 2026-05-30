#include "modes/mode_manager.h"

#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "decoders/CAN_Decoder.h"
#include "Drivers/can_driver.h"
#include "Drivers/rs485_driver.h"
#include "config.h"
#include "decoders/modbusDecoder.h"
#include "orchestrator/orchestrator.h"
#include "protocols/common/battery_model.h"
#include "protocols/pylon/pylon_can_diag_sender.h"
#include "protocols/pylon/pylon_registers_map.h"
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
static working_mode_t s_runningMode = WORKING_MODE_BRIDGE;

static canForwardCtx_t s_canForward12;
static rs485ForwardCtx_t s_rsForward12;

static canSnifferCtx_t s_canSniffer1;
static canSnifferCtx_t s_canSniffer2;
static rs485SnifferCtx_t s_rsSniffer1;
static rs485SnifferCtx_t s_rsSniffer2;

static const char *twaiStateToStr(twai_state_t state)
{
    switch (state) {
        case TWAI_STATE_STOPPED:
            return "STOPPED";
        case TWAI_STATE_RUNNING:
            return "RUNNING";
        case TWAI_STATE_BUS_OFF:
            return "BUS_OFF";
        case TWAI_STATE_RECOVERING:
            return "RECOVERING";
        default:
            return "UNKNOWN";
    }
}

static working_mode_t runtimeModeToWorkingMode(uint8_t runtimeMode)
{
    switch (runtimeMode) {
        case MODE_SNIFFER:
            return WORKING_MODE_SNIFFER;
        case MODE_FORWARD:
            return WORKING_MODE_FORWARD;
        case MODE_BRIDGE:
            return WORKING_MODE_BRIDGE;
        default:
            return s_runningMode;
    }
}

static esp_err_t applyBridgeRuntimeSettings(void)
{
    bridge_runtime_settings_t settings = runtimeSettingsGet();

    ESP_LOGI(EXAMPLE_TAG,
             "Applying bridge settings: mode=%u bms(line=%u prot=%u port=%u) inv(line=%u prot=%u port=%u)",
             (unsigned)settings.mode,
             (unsigned)settings.bms_line,
             (unsigned)settings.bms_protocol,
             (unsigned)settings.bms_port,
             (unsigned)settings.inverter_line,
             (unsigned)settings.inverter_protocol,
             (unsigned)settings.inverter_port);

    (void)orchestratorStop();
    rs485Reinit();
    canReinit();
    esp_err_t err = orchestratorStartFromRuntime(&settings);
    if (err != ESP_OK) {
        ESP_LOGW(EXAMPLE_TAG, "Bridge runtime apply failed (err=0x%x)", (unsigned)err);
    }
    return err;
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

#if CAN_FORWARD_PYLON_16S_TO_8S_ENABLE
static uint16_t getLe16(const uint8_t *src)
{
    return (uint16_t)(((uint16_t)src[1] << 8) | src[0]);
}
#endif

static void putLe16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)((value >> 8) & 0xFFu);
}

#if CAN_FORWARD_PYLON_16S_TO_8S_ENABLE
static uint16_t halfRounded(uint16_t value)
{
    return (uint16_t)((value + 1u) / 2u);
}
#endif

static uint16_t floatToU16Scaled(float value, float scale, uint16_t maxValue)
{
    float scaled = 0.0f;

    if (value <= 0.0f || scale <= 0.0f) {
        return 0u;
    }

    scaled = (value * scale) + 0.5f;
    if (scaled >= (float)maxValue) {
        return maxValue;
    }
    return (uint16_t)scaled;
}

static int16_t floatToI16Scaled(float value, float scale)
{
    float scaled = (value * scale);

    if (scaled >= 0.0f) {
        scaled += 0.5f;
    } else {
        scaled -= 0.5f;
    }

    if (scaled > (float)INT16_MAX) {
        return INT16_MAX;
    }
    if (scaled < (float)INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)scaled;
}

static bool maybeApplyPylonFakeOverride(twai_message_t *msg)
{
    static int64_t s_lastFakeLogUs;
    battery_model_t fake = {0};
    bool enabled = false;
    bool changed = false;

    if (msg == NULL || msg->data_length_code < 8u) {
        return false;
    }

    batteryModelGetDebugOverride(&fake, &enabled);
    if (!enabled || !fake.valid) {
        return false;
    }

    switch ((uint32_t)msg->identifier) {
        case PYLON_CAN_ID_LIMITS_351:
            if (fake.chargeVoltageLimitV > 0.0f) {
                putLe16(&msg->data[PYLON_CAN_351_OFF_CHG_VLIM_DV],
                        floatToU16Scaled(fake.chargeVoltageLimitV, 10.0f, UINT16_MAX));
                changed = true;
            }
            if (fake.chargeCurrentLimitA > 0.0f) {
                putLe16(&msg->data[PYLON_CAN_351_OFF_CHG_ILIM_DA],
                        floatToU16Scaled(fake.chargeCurrentLimitA, 10.0f, UINT16_MAX));
                changed = true;
            }
            if (fake.dischargeCurrentLimitA > 0.0f) {
                putLe16(&msg->data[PYLON_CAN_351_OFF_DIS_ILIM_DA],
                        floatToU16Scaled(fake.dischargeCurrentLimitA, 10.0f, UINT16_MAX));
                changed = true;
            }
            break;

        case PYLON_CAN_ID_SOC_SOH_355:
            putLe16(&msg->data[PYLON_CAN_355_OFF_SOC_PCT], fake.socPct);
            putLe16(&msg->data[PYLON_CAN_355_OFF_SOH_PCT], fake.sohPct);
            changed = true;
            break;

        case PYLON_CAN_ID_PACK_356:
            if (fake.packVoltageV > 0.0f) {
                putLe16(&msg->data[PYLON_CAN_356_OFF_PACK_V_CV],
                        floatToU16Scaled(fake.packVoltageV, 100.0f, UINT16_MAX));
                changed = true;
            }
            putLe16(&msg->data[PYLON_CAN_356_OFF_PACK_I_DA],
                    (uint16_t)floatToI16Scaled(fake.packCurrentA, 10.0f));
            putLe16(&msg->data[PYLON_CAN_356_OFF_TEMP_DECIC],
                    (uint16_t)floatToI16Scaled(fake.temperaturesC[0], 10.0f));
            changed = true;
            break;

        case PYLON_CAN_ID_STATUS_35C: {
            uint8_t status = (uint8_t)(fake.protocolState & 0xFFu);
            status = (uint8_t)(status & (uint8_t)~(0x80u | 0x40u | 0x20u));
            if (fake.chargeEnabled) {
                status = (uint8_t)(status | 0x80u);
            }
            if (fake.dischargeEnabled) {
                status = (uint8_t)(status | 0x40u);
            }
            if (fake.balanceEnabled) {
                status = (uint8_t)(status | 0x20u);
            }
            msg->data[0] = status;
            changed = true;
            break;
        }

        case PYLON_CAN_ID_CELL_TEMP_373:
            if (fake.cellMinV > 0.0f) {
                putLe16(&msg->data[PYLON_CAN_373_OFF_CELL_MIN_MV],
                        floatToU16Scaled(fake.cellMinV, 1000.0f, UINT16_MAX));
                changed = true;
            }
            if (fake.cellMaxV > 0.0f) {
                putLe16(&msg->data[PYLON_CAN_373_OFF_CELL_MAX_MV],
                        floatToU16Scaled(fake.cellMaxV, 1000.0f, UINT16_MAX));
                changed = true;
            }
            putLe16(&msg->data[PYLON_CAN_373_OFF_TEMP1_DECIC],
                    (uint16_t)floatToI16Scaled(fake.temperaturesC[1], 10.0f));
            putLe16(&msg->data[PYLON_CAN_373_OFF_TEMP2_DECIC],
                    (uint16_t)floatToI16Scaled(fake.temperaturesC[2], 10.0f));
            changed = true;
            break;

        default:
            break;
    }

    if (changed) {
        int64_t nowUs = esp_timer_get_time();
        if ((nowUs - s_lastFakeLogUs) >= 5000000LL) {
            ESP_LOGI(EXAMPLE_TAG,
                     "Pylon CAN forward fake override active: soc=%u%% soh=%u%% pack=%.2fV I=%.1fA status=0x%02" PRIX32,
                     (unsigned)fake.socPct,
                     (unsigned)fake.sohPct,
                     (double)fake.packVoltageV,
                     (double)fake.packCurrentA,
                     fake.protocolState & 0xFFu);
            s_lastFakeLogUs = nowUs;
        }
    }

    return changed;
}

static void maybeScalePylon16sTo8s(twai_message_t *msg)
{
#if CAN_FORWARD_PYLON_16S_TO_8S_ENABLE
    static int64_t s_lastLogUs;

    if (msg == NULL || msg->data_length_code < 8u) {
        return;
    }

    const uint32_t id = (uint32_t)msg->identifier;
    if (id == PYLON_CAN_ID_LIMITS_351) {
        const uint16_t inChargeV = getLe16(&msg->data[PYLON_CAN_351_OFF_CHG_VLIM_DV]);
        const uint16_t inDischargeV = getLe16(&msg->data[PYLON_CAN_351_OFF_DIS_VLIM_DV]);
        const uint16_t outChargeV = halfRounded(inChargeV);
        const uint16_t outDischargeV = halfRounded(inDischargeV);

        putLe16(&msg->data[PYLON_CAN_351_OFF_CHG_VLIM_DV], outChargeV);
        putLe16(&msg->data[PYLON_CAN_351_OFF_DIS_VLIM_DV], outDischargeV);

        int64_t nowUs = esp_timer_get_time();
        if ((nowUs - s_lastLogUs) >= 5000000LL) {
            ESP_LOGI(EXAMPLE_TAG,
                     "Pylon CAN forward 16S->8S limits: charge %.1fV->%.1fV discharge %.1fV->%.1fV",
                     (double)inChargeV / 10.0,
                     (double)outChargeV / 10.0,
                     (double)inDischargeV / 10.0,
                     (double)outDischargeV / 10.0);
            s_lastLogUs = nowUs;
        }
        return;
    }

    if (id == PYLON_CAN_ID_PACK_356) {
        const uint16_t inPackV = getLe16(&msg->data[PYLON_CAN_356_OFF_PACK_V_CV]);
        const uint16_t outPackV = halfRounded(inPackV);
        putLe16(&msg->data[PYLON_CAN_356_OFF_PACK_V_CV], outPackV);

        int64_t nowUs = esp_timer_get_time();
        if ((nowUs - s_lastLogUs) >= 5000000LL) {
            ESP_LOGI(EXAMPLE_TAG,
                     "Pylon CAN forward 16S->8S pack: %.2fV->%.2fV",
                     (double)inPackV / 100.0,
                     (double)outPackV / 100.0);
            s_lastLogUs = nowUs;
        }
    }
#else
    (void)msg;
#endif
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

        if (!maybeApplyPylonFakeOverride(&rx)) {
            maybeScalePylon16sTo8s(&rx);
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
    const int chunkBytes = 24;
    int offset = 0;

    ESP_LOGI(EXAMPLE_TAG, "SNIFFER %s: LEN=%d FRAME_BEGIN", ifName, len);

    while (offset < len) {
        char hex[3 * chunkBytes + 1];
        char ascii[chunkBytes + 1];
        int n = len - offset;
        int pos = 0;

        if (n > chunkBytes) {
            n = chunkBytes;
        }

        for (int i = 0; i < n; i++) {
            uint8_t ch = frame[offset + i];
            ascii[i] = (char)((ch >= 32u && ch <= 126u) ? ch : '.');
            pos += snprintf(&hex[pos], sizeof(hex) - (size_t)pos, "%02X ", ch);
            if (pos >= (int)sizeof(hex)) {
                break;
            }
        }
        ascii[n] = '\0';
        if (pos > 0) {
            hex[pos - 1] = '\0';
        } else {
            hex[0] = '\0';
        }

        ESP_LOGI(EXAMPLE_TAG,
                 "SNIFFER %s: CHUNK[%03d..%03d] ASCII=[%s] DATA=[%s]",
                 ifName,
                 offset,
                 offset + n - 1,
                 ascii,
                 hex);

        offset += n;
    }

    ESP_LOGI(EXAMPLE_TAG, "SNIFFER %s: LEN=%d FRAME_END", ifName, len);
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

static void canStatusTask(void *pv)
{
    const canSnifferCtx_t *ctx = (const canSnifferCtx_t *)pv;

    while (1) {
        twai_status_info_t status = {0};
        vTaskDelay(pdMS_TO_TICKS(WORKING_MODE_SNAPSHOT_PERIOD_MS));

        if (ctx == NULL || ctx->bus == NULL) {
            continue;
        }

        esp_err_t err = twai_get_status_info_v2(ctx->bus, &status);
        if (err != ESP_OK) {
            ESP_LOGW(EXAMPLE_TAG,
                     "SNIFFER %s status read failed (err=0x%x)",
                     ctx->ifName ? ctx->ifName : "CAN",
                     (unsigned)err);
            continue;
        }

        ESP_LOGI(EXAMPLE_TAG,
                 "SNIFFER %s status: state=%s txErr=%" PRIu32 " rxErr=%" PRIu32 " txFail=%" PRIu32 " rxMiss=%" PRIu32 " arbLost=%" PRIu32 " busErr=%" PRIu32,
                 ctx->ifName ? ctx->ifName : "CAN",
                 twaiStateToStr(status.state),
                 status.tx_error_counter,
                 status.rx_error_counter,
                 status.tx_failed_count,
                 status.rx_missed_count,
                 status.arb_lost_count,
                 status.bus_error_count);
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
    return applyBridgeRuntimeSettings();
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

        task = NULL;
        if (xTaskCreate(canStatusTask,
                        "snif_can1_stat",
                        WORKING_MODE_SNAPSHOT_TASK_STACK,
                        &s_canSniffer1,
                        WORKING_MODE_SNAPSHOT_TASK_PRIORITY,
                        &task) == pdPASS) {
            pushTaskHandle(task);
        }

        task = NULL;
        if (xTaskCreate(canStatusTask,
                        "snif_can2_stat",
                        WORKING_MODE_SNAPSHOT_TASK_STACK,
                        &s_canSniffer2,
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

#if EASUN_PYLON_24V_DIAG_SENDER_ENABLE
    task = NULL;
    esp_err_t diagErr = pylonCanDiagSenderStart(canGetBus1(), "CAN2", &task);
    if (diagErr == ESP_OK) {
        pushTaskHandle(task);
    } else {
        ESP_LOGW(EXAMPLE_TAG,
                 "EASUN Pylon 24V diagnostic sender start failed (err=0x%x)",
                 (unsigned)diagErr);
    }
#endif

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
        s_runningMode = mode;
    }
    return err;
}

esp_err_t workingModesApplyRuntimeSettings(void)
{
    if (!s_modeStarted) {
        return ESP_ERR_INVALID_STATE;
    }

    bridge_runtime_settings_t settings = runtimeSettingsGet();
    const working_mode_t targetMode = runtimeModeToWorkingMode(settings.mode);

    if (targetMode != s_runningMode) {
        ESP_LOGW(EXAMPLE_TAG,
                 "Runtime mode change requested (%s -> %s), hot switch not available in current build",
                 workingModeToStr(s_runningMode),
                 workingModeToStr(targetMode));
        return ESP_ERR_NOT_SUPPORTED;
    }

    switch (s_runningMode) {
        case WORKING_MODE_BRIDGE:
            return applyBridgeRuntimeSettings();
        case WORKING_MODE_FORWARD:
        case WORKING_MODE_SNIFFER:
            ESP_LOGI(EXAMPLE_TAG, "Runtime settings applied for %s (no task restart needed)",
                     workingModeToStr(s_runningMode));
            return ESP_OK;
        default:
            return ESP_ERR_INVALID_STATE;
    }
}
