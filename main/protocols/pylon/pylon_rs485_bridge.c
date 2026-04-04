#include "pylon_rs485_bridge.h"

#include "../../Web_interface/web_bridge_api.h"
#include "../../config.h"
#include "../../runtime_settings.h"
#include "../../Drivers/RS485/rs485_driver.h"
#include "../common/battery_model.h"
#include "pylon_rs485_protocol.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    const char *rxName;
    const char *txName;
    uart_port_t rxUart;
    uart_port_t txUart;
    gpio_num_t rxDirPin;
    gpio_num_t txDirPin;
    bool isBmsSide;
    bool isInverterSide;
} pylonRs485BridgeCtx_t;

typedef struct {
    bool active;
    uint8_t adr;
    uint8_t cid2;
    int64_t dueUs;
} pylonProbePending_t;

typedef struct {
    bool valid;
    uint8_t cid2;
    uint8_t reqAdr;
    int64_t seenUs;
} pylonForwardPending_t;

typedef struct {
    const char *probeName;
    uart_port_t probeUart;
    gpio_num_t probeDirPin;
} pylonProbeTaskCtx_t;

static pylon_rs485_cache_t s_pylonCache = {
    .valid61 = false,
    .valid62 = true,
    .valid63 = false,
    .info61 = {0},
    .info62 = "00000000",
    .info63 = {0},
};

static pylon_rs485_summary_t s_pylonSummary = {0};
static pylonProbePending_t s_probePending = {0};
static pylonForwardPending_t s_forwardPending = {0};
static TaskHandle_t s_pylonBmsTask = NULL;
static TaskHandle_t s_pylonInvTask = NULL;
static TaskHandle_t s_pylonProbeTaskHandle = NULL;
static char s_pylonDecodedLog[2048];
static int64_t s_lastPylonBmsTrafficUs = 0;
static int64_t s_lastPylonInverterTrafficUs = 0;
static int64_t s_lastCanSourceDiagUs = 0;
static int64_t s_lastCacheBuildDiagUs = 0;
static int64_t s_lastStatus63DiagUs = 0;
static uint8_t s_lastStatus63Value = 0xFFu;
static char s_lastStatus63Reason[48] = "init";

static void telemetryFromSummary(void);
static void updateSummary61(void);
static void updateSummary63(void);
static void logSyntheticPayloadBuild(const char *label,
                                     const universal_battery_model_t *model,
                                     const char *info61,
                                     const char *info62,
                                     const char *info63);

static const char *protocolToStrLocal(uint8_t protocol)
{
    switch (protocol) {
        case PROTOCOL_CAN_GROWATT: return "CAN_GROWATT";
        case PROTOCOL_RS485_GROWATT: return "RS485_GROWATT";
        case PROTOCOL_RS485_PYLON: return "RS485_PYLON";
        case PROTOCOL_CAN_PYLON: return "CAN_PYLON";
        case PROTOCOL_CAN_DEYE: return "CAN_DEYE";
        case PROTOCOL_RS485_JKBMS: return "JKBMS_MODBUS";
        case PROTOCOL_CAN_GOODWE: return "CAN_GOODWE";
        case PROTOCOL_CAN_SOFAR: return "CAN_SOFAR";
        case PROTOCOL_CAN_SMA: return "CAN_SMA";
        case PROTOCOL_CAN_VICTRON: return "CAN_VICTRON";
        default: return "UNKNOWN";
    }
}

static void deleteTaskIfRunning(TaskHandle_t *handle)
{
    if (handle != NULL && *handle != NULL) {
        vTaskDelete(*handle);
        *handle = NULL;
    }
}

static bool pylonProbeModeEnabled(uint8_t mode)
{
    return (mode == MODE_BRIDGE) || (mode == MODE_FORWARD);
}

static bool pylonCanToRs485ModeEnabled(const bridge_runtime_settings_t *settings)
{
    return (settings != NULL) &&
           (settings->bms_line == LINE_CAN) &&
           (settings->inverter_line == LINE_RS485) &&
           (settings->bms_protocol == PROTOCOL_CAN_PYLON) &&
           (settings->inverter_protocol == PROTOCOL_RS485_PYLON);
}

static bool pylonRs485PassthroughModeEnabled(const bridge_runtime_settings_t *settings)
{
    return (settings != NULL) &&
           (settings->bms_line == LINE_RS485) &&
           (settings->inverter_line == LINE_RS485) &&
           (settings->bms_protocol == PROTOCOL_RS485_PYLON) &&
           (settings->inverter_protocol == PROTOCOL_RS485_PYLON);
}

static bool pylonSyntheticSourceModeEnabled(const bridge_runtime_settings_t *settings)
{
    return (settings != NULL) &&
           (settings->mode == MODE_BRIDGE) &&
           (settings->inverter_line == LINE_RS485) &&
           (settings->inverter_protocol == PROTOCOL_RS485_PYLON) &&
           !pylonRs485PassthroughModeEnabled(settings);
}

static bool pylonSourceUsesNativePayloadEncoding(const bridge_runtime_settings_t *settings)
{
    return (settings != NULL) &&
           ((settings->bms_protocol == PROTOCOL_CAN_PYLON) ||
            (settings->bms_protocol == PROTOCOL_RS485_PYLON) ||
            (settings->bms_protocol == PROTOCOL_CAN_DEYE));
}

static bool pylonDiagLogsEnabled(void)
{
    return CAN_DECODER_SHOW_RAW_FRAMES != 0;
}

static bool pylonBmsSourceFresh(const bridge_runtime_settings_t *settings)
{
    int64_t nowUs = esp_timer_get_time();

    if (settings == NULL) {
        return false;
    }

    if (pylonSyntheticSourceModeEnabled(settings)) {
        universal_battery_model_t model = {0};
        uint32_t ageMs = 0u;
        batteryModelGet(&model);
        if (model.updatedMs != 0u) {
            uint32_t nowMs = (uint32_t)(nowUs / 1000LL);
            ageMs = nowMs - model.updatedMs;
        }
        if (pylonDiagLogsEnabled() && ((nowUs - s_lastCanSourceDiagUs) >= 1000000LL)) {
            ESP_LOGI(EXAMPLE_TAG,
                     "PYLON synthetic source check: model.valid=%s updatedMs=%" PRIu32 " ageMs=%" PRIu32 " soc=%u soh=%u I=%.2fA V=%.2fV status=0x%02X",
                     model.valid ? "YES" : "NO",
                     model.updatedMs,
                     ageMs,
                     (unsigned)model.socPct,
                     (unsigned)model.sohPct,
                     (double)model.packCurrentA,
                     (double)model.packVoltageV,
                     (unsigned)(model.protocolState & 0xFFu));
            s_lastCanSourceDiagUs = nowUs;
        }
        return model.valid;
    }

    if (settings->bms_line == LINE_RS485) {
        return (s_lastPylonBmsTrafficUs != 0) &&
               ((nowUs - s_lastPylonBmsTrafficUs) <= ((int64_t)BRIDGE_SOURCE_STALE_MS * 1000LL));
    }

    return false;
}

static bool rs485PortUsesHalfDuplex(uart_port_t uart)
{
    if (uart == RS485_1_UART) return RS485_1_USE_HALF_DUPLEX != 0;
    if (uart == RS485_2_UART) return RS485_2_USE_HALF_DUPLEX != 0;
    return RS485_USE_HALF_DUPLEX != 0;
}

static TickType_t rs485PortPreDelayTicks(uart_port_t uart)
{
    if (uart == RS485_1_UART) return pdMS_TO_TICKS(RS485_1_TX_PRE_DELAY_MS);
    if (uart == RS485_2_UART) return pdMS_TO_TICKS(RS485_2_TX_PRE_DELAY_MS);
    return 0;
}

static TickType_t rs485PortPostDelayTicks(uart_port_t uart)
{
    if (uart == RS485_1_UART) return pdMS_TO_TICKS(RS485_1_TX_POST_DELAY_MS);
    if (uart == RS485_2_UART) return pdMS_TO_TICKS(RS485_2_TX_POST_DELAY_MS);
    return 0;
}

static TickType_t rs485TxTimeoutTicksForFrameLen(int len)
{
    if (len <= 0) {
        return pdMS_TO_TICKS(100);
    }

    /* 8N1 framing + margin for scheduling jitter / turnaround. */
    uint32_t bits = (uint32_t)len * 12u;
    uint32_t txMs = (bits * 1000u + (uint32_t)RS485_BAUDRATE - 1u) / (uint32_t)RS485_BAUDRATE;
    uint32_t timeoutMs = txMs + 50u;
    if (timeoutMs < 100u) {
        timeoutMs = 100u;
    }
    return pdMS_TO_TICKS(timeoutMs);
}

static int rs485PortTxLevel(uart_port_t uart)
{
    if (uart == RS485_1_UART) return RS485_1_DIR_TX_LEVEL;
    if (uart == RS485_2_UART) return RS485_2_DIR_TX_LEVEL;
    return 1;
}

static void rs485SetTxForPort(uart_port_t uart, gpio_num_t dirPin, bool txEn)
{
    int txLevel = rs485PortTxLevel(uart);
    int level = txEn ? txLevel : (txLevel ? 0 : 1);
    gpio_set_level(dirPin, level);
}

static int hexNibble(uint8_t ch)
{
    if (ch >= '0' && ch <= '9') return (int)(ch - '0');
    if (ch >= 'A' && ch <= 'F') return 10 + (int)(ch - 'A');
    if (ch >= 'a' && ch <= 'f') return 10 + (int)(ch - 'a');
    return -1;
}

static bool isLikelyPylonAsciiFrame(const uint8_t *frame, int len)
{
    if (frame == NULL || len < 18) return false;
    if (frame[0] != '~' || frame[len - 1] != '\r') return false;
    for (int i = 1; i < len - 1; i++) {
        if (hexNibble(frame[i]) < 0) return false;
    }
    return true;
}

static bool parseAsciiHexByte(const uint8_t *p, uint8_t *out)
{
    int hi = hexNibble(p[0]);
    int lo = hexNibble(p[1]);
    if (hi < 0 || lo < 0) return false;
    *out = (uint8_t)((hi << 4) | lo);
    return true;
}

static bool parsePylonHeader(const uint8_t *frame,
                             int len,
                             uint8_t *ver,
                             uint8_t *adr,
                             uint8_t *cid1,
                             uint8_t *code)
{
    if (!isLikelyPylonAsciiFrame(frame, len)) return false;
    if (!parseAsciiHexByte(&frame[1], ver) ||
        !parseAsciiHexByte(&frame[3], adr) ||
        !parseAsciiHexByte(&frame[5], cid1) ||
        !parseAsciiHexByte(&frame[7], code)) {
        return false;
    }
    return true;
}

static uint16_t pylonAsciiChecksum(const char *body)
{
    uint32_t sum = 0;
    for (const char *p = body; *p != 0; ++p) {
        sum += (uint8_t)(*p);
    }
    return (uint16_t)((~sum + 1u) & 0xFFFFu);
}

static uint16_t pylonLengthField(int infoAsciiLen)
{
    uint16_t lenid = (uint16_t)(infoAsciiLen & 0x0FFF);
    uint16_t n0 = (uint16_t)((lenid >> 8) & 0x0F);
    uint16_t n1 = (uint16_t)((lenid >> 4) & 0x0F);
    uint16_t n2 = (uint16_t)(lenid & 0x0F);
    uint16_t sum = (uint16_t)((n0 + n1 + n2) & 0x0F);
    uint16_t lchk = (uint16_t)((~sum + 1u) & 0x0F);
    return (uint16_t)((lchk << 12) | lenid);
}

static bool pylonBuildRequest(uint8_t ver,
                              uint8_t adr,
                              uint8_t cid2,
                              uint8_t *out,
                              int outSize,
                              int *outLen)
{
    char body[32];
    uint16_t checksum;
    int len;

    if (out == NULL || outLen == NULL || outSize < 20) {
        return false;
    }

    snprintf(body, sizeof(body), "%02X%02X46%02X0000", ver, adr, cid2);
    checksum = pylonAsciiChecksum(body);
    len = snprintf((char *)out, (size_t)outSize, "~%s%04X\r", body, checksum);
    if (len <= 0 || len >= outSize) {
        return false;
    }

    *outLen = len;
    return true;
}

static int parseHexAsciiPayload(const char *ascii, uint8_t *out, int maxOut)
{
    int n = 0;
    if (ascii == NULL || out == NULL || maxOut <= 0) return 0;
    while (ascii[0] != '\0' && ascii[1] != '\0' && n < maxOut) {
        int hi = hexNibble((uint8_t)ascii[0]);
        int lo = hexNibble((uint8_t)ascii[1]);
        if (hi < 0 || lo < 0) break;
        out[n++] = (uint8_t)((hi << 4) | lo);
        ascii += 2;
    }
    return n;
}

static uint16_t be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static int16_t be16s(const uint8_t *p)
{
    return (int16_t)be16(p);
}

static void putBe16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)((v >> 8) & 0xFFu);
    p[1] = (uint8_t)(v & 0xFFu);
}

static void encodeHexAscii(const uint8_t *bytes, int len, char *out, size_t outSize)
{
    size_t pos = 0;

    if (out == NULL || outSize == 0) {
        return;
    }
    out[0] = '\0';
    if (bytes == NULL || len <= 0) {
        return;
    }

    for (int i = 0; i < len && pos + 3u < outSize; i++) {
        pos += (size_t)snprintf(&out[pos], outSize - pos, "%02X", bytes[i]);
    }
}

static bool buildCanDerivedInfo61(char *out, size_t outSize)
{
    static const uint8_t template61[49] = {
        0x21, 0x8F, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x64,
        0x64, 0x12, 0x65, 0x00, 0x03, 0x11, 0xD4, 0x00, 0x0C, 0x0B,
        0xC2, 0x0B, 0xC7, 0x00, 0x04, 0x0B, 0xBE, 0x00, 0x02, 0x0B,
        0xC0, 0x0B, 0xC0, 0x00, 0x00, 0x0B, 0xC0, 0x00, 0x00, 0x0B,
        0xC0, 0x0B, 0xC0, 0x00, 0x00, 0x0B, 0xC0, 0x00, 0x00
    };
    universal_battery_model_t model;
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    bool forceStaticPayload = pylonCanToRs485ModeEnabled(&settings) && PYLON_CAN_RS485_FORCE_FAKE_ENABLE;
    bool nativePayloadSource = pylonSourceUsesNativePayloadEncoding(&settings);
    uint8_t bytes[sizeof(template61)];
    uint16_t kelvinTemp = 0;
    uint16_t maxMv = 0;
    uint16_t minMv = 0;
    uint8_t maxIdx = 3u;
    uint8_t minIdx = 12u;

    batteryModelGet(&model);
    if (!forceStaticPayload && !model.valid) {
        return false;
    }

    memcpy(bytes, template61, sizeof(bytes));
    if (forceStaticPayload) {
        if (model.valid && model.socPct <= 100u) {
            bytes[4] = model.socPct;
        } else {
            bytes[4] = 95u;
        }
        if (model.valid && model.sohPct <= 100u) {
            bytes[9] = model.sohPct;
        } else {
            bytes[9] = 100u;
        }
    } else if (model.valid) {
        /*
         * Preserve the legacy word0 layout for synthetic Pylon 0x61 frames.
         * Older inverter behaviour proved stable with 0x2190 here, while
         * treating bytes[0..1] as plain pack voltage caused faults.
         */
        bytes[1] = 0x90u;
        if (nativePayloadSource) {
            putBe16(&bytes[2], (uint16_t)((int16_t)(model.packCurrentA * 100.0f)));
            bytes[4] = model.socPct;
            putBe16(&bytes[5], model.cycleCount);
            bytes[9] = model.sohPct;
        } else {
            /*
             * For generic sources like JK Modbus, keep the Pylon-specific
             * current/cycle words stable and only project the least ambiguous
             * percentage-style fields.
             */
            if (model.socPct <= 100u) {
                bytes[4] = model.socPct;
            }
            if (model.sohPct <= 100u) {
                bytes[9] = model.sohPct;
            }
        }
    }

    /*
     * Generic sources such as JK Modbus do not expose native Pylon semantics
     * for the cell/temperature block in 0x61. Keeping the stable template for
     * those fields is safer than projecting incompatible values directly.
     */
    if (nativePayloadSource) {
        maxMv = (uint16_t)(model.cellMaxV > 0.0f ? (model.cellMaxV * 1000.0f) : 0.0f);
        minMv = (uint16_t)(model.cellMinV > 0.0f ? (model.cellMinV * 1000.0f) : 0.0f);
        if (maxMv > 0u) {
            putBe16(&bytes[11], maxMv);
        }
        if (minMv > 0u) {
            putBe16(&bytes[15], minMv);
        }

        if (model.cellMaxIdx >= 1u && model.cellMaxIdx <= 16u) {
            maxIdx = model.cellMaxIdx;
        }
        if (model.cellMinIdx >= 1u && model.cellMinIdx <= 16u) {
            minIdx = model.cellMinIdx;
        }
        putBe16(&bytes[13], maxIdx);
        putBe16(&bytes[17], minIdx);

        if (model.temperaturesC[0] > -100.0f) {
            kelvinTemp = (uint16_t)(model.temperaturesC[0] * 10.0f + 2731.0f);
            putBe16(&bytes[19], kelvinTemp);
        }
        if (model.temperaturesC[1] > -100.0f) {
            kelvinTemp = (uint16_t)(model.temperaturesC[1] * 10.0f + 2731.0f);
            putBe16(&bytes[21], kelvinTemp);
        }
        if (model.temperaturesC[2] > -100.0f) {
            kelvinTemp = (uint16_t)(model.temperaturesC[2] * 10.0f + 2731.0f);
            putBe16(&bytes[25], kelvinTemp);
        }
        if (model.temperaturesC[3] > -100.0f) {
            kelvinTemp = (uint16_t)(model.temperaturesC[3] * 10.0f + 2731.0f);
            putBe16(&bytes[29], kelvinTemp);
        }
        if (model.temperaturesC[4] > -100.0f) {
            kelvinTemp = (uint16_t)(model.temperaturesC[4] * 10.0f + 2731.0f);
            putBe16(&bytes[31], kelvinTemp);
            putBe16(&bytes[35], kelvinTemp);
            putBe16(&bytes[39], kelvinTemp);
            putBe16(&bytes[41], kelvinTemp);
            putBe16(&bytes[45], kelvinTemp);
        }
    }

    encodeHexAscii(bytes, (int)sizeof(bytes), out, outSize);
    return out[0] != '\0';
}

static bool buildCanDerivedInfo63(char *out, size_t outSize)
{
    static const uint8_t template63[9] = {0x05, 0xE0, 0xB1, 0x80, 0x00, 0x00, 0x07, 0x6C, 0x40};
    universal_battery_model_t model;
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    bool forceStaticPayload = pylonCanToRs485ModeEnabled(&settings) && PYLON_CAN_RS485_FORCE_FAKE_ENABLE;
    bool nativeStatusSource = pylonSourceUsesNativePayloadEncoding(&settings);
    uint8_t bytes[sizeof(template63)];
    const char *reason = "template";
    int64_t nowUs = esp_timer_get_time();

    batteryModelGet(&model);
    if (!forceStaticPayload && !model.valid) {
        return false;
    }

    memcpy(bytes, template63, sizeof(bytes));
    if (forceStaticPayload) {
        bytes[8] = 0xE0u; /* Diagnostic: permissive status (charge+discharge+balance) */
        reason = "force_fake_e0";
    } else if (model.valid && nativeStatusSource && model.protocolState != 0u) {
        bytes[8] = (uint8_t)(model.protocolState & 0xFFu);
        reason = "native_protocol_state";
    } else if (model.valid) {
        uint8_t status = 0u;
        bool haveExplicitChargeDischarge = false;
        if (model.chargeEnabled) status |= 0x80u;
        if (model.dischargeEnabled) status |= 0x40u;
        if (model.balanceEnabled) status |= 0x20u;
        if ((model.protocolState & 0x80u) != 0u || model.chargeEnabled) {
            haveExplicitChargeDischarge = true;
        }
        if ((model.protocolState & 0x40u) != 0u || model.dischargeEnabled) {
            haveExplicitChargeDischarge = true;
        }
        if (!nativeStatusSource && !haveExplicitChargeDischarge) {
            /*
             * Generic sources like JK do not expose native Pylon enable bits.
             * Treat missing charge/discharge flags as "unknown", not "OFF".
             * Also avoid projecting generic "balance active" into Pylon 0x63,
             * because the web fake path that works uses a plain 0xC0 status.
             */
            status = 0xC0u;
            reason = "generic_default_c0_ignore_balance";
        } else {
            reason = "derived_from_flags";
        }
        bytes[8] = status;
        if (pylonDiagLogsEnabled()) {
            ESP_LOGI(EXAMPLE_TAG,
                     "PYLON synthetic 0x63 inputs: native=%s explicitCD=%s flags(c=%u d=%u b=%u state=0x%02X) -> status=0x%02X reason=%s",
                     nativeStatusSource ? "YES" : "NO",
                     haveExplicitChargeDischarge ? "YES" : "NO",
                     model.chargeEnabled ? 1u : 0u,
                     model.dischargeEnabled ? 1u : 0u,
                     model.balanceEnabled ? 1u : 0u,
                     (unsigned)(model.protocolState & 0xFFu),
                     (unsigned)bytes[8],
                     reason);
        }
    }

    snprintf(s_lastStatus63Reason, sizeof(s_lastStatus63Reason), "%s", reason);

    if (pylonDiagLogsEnabled() &&
        ((bytes[8] != s_lastStatus63Value) || ((nowUs - s_lastStatus63DiagUs) >= 1000000LL))) {
        ESP_LOGI(EXAMPLE_TAG,
                 "PYLON synthetic 0x63 derive: status=0x%02X reason=%s native=%s forceFake=%s flags(c=%u d=%u b=%u state=0x%02X)",
                 (unsigned)bytes[8],
                 reason,
                 nativeStatusSource ? "YES" : "NO",
                 forceStaticPayload ? "YES" : "NO",
                 model.chargeEnabled ? 1u : 0u,
                 model.dischargeEnabled ? 1u : 0u,
                 model.balanceEnabled ? 1u : 0u,
                 (unsigned)(model.protocolState & 0xFFu));
        s_lastStatus63DiagUs = nowUs;
        s_lastStatus63Value = bytes[8];
    }

    encodeHexAscii(bytes, (int)sizeof(bytes), out, outSize);
    return out[0] != '\0';
}

static void logSyntheticPayloadBuild(const char *label,
                                     const universal_battery_model_t *model,
                                     const char *info61,
                                     const char *info62,
                                     const char *info63)
{
    if (!pylonDiagLogsEnabled() || model == NULL) {
        return;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "%s model: valid=%s V=%.2fV I=%.2fA soc=%u soh=%u cyc=%u max=%.3fV#%u min=%.3fV#%u T=[%.1f %.1f %.1f %.1f %.1f] flags(c=%u d=%u b=%u state=0x%02X)",
             label != NULL ? label : "PYLON synthetic",
             model->valid ? "YES" : "NO",
             (double)model->packVoltageV,
             (double)model->packCurrentA,
             (unsigned)model->socPct,
             (unsigned)model->sohPct,
             (unsigned)model->cycleCount,
             (double)model->cellMaxV,
             (unsigned)model->cellMaxIdx,
             (double)model->cellMinV,
             (unsigned)model->cellMinIdx,
             (double)model->temperaturesC[0],
             (double)model->temperaturesC[1],
             (double)model->temperaturesC[2],
             (double)model->temperaturesC[3],
             (double)model->temperaturesC[4],
             model->chargeEnabled ? 1u : 0u,
             model->dischargeEnabled ? 1u : 0u,
             model->balanceEnabled ? 1u : 0u,
             (unsigned)(model->protocolState & 0xFFu));

    if (info61 != NULL && info61[0] != '\0') {
        ESP_LOGI(EXAMPLE_TAG, "%s 0x61=[%s]", label != NULL ? label : "PYLON synthetic", info61);
    }
    if (info62 != NULL && info62[0] != '\0') {
        ESP_LOGI(EXAMPLE_TAG, "%s 0x62=[%s]", label != NULL ? label : "PYLON synthetic", info62);
    }
    if (info63 != NULL && info63[0] != '\0') {
        ESP_LOGI(EXAMPLE_TAG, "%s 0x63=[%s]", label != NULL ? label : "PYLON synthetic", info63);
    }
}

static void maybeRefreshSyntheticCacheFromUniversal(void)
{
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    universal_battery_model_t model = {0};
    universal_battery_model_t realModel = {0};
    char info61[sizeof(s_pylonCache.info61)] = {0};
    char info63[sizeof(s_pylonCache.info63)] = {0};
    int64_t nowUs = esp_timer_get_time();
    bool debugOverride = batteryModelIsDebugOverrideEnabled();

    if (!pylonSyntheticSourceModeEnabled(&settings)) {
        return;
    }

    batteryModelGet(&model);
    batteryModelGetReal(&realModel);

    if (!pylonBmsSourceFresh(&settings)) {
        if (pylonSyntheticSourceModeEnabled(&settings) && PYLON_CAN_RS485_FORCE_FAKE_ENABLE) {
            if (buildCanDerivedInfo61(info61, sizeof(info61))) {
                snprintf(s_pylonCache.info61, sizeof(s_pylonCache.info61), "%s", info61);
                s_pylonCache.valid61 = true;
            }

            snprintf(s_pylonCache.info62, sizeof(s_pylonCache.info62), "00000000");
            s_pylonCache.valid62 = true;

            if (buildCanDerivedInfo63(info63, sizeof(info63))) {
                snprintf(s_pylonCache.info63, sizeof(s_pylonCache.info63), "%s", info63);
                s_pylonCache.valid63 = true;
            }

            memset(&s_pylonSummary, 0, sizeof(s_pylonSummary));
            if (pylonDiagLogsEnabled()) {
                logSyntheticPayloadBuild("PYLON synthetic forced-fake", &model, info61, s_pylonCache.info62, info63);
                ESP_LOGW(EXAMPLE_TAG,
                         "PYLON synthetic source not fresh: using FORCED fake cache (v61=%s v62=%s v63=%s)",
                         s_pylonCache.valid61 ? "YES" : "NO",
                         s_pylonCache.valid62 ? "YES" : "NO",
                         s_pylonCache.valid63 ? "YES" : "NO");
            }
            return;
        }

        bool old61 = s_pylonCache.valid61;
        bool old62 = s_pylonCache.valid62;
        bool old63 = s_pylonCache.valid63;
        s_pylonCache.valid61 = false;
        s_pylonCache.valid62 = false;
        s_pylonCache.valid63 = false;
        memset(&s_pylonSummary, 0, sizeof(s_pylonSummary));
        if (pylonDiagLogsEnabled() && ((nowUs - s_lastCacheBuildDiagUs) >= 1000000LL)) {
            ESP_LOGW(EXAMPLE_TAG,
                     "PYLON synthetic cache cleared: source not fresh (v61=%s v62=%s v63=%s)",
                     old61 ? "YES" : "NO",
                     old62 ? "YES" : "NO",
                     old63 ? "YES" : "NO");
            s_lastCacheBuildDiagUs = nowUs;
        }
        return;
    }

    if (buildCanDerivedInfo61(info61, sizeof(info61))) {
        snprintf(s_pylonCache.info61, sizeof(s_pylonCache.info61), "%s", info61);
        s_pylonCache.valid61 = true;
        updateSummary61();
        telemetryFromSummary();
    } else if (pylonDiagLogsEnabled() && ((nowUs - s_lastCacheBuildDiagUs) >= 1000000LL)) {
        ESP_LOGW(EXAMPLE_TAG, "PYLON synthetic source failed to build 0x61 from battery model");
        s_lastCacheBuildDiagUs = nowUs;
    }

    snprintf(s_pylonCache.info62, sizeof(s_pylonCache.info62), "00000000");
    s_pylonCache.valid62 = true;

    if (buildCanDerivedInfo63(info63, sizeof(info63))) {
        snprintf(s_pylonCache.info63, sizeof(s_pylonCache.info63), "%s", info63);
        s_pylonCache.valid63 = true;
        updateSummary63();
        telemetryFromSummary();
    } else if (pylonDiagLogsEnabled() && ((nowUs - s_lastCacheBuildDiagUs) >= 1000000LL)) {
        ESP_LOGW(EXAMPLE_TAG, "PYLON synthetic source failed to build 0x63 from battery model");
        s_lastCacheBuildDiagUs = nowUs;
    }

    if (pylonDiagLogsEnabled() && ((nowUs - s_lastCacheBuildDiagUs) >= 1000000LL)) {
        logSyntheticPayloadBuild("PYLON synthetic cache", &model, s_pylonCache.info61, s_pylonCache.info62, s_pylonCache.info63);
        ESP_LOGI(EXAMPLE_TAG,
                 "PYLON synthetic cache state: debugOverride=%s realV=%.2fV effV=%.2fV v61=%s v62=%s v63=%s len61=%u len62=%u len63=%u info63=[%s] flags(c=%u d=%u b=%u state=0x%02X) reason63=%s",
                 debugOverride ? "YES" : "NO",
                 (double)realModel.packVoltageV,
                 (double)model.packVoltageV,
                 s_pylonCache.valid61 ? "YES" : "NO",
                 s_pylonCache.valid62 ? "YES" : "NO",
                 s_pylonCache.valid63 ? "YES" : "NO",
                 (unsigned)strlen(s_pylonCache.info61),
                 (unsigned)strlen(s_pylonCache.info62),
                 (unsigned)strlen(s_pylonCache.info63),
                 s_pylonCache.valid63 ? s_pylonCache.info63 : "",
                 (unsigned)(model.chargeEnabled ? 1u : 0u),
                 (unsigned)(model.dischargeEnabled ? 1u : 0u),
                 (unsigned)(model.balanceEnabled ? 1u : 0u),
                 (unsigned)(model.protocolState & 0xFFu),
                 s_lastStatus63Reason);
        s_lastCacheBuildDiagUs = nowUs;
    }
}

static void telemetryFromSummary(void)
{
    bridgeTelemetrySnapshot_t snap = {0};
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    universal_battery_model_t model = {0};
    char iface[12] = {0};
    bool preferModelTelemetry = pylonSyntheticSourceModeEnabled(&settings) &&
                                !pylonSourceUsesNativePayloadEncoding(&settings);

    if (!s_pylonSummary.valid) {
        ESP_LOGD("PYLON_RS485", "[TELEM_FROM_SUMMARY] Summary invalid, clearing telemetry");
        bridgeSetTelemetrySnapshot(NULL);
        return;
    }

    if (settings.bms_port >= 1u && settings.bms_port <= 2u) {
        if (settings.bms_line == LINE_RS485) {
            snprintf(iface, sizeof(iface), "RS485_%u", (unsigned)settings.bms_port);
        } else {
            snprintf(iface, sizeof(iface), "CAN%u", (unsigned)settings.bms_port);
        }
    }

    if (preferModelTelemetry) {
        batteryModelGet(&model);
    }

    snap.valid = true;
    if (iface[0] != '\0') {
        snprintf(snap.source, sizeof(snap.source), "%s", iface);
    } else {
        snprintf(snap.source, sizeof(snap.source), "BMS");
    }
    snprintf(snap.protocol, sizeof(snap.protocol), "%s", protocolToStrLocal(settings.bms_protocol));
    snap.currentA = (preferModelTelemetry && model.valid) ? model.packCurrentA : s_pylonSummary.current_a;
    snap.packVoltageV = (preferModelTelemetry && model.valid)
                        ? model.packVoltageV
                        : (float)s_pylonSummary.pack_voltage_cv / 100.0f;
    snap.cycles = s_pylonSummary.cycles;
    snap.socPct = s_pylonSummary.soc_pct;
    snap.sohPct = s_pylonSummary.soh_pct;
    snap.cellMaxV = (float)s_pylonSummary.max_cell_mv / 1000.0f;
    snap.cellMinV = (float)s_pylonSummary.min_cell_mv / 1000.0f;
    snap.cellMaxIdx = s_pylonSummary.max_cell_idx;
    snap.cellMinIdx = s_pylonSummary.min_cell_idx;
    snap.deltaV = snap.cellMaxV - snap.cellMinV;
    snap.tempMosC = (float)s_pylonSummary.temp_mos_c10 / 10.0f;
    snap.tempT1C = (float)s_pylonSummary.temp_t1_c10 / 10.0f;
    snap.tempT2C = (float)s_pylonSummary.temp_t2_c10 / 10.0f;
    snap.tempT4C = (float)s_pylonSummary.temp_t4_c10 / 10.0f;
    snap.tempT5C = (float)s_pylonSummary.temp_t5_c10 / 10.0f;
    snap.pylonStatus63 = s_pylonSummary.status_63;

    ESP_LOGI("PYLON_RS485", "[TELEM_FROM_SUMMARY] Setting telemetry from RS485 summary: "
             "valid=%s, soc=%u%%, v=%.2fV (cv=%u), i=%.1fA, preferModel=%s, modelV=%.2fV, flags(c=%u d=%u b=%u state=0x%02X) status63=0x%02X raw61=%s raw63=%s",
             snap.valid ? "YES" : "NO", snap.socPct, (double)snap.packVoltageV,
             s_pylonSummary.pack_voltage_cv, (double)snap.currentA,
             preferModelTelemetry ? "YES" : "NO",
             (double)(model.valid ? model.packVoltageV : 0.0f),
             (unsigned)(model.chargeEnabled ? 1u : 0u),
             (unsigned)(model.dischargeEnabled ? 1u : 0u),
             (unsigned)(model.balanceEnabled ? 1u : 0u),
             (unsigned)(model.protocolState & 0xFFu),
             (unsigned)s_pylonSummary.status_63,
             s_pylonCache.valid61 ? s_pylonCache.info61 : "",
             s_pylonCache.valid63 ? s_pylonCache.info63 : "");

    bridgeSetTelemetrySnapshot(&snap);
}

static void updateDecodedLogSnapshot(void)
{
    int64_t nowS = esp_timer_get_time() / 1000000LL;

    snprintf(s_pylonDecodedLog,
             sizeof(s_pylonDecodedLog),
             "BMS Decoded Logs\n"
             "Updated: %lld s uptime\n\n"
             "Pylon 0x61\n"
             "  valid : %s\n"
             "  pack  : I~=%.2fA  SOC~=%u%%  SOH?~=%u%%  cycles~=%u  w0=0x%04X\n"
             "  cells : max=%.3fV#%02u  min=%.3fV#%02u  dV=%.3fV\n"
             "  temps : MOS?=%.1fC  T1?=%.1fC  T2?=%.1fC  T4?=%.1fC  T5?=%.1fC\n"
             "  raw   : [%s]\n\n"
             "Pylon 0x62\n"
             "  valid : %s\n"
             "  raw   : [%s]\n\n"
             "Pylon 0x63\n"
             "  valid : %s\n"
             "  flags : status=0x%02X  likely(discharge=ON charge/balance=OFF)\n"
             "  raw   : [%s]\n",
             (long long)nowS,
             s_pylonCache.valid61 ? "YES" : "NO",
             (double)s_pylonSummary.current_a,
             (unsigned)s_pylonSummary.soc_pct,
             (unsigned)s_pylonSummary.soh_pct,
             (unsigned)s_pylonSummary.cycles,
             (unsigned)s_pylonSummary.raw_word0,
             (double)s_pylonSummary.max_cell_mv / 1000.0,
             (unsigned)s_pylonSummary.max_cell_idx,
             (double)s_pylonSummary.min_cell_mv / 1000.0,
             (unsigned)s_pylonSummary.min_cell_idx,
             ((double)s_pylonSummary.max_cell_mv - (double)s_pylonSummary.min_cell_mv) / 1000.0,
             (double)s_pylonSummary.temp_mos_c10 / 10.0,
             (double)s_pylonSummary.temp_t1_c10 / 10.0,
             (double)s_pylonSummary.temp_t2_c10 / 10.0,
             (double)s_pylonSummary.temp_t4_c10 / 10.0,
             (double)s_pylonSummary.temp_t5_c10 / 10.0,
             s_pylonCache.valid61 ? s_pylonCache.info61 : "",
             s_pylonCache.valid62 ? "YES" : "NO",
             s_pylonCache.valid62 ? s_pylonCache.info62 : "",
             s_pylonCache.valid63 ? "YES" : "NO",
             (unsigned)s_pylonSummary.status_63,
             s_pylonCache.valid63 ? s_pylonCache.info63 : "");

    bridgeSetDecodedLogSnapshot(s_pylonDecodedLog);
}

static void logPylonFrame(const char *prefix, const uint8_t *frame, int len)
{
    const int maxHexBytes = 64;
    const int maxAsciiBytes = 64;
    int hexBytes = (len < maxHexBytes) ? len : maxHexBytes;
    int asciiBytes = (len < maxAsciiBytes) ? len : maxAsciiBytes;
    char hex[3 * maxHexBytes + 1];
    char ascii[maxAsciiBytes + 1];
    int pos = 0;

    for (int i = 0; i < hexBytes; i++) {
        pos += snprintf(&hex[pos], sizeof(hex) - pos, "%02X ", frame[i]);
        if (pos >= (int)sizeof(hex)) break;
    }
    if (pos > 0) hex[pos - 1] = 0;
    else hex[0] = 0;

    for (int i = 0; i < asciiBytes; i++) {
        uint8_t ch = frame[i];
        ascii[i] = (char)((ch >= 32u && ch <= 126u) ? ch : '.');
    }
    ascii[asciiBytes] = 0;

    if (pylonDiagLogsEnabled()) {
        ESP_LOGI(EXAMPLE_TAG,
                 "%s: len=%d ASCII=[%s] HEX=[%s]%s",
                 prefix,
                 len,
                 ascii,
                 hex,
                 (len > maxHexBytes) ? " ..." : "");
    }
}

static void forwardFrame(const char *rxName,
                         const char *txName,
                         uart_port_t txUart,
                         gpio_num_t txDirPin,
                         const uint8_t *frame,
                         int len)
{
    if (frame == NULL || len <= 0) return;

    logPylonFrame(rxName && txName ? "RS485 FWD" : "RS485 TX", frame, len);
    if (!rs485PortUsesHalfDuplex(txUart)) {
        TickType_t preDelay = rs485PortPreDelayTicks(txUart);
        rs485SetTxForPort(txUart, txDirPin, true);
        if (preDelay > 0) vTaskDelay(preDelay);
    }
    int written = uart_write_bytes(txUart, (const char *)frame, len);
    if (written != len) {
        ESP_LOGW(EXAMPLE_TAG,
                 "RS485 TX short write on %s: wrote=%d expected=%d",
                 txName ? txName : "RS485",
                 written,
                 len);
    }
    esp_err_t waitErr = uart_wait_tx_done(txUart, rs485TxTimeoutTicksForFrameLen(len));
    if (waitErr != ESP_OK) {
        ESP_LOGW(EXAMPLE_TAG,
                 "RS485 TX wait timeout/error on %s: err=%d len=%d",
                 txName ? txName : "RS485",
                 (int)waitErr,
                 len);
    }
    if (!rs485PortUsesHalfDuplex(txUart)) {
        TickType_t postDelay = rs485PortPostDelayTicks(txUart);
        if (postDelay > 0) vTaskDelay(postDelay);
        rs485SetTxForPort(txUart, txDirPin, false);
    }

    if (pylonDiagLogsEnabled() && rxName != NULL && txName != NULL) {
        ESP_LOGI(EXAMPLE_TAG,
                 "RS485 FWD %s -> %s complete",
                 rxName,
                 txName);
    }
}

static void logDecodedPylon(const uint8_t *frame, int len)
{
    uint8_t ver = 0, adr = 0, cid1 = 0, cid2 = 0;
    int infoAsciiLen = len - 18;
    const char *payload = (const char *)&frame[13];

    if (!parsePylonHeader(frame, len, &ver, &adr, &cid1, &cid2)) return;

    if (pylonDiagLogsEnabled()) {
        ESP_LOGI(EXAMPLE_TAG,
                 "PYLON: ver=0x%02X addr=0x%02X cid1=0x%02X cid2=0x%02X infoLen=0x%04X payloadHexLen=%d chk=OK",
                 (unsigned)ver,
                 (unsigned)adr,
                 (unsigned)cid1,
                 (unsigned)cid2,
                 (unsigned)pylonLengthField(infoAsciiLen),
                 infoAsciiLen);
        if (infoAsciiLen > 0) {
            ESP_LOGI(EXAMPLE_TAG, "  payload=[%.*s]", infoAsciiLen, payload);
        }
    }
}

static void updateSummary61(void)
{
    uint8_t bytes[80];
    int n = parseHexAsciiPayload(s_pylonCache.info61, bytes, (int)sizeof(bytes));

    if (!s_pylonCache.valid61 || n < 33) {
        return;
    }

    s_pylonSummary.valid = true;
    s_pylonSummary.raw_word0 = be16(&bytes[0]);
    s_pylonSummary.pack_voltage_cv = be16(&bytes[0]);
    s_pylonSummary.current_a = (float)be16s(&bytes[2]) / 100.0f;
    s_pylonSummary.soc_pct = bytes[4];
    s_pylonSummary.cycles = be16(&bytes[5]);
    s_pylonSummary.soh_pct = bytes[9];
    s_pylonSummary.max_cell_mv = be16(&bytes[11]);
    s_pylonSummary.max_cell_idx = (uint8_t)be16(&bytes[13]);
    s_pylonSummary.min_cell_mv = be16(&bytes[15]);
    s_pylonSummary.min_cell_idx = (uint8_t)be16(&bytes[17]);
    s_pylonSummary.temp_mos_c10 = (int16_t)(be16(&bytes[19]) - 2731);
    s_pylonSummary.temp_t1_c10 = (int16_t)(be16(&bytes[21]) - 2731);
    s_pylonSummary.temp_t2_c10 = (int16_t)(be16(&bytes[25]) - 2731);
    s_pylonSummary.temp_t4_c10 = (int16_t)(be16(&bytes[29]) - 2731);
    s_pylonSummary.temp_t5_c10 = (int16_t)(be16(&bytes[31]) - 2731);

    if (pylonDiagLogsEnabled()) {
        ESP_LOGI(EXAMPLE_TAG, "RS485 PYLON 0x61");
        ESP_LOGI(EXAMPLE_TAG,
                 "  pack : V~=%.2fV I~=%.2fA  SOC~=%u%%  SOH?~=%u%%  cycles~=%u  w0=0x%04X",
                 (double)s_pylonSummary.pack_voltage_cv / 100.0,
                 (double)s_pylonSummary.current_a,
                 (unsigned)s_pylonSummary.soc_pct,
                 (unsigned)s_pylonSummary.soh_pct,
                 (unsigned)s_pylonSummary.cycles,
                 (unsigned)s_pylonSummary.raw_word0);
        ESP_LOGI(EXAMPLE_TAG,
                 "  cells: max=%.3fV#%02u  min=%.3fV#%02u  dV=%.3fV",
                 (double)s_pylonSummary.max_cell_mv / 1000.0,
                 (unsigned)s_pylonSummary.max_cell_idx,
                 (double)s_pylonSummary.min_cell_mv / 1000.0,
                 (unsigned)s_pylonSummary.min_cell_idx,
                 ((double)s_pylonSummary.max_cell_mv - (double)s_pylonSummary.min_cell_mv) / 1000.0);
        ESP_LOGI(EXAMPLE_TAG,
                 "  temps: MOS?=%.1fC  T1?=%.1fC  T2?=%.1fC  T4?=%.1fC  T5?=%.1fC",
                 (double)s_pylonSummary.temp_mos_c10 / 10.0,
                 (double)s_pylonSummary.temp_t1_c10 / 10.0,
                 (double)s_pylonSummary.temp_t2_c10 / 10.0,
                 (double)s_pylonSummary.temp_t4_c10 / 10.0,
                 (double)s_pylonSummary.temp_t5_c10 / 10.0);
        ESP_LOGI(EXAMPLE_TAG, "  raw  : [%s]", s_pylonCache.info61);
    }
    updateDecodedLogSnapshot();
}

static void updateSummary63(void)
{
    uint8_t bytes[32];
    int n = parseHexAsciiPayload(s_pylonCache.info63, bytes, (int)sizeof(bytes));

    if (!s_pylonCache.valid63 || n < 9) {
        return;
    }

    s_pylonSummary.status_63 = bytes[8];
    if (pylonDiagLogsEnabled()) {
        ESP_LOGI(EXAMPLE_TAG, "RS485 PYLON 0x63");
        ESP_LOGI(EXAMPLE_TAG,
                 "  flags: status=0x%02X  likely(discharge=ON charge/balance=OFF)",
                 (unsigned)s_pylonSummary.status_63);
        ESP_LOGI(EXAMPLE_TAG, "  raw  : [%s]", s_pylonCache.info63);
    }
    updateDecodedLogSnapshot();
}

static void cacheResponse(uint8_t requestedCid2, const uint8_t *frame, int len)
{
    int payloadLen = len - 18;
    const uint8_t *payload = &frame[13];

    if (payloadLen <= 0) return;

    switch (requestedCid2) {
        case 0x61:
            if (payloadLen >= (int)sizeof(s_pylonCache.info61)) payloadLen = (int)sizeof(s_pylonCache.info61) - 1;
            memcpy(s_pylonCache.info61, payload, (size_t)payloadLen);
            s_pylonCache.info61[payloadLen] = '\0';
            s_pylonCache.valid61 = true;
            updateSummary61();
            telemetryFromSummary();
            break;
        case 0x62:
            if (payloadLen >= (int)sizeof(s_pylonCache.info62)) payloadLen = (int)sizeof(s_pylonCache.info62) - 1;
            memcpy(s_pylonCache.info62, payload, (size_t)payloadLen);
            s_pylonCache.info62[payloadLen] = '\0';
            s_pylonCache.valid62 = true;
            if (pylonDiagLogsEnabled()) {
                ESP_LOGI(EXAMPLE_TAG, "RS485 PYLON 0x62");
                ESP_LOGI(EXAMPLE_TAG, "  raw  : [%s]", s_pylonCache.info62);
            }
            updateDecodedLogSnapshot();
            break;
        case 0x63:
            if (payloadLen >= (int)sizeof(s_pylonCache.info63)) payloadLen = (int)sizeof(s_pylonCache.info63) - 1;
            memcpy(s_pylonCache.info63, payload, (size_t)payloadLen);
            s_pylonCache.info63[payloadLen] = '\0';
            s_pylonCache.valid63 = true;
            updateSummary63();
            telemetryFromSummary();
            break;
        default:
            break;
    }
}

static bool buildCachedResponse(const uint8_t *request,
                                int requestLen,
                                uint8_t *response,
                                int responseSize,
                                int *outLen,
                                uint8_t *outCid2,
                                uint8_t *outAdr)
{
    uint8_t ver = 0, adr = 0, cid1 = 0, cid2 = 0;
    const char *infoAscii = NULL;
    char body[192];
    uint16_t checksum;
    uint16_t lengthField;
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    bool debugOverride = batteryModelIsDebugOverrideEnabled();

    if (!parsePylonHeader(request, requestLen, &ver, &adr, &cid1, &cid2) || cid1 != 0x46) {
        return false;
    }

    bool sourceFresh = pylonBmsSourceFresh(&settings);
    bool forceFake = pylonSyntheticSourceModeEnabled(&settings) && PYLON_CAN_RS485_FORCE_FAKE_ENABLE;

    maybeRefreshSyntheticCacheFromUniversal();

    if (!sourceFresh && !forceFake) {
        ESP_LOGW(EXAMPLE_TAG,
                 "RS485 PYLON FAKE BMS skip: source not fresh for cid2=0x%02X addr=0x%02X",
                 (unsigned)cid2,
                 (unsigned)adr);
        return false;
    }

    switch (cid2) {
        case 0x61:
            if (!s_pylonCache.valid61) {
                ESP_LOGW(EXAMPLE_TAG, "RS485 PYLON FAKE BMS skip: missing cache 0x61");
                return false;
            }
            infoAscii = s_pylonCache.info61;
            break;
        case 0x62:
            if (!s_pylonCache.valid62) {
                ESP_LOGW(EXAMPLE_TAG, "RS485 PYLON FAKE BMS skip: missing cache 0x62");
                return false;
            }
            infoAscii = s_pylonCache.info62;
            break;
        case 0x63:
            if (!s_pylonCache.valid63) {
                ESP_LOGW(EXAMPLE_TAG, "RS485 PYLON FAKE BMS skip: missing cache 0x63");
                return false;
            }
            infoAscii = s_pylonCache.info63;
            break;
        default:
            ESP_LOGW(EXAMPLE_TAG, "RS485 PYLON FAKE BMS skip: unsupported cid2=0x%02X", (unsigned)cid2);
            return false;
    }

    lengthField = pylonLengthField((int)strlen(infoAscii));
    snprintf(body, sizeof(body), "%02X%02X46%02X%04X%s", ver, adr, 0x00, lengthField, infoAscii);
    checksum = pylonAsciiChecksum(body);
    *outLen = snprintf((char *)response, (size_t)responseSize, "~%s%04X\r", body, checksum);
    if (*outLen <= 0 || *outLen >= responseSize) {
        return false;
    }

    if (pylonDiagLogsEnabled()) {
        ESP_LOGI(EXAMPLE_TAG,
                 "PYLON cached response build: req cid2=0x%02X addr=0x%02X debugOverride=%s sourceFresh=%s forceFake=%s reason63=%s info=[%s] response=[%s]",
                 (unsigned)cid2,
                 (unsigned)adr,
                 debugOverride ? "YES" : "NO",
                 sourceFresh ? "YES" : "NO",
                 forceFake ? "YES" : "NO",
                 s_lastStatus63Reason,
                 infoAscii,
                 response);
    }

    if (outCid2) *outCid2 = cid2;
    if (outAdr) *outAdr = adr;
    return true;
}

static void sendCachedResponse(const pylonRs485BridgeCtx_t *ctx, const uint8_t *request, int requestLen)
{
    uint8_t response[224];
    uint8_t cid2 = 0;
    uint8_t adr = 0;
    int outLen = 0;

    if (!ctx->isInverterSide) return;

    if (!buildCachedResponse(request, requestLen, response, sizeof(response), &outLen, &cid2, &adr)) {
        return;
    }

    if (pylonDiagLogsEnabled()) {
        ESP_LOGI(EXAMPLE_TAG,
                 "RS485 PYLON FAKE BMS on %s: req cid2=0x%02X addr=0x%02X -> rtn=0x00 len=%d reason=cache-or-fallback",
                 ctx->rxName,
                 (unsigned)cid2,
                 (unsigned)adr,
                 outLen);
        logPylonFrame("RS485 FWD PYLON_FAKE_BMS -> inverter", response, outLen);
    }
    forwardFrame("PYLON_FAKE_BMS", ctx->rxName, ctx->rxUart, ctx->rxDirPin, response, outLen);
}

static bool maybeHandleProbeResponse(const pylonRs485BridgeCtx_t *ctx, const uint8_t *frame, int len)
{
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    uint8_t ver = 0, adr = 0, cid1 = 0, code = 0;

    if (!ctx->isBmsSide || !pylonProbeModeEnabled(settings.mode)) return false;
    if (!s_probePending.active) return false;
    if (!parsePylonHeader(frame, len, &ver, &adr, &cid1, &code)) return false;
    if (cid1 != 0x46) return false;
    if (adr != s_probePending.adr) return false;
    if (code != 0x00 && code != 0x90 && code != 0x91) return false;

    if (pylonDiagLogsEnabled()) {
        ESP_LOGI(EXAMPLE_TAG,
                 "RS485 PYLON PROBE RX on %s: ver=0x%02X addr=0x%02X cid1=0x%02X code=0x%02X len=%d",
                 ctx->rxName,
                 (unsigned)ver,
                 (unsigned)adr,
                 (unsigned)cid1,
                 (unsigned)code,
                 len);
    }
    cacheResponse(s_probePending.cid2, frame, len);
    if (pylonDiagLogsEnabled()) {
        ESP_LOGI(EXAMPLE_TAG,
                 "RS485 PYLON CACHE update from %s: reqCid2=0x%02X respAddr=0x%02X len=%d",
                 ctx->rxName,
                 (unsigned)s_probePending.cid2,
                 (unsigned)adr,
                 len);
    }
    s_probePending.active = false;
    return true;
}

static void maybeCheckProbeTimeout(const pylonRs485BridgeCtx_t *ctx, int64_t nowUs)
{
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    if (!ctx->isBmsSide || !pylonProbeModeEnabled(settings.mode)) return;
    if (!s_probePending.active || nowUs < s_probePending.dueUs) return;

    if (pylonDiagLogsEnabled()) {
        ESP_LOGI(EXAMPLE_TAG,
                 "RS485 PYLON PROBE timeout on %s: cid2=0x%02X addr=0x%02X reason=no-response",
                 ctx->rxName,
                 (unsigned)s_probePending.cid2,
                 (unsigned)s_probePending.adr);
    }
    s_probePending.active = false;
}

static void maybeHandleForwardDecode(const pylonRs485BridgeCtx_t *ctx,
                                     const uint8_t *frame,
                                     int len,
                                     int64_t nowUs)
{
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    uint8_t ver = 0, adr = 0, cid1 = 0, code = 0;

    if (settings.mode != MODE_FORWARD) return;
    if (!parsePylonHeader(frame, len, &ver, &adr, &cid1, &code)) return;
    if (cid1 != 0x46) return;

    if (ctx->isInverterSide && (code == 0x61 || code == 0x62 || code == 0x63)) {
        s_forwardPending.valid = true;
        s_forwardPending.cid2 = code;
        s_forwardPending.reqAdr = adr;
        s_forwardPending.seenUs = nowUs;
        return;
    }

    if (ctx->isBmsSide && code == 0x00 && s_forwardPending.valid) {
        if ((nowUs - s_forwardPending.seenUs) <= 1000000LL) {
            cacheResponse(s_forwardPending.cid2, frame, len);
        }
        s_forwardPending.valid = false;
    }
}

static void pylonBridgeTask(void *pv)
{
    pylonRs485BridgeCtx_t *ctx = (pylonRs485BridgeCtx_t *)pv;
    uint8_t rxChunk[RS485_BUF_SIZE];
    uint8_t frameBuf[256];
    uint16_t frameLen = 0;
    bool haveFrame = false;
    int64_t lastByteUs = 0;
    const int64_t gapUs = 5000;

    while (1) {
        int len = uart_read_bytes(ctx->rxUart, rxChunk, RS485_BUF_SIZE, pdMS_TO_TICKS(5));
        int64_t nowUs = esp_timer_get_time();
        bool consumedByProbe = false;

        maybeCheckProbeTimeout(ctx, nowUs);

        if (len > 0) {
            if (haveFrame && (nowUs - lastByteUs) > gapUs) {
                frameLen = 0;
                haveFrame = false;
            }

            if ((size_t)frameLen + (size_t)len > sizeof(frameBuf)) {
                frameLen = 0;
                haveFrame = false;
            }

            if ((size_t)frameLen + (size_t)len <= sizeof(frameBuf)) {
                memcpy(&frameBuf[frameLen], rxChunk, (size_t)len);
                frameLen = (uint16_t)(frameLen + len);
                lastByteUs = nowUs;
                haveFrame = true;
            }

            if (haveFrame && frameLen > 0 && frameBuf[frameLen - 1] == '\r') {
                bridge_runtime_settings_t settings = runtimeSettingsGet();
                if (ctx->isBmsSide) {
                    s_lastPylonBmsTrafficUs = nowUs;
                }
                if (ctx->isInverterSide) {
                    s_lastPylonInverterTrafficUs = nowUs;
                }
                logDecodedPylon(frameBuf, frameLen);
                maybeHandleForwardDecode(ctx, frameBuf, frameLen, nowUs);
                consumedByProbe = maybeHandleProbeResponse(ctx, frameBuf, frameLen);

                if (settings.mode == MODE_FORWARD) {
                    if (!consumedByProbe) {
                        forwardFrame(ctx->rxName, ctx->txName, ctx->txUart, ctx->txDirPin, frameBuf, frameLen);
                    }
                } else if (settings.mode == MODE_BRIDGE) {
                    if (ctx->isInverterSide) {
                        sendCachedResponse(ctx, frameBuf, frameLen);
                    }
                }

                frameLen = 0;
                haveFrame = false;
            }
        } else if (haveFrame && (nowUs - lastByteUs) > gapUs) {
            bridge_runtime_settings_t settings = runtimeSettingsGet();
            if (settings.mode == MODE_FORWARD && frameLen > 0) {
                forwardFrame(ctx->rxName, ctx->txName, ctx->txUart, ctx->txDirPin, frameBuf, frameLen);
            }
            frameLen = 0;
            haveFrame = false;
        }
    }
}

static void pylonProbeTask(void *pv)
{
    static const uint8_t addresses[] = {0x02};
    static const uint8_t cid2Seq[] = {0x61, 0x63, 0x62};
    pylonProbeTaskCtx_t *ctx = (pylonProbeTaskCtx_t *)pv;
    size_t addrIdx = 0;
    size_t cidIdx = 0;
    uint8_t frame[32];
    int frameLen = 0;

    while (1) {
        uint8_t adr = addresses[addrIdx];
        uint8_t cid2 = cid2Seq[cidIdx];
        int64_t nowUs = esp_timer_get_time();
        int64_t latestTrafficUs = s_lastPylonBmsTrafficUs;

        if (s_lastPylonInverterTrafficUs > latestTrafficUs) {
            latestTrafficUs = s_lastPylonInverterTrafficUs;
        }

        if (latestTrafficUs != 0 && (nowUs - latestTrafficUs) < 1500000LL) {
            vTaskDelay(pdMS_TO_TICKS(400));
            continue;
        }

        if (pylonBuildRequest(0x20, adr, cid2, frame, sizeof(frame), &frameLen)) {
            s_probePending.active = true;
            s_probePending.adr = adr;
            s_probePending.cid2 = cid2;
            s_probePending.dueUs = nowUs + 250000;
            if (pylonDiagLogsEnabled()) {
                ESP_LOGI(EXAMPLE_TAG,
                         "RS485 FWD PYLON_PROBE -> %s: len=%d ASCII=[%.*s]",
                         ctx->probeName,
                         frameLen,
                         frameLen,
                         frame);
            }
            forwardFrame("PYLON_PROBE", ctx->probeName, ctx->probeUart, ctx->probeDirPin, frame, frameLen);
        }

        addrIdx = (addrIdx + 1u) % (sizeof(addresses) / sizeof(addresses[0]));
        if (addrIdx == 0u) {
            cidIdx = (cidIdx + 1u) % (sizeof(cid2Seq) / sizeof(cid2Seq[0]));
        }
        vTaskDelay(pdMS_TO_TICKS(800));
    }
}

bool pylonRs485BridgeSupportsRoute(const bridge_runtime_settings_t *settings)
{
    if (settings == NULL) {
        return false;
    }

    return ((settings->bms_line == LINE_RS485) &&
            (settings->inverter_line == LINE_RS485) &&
            (settings->bms_protocol == PROTOCOL_RS485_PYLON) &&
            (settings->inverter_protocol == PROTOCOL_RS485_PYLON)) ||
           pylonCanToRs485ModeEnabled(settings);
}

bool pylonRs485BridgeHandlesCurrentConfig(void)
{
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    return pylonRs485BridgeSupportsRoute(&settings);
}

void pylonRs485BridgeEnable(void)
{
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    static pylonRs485BridgeCtx_t bmsCtx;
    static pylonRs485BridgeCtx_t inverterCtx;
    static pylonProbeTaskCtx_t probeCtx;

    memset(&s_pylonCache, 0, sizeof(s_pylonCache));
    memset(&s_pylonSummary, 0, sizeof(s_pylonSummary));
    memset(&s_probePending, 0, sizeof(s_probePending));
    memset(&s_forwardPending, 0, sizeof(s_forwardPending));
    s_lastPylonBmsTrafficUs = 0;
    s_lastPylonInverterTrafficUs = 0;
    snprintf(s_pylonCache.info62, sizeof(s_pylonCache.info62), "00000000");
    s_pylonCache.valid62 = true;
    bridgeSetTelemetrySnapshot(NULL);
    bridgeSetDecodedLogSnapshot("");

    deleteTaskIfRunning(&s_pylonBmsTask);
    deleteTaskIfRunning(&s_pylonInvTask);
    deleteTaskIfRunning(&s_pylonProbeTaskHandle);

    bmsCtx.rxName = (settings.bms_port == 1) ? "RS485_1" : "RS485_2";
    bmsCtx.txName = (settings.inverter_port == 1) ? "RS485_1" : "RS485_2";
    bmsCtx.rxUart = (settings.bms_port == 1) ? rs485GetUart1() : rs485GetUart2();
    bmsCtx.txUart = (settings.inverter_port == 1) ? rs485GetUart1() : rs485GetUart2();
    bmsCtx.rxDirPin = (settings.bms_port == 1) ? rs485GetDir1() : rs485GetDir2();
    bmsCtx.txDirPin = (settings.inverter_port == 1) ? rs485GetDir1() : rs485GetDir2();
    bmsCtx.isBmsSide = true;
    bmsCtx.isInverterSide = false;

    inverterCtx.rxName = (settings.inverter_port == 1) ? "RS485_1" : "RS485_2";
    inverterCtx.txName = (settings.bms_port == 1) ? "RS485_1" : "RS485_2";
    inverterCtx.rxUart = (settings.inverter_port == 1) ? rs485GetUart1() : rs485GetUart2();
    inverterCtx.txUart = (settings.bms_port == 1) ? rs485GetUart1() : rs485GetUart2();
    inverterCtx.rxDirPin = (settings.inverter_port == 1) ? rs485GetDir1() : rs485GetDir2();
    inverterCtx.txDirPin = (settings.bms_port == 1) ? rs485GetDir1() : rs485GetDir2();
    inverterCtx.isBmsSide = false;
    inverterCtx.isInverterSide = true;

    if (pylonRs485PassthroughModeEnabled(&settings)) {
        xTaskCreate(pylonBridgeTask, "pylon_bms_rx", 6144, &bmsCtx, 9, &s_pylonBmsTask);
    }
    xTaskCreate(pylonBridgeTask, "pylon_inv_rx", 6144, &inverterCtx, 9, &s_pylonInvTask);

    if (pylonRs485PassthroughModeEnabled(&settings) && pylonProbeModeEnabled(settings.mode)) {
        probeCtx.probeName = bmsCtx.rxName;
        probeCtx.probeUart = bmsCtx.rxUart;
        probeCtx.probeDirPin = bmsCtx.rxDirPin;
        xTaskCreate(pylonProbeTask, "pylon_probe", 4096, &probeCtx, 8, &s_pylonProbeTaskHandle);
        ESP_LOGI(EXAMPLE_TAG, "Pylon active probe enabled on %s (mode=%d)", probeCtx.probeName, settings.mode);
    }

    maybeRefreshSyntheticCacheFromUniversal();
    if (pylonCanToRs485ModeEnabled(&settings)) {
        ESP_LOGI(EXAMPLE_TAG, "Pylon CAN->RS485 translator armed (BMS=CAN_PYLON inverter=RS485_PYLON)");
        if (settings.mode != MODE_BRIDGE) {
            ESP_LOGW(EXAMPLE_TAG, "Pylon CAN->RS485 translation requires Mode=bridge; forward will not answer inverter requests");
        }
    } else if (pylonSyntheticSourceModeEnabled(&settings)) {
        ESP_LOGI(EXAMPLE_TAG,
                 "Pylon synthetic responder armed (BMS protocol=%s inverter=RS485_PYLON)",
                 protocolToStrLocal(settings.bms_protocol));
    }

    ESP_LOGI(EXAMPLE_TAG,
             "Pylon RS485 mode enabled (mode=%d bms=%s[P%d] inverter=%s[P%d])",
             settings.mode,
             bmsCtx.rxName,
             settings.bms_port,
             inverterCtx.rxName,
             settings.inverter_port);
}

void pylonRs485BridgeStop(void)
{
    deleteTaskIfRunning(&s_pylonBmsTask);
    deleteTaskIfRunning(&s_pylonInvTask);
    deleteTaskIfRunning(&s_pylonProbeTaskHandle);
    memset(&s_forwardPending, 0, sizeof(s_forwardPending));
    bridgeSetTelemetrySnapshot(NULL);
    bridgeSetDecodedLogSnapshot("");
}
