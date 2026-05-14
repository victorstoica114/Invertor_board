#include "protocols/seplos_rs485/seplos_rs485_bms_task.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "Drivers/rs485_driver.h"
#include "config.h"
#include "protocols/common/battery_model.h"
#include "runtime_settings.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    enum {
        SEPLOS_POLL_KIND_ASCII = 0,
        SEPLOS_POLL_KIND_MODBUS_RTU_V3,
    } kind;
    uint8_t protocolVersion;
    uint8_t address;
    uint8_t requestInfo;
    seplos_rs485_request_style_t style;
    bool invertSignals;
    const char *name;
} seplosPollCandidate_t;

typedef struct {
    QueueHandle_t outQueue;
    uint32_t sequence;
    int64_t lastPollUs;
    int64_t lastFrameUs;
    uint8_t nextCid2;
    uint8_t lastRequestedCid2;
    uint8_t candidateIndex;
    uint8_t lastPolledCandidate;
    bool candidateLocked;
    bool lineInverted;
    bool lastPollWasModbus;
    uint8_t lastModbusAddress;
    uint8_t lastModbusFunction;
    uint16_t lastModbusRegister;
    uint16_t lastModbusCount;
    uint32_t pollCount;
    uint8_t rxBuf[SEPLOS_RS485_MAX_FRAME_LEN];
    size_t rxLen;
    seplos_rs485_snapshot_t workingSnapshot;
} seplosRs485BmsTaskCtx_t;

static const seplosPollCandidate_t kSeplosPollCandidates[] = {
    {SEPLOS_POLL_KIND_ASCII, 0x20u, SEPLOS_BMS_ADDRESS, SEPLOS_BMS_REQUEST_INFO, SEPLOS_RS485_REQUEST_STYLE_LEN_CHECK, false, "v20-default-p00"},
    {SEPLOS_POLL_KIND_ASCII, 0x20u, SEPLOS_BMS_ADDRESS, SEPLOS_BMS_REQUEST_INFO, SEPLOS_RS485_REQUEST_STYLE_LEN_CHECK, true, "v20-default-p00-inv"},
    {SEPLOS_POLL_KIND_ASCII, 0x20u, 0x00u, 0x01u, SEPLOS_RS485_REQUEST_STYLE_LEN_CHECK, false, "v20-a00-p01"},
    {SEPLOS_POLL_KIND_ASCII, 0x20u, 0x00u, 0x01u, SEPLOS_RS485_REQUEST_STYLE_LEN_CHECK, true, "v20-a00-p01-inv"},
    {SEPLOS_POLL_KIND_ASCII, 0x20u, 0x01u, 0x01u, SEPLOS_RS485_REQUEST_STYLE_LEN_CHECK, false, "v20-a01-p01"},
    {SEPLOS_POLL_KIND_ASCII, 0x20u, 0x01u, 0x01u, SEPLOS_RS485_REQUEST_STYLE_LEN_CHECK, true, "v20-a01-p01-inv"},
    {SEPLOS_POLL_KIND_ASCII, 0x26u, 0x00u, 0x01u, SEPLOS_RS485_REQUEST_STYLE_LEN_CHECK, false, "v26-a00-p01"},
    {SEPLOS_POLL_KIND_ASCII, 0x26u, 0x00u, 0x01u, SEPLOS_RS485_REQUEST_STYLE_LEN_CHECK, true, "v26-a00-p01-inv"},
    {SEPLOS_POLL_KIND_ASCII, 0x20u, 0x00u, 0x01u, SEPLOS_RS485_REQUEST_STYLE_SIMPLE_LEN_NIBBLE, false, "simple-v20-a00-p1"},
    {SEPLOS_POLL_KIND_ASCII, 0x20u, 0x00u, 0x01u, SEPLOS_RS485_REQUEST_STYLE_SIMPLE_LEN_NIBBLE, true, "simple-v20-a00-p1-inv"},
    {SEPLOS_POLL_KIND_ASCII, 0x20u, 0x01u, 0x01u, SEPLOS_RS485_REQUEST_STYLE_SIMPLE_LEN_NIBBLE, false, "simple-v20-a01-p1"},
    {SEPLOS_POLL_KIND_ASCII, 0x20u, 0x01u, 0x01u, SEPLOS_RS485_REQUEST_STYLE_SIMPLE_LEN_NIBBLE, true, "simple-v20-a01-p1-inv"},
    {SEPLOS_POLL_KIND_MODBUS_RTU_V3, 0x00u, 0x00u, 0x00u, SEPLOS_RS485_REQUEST_STYLE_LEN_CHECK, false, "rtu-v3-id00"},
    {SEPLOS_POLL_KIND_MODBUS_RTU_V3, 0x00u, 0x00u, 0x00u, SEPLOS_RS485_REQUEST_STYLE_LEN_CHECK, true, "rtu-v3-id00-inv"},
    {SEPLOS_POLL_KIND_MODBUS_RTU_V3, 0x00u, 0x01u, 0x00u, SEPLOS_RS485_REQUEST_STYLE_LEN_CHECK, false, "rtu-v3-id01"},
    {SEPLOS_POLL_KIND_MODBUS_RTU_V3, 0x00u, 0x01u, 0x00u, SEPLOS_RS485_REQUEST_STYLE_LEN_CHECK, true, "rtu-v3-id01-inv"},
};

static seplosRs485BmsTaskCtx_t g_seplosRs485BmsCtx;
static TaskHandle_t g_seplosRs485BmsTaskHandle;
static portMUX_TYPE g_latestMux = portMUX_INITIALIZER_UNLOCKED;
static bool g_haveLatestPacket;
static bms_decoded_packet_t g_latestPacket;
static bool g_haveLatestSnapshot;
static seplos_rs485_snapshot_t g_latestSnapshot;
static int64_t g_lastSourceStaleLogUs;

static size_t seplosCandidateCount(void)
{
    return sizeof(kSeplosPollCandidates) / sizeof(kSeplosPollCandidates[0]);
}

static void logRawBytes(const char *label, const uint8_t *data, size_t len)
{
#if SEPLOS_BMS_RAW_LOGS
    char ascii[161];
    char hex[145];
    size_t asciiLen = (len < sizeof(ascii) - 1u) ? len : sizeof(ascii) - 1u;
    size_t hexBytes = (len < 48u) ? len : 48u;
    size_t hexPos = 0u;

    if (label == NULL || data == NULL || len == 0u) {
        return;
    }

    for (size_t i = 0u; i < asciiLen; i++) {
        uint8_t ch = data[i];
        if (ch >= 0x20u && ch <= 0x7Eu) {
            ascii[i] = (char)ch;
        } else if (ch == '\r') {
            ascii[i] = '|';
        } else {
            ascii[i] = '.';
        }
    }
    ascii[asciiLen] = '\0';

    for (size_t i = 0u; i < hexBytes && hexPos + 4u < sizeof(hex); i++) {
        int written = snprintf(hex + hexPos,
                               sizeof(hex) - hexPos,
                               "%s%02X",
                               (i == 0u) ? "" : " ",
                               (unsigned)data[i]);
        if (written <= 0) {
            break;
        }
        hexPos += (size_t)written;
    }
    hex[hexPos] = '\0';

    ESP_LOGI(EXAMPLE_TAG,
             "%s len=%u ascii=\"%s%s\" hex=%s%s",
             label,
             (unsigned)len,
             ascii,
             (asciiLen < len) ? "..." : "",
             hex,
             (hexBytes < len) ? " ..." : "");
#else
    (void)label;
    (void)data;
    (void)len;
#endif
}

static void lockPollCandidate(seplosRs485BmsTaskCtx_t *ctx)
{
    const size_t count = seplosCandidateCount();

    if (ctx == NULL || ctx->candidateLocked || ctx->lastPolledCandidate >= count) {
        return;
    }

    ctx->candidateIndex = ctx->lastPolledCandidate;
    ctx->candidateLocked = true;
    ESP_LOGI(EXAMPLE_TAG,
             "Seplos locked request candidate: %s ver=0x%02X addr=0x%02X info=0x%02X invert=%s",
             kSeplosPollCandidates[ctx->candidateIndex].name,
             (unsigned)kSeplosPollCandidates[ctx->candidateIndex].protocolVersion,
             (unsigned)kSeplosPollCandidates[ctx->candidateIndex].address,
             (unsigned)kSeplosPollCandidates[ctx->candidateIndex].requestInfo,
             kSeplosPollCandidates[ctx->candidateIndex].invertSignals ? "YES" : "NO");
}

static const seplosPollCandidate_t *selectPollCandidate(seplosRs485BmsTaskCtx_t *ctx)
{
    const size_t count = seplosCandidateCount();
    size_t index = 0u;

    if (ctx == NULL || count == 0u) {
        return NULL;
    }

    if (ctx->candidateLocked && ctx->candidateIndex < count) {
        index = ctx->candidateIndex;
    } else {
        uint32_t period = SEPLOS_BMS_REQUEST_CANDIDATE_PERIOD;
        if (period == 0u) {
            period = 1u;
        }
        index = (size_t)((ctx->pollCount / period) % (uint32_t)count);
        ctx->candidateIndex = (uint8_t)index;
    }

    return &kSeplosPollCandidates[index];
}

static void applyLineInversion(uart_port_t uart,
                               seplosRs485BmsTaskCtx_t *ctx,
                               const seplosPollCandidate_t *candidate)
{
    bool invert = false;
    uint32_t mask = UART_SIGNAL_INV_DISABLE;
    esp_err_t err = ESP_OK;

    if (ctx == NULL || candidate == NULL) {
        return;
    }

    invert = candidate->invertSignals;
    if (ctx->lineInverted == invert) {
        return;
    }

    if (invert) {
        mask = UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV;
    }
    err = uart_set_line_inverse(uart, mask);
    if (err != ESP_OK) {
        ESP_LOGW(EXAMPLE_TAG,
                 "Seplos UART inversion switch failed (invert=%s err=0x%x)",
                 invert ? "YES" : "NO",
                 (unsigned)err);
        return;
    }

    ctx->lineInverted = invert;
    ESP_LOGI(EXAMPLE_TAG, "Seplos UART TX/RX inversion %s", invert ? "enabled" : "disabled");
}

static uint16_t modbusCrc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;

    if (data == NULL) {
        return 0u;
    }

    for (size_t i = 0u; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0u; bit < 8u; bit++) {
            if ((crc & 0x0001u) != 0u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

static size_t buildModbusReadRequest(uint8_t address,
                                     uint8_t function,
                                     uint16_t startRegister,
                                     uint16_t count,
                                     uint8_t *out,
                                     size_t outSize)
{
    uint16_t crc = 0u;

    if (out == NULL || outSize < 8u) {
        return 0u;
    }

    out[0] = address;
    out[1] = function;
    out[2] = (uint8_t)((startRegister >> 8) & 0xFFu);
    out[3] = (uint8_t)(startRegister & 0xFFu);
    out[4] = (uint8_t)((count >> 8) & 0xFFu);
    out[5] = (uint8_t)(count & 0xFFu);
    crc = modbusCrc16(out, 6u);
    out[6] = (uint8_t)(crc & 0xFFu);
    out[7] = (uint8_t)((crc >> 8) & 0xFFu);

    return 8u;
}

static uint16_t be16u(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static int16_t be16i(const uint8_t *p)
{
    return (int16_t)be16u(p);
}

static void recomputeModbusCells(seplos_rs485_snapshot_t *snapshot)
{
    uint32_t sum = 0u;
    uint8_t counted = 0u;
    uint16_t minMv = UINT16_MAX;
    uint16_t maxMv = 0u;
    uint8_t minIdx = 0u;
    uint8_t maxIdx = 0u;

    if (snapshot == NULL || snapshot->cellCount == 0u) {
        return;
    }

    for (uint8_t i = 0u; i < snapshot->cellCount && i < SEPLOS_RS485_MAX_CELLS; i++) {
        uint16_t mv = snapshot->cellMv[i];
        if (mv == 0u) {
            continue;
        }
        sum += mv;
        counted++;
        if (mv < minMv) {
            minMv = mv;
            minIdx = (uint8_t)(i + 1u);
        }
        if (mv > maxMv) {
            maxMv = mv;
            maxIdx = (uint8_t)(i + 1u);
        }
    }

    if (counted == 0u) {
        return;
    }

    snapshot->hasCellExtremes = true;
    snapshot->minCellMv = minMv;
    snapshot->maxCellMv = maxMv;
    snapshot->minCellIndex = minIdx;
    snapshot->maxCellIndex = maxIdx;
    snapshot->hasCellAvgMv = true;
    snapshot->cellAvgMv = (uint16_t)((sum + (uint32_t)(counted / 2u)) / (uint32_t)counted);
    snapshot->hasCellDiffMv = true;
    snapshot->cellDiffMv = (uint16_t)(maxMv - minMv);
}

static bool applyModbusPia(seplos_rs485_snapshot_t *snapshot, const uint8_t *payload, size_t payloadLen)
{
    if (snapshot == NULL || payload == NULL || payloadLen < 36u) {
        return false;
    }

    snapshot->hasPackVoltageCv = true;
    snapshot->packVoltageCv = be16u(&payload[0]);
    snapshot->hasPackCurrentCa = true;
    snapshot->packCurrentCa = be16i(&payload[2]);
    snapshot->hasRemainingCapacityCah = true;
    snapshot->remainingCapacityCah = be16u(&payload[4]);
    snapshot->hasFullCapacityCah = true;
    snapshot->fullCapacityCah = be16u(&payload[6]);
    snapshot->hasSocDeciPct = true;
    snapshot->socDeciPct = be16u(&payload[10]);
    snapshot->hasSohDeciPct = true;
    snapshot->sohDeciPct = be16u(&payload[12]);
    snapshot->hasCycles = true;
    snapshot->cycles = be16u(&payload[14]);
    snapshot->hasCellAvgMv = true;
    snapshot->cellAvgMv = be16u(&payload[16]);
    snapshot->hasCellExtremes = true;
    snapshot->maxCellMv = be16u(&payload[20]);
    snapshot->minCellMv = be16u(&payload[22]);
    snapshot->hasCellDiffMv = snapshot->maxCellMv >= snapshot->minCellMv;
    if (snapshot->hasCellDiffMv) {
        snapshot->cellDiffMv = (uint16_t)(snapshot->maxCellMv - snapshot->minCellMv);
    }
    snapshot->tempCount = 3u;
    snapshot->tempDeciC[0] = be16i(&payload[18]);
    snapshot->tempDeciC[1] = be16i(&payload[24]);
    snapshot->tempDeciC[2] = be16i(&payload[26]);

    if (snapshot->hasPackVoltageCv && snapshot->hasPackCurrentCa) {
        snapshot->hasPackPowerW = true;
        snapshot->packPowerW = (int32_t)(((int32_t)snapshot->packVoltageCv *
                                          (int32_t)snapshot->packCurrentCa) /
                                         10000);
    }

    snapshot->valid = true;
    snapshot->hasTelemetry = true;
    return true;
}

static bool applyModbusPib(seplos_rs485_snapshot_t *snapshot, const uint8_t *payload, size_t payloadLen)
{
    if (snapshot == NULL || payload == NULL || payloadLen < 52u) {
        return false;
    }

    snapshot->cellCount = SEPLOS_RS485_MAX_CELLS;
    for (uint8_t i = 0u; i < SEPLOS_RS485_MAX_CELLS; i++) {
        snapshot->cellMv[i] = be16u(&payload[(size_t)i * 2u]);
    }
    recomputeModbusCells(snapshot);

    snapshot->tempCount = 6u;
    for (uint8_t i = 0u; i < snapshot->tempCount; i++) {
        snapshot->tempDeciC[i] = be16i(&payload[32u + ((size_t)i * 2u)]);
    }

    snapshot->valid = true;
    snapshot->hasTelemetry = true;
    return true;
}

static uint8_t pctFromDeci(uint16_t deciPct)
{
    uint16_t pct = (uint16_t)((deciPct + 5u) / 10u);
    return (uint8_t)((pct > 100u) ? 100u : pct);
}

static void storeLatestPacket(const bms_decoded_packet_t *packet)
{
    if (packet == NULL) {
        return;
    }

    portENTER_CRITICAL(&g_latestMux);
    g_latestPacket = *packet;
    g_haveLatestPacket = true;
    portEXIT_CRITICAL(&g_latestMux);
}

static void storeLatestSnapshot(const seplos_rs485_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    portENTER_CRITICAL(&g_latestMux);
    g_latestSnapshot = *snapshot;
    g_haveLatestSnapshot = true;
    portEXIT_CRITICAL(&g_latestMux);
}

static void clearLatestData(void)
{
    portENTER_CRITICAL(&g_latestMux);
    g_haveLatestPacket = false;
    memset(&g_latestPacket, 0, sizeof(g_latestPacket));
    g_haveLatestSnapshot = false;
    memset(&g_latestSnapshot, 0, sizeof(g_latestSnapshot));
    portEXIT_CRITICAL(&g_latestMux);
}

static void publishBatteryModel(const seplos_rs485_snapshot_t *snapshot, int64_t sourceUs)
{
    battery_model_t model = {0};

    if (snapshot == NULL || !snapshot->valid || !snapshot->hasTelemetry) {
        return;
    }

    model.valid = true;
    model.updatedMs = (uint32_t)(sourceUs / 1000LL);

    if (snapshot->hasPackVoltageCv) {
        model.packVoltageV = (float)snapshot->packVoltageCv / 100.0f;
    }
    if (snapshot->hasPackCurrentCa) {
        model.packCurrentA = (float)snapshot->packCurrentCa / 100.0f;
    }
    if (snapshot->hasSocDeciPct) {
        model.socPct = pctFromDeci(snapshot->socDeciPct);
    }
    if (snapshot->hasSohDeciPct) {
        model.sohPct = pctFromDeci(snapshot->sohDeciPct);
    } else {
        model.sohPct = 100u;
    }
    if (snapshot->hasCycles) {
        model.cycleCount = snapshot->cycles;
    }
    if (snapshot->hasCellExtremes) {
        model.cellMaxV = (float)snapshot->maxCellMv / 1000.0f;
        model.cellMinV = (float)snapshot->minCellMv / 1000.0f;
        model.cellMaxIdx = snapshot->maxCellIndex;
        model.cellMinIdx = snapshot->minCellIndex;
    }
    if (snapshot->hasCellDiffMv) {
        model.cellDeltaV = (float)snapshot->cellDiffMv / 1000.0f;
    }

    for (uint8_t i = 0u; i < UNIVERSAL_BATTERY_TEMP_SENSORS; i++) {
        model.temperaturesC[i] = -100.0f;
    }
    if (snapshot->tempCount > 5u) {
        model.temperaturesC[0] = (float)snapshot->tempDeciC[5] / 10.0f; /* MOS */
    }
    if (snapshot->tempCount > 0u) {
        model.temperaturesC[1] = (float)snapshot->tempDeciC[0] / 10.0f;
    }
    if (snapshot->tempCount > 1u) {
        model.temperaturesC[2] = (float)snapshot->tempDeciC[1] / 10.0f;
    }
    if (snapshot->tempCount > 2u) {
        model.temperaturesC[3] = (float)snapshot->tempDeciC[2] / 10.0f;
    }
    if (snapshot->tempCount > 4u) {
        model.temperaturesC[4] = (float)snapshot->tempDeciC[4] / 10.0f; /* Environment */
    }

    if (snapshot->hasAlarms) {
        model.chargeEnabled = snapshot->chargeEnabled;
        model.dischargeEnabled = snapshot->dischargeEnabled;
        model.balanceEnabled = snapshot->balanceFlags != 0u;
        model.protocolState = snapshot->systemStatus;

        uint32_t rawWarnings = ((uint32_t)snapshot->warningBytes[0]) |
                               ((uint32_t)snapshot->warningBytes[1] << 8) |
                               ((uint32_t)snapshot->warningBytes[2] << 16) |
                               ((uint32_t)snapshot->warningBytes[3] << 24);
        uint32_t rawAlarms = ((uint32_t)snapshot->warningBytes[4]) |
                             ((uint32_t)snapshot->warningBytes[5] << 8) |
                             ((uint32_t)snapshot->warningBytes[6] << 16) |
                             ((uint32_t)snapshot->warningBytes[7] << 24);
        model.warningsMask = rawWarnings;
        model.alarmsMask = rawAlarms;
    }

    batteryModelSet(&model);
}

static bool buildPacketFromSnapshot(const seplos_rs485_snapshot_t *snapshot,
                                    uint32_t sequence,
                                    int64_t sourceUs,
                                    bms_decoded_packet_t *outPacket)
{
    if (snapshot == NULL || outPacket == NULL || !snapshot->valid || !snapshot->hasTelemetry) {
        return false;
    }

    memset(outPacket, 0, sizeof(*outPacket));
    outPacket->sourceProtocol = PROTOCOL_ID_SEPLOS;
    outPacket->sequence = sequence;
    outPacket->timestampUs = sourceUs;

    if (snapshot->hasSocDeciPct) {
        outPacket->hasSoc = true;
        outPacket->socPct = pctFromDeci(snapshot->socDeciPct);
    }
    if (snapshot->tempCount > 0u) {
        outPacket->hasTemperatureC = true;
        outPacket->temperatureC = (int16_t)(snapshot->tempDeciC[0] / 10);
        outPacket->tempCount = snapshot->tempCount;
        for (uint8_t i = 0u; i < snapshot->tempCount && i < BMS_DECODED_PACKET_MAX_TEMPS; i++) {
            outPacket->tempDeciC[i] = snapshot->tempDeciC[i];
        }
    }
    if (snapshot->hasPackVoltageCv) {
        outPacket->hasPackVoltageCv = true;
        outPacket->packVoltageCv = snapshot->packVoltageCv;
    }
    if (snapshot->hasCellExtremes) {
        outPacket->hasCellExtremes = true;
        outPacket->minCellMv = snapshot->minCellMv;
        outPacket->maxCellMv = snapshot->maxCellMv;
        outPacket->minCellIndex = snapshot->minCellIndex;
        outPacket->maxCellIndex = snapshot->maxCellIndex;
    }
    if (snapshot->cellCount > 0u) {
        uint8_t limit = (snapshot->cellCount > BMS_DECODED_PACKET_MAX_CELLS)
                            ? BMS_DECODED_PACKET_MAX_CELLS
                            : snapshot->cellCount;
        outPacket->cellCount = limit;
        memcpy(outPacket->cellMv, snapshot->cellMv, (size_t)limit * sizeof(outPacket->cellMv[0]));
    }
    if (snapshot->hasAlarms) {
        outPacket->hasWarningFlags = true;
        outPacket->warningFlags = (uint16_t)(((uint16_t)snapshot->warningBytes[0]) |
                                             ((uint16_t)snapshot->warningBytes[1] << 8));
        outPacket->hasProtectionFlags = true;
        outPacket->protectionFlags = (uint16_t)(((uint16_t)snapshot->warningBytes[2]) |
                                                ((uint16_t)snapshot->warningBytes[3] << 8));
        outPacket->hasStatusFlags = true;
        outPacket->statusFlags = snapshot->systemStatus;
        outPacket->hasBalanceFlags = true;
        outPacket->balanceFlags = snapshot->balanceFlags;
    }

    return outPacket->hasSoc ||
           outPacket->hasTemperatureC ||
           outPacket->hasPackVoltageCv ||
           outPacket->hasCellExtremes ||
           (outPacket->cellCount > 0u) ||
           outPacket->hasWarningFlags ||
           outPacket->hasStatusFlags;
}

static void publishSnapshot(seplosRs485BmsTaskCtx_t *ctx,
                            const seplos_rs485_snapshot_t *snapshot,
                            int64_t sourceUs)
{
    bms_decoded_packet_t packet = {0};

    if (ctx == NULL || snapshot == NULL || !snapshot->valid) {
        return;
    }

    storeLatestSnapshot(snapshot);
    publishBatteryModel(snapshot, sourceUs);
    ctx->lastFrameUs = sourceUs;

    if (buildPacketFromSnapshot(snapshot, ++ctx->sequence, sourceUs, &packet)) {
        storeLatestPacket(&packet);
        if (xQueueOverwrite(ctx->outQueue, &packet) != pdPASS) {
            ESP_LOGW(EXAMPLE_TAG, "Seplos RS485 output queue overwrite failed");
        }
    }
}

static void dropRxPrefix(seplosRs485BmsTaskCtx_t *ctx, size_t count)
{
    if (ctx == NULL || count == 0u) {
        return;
    }
    if (count >= ctx->rxLen) {
        ctx->rxLen = 0u;
        return;
    }
    memmove(ctx->rxBuf, &ctx->rxBuf[count], ctx->rxLen - count);
    ctx->rxLen -= count;
}

static bool consumeModbusRx(seplosRs485BmsTaskCtx_t *ctx, int64_t nowUs)
{
    uint8_t byteCount = 0u;
    size_t frameLen = 0u;
    uint16_t receivedCrc = 0u;
    uint16_t expectedCrc = 0u;

    if (ctx == NULL || !ctx->lastPollWasModbus || ctx->rxLen == 0u) {
        return false;
    }

    while (ctx->rxLen > 0u &&
           (ctx->rxBuf[0] != ctx->lastModbusAddress ||
            ctx->rxLen < 2u ||
            ctx->rxBuf[1] != ctx->lastModbusFunction)) {
        dropRxPrefix(ctx, 1u);
    }

    if (ctx->rxLen < 5u) {
        return true;
    }

    if ((ctx->rxBuf[1] & 0x80u) != 0u) {
        if (ctx->rxLen < 5u) {
            return true;
        }
        ESP_LOGW(EXAMPLE_TAG,
                 "Seplos Modbus RTU exception: addr=0x%02X function=0x%02X code=0x%02X",
                 (unsigned)ctx->rxBuf[0],
                 (unsigned)ctx->rxBuf[1],
                 (unsigned)ctx->rxBuf[2]);
        logRawBytes("Seplos Modbus RTU exception frame", ctx->rxBuf, 5u);
        dropRxPrefix(ctx, 5u);
        return true;
    }

    byteCount = ctx->rxBuf[2];
    frameLen = (size_t)byteCount + 5u;
    if (byteCount == 0u || byteCount > 120u) {
        logRawBytes("Seplos Modbus RTU invalid header", ctx->rxBuf, ctx->rxLen);
        dropRxPrefix(ctx, 1u);
        return false;
    }
    if (ctx->rxLen < frameLen) {
        return true;
    }

    receivedCrc = (uint16_t)ctx->rxBuf[frameLen - 2u] |
                  (uint16_t)((uint16_t)ctx->rxBuf[frameLen - 1u] << 8);
    expectedCrc = modbusCrc16(ctx->rxBuf, frameLen - 2u);
    if (receivedCrc != expectedCrc) {
        ESP_LOGW(EXAMPLE_TAG,
                 "Seplos Modbus RTU CRC failed: got=0x%04X expected=0x%04X",
                 (unsigned)receivedCrc,
                 (unsigned)expectedCrc);
        logRawBytes("Seplos Modbus RTU bad frame", ctx->rxBuf, frameLen);
        dropRxPrefix(ctx, 1u);
        return false;
    }

    logRawBytes("Seplos Modbus RTU valid frame", ctx->rxBuf, frameLen);
    lockPollCandidate(ctx);

    if (ctx->lastModbusRegister == 0x1000u) {
        if (applyModbusPia(&ctx->workingSnapshot, &ctx->rxBuf[3], byteCount)) {
            publishSnapshot(ctx, &ctx->workingSnapshot, nowUs);
            ESP_LOGI(EXAMPLE_TAG,
                     "Seplos V3 RTU PIA: soc=%u.%u%% pack=%.2fV current=%.2fA",
                     (unsigned)(ctx->workingSnapshot.socDeciPct / 10u),
                     (unsigned)(ctx->workingSnapshot.socDeciPct % 10u),
                     (double)ctx->workingSnapshot.packVoltageCv / 100.0,
                     (double)ctx->workingSnapshot.packCurrentCa / 100.0);
        }
    } else if (ctx->lastModbusRegister == 0x1100u) {
        if (applyModbusPib(&ctx->workingSnapshot, &ctx->rxBuf[3], byteCount)) {
            publishSnapshot(ctx, &ctx->workingSnapshot, nowUs);
            ESP_LOGI(EXAMPLE_TAG,
                     "Seplos V3 RTU PIB: cells=%u max=%.3fV min=%.3fV",
                     (unsigned)ctx->workingSnapshot.cellCount,
                     (double)ctx->workingSnapshot.maxCellMv / 1000.0,
                     (double)ctx->workingSnapshot.minCellMv / 1000.0);
        }
    }

    dropRxPrefix(ctx, frameLen);
    return true;
}

static uint8_t inferCid2FromFrame(const seplos_rs485_frame_t *frame, uint8_t lastRequestedCid2)
{
    if (frame == NULL) {
        return lastRequestedCid2;
    }
    if (frame->infoLen >= 60u) {
        return SEPLOS_RS485_CID2_TELEMETRY;
    }
    if (frame->infoLen >= 24u) {
        return SEPLOS_RS485_CID2_ALARMS;
    }
    return lastRequestedCid2;
}

static void consumeRx(seplosRs485BmsTaskCtx_t *ctx, const uint8_t *data, size_t len, int64_t nowUs)
{
    if (ctx == NULL || data == NULL || len == 0u) {
        return;
    }

    logRawBytes("Seplos RX chunk", data, len);

    if (len > sizeof(ctx->rxBuf) - ctx->rxLen) {
        ctx->rxLen = 0u;
    }
    if (len > sizeof(ctx->rxBuf)) {
        data += len - sizeof(ctx->rxBuf);
        len = sizeof(ctx->rxBuf);
    }

    memcpy(&ctx->rxBuf[ctx->rxLen], data, len);
    ctx->rxLen += len;

    if (consumeModbusRx(ctx, nowUs) && ctx->rxLen < 2u) {
        return;
    }

    while (ctx->rxLen > 0u) {
        size_t start = 0u;
        while (start < ctx->rxLen && ctx->rxBuf[start] != '~') {
            start++;
        }
        if (start > 0u) {
            dropRxPrefix(ctx, start);
        }
        if (ctx->rxLen < 2u) {
            return;
        }

        size_t end = 0u;
        while (end < ctx->rxLen && ctx->rxBuf[end] != '\r') {
            end++;
        }
        if (end >= ctx->rxLen) {
            return;
        }

        const size_t frameLen = end + 1u;
        seplos_rs485_frame_t frame = {0};
        if (seplosRs485DecodeFrame(ctx->rxBuf, frameLen, &frame)) {
            uint8_t cid2 = inferCid2FromFrame(&frame, ctx->lastRequestedCid2);
            lockPollCandidate(ctx);
            if (cid2 == SEPLOS_RS485_CID2_TELEMETRY) {
                seplos_rs485_snapshot_t next = {0};
                bool hadAlarms = ctx->workingSnapshot.hasAlarms;
                seplos_rs485_snapshot_t previous = ctx->workingSnapshot;
                if (seplosRs485DecodeTelemetryInfo(frame.info, frame.infoLen, &next)) {
                    if (hadAlarms) {
                        memcpy(next.cellAlarmFlags,
                               previous.cellAlarmFlags,
                               sizeof(next.cellAlarmFlags));
                        memcpy(next.tempAlarmFlags,
                               previous.tempAlarmFlags,
                               sizeof(next.tempAlarmFlags));
                        next.currentAlarmFlags = previous.currentAlarmFlags;
                        next.voltageAlarmFlags = previous.voltageAlarmFlags;
                        next.customAlarmFlags = previous.customAlarmFlags;
                        memcpy(next.warningBytes, previous.warningBytes, sizeof(next.warningBytes));
                        next.powerStatus = previous.powerStatus;
                        next.balanceFlags = previous.balanceFlags;
                        next.systemStatus = previous.systemStatus;
                        next.chargeEnabled = previous.chargeEnabled;
                        next.dischargeEnabled = previous.dischargeEnabled;
                        next.sleepMode = previous.sleepMode;
                        next.hasAlarms = true;
                    }
                    ctx->workingSnapshot = next;
                    publishSnapshot(ctx, &ctx->workingSnapshot, nowUs);
                    ESP_LOGI(EXAMPLE_TAG,
                             "Seplos telemetry: cells=%u soc=%u.%u%% pack=%.2fV current=%.2fA",
                             (unsigned)ctx->workingSnapshot.cellCount,
                             (unsigned)(ctx->workingSnapshot.hasSocDeciPct
                                            ? (ctx->workingSnapshot.socDeciPct / 10u)
                                            : 0u),
                             (unsigned)(ctx->workingSnapshot.hasSocDeciPct
                                            ? (ctx->workingSnapshot.socDeciPct % 10u)
                                            : 0u),
                             ctx->workingSnapshot.hasPackVoltageCv
                                 ? ((double)ctx->workingSnapshot.packVoltageCv / 100.0)
                                 : 0.0,
                             ctx->workingSnapshot.hasPackCurrentCa
                                 ? ((double)ctx->workingSnapshot.packCurrentCa / 100.0)
                                 : 0.0);
                } else {
                    ESP_LOGW(EXAMPLE_TAG, "Seplos telemetry decode failed (info_len=%u)", (unsigned)frame.infoLen);
                }
            } else if (cid2 == SEPLOS_RS485_CID2_ALARMS) {
                if (seplosRs485DecodeAlarmInfo(frame.info, frame.infoLen, &ctx->workingSnapshot)) {
                    publishSnapshot(ctx, &ctx->workingSnapshot, nowUs);
                } else {
                    ESP_LOGW(EXAMPLE_TAG, "Seplos alarm decode failed (info_len=%u)", (unsigned)frame.infoLen);
                }
            }
        } else {
            ESP_LOGW(EXAMPLE_TAG, "Seplos RS485 frame decode failed (len=%u)", (unsigned)frameLen);
            logRawBytes("Seplos RX bad frame", ctx->rxBuf, frameLen);
        }

        dropRxPrefix(ctx, frameLen);
    }
}

static void pollSeplos(uart_port_t uart, gpio_num_t dirPin, seplosRs485BmsTaskCtx_t *ctx, int64_t nowUs)
{
    uint8_t req[32];
    size_t reqLen = 0u;
    uint8_t cid2 = 0u;
    char txLabel[80];
    const seplosPollCandidate_t *candidate = NULL;

    if (ctx == NULL) {
        return;
    }

    if (ctx->lastPollUs != 0 &&
        (nowUs - ctx->lastPollUs) < ((int64_t)SEPLOS_BMS_QUERY_PERIOD_MS * 1000LL)) {
        return;
    }

    cid2 = (ctx->nextCid2 == SEPLOS_RS485_CID2_ALARMS)
               ? SEPLOS_RS485_CID2_ALARMS
               : SEPLOS_RS485_CID2_TELEMETRY;
    candidate = selectPollCandidate(ctx);
    if (candidate == NULL) {
        return;
    }
    applyLineInversion(uart, ctx, candidate);

    if (candidate->kind == SEPLOS_POLL_KIND_MODBUS_RTU_V3) {
        uint16_t startRegister = (cid2 == SEPLOS_RS485_CID2_ALARMS) ? 0x1100u : 0x1000u;
        uint16_t registerCount = (cid2 == SEPLOS_RS485_CID2_ALARMS) ? 0x001Au : 0x0012u;
        reqLen = buildModbusReadRequest(candidate->address,
                                        0x04u,
                                        startRegister,
                                        registerCount,
                                        req,
                                        sizeof(req));
        ctx->lastPollWasModbus = true;
        ctx->lastModbusAddress = candidate->address;
        ctx->lastModbusFunction = 0x04u;
        ctx->lastModbusRegister = startRegister;
        ctx->lastModbusCount = registerCount;
    } else {
        reqLen = seplosRs485BuildRequestWithStyle(cid2,
                                                  candidate->address,
                                                  candidate->requestInfo,
                                                  candidate->protocolVersion,
                                                  candidate->style,
                                                  req,
                                                  sizeof(req));
        ctx->lastPollWasModbus = false;
    }
    if (reqLen == 0u) {
        ESP_LOGW(EXAMPLE_TAG, "Seplos request build failed");
        return;
    }

    (void)snprintf(txLabel,
                   sizeof(txLabel),
                   "Seplos TX %s cid2=0x%02X",
                   candidate->name,
                   (unsigned)cid2);
    if (candidate->kind == SEPLOS_POLL_KIND_MODBUS_RTU_V3) {
        (void)snprintf(txLabel,
                       sizeof(txLabel),
                       "Seplos TX %s reg=0x%04X count=%u",
                       candidate->name,
                       (unsigned)ctx->lastModbusRegister,
                       (unsigned)ctx->lastModbusCount);
    }
    logRawBytes(txLabel, req, reqLen);

    esp_err_t err = rs485WriteBytes(uart, dirPin, req, (int)reqLen, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGW(EXAMPLE_TAG, "Seplos poll TX failed (cid2=0x%02X err=0x%x)", (unsigned)cid2, (unsigned)err);
        return;
    }

    ctx->lastRequestedCid2 = cid2;
    ctx->lastPolledCandidate = ctx->candidateIndex;
    ctx->pollCount++;
    ctx->nextCid2 = (cid2 == SEPLOS_RS485_CID2_TELEMETRY)
                        ? SEPLOS_RS485_CID2_ALARMS
                        : SEPLOS_RS485_CID2_TELEMETRY;
    ctx->lastPollUs = nowUs;
}

static void maybeClearStaleSource(seplosRs485BmsTaskCtx_t *ctx, int64_t nowUs)
{
    bool haveData = false;

    if (ctx == NULL) {
        return;
    }

    if (ctx->lastFrameUs == 0) {
        if ((nowUs - g_lastSourceStaleLogUs) >= 1000000LL) {
            ESP_LOGW(EXAMPLE_TAG,
                     "Seplos RS485 source stale: clearing published data (no valid BMS response yet)");
            g_lastSourceStaleLogUs = nowUs;
        }
        return;
    }

    portENTER_CRITICAL(&g_latestMux);
    haveData = g_haveLatestSnapshot || g_haveLatestPacket;
    portEXIT_CRITICAL(&g_latestMux);

    if (haveData &&
        (nowUs - ctx->lastFrameUs) > ((int64_t)BRIDGE_SOURCE_STALE_MS * 1000LL)) {
        if (ctx->candidateLocked) {
            ctx->candidateLocked = false;
            ESP_LOGW(EXAMPLE_TAG, "Seplos request candidate unlocked after source timeout");
        }
        batteryModelClear();
        clearLatestData();
        memset(&ctx->workingSnapshot, 0, sizeof(ctx->workingSnapshot));
        if ((nowUs - g_lastSourceStaleLogUs) >= 1000000LL) {
            const uint32_t ageMs = (uint32_t)((nowUs - ctx->lastFrameUs) / 1000LL);
            ESP_LOGW(EXAMPLE_TAG,
                     "Seplos RS485 source stale: clearing published data (last_rx_age=%u ms)",
                     (unsigned)ageMs);
            g_lastSourceStaleLogUs = nowUs;
        }
    }
}

static void seplosRs485BmsTask(void *pv)
{
    seplosRs485BmsTaskCtx_t *ctx = (seplosRs485BmsTaskCtx_t *)pv;
    uint8_t rxChunk[128];
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    const uint8_t bmsPort = (settings.bms_port == 2u) ? 2u : 1u;
    const uart_port_t rxUart = (bmsPort == 2u) ? rs485GetUart2() : rs485GetUart1();
    const gpio_num_t dirPin = (bmsPort == 2u) ? rs485GetDir2() : rs485GetDir1();

    uart_flush_input(rxUart);

    while (1) {
        int len = uart_read_bytes(rxUart, rxChunk, sizeof(rxChunk), pdMS_TO_TICKS(10));
        int64_t nowUs = esp_timer_get_time();

        if (len > 0) {
            consumeRx(ctx, rxChunk, (size_t)len, nowUs);
        }

        pollSeplos(rxUart, dirPin, ctx, nowUs);
        maybeClearStaleSource(ctx, nowUs);
    }
}

esp_err_t seplosRs485BmsTaskStart(QueueHandle_t outQueue)
{
    if (outQueue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_seplosRs485BmsTaskHandle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&g_seplosRs485BmsCtx, 0, sizeof(g_seplosRs485BmsCtx));
    g_seplosRs485BmsCtx.outQueue = outQueue;
    g_seplosRs485BmsCtx.nextCid2 = SEPLOS_RS485_CID2_TELEMETRY;
    g_lastSourceStaleLogUs = 0;
    batteryModelClear();
    clearLatestData();

    BaseType_t taskOk =
        xTaskCreate(seplosRs485BmsTask,
                    "seplos_rs485",
                    SEPLOS_BMS_TASK_STACK,
                    &g_seplosRs485BmsCtx,
                    SEPLOS_BMS_TASK_PRIORITY,
                    &g_seplosRs485BmsTaskHandle);
    if (taskOk != pdPASS) {
        g_seplosRs485BmsTaskHandle = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "Seplos RS485 BMS task started (addr=0x%02X info=0x%02X candidates=%u poll=%dms)",
             (unsigned)SEPLOS_BMS_ADDRESS,
             (unsigned)SEPLOS_BMS_REQUEST_INFO,
             (unsigned)seplosCandidateCount(),
             SEPLOS_BMS_QUERY_PERIOD_MS);
    return ESP_OK;
}

esp_err_t seplosRs485BmsTaskStop(void)
{
    if (g_seplosRs485BmsTaskHandle == NULL) {
        return ESP_OK;
    }

    vTaskDelete(g_seplosRs485BmsTaskHandle);
    g_seplosRs485BmsTaskHandle = NULL;
    (void)uart_set_line_inverse(rs485GetUart1(), UART_SIGNAL_INV_DISABLE);
    (void)uart_set_line_inverse(rs485GetUart2(), UART_SIGNAL_INV_DISABLE);
    memset(&g_seplosRs485BmsCtx, 0, sizeof(g_seplosRs485BmsCtx));
    batteryModelClear();
    clearLatestData();
    return ESP_OK;
}

bool seplosRs485BmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket)
{
    bool hasPacket = false;

    if (outPacket == NULL) {
        return false;
    }

    portENTER_CRITICAL(&g_latestMux);
    hasPacket = g_haveLatestPacket;
    if (hasPacket) {
        *outPacket = g_latestPacket;
    }
    portEXIT_CRITICAL(&g_latestMux);

    return hasPacket;
}

bool seplosRs485BmsTaskGetLatestSnapshot(seplos_rs485_snapshot_t *outSnapshot)
{
    bool hasSnapshot = false;

    if (outSnapshot == NULL) {
        return false;
    }

    portENTER_CRITICAL(&g_latestMux);
    hasSnapshot = g_haveLatestSnapshot;
    if (hasSnapshot) {
        *outSnapshot = g_latestSnapshot;
    }
    portEXIT_CRITICAL(&g_latestMux);

    return hasSnapshot;
}
