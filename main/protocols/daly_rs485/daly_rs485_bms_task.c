#include "protocols/daly_rs485/daly_rs485_bms_task.h"

#include <inttypes.h>
#include <limits.h>
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

#define DALY_FRAME_LEN 13u
#define DALY_START_BYTE 0xA5u
#define DALY_PAYLOAD_LEN 0x08u
#define DALY_RS485_REQUEST_ADDR ((uint8_t)(DALY_RS485_BMS_ID + 0x3Fu))
#define DALY_UART_REQUEST_ADDR 0x80u
#define DALY_MODBUS_REQUEST_ADDR 0x81u
#define DALY_MODBUS_ALT_REQUEST_ADDR 0xD2u
#define DALY_MODBUS_OBSERVED_RESPONSE_ADDR 0x51u
#define DALY_MODBUS_FUNC_READ_HOLDING 0x03u
#define DALY_MODBUS_READ_CELLS_START 0x0000u
#define DALY_MODBUS_READ_CELLS_COUNT 0x007Fu
#define DALY_MODBUS_READ_INFO_START 0x0100u
#define DALY_MODBUS_READ_INFO_COUNT 0x0078u
#define DALY_MODBUS_TEMP_SENSOR_BASE_REG_INDEX 48u
#define DALY_MODBUS_TEMP_SENSOR_REG_COUNT 4u
#define DALY_MODBUS_TEMP_SENSOR_COUNT_REG_INDEX 61u
#define DALY_MODBUS_MOS_TEMP_RAW_REG_INDEX 90u
#define DALY_MODBUS_SOC_DECI_REG_INDEX 58u
#define DALY_MODBUS_RESPONSE_TIMEOUT_MS 2500
#define DALY_MODBUS_INTER_BYTE_TIMEOUT_MS 500
#define DALY_MODBUS_MAX_REGS 127u
#define DALY_MODBUS_MAX_FRAME_LEN (3u + (DALY_MODBUS_MAX_REGS * 2u) + 2u)
#define DALY_MODBUS_RX_BUF_LEN 320u

typedef enum {
    DALY_CMD_RATED_CAPACITY_CELL_VOLTAGE = 0x50,
    DALY_CMD_BATTERY_TYPE_INFO = 0x53,
    DALY_CMD_MIN_MAX_PACK_VOLTAGE = 0x5A,
    DALY_CMD_MAX_DISCHARGE_CHARGE_CURRENT = 0x5B,
    DALY_CMD_VOUT_IOUT_SOC = 0x90,
    DALY_CMD_MIN_MAX_CELL_VOLTAGE = 0x91,
    DALY_CMD_MIN_MAX_TEMPERATURE = 0x92,
    DALY_CMD_MOS_STATUS = 0x93,
    DALY_CMD_STATUS_INFO = 0x94,
    DALY_CMD_CELL_VOLTAGES = 0x95,
    DALY_CMD_CELL_TEMPERATURES = 0x96,
    DALY_CMD_CELL_BALANCE_STATE = 0x97,
    DALY_CMD_FAILURE_CODES = 0x98,
} daly_cmd_t;

typedef struct {
    QueueHandle_t outQueue;
    uint32_t sequence;
    int64_t lastFrameUs;
    int64_t lastPublishUs;
    daly_rs485_snapshot_t snapshot;
} daly_task_ctx_t;

typedef struct {
    uint8_t bytes[DALY_FRAME_LEN];
    uint8_t len;
} daly_frame_parser_t;

static daly_task_ctx_t g_dalyCtx;
static TaskHandle_t g_dalyTaskHandle;
static portMUX_TYPE g_latestMux = portMUX_INITIALIZER_UNLOCKED;
static bool g_haveLatestPacket;
static bms_decoded_packet_t g_latestPacket;
static bool g_haveLatestSnapshot;
static daly_rs485_snapshot_t g_latestSnapshot;
static int64_t g_lastStaleLogUs;
static int64_t g_lastDecodeLogUs;
static int64_t g_lastModbusLogUs;

static uint16_t getBe16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint32_t getBe32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) |
           (uint32_t)p[3];
}

static void putBe16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFFu);
}

static uint16_t dalyModbusCrc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0u; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0u; bit < 8u; bit++) {
            if ((crc & 0x0001u) != 0u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}

static bool dalyModbusCheckCrc(const uint8_t *frame, size_t len)
{
    if (frame == NULL || len < 4u) {
        return false;
    }

    uint16_t calc = dalyModbusCrc16(frame, len - 2u);
    uint16_t got = (uint16_t)frame[len - 2u] | ((uint16_t)frame[len - 1u] << 8);
    return calc == got;
}

static void dalyBuildModbusReadRequest(uint8_t requestAddr,
                                       uint16_t start,
                                       uint16_t count,
                                       uint8_t outFrame[8])
{
    outFrame[0] = requestAddr;
    outFrame[1] = DALY_MODBUS_FUNC_READ_HOLDING;
    putBe16(&outFrame[2], start);
    putBe16(&outFrame[4], count);
    uint16_t crc = dalyModbusCrc16(outFrame, 6u);
    outFrame[6] = (uint8_t)(crc & 0xFFu);
    outFrame[7] = (uint8_t)(crc >> 8);
}

static uint8_t clampPctFromDeci(uint16_t deciPct)
{
    uint16_t pct = (uint16_t)((deciPct + 5u) / 10u);
    return (uint8_t)((pct > 100u) ? 100u : pct);
}

static uint8_t dalyChecksum(const uint8_t *frame)
{
    uint16_t sum = 0u;
    for (uint8_t i = 0u; i < (DALY_FRAME_LEN - 1u); i++) {
        sum = (uint16_t)(sum + frame[i]);
    }
    return (uint8_t)(sum & 0xFFu);
}

static bool dalyFrameValid(const uint8_t *frame)
{
    return frame != NULL &&
           frame[0] == DALY_START_BYTE &&
           frame[3] == DALY_PAYLOAD_LEN &&
           frame[12] == dalyChecksum(frame);
}

static void dalyBuildRequest(uint8_t requestAddr, uint8_t cmd, uint8_t outFrame[DALY_FRAME_LEN])
{
    memset(outFrame, 0, DALY_FRAME_LEN);
    outFrame[0] = DALY_START_BYTE;
    outFrame[1] = requestAddr;
    outFrame[2] = cmd;
    outFrame[3] = DALY_PAYLOAD_LEN;
    outFrame[12] = dalyChecksum(outFrame);
}

static bool dalyParserFeed(daly_frame_parser_t *parser, uint8_t byte, uint8_t outFrame[DALY_FRAME_LEN])
{
    if (parser == NULL || outFrame == NULL) {
        return false;
    }

    if (parser->len == 0u) {
        if (byte != DALY_START_BYTE) {
            return false;
        }
        parser->bytes[parser->len++] = byte;
        return false;
    }

    parser->bytes[parser->len++] = byte;
    if (parser->len < DALY_FRAME_LEN) {
        return false;
    }

    memcpy(outFrame, parser->bytes, DALY_FRAME_LEN);
    parser->len = 0u;

    if (dalyFrameValid(outFrame)) {
        return true;
    }

    for (uint8_t i = 1u; i < DALY_FRAME_LEN; i++) {
        if (outFrame[i] == DALY_START_BYTE) {
            parser->bytes[0] = DALY_START_BYTE;
            parser->len = 1u;
            break;
        }
    }
    return false;
}

static bool dalyFrameAddressMatches(uint8_t address)
{
    return address == DALY_RS485_BMS_ID ||
           address == DALY_RS485_REQUEST_ADDR ||
           address == DALY_UART_REQUEST_ADDR;
}

static uint32_t dalyFailureMask(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24) |
           ((uint32_t)data[1] << 16) |
           ((uint32_t)data[2] << 8) |
           (uint32_t)data[3];
}

static void dalyDecodeFrame(daly_task_ctx_t *ctx, const uint8_t frame[DALY_FRAME_LEN])
{
    daly_rs485_snapshot_t *s = &ctx->snapshot;
    const uint8_t cmd = frame[2];
    const uint8_t *d = &frame[4];

    if (!dalyFrameAddressMatches(frame[1])) {
        return;
    }

    s->valid = true;
    s->timestampUs = esp_timer_get_time();
    ctx->lastFrameUs = s->timestampUs;

    switch (cmd) {
        case DALY_CMD_RATED_CAPACITY_CELL_VOLTAGE:
            s->hasCapacity = true;
            s->ratedCapacityMah = getBe32(&d[0]);
            break;

        case DALY_CMD_MAX_DISCHARGE_CHARGE_CURRENT:
            break;

        case DALY_CMD_VOUT_IOUT_SOC: {
            int32_t currentDeciA = (int32_t)getBe16(&d[4]) - 30000;
            s->hasPackVoltageCv = true;
            s->packVoltageCv = (uint16_t)(getBe16(&d[0]) * 10u);
            s->hasCurrentDeciA = true;
            if (currentDeciA < INT16_MIN) currentDeciA = INT16_MIN;
            if (currentDeciA > INT16_MAX) currentDeciA = INT16_MAX;
            s->currentDeciA = (int16_t)currentDeciA;
            s->hasSocDeciPct = true;
            s->socDeciPct = getBe16(&d[6]);
            break;
        }

        case DALY_CMD_MIN_MAX_CELL_VOLTAGE:
            s->hasCellExtremes = true;
            s->maxCellMv = getBe16(&d[0]);
            s->maxCellIndex = d[2];
            s->minCellMv = getBe16(&d[3]);
            s->minCellIndex = d[5];
            break;

        case DALY_CMD_MIN_MAX_TEMPERATURE:
            s->tempCount = (s->tempCount > 0u) ? s->tempCount : 2u;
            s->tempDeciC[0] = (int16_t)(((int16_t)d[0] - 40) * 10);
            s->tempDeciC[1] = (int16_t)(((int16_t)d[2] - 40) * 10);
            break;

        case DALY_CMD_MOS_STATUS:
            s->protocolState = d[0];
            s->chargeEnabled = d[1] == 1u;
            s->dischargeEnabled = d[2] == 1u;
            s->hasCapacity = true;
            s->remainingCapacityMah = getBe32(&d[4]);
            break;

        case DALY_CMD_STATUS_INFO:
            if (d[0] > 0u) {
                s->cellCount = (d[0] > BMS_DECODED_PACKET_MAX_CELLS)
                                   ? BMS_DECODED_PACKET_MAX_CELLS
                                   : d[0];
            }
            if (d[1] > 0u) {
                s->tempCount = (d[1] > BMS_DECODED_PACKET_MAX_TEMPS)
                                   ? BMS_DECODED_PACKET_MAX_TEMPS
                                   : d[1];
            }
            s->chargeEnabled = d[2] == 1u;
            s->dischargeEnabled = d[3] == 1u;
            s->hasCycles = true;
            s->cycles = getBe16(&d[5]);
            break;

        case DALY_CMD_CELL_VOLTAGES: {
            uint8_t frameNo = d[0];
            if (frameNo == 0u) {
                break;
            }
            uint8_t firstCell = (uint8_t)((frameNo - 1u) * 3u);
            for (uint8_t i = 0u; i < 3u; i++) {
                uint8_t cell = (uint8_t)(firstCell + i);
                uint16_t mv = getBe16(&d[1u + (uint8_t)(i * 2u)]);
                if (cell >= BMS_DECODED_PACKET_MAX_CELLS || mv < 1000u || mv > 6000u) {
                    continue;
                }
                s->cellMv[cell] = mv;
                if (s->cellCount < (uint8_t)(cell + 1u)) {
                    s->cellCount = (uint8_t)(cell + 1u);
                }
            }
            break;
        }

        case DALY_CMD_CELL_TEMPERATURES: {
            uint8_t frameNo = d[0];
            if (frameNo == 0u) {
                break;
            }
            uint8_t firstTemp = (uint8_t)((frameNo - 1u) * 7u);
            for (uint8_t i = 0u; i < 7u; i++) {
                uint8_t temp = (uint8_t)(firstTemp + i);
                if (temp >= BMS_DECODED_PACKET_MAX_TEMPS) {
                    break;
                }
                s->tempDeciC[temp] = (int16_t)(((int16_t)d[1u + i] - 40) * 10);
                if (s->tempCount < (uint8_t)(temp + 1u)) {
                    s->tempCount = (uint8_t)(temp + 1u);
                }
            }
            break;
        }

        case DALY_CMD_CELL_BALANCE_STATE:
            s->balanceEnabled = false;
            for (uint8_t i = 0u; i < 6u; i++) {
                if (d[i] != 0u) {
                    s->balanceEnabled = true;
                    break;
                }
            }
            break;

        case DALY_CMD_FAILURE_CODES:
            s->alarmMask = dalyFailureMask(d);
            s->warningMask = ((uint32_t)d[4] << 16) | ((uint32_t)d[5] << 8) | (uint32_t)d[6];
            break;

        default:
            break;
    }
}

static uint8_t dalyResponseFrameCount(uint8_t cmd, const daly_rs485_snapshot_t *snapshot)
{
    if (cmd == DALY_CMD_CELL_VOLTAGES) {
        uint8_t cells = (snapshot != NULL && snapshot->cellCount > 0u) ? snapshot->cellCount : 16u;
        return (uint8_t)((cells + 2u) / 3u);
    }
    return 1u;
}

static uint8_t dalyPollCommand(daly_task_ctx_t *ctx,
                               uart_port_t uart,
                               gpio_num_t dirPin,
                               uint8_t requestAddr,
                               uint8_t cmd)
{
    uint8_t request[DALY_FRAME_LEN];
    uint8_t rxChunk[64];
    uint8_t frame[DALY_FRAME_LEN];
    daly_frame_parser_t parser = {0};
    uint8_t matched = 0u;
    uint8_t expected = dalyResponseFrameCount(cmd, &ctx->snapshot);
    int64_t deadlineUs = 0;
    int64_t lastMatchUs = 0;

    dalyBuildRequest(requestAddr, cmd, request);
    uart_flush_input(uart);
    esp_err_t err = rs485WriteBytes(uart, dirPin, request, (int)sizeof(request), pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGW(EXAMPLE_TAG, "DALY RS485 TX failed cmd=0x%02X err=0x%x", cmd, (unsigned)err);
        return 0u;
    }

    deadlineUs = esp_timer_get_time() + ((int64_t)DALY_RS485_RESPONSE_TIMEOUT_MS * 1000LL);
    while (esp_timer_get_time() < deadlineUs) {
        int len = uart_read_bytes(uart, rxChunk, sizeof(rxChunk), pdMS_TO_TICKS(10));
        int64_t nowUs = esp_timer_get_time();
        if (len <= 0) {
            if (matched >= expected && lastMatchUs > 0 &&
                (nowUs - lastMatchUs) > 30000LL) {
                break;
            }
            continue;
        }

        for (int i = 0; i < len; i++) {
            if (!dalyParserFeed(&parser, rxChunk[i], frame)) {
                continue;
            }
            if (frame[2] == cmd) {
                dalyDecodeFrame(ctx, frame);
                matched++;
                lastMatchUs = nowUs;
            } else {
                dalyDecodeFrame(ctx, frame);
            }
        }
    }

    return matched;
}

static bool dalyModbusResponseAddressMatches(uint8_t address)
{
    return address == DALY_MODBUS_OBSERVED_RESPONSE_ADDR ||
           address == DALY_MODBUS_ALT_REQUEST_ADDR ||
           address == DALY_MODBUS_REQUEST_ADDR;
}

static bool dalyModbusDatePattern(uint16_t word0, uint16_t word1)
{
    uint8_t year = (uint8_t)(word0 >> 8);
    uint8_t month = (uint8_t)(word0 & 0xFFu);
    uint8_t day = (uint8_t)(word1 >> 8);
    uint8_t hour = (uint8_t)(word1 & 0xFFu);

    return year >= 20u && year <= 40u &&
           month >= 1u && month <= 12u &&
           day >= 1u && day <= 31u &&
           hour <= 23u;
}

static bool dalyModbusLooksLikeCellMv(uint16_t v)
{
    return v >= 1800u && v <= 4500u;
}

static bool dalyModbusDecodeOffsetTemp(uint16_t raw, int16_t *outDeciC)
{
    if (outDeciC == NULL || raw == 0xFFFFu || raw < 40u || raw > 160u) {
        return false;
    }
    *outDeciC = (int16_t)(((int16_t)raw - 40) * 10);
    return true;
}

static void dalyUpdateExtremesFromCells(daly_rs485_snapshot_t *s)
{
    if (s == NULL || s->cellCount == 0u) {
        return;
    }

    uint16_t minMv = UINT16_MAX;
    uint16_t maxMv = 0u;
    uint8_t minIdx = 1u;
    uint8_t maxIdx = 1u;
    uint32_t sumMv = 0u;

    for (uint8_t i = 0u; i < s->cellCount; i++) {
        uint16_t mv = s->cellMv[i];
        sumMv += mv;
        if (mv < minMv) {
            minMv = mv;
            minIdx = (uint8_t)(i + 1u);
        }
        if (mv > maxMv) {
            maxMv = mv;
            maxIdx = (uint8_t)(i + 1u);
        }
    }

    s->hasCellExtremes = true;
    s->minCellMv = minMv;
    s->maxCellMv = maxMv;
    s->minCellIndex = minIdx;
    s->maxCellIndex = maxIdx;
    s->hasPackVoltageCv = true;
    s->packVoltageCv = (uint16_t)((sumMv + 5u) / 10u);
}

static void dalyDecodeModbusTemperatures(daly_rs485_snapshot_t *s,
                                         const uint16_t *regs,
                                         uint16_t count)
{
    int16_t tempDeciC = 0;

    if (s == NULL || regs == NULL || count <= DALY_MODBUS_MOS_TEMP_RAW_REG_INDEX) {
        return;
    }

    if (!dalyModbusDecodeOffsetTemp(regs[DALY_MODBUS_MOS_TEMP_RAW_REG_INDEX], &tempDeciC)) {
        return;
    }

    memset(s->tempDeciC, 0, sizeof(s->tempDeciC));
    s->tempDeciC[0] = tempDeciC;
    s->tempCount = 1u;

    uint16_t sensorCount = 0u;
    if (count > DALY_MODBUS_TEMP_SENSOR_COUNT_REG_INDEX &&
        regs[DALY_MODBUS_TEMP_SENSOR_COUNT_REG_INDEX] > 0u &&
        regs[DALY_MODBUS_TEMP_SENSOR_COUNT_REG_INDEX] <= DALY_MODBUS_TEMP_SENSOR_REG_COUNT) {
        sensorCount = regs[DALY_MODBUS_TEMP_SENSOR_COUNT_REG_INDEX];
    } else {
        sensorCount = DALY_MODBUS_TEMP_SENSOR_REG_COUNT;
    }

    for (uint16_t i = 0u;
         i < sensorCount &&
         i < DALY_MODBUS_TEMP_SENSOR_REG_COUNT &&
         (DALY_MODBUS_TEMP_SENSOR_BASE_REG_INDEX + i) < count &&
         (i + 1u) < BMS_DECODED_PACKET_MAX_TEMPS;
         i++) {
        uint16_t raw = regs[DALY_MODBUS_TEMP_SENSOR_BASE_REG_INDEX + i];
        if (dalyModbusDecodeOffsetTemp(raw, &tempDeciC)) {
            s->tempDeciC[i + 1u] = tempDeciC;
            s->tempCount = (uint8_t)(i + 2u);
        }
    }
}

static bool dalyDecodeModbusRegisters(daly_task_ctx_t *ctx,
                                      uint16_t start,
                                      const uint16_t *regs,
                                      uint16_t count)
{
    if (ctx == NULL || regs == NULL || count == 0u) {
        return false;
    }

    daly_rs485_snapshot_t *s = &ctx->snapshot;
    bool updated = false;
    uint16_t socCandidateIndex = UINT16_MAX;
    uint16_t scanLimit = count;
    if (scanLimit > BMS_DECODED_PACKET_MAX_CELLS) {
        scanLimit = BMS_DECODED_PACKET_MAX_CELLS;
    }

    if (start == DALY_MODBUS_READ_CELLS_START) {
        uint8_t cellCount = 0u;

        for (uint16_t i = 0u; i < scanLimit; i++) {
            if (!dalyModbusLooksLikeCellMv(regs[i])) {
                break;
            }
            s->cellMv[cellCount++] = regs[i];
        }
        if (cellCount >= 4u) {
            s->cellCount = cellCount;
            dalyUpdateExtremesFromCells(s);
            updated = true;
        }

        bool haveSocCurrentCandidate = false;
        uint16_t bestSocDeciPct = 0u;
        uint16_t bestCurrentRaw = 30000u;
        uint16_t bestCandidateIndex = 0u;

        for (uint16_t i = 0u; i + 3u < count; i++) {
            if (regs[i] <= 1000u &&
                regs[i + 1u] >= 25000u &&
                regs[i + 1u] <= 35000u &&
                dalyModbusDatePattern(regs[i + 2u], regs[i + 3u])) {
                bool candidateUsable = regs[i] >= 10u;
                if (!haveSocCurrentCandidate ||
                    (candidateUsable && bestSocDeciPct < 10u) ||
                    (candidateUsable && regs[i] < bestSocDeciPct)) {
                    haveSocCurrentCandidate = true;
                    bestSocDeciPct = regs[i];
                    bestCurrentRaw = regs[i + 1u];
                    bestCandidateIndex = i;
                }
            }
        }
        if (haveSocCurrentCandidate) {
            int32_t currentDeciA = (int32_t)bestCurrentRaw - 30000;

            s->hasSocDeciPct = true;
            s->socDeciPct = bestSocDeciPct;
            s->hasCurrentDeciA = true;
            if (currentDeciA < INT16_MIN) currentDeciA = INT16_MIN;
            if (currentDeciA > INT16_MAX) currentDeciA = INT16_MAX;
            s->currentDeciA = (int16_t)currentDeciA;
            socCandidateIndex = bestCandidateIndex;
            updated = true;
        }

        if (count > DALY_MODBUS_SOC_DECI_REG_INDEX &&
            regs[DALY_MODBUS_SOC_DECI_REG_INDEX] > 0u &&
            regs[DALY_MODBUS_SOC_DECI_REG_INDEX] <= 1000u) {
            s->hasSocDeciPct = true;
            s->socDeciPct = regs[DALY_MODBUS_SOC_DECI_REG_INDEX];
            socCandidateIndex = DALY_MODBUS_SOC_DECI_REG_INDEX;
            updated = true;
        }

        bool haveSocScanCandidate = false;
        uint16_t socScanValue = 0u;
        uint16_t socScanIndex = UINT16_MAX;
        for (uint16_t i = scanLimit; i < count; i++) {
            if (regs[i] < 900u || regs[i] > 1000u) {
                continue;
            }
            if (!haveSocScanCandidate || (socScanValue == 1000u && regs[i] != 1000u)) {
                haveSocScanCandidate = true;
                socScanValue = regs[i];
                socScanIndex = i;
            }
            if (regs[i] != 1000u) {
                break;
            }
        }
        if (haveSocScanCandidate &&
            (socCandidateIndex == UINT16_MAX) &&
            (socScanValue != 1000u || !s->hasSocDeciPct || s->socDeciPct < 100u)) {
            s->hasSocDeciPct = true;
            s->socDeciPct = socScanValue;
            socCandidateIndex = socScanIndex;
            updated = true;
        }

        uint8_t prevTempCount = s->tempCount;
        dalyDecodeModbusTemperatures(s, regs, count);
        if (s->tempCount != prevTempCount || s->tempCount > 0u) {
            updated = true;
        }
    }

    if (updated) {
        s->valid = true;
        s->timestampUs = esp_timer_get_time();
        s->chargeEnabled = true;
        s->dischargeEnabled = true;
        ctx->lastFrameUs = s->timestampUs;

        if ((s->timestampUs - g_lastModbusLogUs) >= 2000000LL) {
            ESP_LOGI(EXAMPLE_TAG,
                     "DALY Modbus decoded block=0x%04X regs=%u cells=%u pack=%.2fV soc=%u.%u%% current=%.1fA socIdx=%u first=[%04X %04X %04X %04X]",
                     (unsigned)start,
                     (unsigned)count,
                     (unsigned)s->cellCount,
                     s->hasPackVoltageCv ? ((double)s->packVoltageCv / 100.0) : 0.0,
                     s->hasSocDeciPct ? (unsigned)(s->socDeciPct / 10u) : 0u,
                     s->hasSocDeciPct ? (unsigned)(s->socDeciPct % 10u) : 0u,
                     s->hasCurrentDeciA ? ((double)s->currentDeciA / 10.0) : 0.0,
                     (socCandidateIndex == UINT16_MAX) ? 0u : (unsigned)socCandidateIndex,
                     (unsigned)((count > 0u) ? regs[0] : 0u),
                     (unsigned)((count > 1u) ? regs[1] : 0u),
                     (unsigned)((count > 2u) ? regs[2] : 0u),
                     (unsigned)((count > 3u) ? regs[3] : 0u));
            g_lastModbusLogUs = s->timestampUs;
        }
    }

    return updated;
}

static bool dalyDecodeModbusPayload(daly_task_ctx_t *ctx,
                                    uint16_t start,
                                    const uint8_t *payload,
                                    uint16_t regsAvailable)
{
    uint16_t regs[DALY_MODBUS_MAX_REGS] = {0};

    if (payload == NULL || regsAvailable == 0u || regsAvailable > DALY_MODBUS_MAX_REGS) {
        return false;
    }

    for (uint16_t i = 0u; i < regsAvailable; i++) {
        regs[i] = getBe16(&payload[(size_t)i * 2u]);
    }

    return dalyDecodeModbusRegisters(ctx, start, regs, regsAvailable);
}

static bool dalyHandleModbusFrame(daly_task_ctx_t *ctx,
                                  const uint8_t *frame,
                                  size_t frameLen,
                                  uint16_t start,
                                  uint16_t count)
{
    uint8_t byteCount = 0u;

    if (frame == NULL || frameLen < 5u || count > DALY_MODBUS_MAX_REGS) {
        return false;
    }
    if (!dalyModbusResponseAddressMatches(frame[0]) ||
        frame[1] != DALY_MODBUS_FUNC_READ_HOLDING) {
        return false;
    }
    byteCount = frame[2];
    if (byteCount != (uint8_t)(count * 2u) ||
        frameLen != (size_t)(3u + byteCount + 2u) ||
        !dalyModbusCheckCrc(frame, frameLen)) {
        return false;
    }

    bool decoded = dalyDecodeModbusPayload(ctx, start, &frame[3], count);
    int64_t nowUs = esp_timer_get_time();
    if (!decoded && (nowUs - g_lastModbusLogUs) >= 2000000LL) {
        ESP_LOGI(EXAMPLE_TAG,
                 "DALY Modbus frame ok but unmapped: resp=0x%02X start=0x%04X regs=%u first=[%04X %04X %04X %04X]",
                 (unsigned)frame[0],
                 (unsigned)start,
                 (unsigned)count,
                 (unsigned)((count > 0u) ? getBe16(&frame[3]) : 0u),
                 (unsigned)((count > 1u) ? getBe16(&frame[5]) : 0u),
                 (unsigned)((count > 2u) ? getBe16(&frame[7]) : 0u),
                 (unsigned)((count > 3u) ? getBe16(&frame[9]) : 0u));
        g_lastModbusLogUs = nowUs;
    }
    (void)decoded;
    return true;
}

static uint16_t dalyPartialMinRegs(uint16_t start)
{
    if (start == DALY_MODBUS_READ_CELLS_START) {
        return (uint16_t)(DALY_MODBUS_SOC_DECI_REG_INDEX + 1u);
    }
    return 0u;
}

static bool dalyTryDecodePartialModbusFrame(daly_task_ctx_t *ctx,
                                            const uint8_t *frame,
                                            size_t frameLen,
                                            uint16_t start,
                                            uint16_t expectedCount,
                                            size_t expectedLen)
{
    if (ctx == NULL || frame == NULL || frameLen < 5u || expectedCount > DALY_MODBUS_MAX_REGS) {
        return false;
    }
    if (!dalyModbusResponseAddressMatches(frame[0]) ||
        frame[1] != DALY_MODBUS_FUNC_READ_HOLDING ||
        frame[2] != (uint8_t)(expectedCount * 2u)) {
        return false;
    }

    uint16_t minRegs = dalyPartialMinRegs(start);
    if (minRegs == 0u || frameLen <= 3u) {
        return false;
    }

    uint16_t regsAvailable = (uint16_t)((frameLen - 3u) / 2u);
    if (regsAvailable > expectedCount) {
        regsAvailable = expectedCount;
    }
    if (regsAvailable < minRegs) {
        return false;
    }

    return dalyDecodeModbusPayload(ctx, start, &frame[3], regsAvailable);
}

static bool dalyPollModbusRead(daly_task_ctx_t *ctx,
                               uart_port_t uart,
                               gpio_num_t dirPin,
                               uint8_t requestAddr,
                               uint16_t start,
                               uint16_t count)
{
    uint8_t request[8];
    uint8_t chunk[64];
    uint8_t rxBuf[DALY_MODBUS_RX_BUF_LEN];
    size_t rxLen = 0u;
    const uint8_t expectedByteCount = (uint8_t)(count * 2u);
    const size_t expectedLen = (size_t)(3u + expectedByteCount + 2u);
    int64_t deadlineUs = 0;
    int64_t hardDeadlineUs = 0;

    if (ctx == NULL || count == 0u || count > DALY_MODBUS_MAX_REGS ||
        expectedLen > DALY_MODBUS_MAX_FRAME_LEN) {
        return false;
    }

    dalyBuildModbusReadRequest(requestAddr, start, count, request);
    uart_flush_input(uart);
    esp_err_t err = rs485WriteBytes(uart, dirPin, request, (int)sizeof(request), pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGW(EXAMPLE_TAG,
                 "DALY Modbus TX failed addr=0x%02X start=0x%04X count=%u err=0x%x",
                 (unsigned)requestAddr,
                 (unsigned)start,
                 (unsigned)count,
                 (unsigned)err);
        return false;
    }

    deadlineUs = esp_timer_get_time() + ((int64_t)DALY_MODBUS_RESPONSE_TIMEOUT_MS * 1000LL);
    hardDeadlineUs = deadlineUs + ((int64_t)DALY_MODBUS_RESPONSE_TIMEOUT_MS * 1000LL);
    while (esp_timer_get_time() < deadlineUs) {
        int len = uart_read_bytes(uart, chunk, sizeof(chunk), pdMS_TO_TICKS(20));
        if (len <= 0) {
            continue;
        }
        int64_t nowUs = esp_timer_get_time();
        int64_t idleDeadlineUs = nowUs + ((int64_t)DALY_MODBUS_INTER_BYTE_TIMEOUT_MS * 1000LL);
        deadlineUs = (idleDeadlineUs < hardDeadlineUs) ? idleDeadlineUs : hardDeadlineUs;

        if ((rxLen + (size_t)len) > sizeof(rxBuf)) {
            size_t keep = (rxLen > expectedLen) ? expectedLen : rxLen;
            memmove(rxBuf, &rxBuf[rxLen - keep], keep);
            rxLen = keep;
        }
        if ((size_t)len > (sizeof(rxBuf) - rxLen)) {
            len = (int)(sizeof(rxBuf) - rxLen);
        }
        memcpy(&rxBuf[rxLen], chunk, (size_t)len);
        rxLen += (size_t)len;

        if (rxLen < 5u) {
            continue;
        }

        for (size_t pos = 0u; (pos + 3u) <= rxLen; pos++) {
            if (!dalyModbusResponseAddressMatches(rxBuf[pos]) ||
                rxBuf[pos + 1u] != DALY_MODBUS_FUNC_READ_HOLDING ||
                rxBuf[pos + 2u] != expectedByteCount) {
                continue;
            }
            size_t available = rxLen - pos;
            if (available >= expectedLen &&
                dalyHandleModbusFrame(ctx, &rxBuf[pos], expectedLen, start, count)) {
                return true;
            }
        }
    }

    for (size_t pos = 0u; (pos + 3u) <= rxLen; pos++) {
        if (!dalyModbusResponseAddressMatches(rxBuf[pos]) ||
            rxBuf[pos + 1u] != DALY_MODBUS_FUNC_READ_HOLDING ||
            rxBuf[pos + 2u] != expectedByteCount) {
            continue;
        }
        if (dalyTryDecodePartialModbusFrame(ctx,
                                            &rxBuf[pos],
                                            rxLen - pos,
                                            start,
                                            count,
                                            expectedLen)) {
            return true;
        }
    }

    return false;
}

bool dalyRs485BuildDecodedPacket(const daly_rs485_snapshot_t *snapshot,
                                 uint32_t sequence,
                                 bms_decoded_packet_t *outPacket)
{
    if (snapshot == NULL || outPacket == NULL || !snapshot->valid) {
        return false;
    }

    memset(outPacket, 0, sizeof(*outPacket));
    outPacket->sourceProtocol = PROTOCOL_ID_DALY;
    outPacket->sequence = sequence;
    outPacket->timestampUs = snapshot->timestampUs;

    if (snapshot->hasSocDeciPct) {
        outPacket->hasSoc = true;
        outPacket->socPct = clampPctFromDeci(snapshot->socDeciPct);
    }
    if (snapshot->hasPackVoltageCv) {
        outPacket->hasPackVoltageCv = true;
        outPacket->packVoltageCv = snapshot->packVoltageCv;
    }
    if (snapshot->hasCellExtremes) {
        outPacket->hasCellExtremes = true;
        outPacket->maxCellMv = snapshot->maxCellMv;
        outPacket->minCellMv = snapshot->minCellMv;
        outPacket->maxCellIndex = snapshot->maxCellIndex;
        outPacket->minCellIndex = snapshot->minCellIndex;
    }
    if (snapshot->tempCount > 0u) {
        int32_t sum = 0;
        uint8_t count = snapshot->tempCount;
        if (count > BMS_DECODED_PACKET_MAX_TEMPS) {
            count = BMS_DECODED_PACKET_MAX_TEMPS;
        }
        outPacket->tempCount = count;
        for (uint8_t i = 0u; i < count; i++) {
            outPacket->tempDeciC[i] = snapshot->tempDeciC[i];
            sum += snapshot->tempDeciC[i];
        }
        outPacket->hasTemperatureC = true;
        outPacket->temperatureC = (int16_t)((sum / (int32_t)count) / 10);
    }
    if (snapshot->cellCount > 0u) {
        uint8_t count = snapshot->cellCount;
        if (count > BMS_DECODED_PACKET_MAX_CELLS) {
            count = BMS_DECODED_PACKET_MAX_CELLS;
        }
        outPacket->cellCount = count;
        memcpy(outPacket->cellMv, snapshot->cellMv, (size_t)count * sizeof(outPacket->cellMv[0]));
    }
    if (snapshot->warningMask != 0u) {
        outPacket->hasWarningFlags = true;
        outPacket->warningFlags = (uint16_t)(snapshot->warningMask & 0xFFFFu);
    }
    if (snapshot->alarmMask != 0u) {
        outPacket->hasProtectionFlags = true;
        outPacket->protectionFlags = (uint16_t)(snapshot->alarmMask & 0xFFFFu);
    }
    if (snapshot->protocolState != 0u || snapshot->chargeEnabled || snapshot->dischargeEnabled) {
        outPacket->hasStatusFlags = true;
        outPacket->statusFlags = (uint16_t)(snapshot->protocolState & 0xFFFFu);
    }
    if (snapshot->balanceEnabled) {
        outPacket->hasBalanceFlags = true;
        outPacket->balanceFlags = 1u;
    }

    return outPacket->hasSoc ||
           outPacket->hasPackVoltageCv ||
           outPacket->hasTemperatureC ||
           outPacket->hasCellExtremes ||
           outPacket->cellCount > 0u;
}

static void dalyPublishBatteryModel(const daly_rs485_snapshot_t *snapshot)
{
    battery_model_t model = {0};

    if (snapshot == NULL || !snapshot->valid) {
        return;
    }

    model.valid = true;
    model.updatedMs = (uint32_t)(esp_timer_get_time() / 1000LL);
    model.sohPct = 100u;

    if (snapshot->hasPackVoltageCv) {
        model.packVoltageV = (float)snapshot->packVoltageCv / 100.0f;
    }
    if (snapshot->hasCurrentDeciA) {
        model.packCurrentA = (float)snapshot->currentDeciA / 10.0f;
    }
    if (snapshot->hasSocDeciPct) {
        model.socPct = clampPctFromDeci(snapshot->socDeciPct);
    }
    if (snapshot->hasCycles) {
        model.cycleCount = snapshot->cycles;
    }
    if (snapshot->hasCellExtremes) {
        model.cellMaxV = (float)snapshot->maxCellMv / 1000.0f;
        model.cellMinV = (float)snapshot->minCellMv / 1000.0f;
        model.cellMaxIdx = snapshot->maxCellIndex;
        model.cellMinIdx = snapshot->minCellIndex;
        model.cellDeltaV = (float)(snapshot->maxCellMv - snapshot->minCellMv) / 1000.0f;
    }
    for (uint8_t i = 0u; i < UNIVERSAL_BATTERY_TEMP_SENSORS; i++) {
        model.temperaturesC[i] = -100.0f;
    }
    for (uint8_t i = 0u; i < snapshot->tempCount && i < UNIVERSAL_BATTERY_TEMP_SENSORS; i++) {
        model.temperaturesC[i] = (float)snapshot->tempDeciC[i] / 10.0f;
    }
    model.chargeEnabled = snapshot->chargeEnabled;
    model.dischargeEnabled = snapshot->dischargeEnabled;
    model.balanceEnabled = snapshot->balanceEnabled;
    model.alarmsMask = snapshot->alarmMask;
    model.warningsMask = snapshot->warningMask;
    model.protocolState =
        (snapshot->chargeEnabled ? 0x80u : 0u) |
        (snapshot->dischargeEnabled ? 0x40u : 0u) |
        (snapshot->balanceEnabled ? 0x20u : 0u);

    batteryModelSet(&model);
}

static void dalyStoreLatest(const daly_rs485_snapshot_t *snapshot,
                            const bms_decoded_packet_t *packet)
{
    portENTER_CRITICAL(&g_latestMux);
    if (snapshot != NULL) {
        g_latestSnapshot = *snapshot;
        g_haveLatestSnapshot = true;
    }
    if (packet != NULL) {
        g_latestPacket = *packet;
        g_haveLatestPacket = true;
    }
    portEXIT_CRITICAL(&g_latestMux);
}

static void dalyClearLatest(void)
{
    portENTER_CRITICAL(&g_latestMux);
    g_haveLatestSnapshot = false;
    g_haveLatestPacket = false;
    memset(&g_latestSnapshot, 0, sizeof(g_latestSnapshot));
    memset(&g_latestPacket, 0, sizeof(g_latestPacket));
    portEXIT_CRITICAL(&g_latestMux);
}

static void dalyLogSnapshot(const daly_rs485_snapshot_t *s, const bms_decoded_packet_t *packet)
{
    int64_t nowUs = esp_timer_get_time();

    if ((nowUs - g_lastDecodeLogUs) < 1000000LL || s == NULL || packet == NULL) {
        return;
    }
    g_lastDecodeLogUs = nowUs;

    ESP_LOGI(EXAMPLE_TAG,
             "DALY RS485 decoded: soc=%u.%u%% pack=%.2fV current=%.1fA cells=%u tempCount=%u min=%.3fV#%u max=%.3fV#%u chg=%s dsg=%s bal=%s alarms=0x%08" PRIX32,
             s->hasSocDeciPct ? (unsigned)(s->socDeciPct / 10u) : 0u,
             s->hasSocDeciPct ? (unsigned)(s->socDeciPct % 10u) : 0u,
             s->hasPackVoltageCv ? ((double)s->packVoltageCv / 100.0) : 0.0,
             s->hasCurrentDeciA ? ((double)s->currentDeciA / 10.0) : 0.0,
             (unsigned)s->cellCount,
             (unsigned)s->tempCount,
             s->hasCellExtremes ? ((double)s->minCellMv / 1000.0) : 0.0,
             s->hasCellExtremes ? (unsigned)s->minCellIndex : 0u,
             s->hasCellExtremes ? ((double)s->maxCellMv / 1000.0) : 0.0,
             s->hasCellExtremes ? (unsigned)s->maxCellIndex : 0u,
             s->chargeEnabled ? "ON" : "OFF",
             s->dischargeEnabled ? "ON" : "OFF",
             s->balanceEnabled ? "ON" : "OFF",
             s->alarmMask);
}

static void dalyTask(void *pv)
{
    daly_task_ctx_t *ctx = (daly_task_ctx_t *)pv;
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    const uint8_t bmsPort = (settings.bms_port == 2u) ? 2u : 1u;
    const uart_port_t uart = (bmsPort == 2u) ? rs485GetUart2() : rs485GetUart1();
    const gpio_num_t dirPin = (bmsPort == 2u) ? rs485GetDir2() : rs485GetDir1();
    const char *ifName = (bmsPort == 2u) ? "DALY_RS485_2" : "DALY_RS485_1";
    static const uint8_t initCmds[] = {
        DALY_CMD_RATED_CAPACITY_CELL_VOLTAGE,
        DALY_CMD_BATTERY_TYPE_INFO,
        DALY_CMD_MIN_MAX_PACK_VOLTAGE,
        DALY_CMD_MAX_DISCHARGE_CHARGE_CURRENT,
    };
    static const uint8_t runCmds[] = {
        DALY_CMD_VOUT_IOUT_SOC,
        DALY_CMD_MIN_MAX_CELL_VOLTAGE,
        DALY_CMD_MIN_MAX_TEMPERATURE,
        DALY_CMD_MOS_STATUS,
        DALY_CMD_STATUS_INFO,
        DALY_CMD_CELL_VOLTAGES,
        DALY_CMD_CELL_TEMPERATURES,
        DALY_CMD_CELL_BALANCE_STATE,
        DALY_CMD_FAILURE_CODES,
    };

    uart_flush_input(uart);
    ESP_LOGI(EXAMPLE_TAG,
             "DALY RS485 BMS task active on %s (bms_id=%u req_addr=0x%02X modbus_addr=0x%02X baud=%u)",
             ifName,
             (unsigned)DALY_RS485_BMS_ID,
             (unsigned)DALY_RS485_REQUEST_ADDR,
             (unsigned)DALY_MODBUS_REQUEST_ADDR,
             (unsigned)rs485GetBaudRate(uart));

    for (uint8_t i = 0u; i < sizeof(initCmds); i++) {
        uint8_t matched = dalyPollCommand(ctx, uart, dirPin, DALY_RS485_REQUEST_ADDR, initCmds[i]);
        if (matched == 0u) {
            (void)dalyPollCommand(ctx, uart, dirPin, DALY_UART_REQUEST_ADDR, initCmds[i]);
        }
        vTaskDelay(pdMS_TO_TICKS(DALY_RS485_QUERY_PERIOD_MS));
    }

    while (1) {
        uint8_t matchedThisRound = 0u;

        if (dalyPollModbusRead(ctx,
                               uart,
                               dirPin,
                               DALY_MODBUS_REQUEST_ADDR,
                               DALY_MODBUS_READ_CELLS_START,
                               DALY_MODBUS_READ_CELLS_COUNT)) {
            matchedThisRound++;
        } else if (dalyPollModbusRead(ctx,
                                      uart,
                                      dirPin,
                                      DALY_MODBUS_ALT_REQUEST_ADDR,
                                      DALY_MODBUS_READ_CELLS_START,
                                      DALY_MODBUS_READ_CELLS_COUNT)) {
            matchedThisRound++;
        }
        vTaskDelay(pdMS_TO_TICKS(DALY_RS485_QUERY_PERIOD_MS));

        if (matchedThisRound == 0u) {
            for (uint8_t i = 0u; i < sizeof(runCmds); i++) {
                uint8_t matched = dalyPollCommand(ctx, uart, dirPin, DALY_RS485_REQUEST_ADDR, runCmds[i]);
                if (matched == 0u) {
                    matched = dalyPollCommand(ctx, uart, dirPin, DALY_UART_REQUEST_ADDR, runCmds[i]);
                }
                matchedThisRound = (uint8_t)(matchedThisRound + matched);
                vTaskDelay(pdMS_TO_TICKS(DALY_RS485_QUERY_PERIOD_MS));
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(DALY_RS485_QUERY_PERIOD_MS));
        }

        int64_t nowUs = esp_timer_get_time();
        bool fresh = ctx->lastFrameUs > 0 &&
                     (nowUs - ctx->lastFrameUs) <= ((int64_t)DALY_RS485_SOURCE_STALE_MS * 1000LL);

        if (!fresh) {
            batteryModelClear();
            dalyClearLatest();
            if ((nowUs - g_lastStaleLogUs) >= 1000000LL) {
                ESP_LOGW(EXAMPLE_TAG,
                         "DALY RS485 source stale: no valid Daly classic/Modbus frames yet (matched_round=%u)",
                         (unsigned)matchedThisRound);
                g_lastStaleLogUs = nowUs;
            }
            continue;
        }

        if ((nowUs - ctx->lastPublishUs) >= ((int64_t)DALY_RS485_PUBLISH_PERIOD_MS * 1000LL)) {
            bms_decoded_packet_t packet = {0};
            ctx->snapshot.sequence = ++ctx->sequence;
            if (dalyRs485BuildDecodedPacket(&ctx->snapshot, ctx->sequence, &packet)) {
                dalyPublishBatteryModel(&ctx->snapshot);
                dalyStoreLatest(&ctx->snapshot, &packet);
                dalyLogSnapshot(&ctx->snapshot, &packet);
                if (ctx->outQueue != NULL && xQueueOverwrite(ctx->outQueue, &packet) != pdPASS) {
                    ESP_LOGW(EXAMPLE_TAG, "DALY RS485 output queue overwrite failed");
                }
            }
            ctx->lastPublishUs = nowUs;
        }
    }
}

esp_err_t dalyRs485BmsTaskStart(QueueHandle_t outQueue)
{
    if (outQueue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_dalyTaskHandle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&g_dalyCtx, 0, sizeof(g_dalyCtx));
    g_dalyCtx.outQueue = outQueue;
    g_lastStaleLogUs = 0;
    g_lastDecodeLogUs = 0;
    g_lastModbusLogUs = 0;
    batteryModelClear();
    dalyClearLatest();

    BaseType_t ok = xTaskCreate(dalyTask,
                                "daly_rs485",
                                DALY_RS485_TASK_STACK,
                                &g_dalyCtx,
                                DALY_RS485_TASK_PRIORITY,
                                &g_dalyTaskHandle);
    if (ok != pdPASS) {
        g_dalyTaskHandle = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t dalyRs485BmsTaskStop(void)
{
    if (g_dalyTaskHandle != NULL) {
        vTaskDelete(g_dalyTaskHandle);
        g_dalyTaskHandle = NULL;
    }
    memset(&g_dalyCtx, 0, sizeof(g_dalyCtx));
    dalyClearLatest();
    return ESP_OK;
}

bool dalyRs485BmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket)
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

bool dalyRs485BmsTaskGetLatestSnapshot(daly_rs485_snapshot_t *outSnapshot)
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
