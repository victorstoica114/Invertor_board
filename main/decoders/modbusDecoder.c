#include "decoders/modbusDecoder.h"
#include "config.h"
#include "protocols/growatt/growatt_registers_map.h"
#include "protocols/voltronic_modbus/voltronic_modbus_registers_map.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

#ifndef EXAMPLE_TAG
#define EXAMPLE_TAG "SNIFFER_BRIDGE"
#endif

#define MODBUS_DECODER_REQ_MAX_AGE_US 1000000LL
#define MODBUS_DECODER_LAST_REQ_MAX_AGE_US 500000LL

#if MODBUS_DECODER_SNAPSHOT_ONLY
#define MODBUS_RUNTIME_LOGI(...) do { if (0) { ESP_LOGI(EXAMPLE_TAG, __VA_ARGS__); } } while (0)
#else
#define MODBUS_RUNTIME_LOGI(...) ESP_LOGI(EXAMPLE_TAG, __VA_ARGS__)
#endif

static uint16_t be16(const uint8_t *p)
{
    return (uint16_t)((p[0] << 8) | p[1]);
}

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

static bool modbusCheckCrc(const uint8_t *frame, int len)
{
    if (len < 3) {
        return false;
    }
    uint16_t got = (uint16_t)(frame[len - 2] | (frame[len - 1] << 8));
    uint16_t calc = modbusCrc16(frame, len - 2);
    return got == calc;
}

static void dumpHexBrief(const uint8_t *buf, int len, char *out, int outSize)
{
    int pos = 0;
    int max = len;
    if (max > 64) {
        max = 64;
    }

    for (int i = 0; i < max; i++) {
        pos += snprintf(&out[pos], outSize - pos, "%02X ", buf[i]);
        if (pos >= outSize) {
            break;
        }
    }

    if (pos > 0) {
        out[pos - 1] = 0;
    } else {
        out[0] = 0;
    }
}

static void cacheStoreReg(modbusDecoder_t *d, uint16_t addr, uint16_t val, int64_t tsUs)
{
    int freeIdx = -1;
    int oldestIdx = 0;
    int64_t oldestTs = INT64_MAX;

    for (int i = 0; i < MODBUS_DECODER_CACHE_MAX_REGS; i++) {
        if (d->cacheValid[i]) {
            if (d->cacheAddr[i] == addr) {
                d->cacheVal[i] = val;
                d->cacheTsUs[i] = tsUs;
                return;
            }
            if (d->cacheTsUs[i] < oldestTs) {
                oldestTs = d->cacheTsUs[i];
                oldestIdx = i;
            }
        } else if (freeIdx < 0) {
            freeIdx = i;
        }
    }

    int idx = (freeIdx >= 0) ? freeIdx : oldestIdx;
    d->cacheAddr[idx] = addr;
    d->cacheVal[idx] = val;
    d->cacheTsUs[idx] = tsUs;
    d->cacheValid[idx] = 1;
}

static bool looksLikeVoltronicStatusBlock(const uint8_t *data, int regCount)
{
    if (data == NULL || regCount < 38) {
        return false;
    }

    const uint16_t cellCount = be16(&data[0]);
    if (cellCount == 0u || cellCount > VOLTRONIC_MB_MAX_CELLS ||
        cellCount >= (uint16_t)regCount) {
        return false;
    }

    int plausibleCells = 0;
    for (uint16_t i = 0u; i < cellCount; i++) {
        const uint16_t raw = be16(&data[(1u + i) * 2u]);
        if ((raw >= 10u && raw <= 60u) ||
            (raw >= 100u && raw <= 600u) ||
            (raw >= 1000u && raw <= 6000u)) {
            plausibleCells++;
        }
    }

    if (plausibleCells < ((cellCount >= 4u) ? 4 : (int)cellCount)) {
        return false;
    }

    const uint16_t soc = be16(&data[35 * 2]);
    if (soc > 100u) {
        return false;
    }

    return true;
}

static void reqQueueClear(modbusDecoder_t *d)
{
    if (d == NULL) {
        return;
    }

    d->reqQHead = 0u;
    d->reqQSize = 0u;
}

static void reqQueueDropAt(modbusDecoder_t *d, uint8_t logicalIndex)
{
    if (d == NULL || logicalIndex >= d->reqQSize) {
        return;
    }

    if (logicalIndex == 0u) {
        d->reqQHead = (uint8_t)((d->reqQHead + 1u) % MODBUS_DECODER_REQ_QUEUE_LEN);
        d->reqQSize--;
        return;
    }

    for (uint8_t i = logicalIndex; (uint8_t)(i + 1u) < d->reqQSize; i++) {
        const uint8_t from = (uint8_t)((d->reqQHead + i + 1u) % MODBUS_DECODER_REQ_QUEUE_LEN);
        const uint8_t to = (uint8_t)((d->reqQHead + i) % MODBUS_DECODER_REQ_QUEUE_LEN);
        d->reqQSlave[to] = d->reqQSlave[from];
        d->reqQFunc[to] = d->reqQFunc[from];
        d->reqQStart[to] = d->reqQStart[from];
        d->reqQCount[to] = d->reqQCount[from];
        d->reqQTsUs[to] = d->reqQTsUs[from];
    }

    d->reqQSize--;
}

static void reqQueuePruneExpired(modbusDecoder_t *d, int64_t nowUs)
{
    if (d == NULL) {
        return;
    }

    uint8_t i = 0u;
    while (i < d->reqQSize) {
        const uint8_t idx = (uint8_t)((d->reqQHead + i) % MODBUS_DECODER_REQ_QUEUE_LEN);
        const int64_t ageUs = nowUs - d->reqQTsUs[idx];
        if (ageUs > MODBUS_DECODER_REQ_MAX_AGE_US) {
            reqQueueDropAt(d, i);
            continue;
        }
        i++;
    }
}

static void reqQueuePush(modbusDecoder_t *d,
                         uint8_t slave,
                         uint8_t func,
                         uint16_t start,
                         uint16_t count,
                         int64_t tsUs)
{
    if (d == NULL || count == 0u) {
        return;
    }

    reqQueuePruneExpired(d, tsUs);

    if (d->reqQSize > 0u) {
        const uint8_t tailPrev = (uint8_t)((d->reqQHead + d->reqQSize - 1u) % MODBUS_DECODER_REQ_QUEUE_LEN);
        if (d->reqQSlave[tailPrev] == slave &&
            d->reqQFunc[tailPrev] == func &&
            d->reqQStart[tailPrev] == start &&
            d->reqQCount[tailPrev] == count) {
            int64_t dUs = tsUs - d->reqQTsUs[tailPrev];
            if (dUs < 0) {
                dUs = -dUs;
            }
            if (dUs < 50000LL) {
                return;
            }
        }
    }

    if (d->reqQSize >= MODBUS_DECODER_REQ_QUEUE_LEN) {
        d->reqQHead = (uint8_t)((d->reqQHead + 1u) % MODBUS_DECODER_REQ_QUEUE_LEN);
        d->reqQSize--;
    }

    const uint8_t tail = (uint8_t)((d->reqQHead + d->reqQSize) % MODBUS_DECODER_REQ_QUEUE_LEN);
    d->reqQSlave[tail] = slave;
    d->reqQFunc[tail] = func;
    d->reqQStart[tail] = start;
    d->reqQCount[tail] = count;
    d->reqQTsUs[tail] = tsUs;
    d->reqQSize++;
}

static bool reqQueuePopForResp(modbusDecoder_t *d,
                               uint8_t slave,
                               uint8_t func,
                               uint16_t respRegCount,
                               int64_t respTsUs,
                               uint16_t *startOut)
{
    if (d == NULL || d->reqQSize == 0u) {
        return false;
    }

    reqQueuePruneExpired(d, respTsUs);

    for (uint8_t n = 0u; n < d->reqQSize; n++) {
        const uint8_t i = (uint8_t)(d->reqQSize - 1u - n);
        const uint8_t idx = (uint8_t)((d->reqQHead + i) % MODBUS_DECODER_REQ_QUEUE_LEN);
        if (d->reqQSlave[idx] != slave || d->reqQFunc[idx] != func) {
            continue;
        }

        const int64_t ageUs = respTsUs - d->reqQTsUs[idx];
        if (ageUs < 0 || ageUs > MODBUS_DECODER_REQ_MAX_AGE_US) {
            continue;
        }

        const uint16_t reqCount = d->reqQCount[idx];
        if (reqCount != respRegCount) {
            continue;
        }

        if (startOut != NULL) {
            *startOut = d->reqQStart[idx];
        }

        reqQueueDropAt(d, i);
        return true;
    }

    return false;
}

static bool reqQueueHasForResp(modbusDecoder_t *d,
                               uint8_t slave,
                               uint8_t func,
                               uint16_t respRegCount,
                               int64_t respTsUs)
{
    if (d == NULL || d->reqQSize == 0u) {
        return false;
    }

    reqQueuePruneExpired(d, respTsUs);

    for (uint8_t i = 0u; i < d->reqQSize; i++) {
        const uint8_t idx = (uint8_t)((d->reqQHead + i) % MODBUS_DECODER_REQ_QUEUE_LEN);
        if (d->reqQSlave[idx] != slave || d->reqQFunc[idx] != func) {
            continue;
        }

        const int64_t ageUs = respTsUs - d->reqQTsUs[idx];
        if (ageUs < 0 || ageUs > MODBUS_DECODER_REQ_MAX_AGE_US) {
            continue;
        }

        if (d->reqQCount[idx] == respRegCount) {
            return true;
        }
    }

    return false;
}

static bool reqQueuePopFirstForFunc(modbusDecoder_t *d,
                                    uint8_t slave,
                                    uint8_t func,
                                    int64_t respTsUs,
                                    uint16_t *startOut,
                                    uint16_t *countOut)
{
    if (d == NULL || d->reqQSize == 0u) {
        return false;
    }

    reqQueuePruneExpired(d, respTsUs);

    for (uint8_t i = 0u; i < d->reqQSize; i++) {
        const uint8_t idx = (uint8_t)((d->reqQHead + i) % MODBUS_DECODER_REQ_QUEUE_LEN);
        if (d->reqQSlave[idx] != slave || d->reqQFunc[idx] != func) {
            continue;
        }

        const int64_t ageUs = respTsUs - d->reqQTsUs[idx];
        if (ageUs < 0 || ageUs > MODBUS_DECODER_REQ_MAX_AGE_US) {
            continue;
        }

        if (startOut != NULL) {
            *startOut = d->reqQStart[idx];
        }
        if (countOut != NULL) {
            *countOut = d->reqQCount[idx];
        }

        reqQueueDropAt(d, i);
        return true;
    }

    return false;
}

static void printReq03(modbusDecoder_t *d, const uint8_t *f, int len, bool crcOk)
{
    if (len < 8) {
        return;
    }

    uint8_t slave = f[0];
    uint8_t func = f[1];
    uint16_t start = be16(&f[2]);
    uint16_t count = be16(&f[4]);

    MODBUS_RUNTIME_LOGI(
             "REQ on %s: slave=%u func=0x%02X start=0x%04X count=0x%04X crc=%s",
             d->ifName,
             (unsigned)slave,
             (unsigned)func,
             (unsigned)start,
             (unsigned)count,
             crcOk ? "OK" : "BAD");

    d->lastReqValid = true;
    d->lastReqSlave = slave;
    d->lastReqFunc = func;
    d->lastReqStart = start;
    d->lastReqCount = count;
    d->lastReqUs = d->lastByteUs;
    reqQueuePush(d, slave, func, start, count, d->lastByteUs);
}

static bool isReadRegsFunc(uint8_t func)
{
    return func == 0x03u || func == 0x04u;
}

static void printExceptionResp(modbusDecoder_t *d, const uint8_t *frame, int len, bool crcOk)
{
    if (d == NULL || frame == NULL || len < 5) {
        return;
    }

    const uint8_t slave = frame[0];
    const uint8_t exceptionFunc = frame[1];
    const uint8_t func = (uint8_t)(exceptionFunc & 0x7Fu);
    const uint8_t code = frame[2];
    uint16_t start = 0xFFFFu;
    uint16_t count = 0u;

    if (isReadRegsFunc(func)) {
        (void)reqQueuePopFirstForFunc(d, slave, func, d->lastByteUs, &start, &count);
    }

    MODBUS_RUNTIME_LOGI(
             "EXCEPTION on %s: slave=%u func=0x%02X code=0x%02X start=0x%04X count=%u crc=%s",
             d->ifName,
             (unsigned)slave,
             (unsigned)func,
             (unsigned)code,
             (unsigned)start,
             (unsigned)count,
             crcOk ? "OK" : "BAD");
}

static void printResp03(modbusDecoder_t *d, const uint8_t *frame, int len, bool crcOk)
{
    if (len < 5) {
        MODBUS_RUNTIME_LOGI( "RESP on %s: too short (len=%d)", d->ifName, len);
        return;
    }

    uint8_t slave = frame[0];
    uint8_t func = frame[1];

    if (!isReadRegsFunc(func)) {
        MODBUS_RUNTIME_LOGI( "RESP on %s: func=0x%02X (not read-registers)", d->ifName, func);
        return;
    }

    uint8_t byteCount = frame[2];
    int dataLen = (int)byteCount;

    if (3 + dataLen + 2 > len) {
        MODBUS_RUNTIME_LOGI(
                 "RESP on %s: length mismatch (len=%d byteCount=%u)",
                 d->ifName,
                 len,
                 (unsigned)byteCount);
        return;
    }

    int regCount = dataLen / 2;
    const uint8_t *data = &frame[3];

    uint16_t startBase = 0xFFFF;
    const bool isVoltronicStatusPayload = looksLikeVoltronicStatusBlock(data, regCount);
    if (isVoltronicStatusPayload) {
        startBase = VOLTRONIC_MB_REG_STATUS_START;
        reqQueueClear(d);
        MODBUS_RUNTIME_LOGI(
                 "RESP on %s: slave=%u func=0x%02X start=0x%04X regs=%d crc=%s (auto Voltronic status)",
                 d->ifName,
                 (unsigned)slave,
                 (unsigned)func,
                 (unsigned)startBase,
                 regCount,
                 crcOk ? "OK" : "BAD");
    } else if (reqQueuePopForResp(d, slave, func, (uint16_t)regCount, d->lastByteUs, &startBase)) {
        MODBUS_RUNTIME_LOGI(
                 "RESP on %s: slave=%u func=0x%02X start=0x%04X regs=%d crc=%s",
                 d->ifName,
                 (unsigned)slave,
                 (unsigned)func,
                 (unsigned)startBase,
                 regCount,
                 crcOk ? "OK" : "BAD");
    } else if (d->reqQSize == 0u &&
               d->lastReqValid &&
               d->lastReqSlave == slave &&
               d->lastReqFunc == func &&
               d->lastReqCount == (uint16_t)regCount &&
               (d->lastByteUs - d->lastReqUs) >= 0 &&
               (d->lastByteUs - d->lastReqUs) < MODBUS_DECODER_LAST_REQ_MAX_AGE_US) {
        startBase = d->lastReqStart;
        MODBUS_RUNTIME_LOGI(
                 "RESP on %s: slave=%u func=0x%02X start=0x%04X regs=%d crc=%s",
                 d->ifName,
                 (unsigned)slave,
                 (unsigned)func,
                 (unsigned)startBase,
                 regCount,
                 crcOk ? "OK" : "BAD");
    } else {
        MODBUS_RUNTIME_LOGI(
                 "RESP on %s: slave=%u func=0x%02X regs=%d crc=%s",
                 d->ifName,
                 (unsigned)slave,
                 (unsigned)func,
                 regCount,
                 crcOk ? "OK" : "BAD");
    }

    bool isCellBlock = (startBase != 0xFFFF && startBase == GROWATT_MB_REG_CELL_BASE);
    bool isMainBlock = (startBase != 0xFFFF && startBase == GROWATT_MB_REG_MAIN_START);
    bool isInfoBlock = (startBase != 0xFFFF && startBase == GROWATT_MB_REG_INFO_0001);

    if (isCellBlock) {
        int cellRegs = regCount;
        if (cellRegs > 16) {
            cellRegs = 16;
        }

        for (int i = 0; i < cellRegs; i++) {
            uint16_t mv = be16(&data[i * 2]);
            uint16_t addr = (uint16_t)(startBase + i);
            uint16_t cellIndex = (uint16_t)(i + 1);

            cacheStoreReg(d, addr, mv, d->lastByteUs);

            MODBUS_RUNTIME_LOGI(
                     "  Cell%02u @0x%04X = %.3f V (%u mV)",
                     (unsigned)cellIndex,
                     (unsigned)addr,
                     (double)mv / 1000.0,
                     (unsigned)mv);
        }

        for (int i = 16; i < regCount; i++) {
            uint16_t v = be16(&data[i * 2]);
            uint16_t addr = (uint16_t)(startBase + i);
            cacheStoreReg(d, addr, v, d->lastByteUs);
            MODBUS_RUNTIME_LOGI(
                     "  reg[0x%04X] = 0x%04X (%u)",
                     (unsigned)addr,
                     (unsigned)v,
                     (unsigned)v);
        }

        return;
    }

    if (isMainBlock) {
        bool socValid = false;
        bool packValid = false;
        bool tempValid = false;
        bool sohValid = false;
        bool cvTargetValid = false;
        bool remainCapValid = false;
        bool fullCapValid = false;
        bool packAbsIValid = false;
        bool cycleCountValid = false;
        bool ichgLimValid = false;
        bool idisLimValid = false;
        bool minCellValid = false;
        bool maxCellValid = false;
        bool minIdxValid = false;
        bool maxIdxValid = false;

        uint16_t socPct = 0;
        uint16_t packCv = 0;
        int16_t tempC = 0;
        uint16_t sohPct = 0;
        uint16_t cvTargetCv = 0;
        uint16_t remainCapCah = 0;
        uint16_t fullCapCah = 0;
        uint16_t packAbsICa = 0;
        uint16_t cycleCount = 0;
        uint16_t ichgLimCa = 0;
        uint16_t idisLimCa = 0;

        uint16_t minCellMv = 0;
        uint16_t maxCellMv = 0;
        uint16_t minCellIdx = 0;
        uint16_t maxCellIdx = 0;

        for (int i = 0; i < regCount; i++) {
            uint16_t v = be16(&data[i * 2]);
            uint16_t addr = (uint16_t)(startBase + i);
            cacheStoreReg(d, addr, v, d->lastByteUs);

            switch (addr) {
                case GROWATT_MB_REG_SOC_PCT:
                    socPct = v;
                    socValid = true;
                    break;
                case GROWATT_MB_REG_PACK_V_CV:
                    packCv = v;
                    packValid = true;
                    break;
                case GROWATT_MB_REG_TEMP_C:
                    tempC = (int16_t)v;
                    tempValid = true;
                    break;
                case GROWATT_MB_REG_SOH_PCT:
                    sohPct = v;
                    sohValid = true;
                    break;
                case GROWATT_MB_REG_CV_TARGET_CV:
                    cvTargetCv = v;
                    cvTargetValid = true;
                    break;
                case GROWATT_MB_REG_REMAIN_CAP_CAH:
                    remainCapCah = v;
                    remainCapValid = true;
                    break;
                case GROWATT_MB_REG_FULL_CAP_CAH:
                    fullCapCah = v;
                    fullCapValid = true;
                    break;
                case GROWATT_MB_REG_PACK_I_ABS_CA_TENTATIVE:
                    packAbsICa = v;
                    packAbsIValid = true;
                    break;
                case GROWATT_MB_REG_CYCLE_COUNT_TENTATIVE:
                    cycleCount = v;
                    cycleCountValid = true;
                    break;
                case GROWATT_MB_REG_ICHG_LIM_CA_TENTATIVE:
                    ichgLimCa = v;
                    ichgLimValid = true;
                    break;
                case GROWATT_MB_REG_IDIS_LIM_CA_TENTATIVE:
                    idisLimCa = v;
                    idisLimValid = true;
                    break;
                case GROWATT_MB_REG_CELL_MAX_MV:
                    maxCellMv = v;
                    maxCellValid = true;
                    break;
                case GROWATT_MB_REG_CELL_MIN_MV:
                    minCellMv = v;
                    minCellValid = true;
                    break;
                case GROWATT_MB_REG_CELL_MAX_IDX:
                    maxCellIdx = v;
                    maxIdxValid = true;
                    break;
                case GROWATT_MB_REG_CELL_MIN_IDX:
                    minCellIdx = v;
                    minIdxValid = true;
                    break;
                default:
#if REG_RAW_VALUES
                    MODBUS_RUNTIME_LOGI(
                             "  reg[0x%04X] = 0x%04X (%u)",
                             (unsigned)addr,
                             (unsigned)v,
                             (unsigned)v);
#endif
                    break;
            }
        }

        if (packValid && socValid && tempValid && minCellValid && maxCellValid && minIdxValid && maxIdxValid) {
            double packV = (double)packCv / 100.0;
            double minV = (double)minCellMv / 1000.0;
            double maxV = (double)maxCellMv / 1000.0;
            double deltaV = (double)(maxCellMv - minCellMv) / 1000.0;

            MODBUS_RUNTIME_LOGI(
                     "BMS: %.2fV | SOC %u%% | %dC | Cmin %.3fV(C%u) | Cmax %.3fV(C%u) | dV %.3fV",
                     packV,
                     socPct,
                     tempC,
                     minV,
                     minCellIdx,
                     maxV,
                     maxCellIdx,
                     deltaV);
        }

        if (remainCapValid && fullCapValid) {
            MODBUS_RUNTIME_LOGI(
                     "BMS-CAP: RM %.2fAh | FCC %.2fAh",
                     (double)remainCapCah / 100.0,
                     (double)fullCapCah / 100.0);
        }

        if (cvTargetValid || sohValid) {
            if (cvTargetValid && sohValid) {
                MODBUS_RUNTIME_LOGI(
                         "BMS-EXT: CVtarget %.2fV | SOH %u%%",
                         (double)cvTargetCv / 100.0,
                         (unsigned)sohPct);
            } else if (cvTargetValid) {
                MODBUS_RUNTIME_LOGI("BMS-EXT: CVtarget %.2fV", (double)cvTargetCv / 100.0);
            } else {
                MODBUS_RUNTIME_LOGI("BMS-EXT: SOH %u%%", (unsigned)sohPct);
            }
        }

        if (packAbsIValid) {
            MODBUS_RUNTIME_LOGI("BMS-TENT: |Ipack| %.2fA", (double)packAbsICa / 100.0);
        }
        if (cycleCountValid) {
            MODBUS_RUNTIME_LOGI("BMS-TENT: Cycles %u", (unsigned)cycleCount);
        }
        if (ichgLimValid || idisLimValid) {
            MODBUS_RUNTIME_LOGI(
                     "BMS-TENT: IchgLim %.2fA | IdisLim %.2fA",
                     ichgLimValid ? ((double)ichgLimCa / 100.0) : -1.0,
                     idisLimValid ? ((double)idisLimCa / 100.0) : -1.0);
        }

        return;
    }

    if (isInfoBlock) {
        uint16_t f0 = be16(&data[0]);
        uint16_t f1 = be16(&data[2]);
        uint16_t f2 = be16(&data[4]);
        uint16_t f3 = be16(&data[6]);

        MODBUS_RUNTIME_LOGI(
                 "BMS-INFO: r0001..0004 = %04X %04X %04X %04X",
                 (unsigned)f0,
                 (unsigned)f1,
                 (unsigned)f2,
                 (unsigned)f3);

        for (int i = 0; i < regCount; i++) {
            uint16_t v = be16(&data[i * 2]);
            uint16_t addr = (uint16_t)(startBase + i);
            cacheStoreReg(d, addr, v, d->lastByteUs);
#if REG_RAW_VALUES
            MODBUS_RUNTIME_LOGI(
                     "  reg[0x%04X] = 0x%04X (%u)",
                     (unsigned)addr,
                     (unsigned)v,
                     (unsigned)v);
#endif
        }
        return;
    }

    for (int i = 0; i < regCount; i++) {
        uint16_t v = be16(&data[i * 2]);
        uint16_t addr = (startBase != 0xFFFF) ? (uint16_t)(startBase + i) : 0xFFFF;

        if (addr != 0xFFFF) {
            cacheStoreReg(d, addr, v, d->lastByteUs);
            MODBUS_RUNTIME_LOGI(
                     "  reg[0x%04X] = 0x%04X (%u)",
                     (unsigned)addr,
                     (unsigned)v,
                     (unsigned)v);
        } else {
            MODBUS_RUNTIME_LOGI(
                     "  reg[%02d] = 0x%04X (%u)",
                     i,
                     (unsigned)v,
                     (unsigned)v);
        }
    }
}

static void printFrameGeneric(modbusDecoder_t *d, const uint8_t *f, int len, bool crcOk)
{
    char hex[3 * 64 + 1];
    dumpHexBrief(f, len, hex, sizeof(hex));
    MODBUS_RUNTIME_LOGI(
             "FRAME on %s: len=%d crc=%s HEX(first64)=[%s]%s",
             d->ifName,
             len,
             crcOk ? "OK" : "BAD",
             hex,
             (len > 64) ? " ..." : "");
}

static bool isReq03Frame(const uint8_t *f, int len)
{
    if (f == NULL || len != 8) {
        return false;
    }
    if (!isReadRegsFunc(f[1])) {
        return false;
    }
    const uint16_t count = be16(&f[4]);
    if (count == 0u || count > 125u) {
        return false;
    }
    return modbusCheckCrc(f, len);
}

static bool isVoltronicReq03Frame(const uint8_t *f, int len)
{
    if (f == NULL || len != 8) {
        return false;
    }
    if (!isReadRegsFunc(f[0]) || f[1] == 0u || f[1] > 16u) {
        return false;
    }
    const uint16_t count = be16(&f[4]);
    if (count == 0u || count > 125u) {
        return false;
    }
    return modbusCheckCrc(f, len);
}

static bool isResp03Frame(const uint8_t *f, int len)
{
    if (f == NULL || len < 5) {
        return false;
    }
    if (!isReadRegsFunc(f[1])) {
        return false;
    }
    const int expLen = (int)f[2] + 5;
    if (len != expLen) {
        return false;
    }
    return modbusCheckCrc(f, len);
}

static bool isWordCountResp03Frame(const uint8_t *f, int len)
{
    if (f == NULL || len < 6) {
        return false;
    }
    if (!isReadRegsFunc(f[1])) {
        return false;
    }
    const uint16_t regCount = be16(&f[2]);
    if (regCount == 0u || regCount > 125u) {
        return false;
    }
    const int expLen = 4 + ((int)regCount * 2) + 2;
    if (len != expLen) {
        return false;
    }
    return modbusCheckCrc(f, len);
}

static bool isWideByteCountResp03Frame(const uint8_t *f, int len)
{
    if (f == NULL || len < 6) {
        return false;
    }
    if (!isReadRegsFunc(f[1])) {
        return false;
    }
    const uint16_t byteCount = be16(&f[2]);
    if (byteCount == 0u || (byteCount & 1u) != 0u || byteCount > 250u) {
        return false;
    }
    const int expLen = 4 + (int)byteCount + 2;
    if (len != expLen) {
        return false;
    }
    return modbusCheckCrc(f, len);
}

static bool isExceptionRespFrame(const uint8_t *f, int len)
{
    if (f == NULL || len != 5) {
        return false;
    }
    if ((f[1] & 0x80u) == 0u || !isReadRegsFunc((uint8_t)(f[1] & 0x7Fu))) {
        return false;
    }
    return modbusCheckCrc(f, len);
}

static bool isVoltronicResp03Frame(const uint8_t *f, int len)
{
    if (f == NULL || len < 5) {
        return false;
    }
    if (!isReadRegsFunc(f[0]) || f[1] == 0u || f[1] > 16u) {
        return false;
    }
    const int expLen = (int)f[2] + 5;
    if (len != expLen) {
        return false;
    }
    return modbusCheckCrc(f, len);
}

static bool isVoltronicWordCountResp03Frame(const uint8_t *f, int len)
{
    if (f == NULL || len < 6) {
        return false;
    }
    if (!isReadRegsFunc(f[0]) || f[1] == 0u || f[1] > 16u) {
        return false;
    }
    const uint16_t regCount = be16(&f[2]);
    if (regCount == 0u || regCount > 125u) {
        return false;
    }
    const int expLen = 4 + ((int)regCount * 2) + 2;
    if (len != expLen) {
        return false;
    }
    return modbusCheckCrc(f, len);
}

static bool isVoltronicWideByteCountResp03Frame(const uint8_t *f, int len)
{
    if (f == NULL || len < 6) {
        return false;
    }
    if (!isReadRegsFunc(f[0]) || f[1] == 0u || f[1] > 16u) {
        return false;
    }
    const uint16_t byteCount = be16(&f[2]);
    if (byteCount == 0u || (byteCount & 1u) != 0u || byteCount > 250u) {
        return false;
    }
    const int expLen = 4 + (int)byteCount + 2;
    if (len != expLen) {
        return false;
    }
    return modbusCheckCrc(f, len);
}

static void normalizeVoltronic03Frame(const uint8_t *src, int len, uint8_t *dst, int dstCap)
{
    if (src == NULL || dst == NULL || len <= 0 || dstCap < len) {
        return;
    }

    dst[0] = src[1];
    dst[1] = src[0];
    if (len > 2) {
        memcpy(&dst[2], &src[2], (size_t)(len - 2));
    }
}

static int normalizeWordCountResp03Frame(const uint8_t *src,
                                         int len,
                                         bool functionFirst,
                                         uint8_t *dst,
                                         int dstCap)
{
    if (src == NULL || dst == NULL || len < 6) {
        return 0;
    }

    const uint16_t regCount = be16(&src[2]);
    const int dataLen = (int)regCount * 2;
    const int normLen = 3 + dataLen + 2;
    if (regCount == 0u || regCount > 125u ||
        (4 + dataLen + 2) > len ||
        dstCap < normLen) {
        return 0;
    }

    if (functionFirst) {
        dst[0] = src[1];
        dst[1] = src[0];
    } else {
        dst[0] = src[0];
        dst[1] = src[1];
    }
    dst[2] = (uint8_t)dataLen;
    memcpy(&dst[3], &src[4], (size_t)dataLen);
    dst[normLen - 2] = src[len - 2];
    dst[normLen - 1] = src[len - 1];
    return normLen;
}

static int normalizeWideByteCountResp03Frame(const uint8_t *src,
                                             int len,
                                             bool functionFirst,
                                             uint8_t *dst,
                                             int dstCap)
{
    if (src == NULL || dst == NULL || len < 6) {
        return 0;
    }

    const uint16_t byteCount = be16(&src[2]);
    const int dataLen = (int)byteCount;
    const int normLen = 3 + dataLen + 2;
    if (byteCount == 0u || (byteCount & 1u) != 0u || byteCount > 250u ||
        (4 + dataLen + 2) > len ||
        dstCap < normLen) {
        return 0;
    }

    if (functionFirst) {
        dst[0] = src[1];
        dst[1] = src[0];
    } else {
        dst[0] = src[0];
        dst[1] = src[1];
    }
    dst[2] = (uint8_t)dataLen;
    memcpy(&dst[3], &src[4], (size_t)dataLen);
    dst[normLen - 2] = src[len - 2];
    dst[normLen - 1] = src[len - 1];
    return normLen;
}

static int decodeFrame(modbusDecoder_t *d, const uint8_t *f, int len, bool fallbackLog)
{
    if (len < 4) {
        if (fallbackLog) {
            printFrameGeneric(d, f, len, false);
        }
        return 0;
    }

    /* Handle concatenated RTU frames (e.g. echoed request + response in one buffer). */
    int off = 0;
    int consumedTo = 0;
    bool decodedAny = false;
    while ((len - off) >= 4) {
        const uint8_t *cur = &f[off];
        const int rem = len - off;

        if (rem >= 6) {
            const uint16_t byteCount = be16(&cur[2]);
            const uint16_t respRegCount = (uint16_t)(byteCount / 2u);
            const int wideRespLen = 4 + (int)byteCount + 2;

            if (wideRespLen >= 6 &&
                wideRespLen <= rem &&
                isWideByteCountResp03Frame(cur, wideRespLen) &&
                reqQueueHasForResp(d, cur[0], cur[1], respRegCount, d->lastByteUs)) {
                uint8_t normalized[256];
                int normLen = normalizeWideByteCountResp03Frame(cur,
                                                                wideRespLen,
                                                                false,
                                                                normalized,
                                                                sizeof(normalized));
                if (normLen > 0) {
                    printResp03(d, normalized, normLen, true);
                    off += wideRespLen;
                    consumedTo = off;
                    decodedAny = true;
                    continue;
                }
            }

            if (wideRespLen >= 6 &&
                wideRespLen <= rem &&
                isVoltronicWideByteCountResp03Frame(cur, wideRespLen) &&
                reqQueueHasForResp(d, cur[1], cur[0], respRegCount, d->lastByteUs)) {
                uint8_t normalized[256];
                int normLen = normalizeWideByteCountResp03Frame(cur,
                                                                wideRespLen,
                                                                true,
                                                                normalized,
                                                                sizeof(normalized));
                if (normLen > 0) {
                    printResp03(d, normalized, normLen, true);
                    off += wideRespLen;
                    consumedTo = off;
                    decodedAny = true;
                    continue;
                }
            }
        }

        if (rem >= 8 && isReq03Frame(cur, 8)) {
            printReq03(d, cur, 8, true);
            off += 8;
            consumedTo = off;
            decodedAny = true;
            continue;
        }

        if (rem >= 8 && isVoltronicReq03Frame(cur, 8)) {
            uint8_t normalized[8];
            normalizeVoltronic03Frame(cur, 8, normalized, sizeof(normalized));
            printReq03(d, normalized, 8, true);
            off += 8;
            consumedTo = off;
            decodedAny = true;
            continue;
        }

        if (rem >= 5) {
            if (isExceptionRespFrame(cur, 5)) {
                printExceptionResp(d, cur, 5, true);
                off += 5;
                consumedTo = off;
                decodedAny = true;
                continue;
            }

            const int respLen = (int)cur[2] + 5;
            if (respLen >= 5 && respLen <= rem && isResp03Frame(cur, respLen)) {
                printResp03(d, cur, respLen, true);
                off += respLen;
                consumedTo = off;
                decodedAny = true;
                continue;
            }

            if (respLen >= 5 && respLen <= rem && isVoltronicResp03Frame(cur, respLen)) {
                uint8_t normalized[256];
                normalizeVoltronic03Frame(cur, respLen, normalized, sizeof(normalized));
                printResp03(d, normalized, respLen, true);
                off += respLen;
                consumedTo = off;
                decodedAny = true;
                continue;
            }

            const uint16_t byteCount = be16(&cur[2]);
            const int wideRespLen = 4 + (int)byteCount + 2;
            if (wideRespLen >= 6 && wideRespLen <= rem && isWideByteCountResp03Frame(cur, wideRespLen)) {
                uint8_t normalized[256];
                int normLen = normalizeWideByteCountResp03Frame(cur,
                                                                wideRespLen,
                                                                false,
                                                                normalized,
                                                                sizeof(normalized));
                if (normLen > 0) {
                    printResp03(d, normalized, normLen, true);
                    off += wideRespLen;
                    consumedTo = off;
                    decodedAny = true;
                    continue;
                }
            }

            if (wideRespLen >= 6 &&
                wideRespLen <= rem &&
                isVoltronicWideByteCountResp03Frame(cur, wideRespLen)) {
                uint8_t normalized[256];
                int normLen = normalizeWideByteCountResp03Frame(cur,
                                                                wideRespLen,
                                                                true,
                                                                normalized,
                                                                sizeof(normalized));
                if (normLen > 0) {
                    printResp03(d, normalized, normLen, true);
                    off += wideRespLen;
                    consumedTo = off;
                    decodedAny = true;
                    continue;
                }
            }

            const uint16_t regCount = be16(&cur[2]);
            const int wordRespLen = 4 + ((int)regCount * 2) + 2;
            if (wordRespLen >= 6 && wordRespLen <= rem && isWordCountResp03Frame(cur, wordRespLen)) {
                uint8_t normalized[256];
                int normLen = normalizeWordCountResp03Frame(cur,
                                                            wordRespLen,
                                                            false,
                                                            normalized,
                                                            sizeof(normalized));
                if (normLen > 0) {
                    printResp03(d, normalized, normLen, true);
                    off += wordRespLen;
                    consumedTo = off;
                    decodedAny = true;
                    continue;
                }
            }

            if (wordRespLen >= 6 &&
                wordRespLen <= rem &&
                isVoltronicWordCountResp03Frame(cur, wordRespLen)) {
                uint8_t normalized[256];
                int normLen = normalizeWordCountResp03Frame(cur,
                                                            wordRespLen,
                                                            true,
                                                            normalized,
                                                            sizeof(normalized));
                if (normLen > 0) {
                    printResp03(d, normalized, normLen, true);
                    off += wordRespLen;
                    consumedTo = off;
                    decodedAny = true;
                    continue;
                }
            }
        }

        off++;
    }

    if (decodedAny) {
        return consumedTo;
    }

    if (!fallbackLog) {
        return 0;
    }

    bool crcOk = modbusCheckCrc(f, len);
    uint8_t func = f[1];

    if (isReadRegsFunc(func) && len == 8) {
        printReq03(d, f, len, crcOk);
    } else if (isExceptionRespFrame(f, len)) {
        printExceptionResp(d, f, len, crcOk);
    } else if (isReadRegsFunc(func) && len >= 5) {
        printResp03(d, f, len, crcOk);
    } else if (len == 8 && isVoltronicReq03Frame(f, len)) {
        uint8_t normalized[8];
        normalizeVoltronic03Frame(f, len, normalized, sizeof(normalized));
        printReq03(d, normalized, len, true);
    } else if (len >= 5 && isVoltronicResp03Frame(f, len)) {
        uint8_t normalized[256];
        normalizeVoltronic03Frame(f, len, normalized, sizeof(normalized));
        printResp03(d, normalized, len, true);
    } else if (len >= 6 && isWordCountResp03Frame(f, len)) {
        uint8_t normalized[256];
        int normLen = normalizeWordCountResp03Frame(f, len, false, normalized, sizeof(normalized));
        if (normLen > 0) {
            printResp03(d, normalized, normLen, true);
        } else {
            printFrameGeneric(d, f, len, crcOk);
        }
    } else if (len >= 6 && isWideByteCountResp03Frame(f, len)) {
        uint8_t normalized[256];
        int normLen = normalizeWideByteCountResp03Frame(f, len, false, normalized, sizeof(normalized));
        if (normLen > 0) {
            printResp03(d, normalized, normLen, true);
        } else {
            printFrameGeneric(d, f, len, crcOk);
        }
    } else if (len >= 6 && isVoltronicWideByteCountResp03Frame(f, len)) {
        uint8_t normalized[256];
        int normLen = normalizeWideByteCountResp03Frame(f, len, true, normalized, sizeof(normalized));
        if (normLen > 0) {
            printResp03(d, normalized, normLen, true);
        } else {
            printFrameGeneric(d, f, len, crcOk);
        }
    } else if (len >= 6 && isVoltronicWordCountResp03Frame(f, len)) {
        uint8_t normalized[256];
        int normLen = normalizeWordCountResp03Frame(f, len, true, normalized, sizeof(normalized));
        if (normLen > 0) {
            printResp03(d, normalized, normLen, true);
        } else {
            printFrameGeneric(d, f, len, crcOk);
        }
    } else {
        printFrameGeneric(d, f, len, crcOk);
    }

    return 0;
}

void modbusDecoderInit(modbusDecoder_t *d, const char *ifName, uint32_t gapUs)
{
    memset(d, 0, sizeof(*d));
    d->ifName = ifName;
    d->gapUs = gapUs;
}

void modbusDecoderRecordRequest(modbusDecoder_t *d,
                                uint8_t slave,
                                uint8_t func,
                                uint16_t start,
                                uint16_t count,
                                int64_t tsUs)
{
    if (d == NULL || count == 0u) {
        return;
    }

    d->lastReqValid = true;
    d->lastReqSlave = slave;
    d->lastReqFunc = func;
    d->lastReqStart = start;
    d->lastReqCount = count;
    d->lastReqUs = tsUs;

    reqQueuePush(d, slave, func, start, count, tsUs);
}

static void finalizeIfAny(modbusDecoder_t *d)
{
    if (d->len > 0) {
        (void)decodeFrame(d, d->buf, d->len, true);
        d->len = 0;
    }
}

void modbusDecoderFeed(modbusDecoder_t *d, const uint8_t *data, int len, int64_t rxUs)
{
    if (len <= 0) {
        return;
    }

    if (d->haveLastByte) {
        int64_t delta = rxUs - d->lastByteUs;
        if (delta > (int64_t)d->gapUs) {
            finalizeIfAny(d);
        }
    }

    d->lastByteUs = rxUs;
    d->haveLastByte = true;

    for (int i = 0; i < len; i++) {
        if (d->len >= sizeof(d->buf)) {
            if (decodeFrame(d, d->buf, d->len, false) > 0) {
                ESP_LOGW(EXAMPLE_TAG,
                         "Decoder on %s recovered valid Modbus frame before overflow",
                         d->ifName);
            } else {
                ESP_LOGW(EXAMPLE_TAG, "Decoder on %s overflow, dropping frame", d->ifName);
            }
            d->len = 0;
        }
        d->buf[d->len++] = data[i];
    }

    if (d->len > 192u) {
        const int consumed = decodeFrame(d, d->buf, d->len, false);
        if (consumed > 0) {
            if (consumed < (int)d->len) {
                const uint16_t remaining = (uint16_t)((int)d->len - consumed);
                memmove(d->buf, &d->buf[consumed], remaining);
                d->len = remaining;
            } else {
                d->len = 0;
            }
        }
    }
}

void modbusDecoderFlush(modbusDecoder_t *d)
{
    finalizeIfAny(d);
    d->haveLastByte = false;
}

static const char *snapshotIfName(const modbusDecoder_t *d)
{
    return (d != NULL && d->ifName != NULL) ? d->ifName : "RS485";
}

bool modbusDecoderGetCachedReg(const modbusDecoder_t *d, uint16_t addr, uint16_t *valOut)
{
    if (d == NULL) {
        return false;
    }

    for (int i = 0; i < MODBUS_DECODER_CACHE_MAX_REGS; i++) {
        if (!d->cacheValid[i]) {
            continue;
        }
        if (d->cacheAddr[i] != addr) {
            continue;
        }
        if (valOut) {
            *valOut = d->cacheVal[i];
        }
        return true;
    }

    return false;
}

int64_t modbusDecoderGetNewestCacheTsUs(const modbusDecoder_t *d)
{
    int64_t newestTsUs = 0;

    if (d == NULL) {
        return 0;
    }

    for (int i = 0; i < MODBUS_DECODER_CACHE_MAX_REGS; i++) {
        if (!d->cacheValid[i]) {
            continue;
        }
        if (d->cacheTsUs[i] > newestTsUs) {
            newestTsUs = d->cacheTsUs[i];
        }
    }

    return newestTsUs;
}

static void printSnapshotDecoded(modbusDecoder_t *d)
{
    const char *ifn = snapshotIfName(d);

    uint16_t r0001 = 0;
    uint16_t r0002 = 0;
    uint16_t r0003 = 0;
    uint16_t r0004 = 0;
    if (modbusDecoderGetCachedReg(d, GROWATT_MB_REG_INFO_0001, &r0001) &&
        modbusDecoderGetCachedReg(d, GROWATT_MB_REG_INFO_0002, &r0002) &&
        modbusDecoderGetCachedReg(d, GROWATT_MB_REG_INFO_0003, &r0003) &&
        modbusDecoderGetCachedReg(d, GROWATT_MB_REG_INFO_0004, &r0004)) {
        ESP_LOGI(EXAMPLE_TAG,
                 "%s BMS-INFO: r0001..0004 = %04X %04X %04X %04X",
                 ifn,
                 (unsigned)r0001,
                 (unsigned)r0002,
                 (unsigned)r0003,
                 (unsigned)r0004);
    }

    uint16_t soc = 0;
    uint16_t packCv = 0;
    uint16_t temp = 0;
    uint16_t cmaxMv = 0;
    uint16_t cminMv = 0;
    uint16_t cmaxIdx = 0;
    uint16_t cminIdx = 0;
    uint16_t soh = 0;
    uint16_t cvTargetCv = 0;
    uint16_t remainCapCah = 0;
    uint16_t fullCapCah = 0;
    uint16_t packAbsICa = 0;
    uint16_t cycleCount = 0;
    uint16_t ichgLimCa = 0;
    uint16_t idisLimCa = 0;

    if (modbusDecoderGetCachedReg(d, GROWATT_MB_REG_SOC_PCT, &soc) &&
        modbusDecoderGetCachedReg(d, GROWATT_MB_REG_PACK_V_CV, &packCv) &&
        modbusDecoderGetCachedReg(d, GROWATT_MB_REG_TEMP_C, &temp) &&
        modbusDecoderGetCachedReg(d, GROWATT_MB_REG_CELL_MAX_MV, &cmaxMv) &&
        modbusDecoderGetCachedReg(d, GROWATT_MB_REG_CELL_MIN_MV, &cminMv) &&
        modbusDecoderGetCachedReg(d, GROWATT_MB_REG_CELL_MAX_IDX, &cmaxIdx) &&
        modbusDecoderGetCachedReg(d, GROWATT_MB_REG_CELL_MIN_IDX, &cminIdx)) {
        ESP_LOGI(EXAMPLE_TAG,
                 "%s BMS: %.2fV | SOC %u%% | %dC | Cmin %.3fV(C%u) | Cmax %.3fV(C%u) | dV %.3fV",
                 ifn,
                 (double)packCv / 100.0,
                 (unsigned)soc,
                 (int16_t)temp,
                 (double)cminMv / 1000.0,
                 (unsigned)cminIdx,
                 (double)cmaxMv / 1000.0,
                 (unsigned)cmaxIdx,
                 (double)(cmaxMv - cminMv) / 1000.0);
    }

    if (modbusDecoderGetCachedReg(d, GROWATT_MB_REG_REMAIN_CAP_CAH, &remainCapCah) &&
        modbusDecoderGetCachedReg(d, GROWATT_MB_REG_FULL_CAP_CAH, &fullCapCah)) {
        ESP_LOGI(EXAMPLE_TAG,
                 "%s BMS-CAP: RM %.2fAh | FCC %.2fAh",
                 ifn,
                 (double)remainCapCah / 100.0,
                 (double)fullCapCah / 100.0);
    }

    if (modbusDecoderGetCachedReg(d, GROWATT_MB_REG_CV_TARGET_CV, &cvTargetCv) &&
        modbusDecoderGetCachedReg(d, GROWATT_MB_REG_SOH_PCT, &soh)) {
        ESP_LOGI(EXAMPLE_TAG,
                 "%s BMS-EXT: CVtarget %.2fV | SOH %u%%",
                 ifn,
                 (double)cvTargetCv / 100.0,
                 (unsigned)soh);
    }

    if (modbusDecoderGetCachedReg(d, GROWATT_MB_REG_PACK_I_ABS_CA_TENTATIVE, &packAbsICa)) {
        ESP_LOGI(EXAMPLE_TAG,
                 "%s BMS-TENT: |Ipack| %.2fA",
                 ifn,
                 (double)packAbsICa / 100.0);
    }
    if (modbusDecoderGetCachedReg(d, GROWATT_MB_REG_CYCLE_COUNT_TENTATIVE, &cycleCount)) {
        ESP_LOGI(EXAMPLE_TAG,
                 "%s BMS-TENT: Cycles %u",
                 ifn,
                 (unsigned)cycleCount);
    }
    bool hasIchgLim = modbusDecoderGetCachedReg(d, GROWATT_MB_REG_ICHG_LIM_CA_TENTATIVE, &ichgLimCa);
    bool hasIdisLim = modbusDecoderGetCachedReg(d, GROWATT_MB_REG_IDIS_LIM_CA_TENTATIVE, &idisLimCa);
    if (hasIchgLim || hasIdisLim) {
        ESP_LOGI(EXAMPLE_TAG,
                 "%s BMS-TENT: IchgLim %.2fA | IdisLim %.2fA",
                 ifn,
                 hasIchgLim ? ((double)ichgLimCa / 100.0) : -1.0,
                 hasIdisLim ? ((double)idisLimCa / 100.0) : -1.0);
    }

    for (int i = 0; i < 16; i++) {
        uint16_t addr = (uint16_t)(GROWATT_MB_REG_CELL_BASE + i);
        uint16_t mv = 0;
        if (!modbusDecoderGetCachedReg(d, addr, &mv)) {
            continue;
        }

        uint16_t cell = (uint16_t)(i + 1);

#if REG_RAW_VALUES
        ESP_LOGI(EXAMPLE_TAG,
                 "%s reg[0x%04X] = 0x%04X (%u)  |  Cell%02u @0x%04X = %.3f V (%u mV)",
                 ifn,
                 (unsigned)addr,
                 (unsigned)mv,
                 (unsigned)mv,
                 (unsigned)cell,
                 (unsigned)addr,
                 (double)mv / 1000.0,
                 (unsigned)mv);
#else
        ESP_LOGI(EXAMPLE_TAG,
                 "%s Cell%02u @0x%04X = %.3f V (%u mV)",
                 ifn,
                 (unsigned)cell,
                 (unsigned)addr,
                 (double)mv / 1000.0,
                 (unsigned)mv);
#endif
    }

#if REG_RAW_VALUES
    {
        uint16_t r0013 = 0;
        uint16_t cell13 = 0;
        if (modbusDecoderGetCachedReg(d, GROWATT_MB_REG_MAIN_RAW_0013, &r0013) &&
            modbusDecoderGetCachedReg(d, GROWATT_MB_REG_CELL_N(13), &cell13)) {
            ESP_LOGI(EXAMPLE_TAG,
                     "%s reg[0x0013] = 0x%04X (%u)  |  Cell13 @0x%04X = %.3f V (%u mV)",
                     ifn,
                     (unsigned)r0013,
                     (unsigned)r0013,
                     (unsigned)GROWATT_MB_REG_CELL_N(13u),
                     (double)cell13 / 1000.0,
                     (unsigned)cell13);
        }
    }
#endif
}

void modbusDecoderPrintSnapshot(modbusDecoder_t *d)
{
    int n = 0;
    const char *ifn = snapshotIfName(d);
#if REG_RAW_VALUES
    int idx[MODBUS_DECODER_CACHE_MAX_REGS];
#endif

    for (int i = 0; i < MODBUS_DECODER_CACHE_MAX_REGS; i++) {
        if (!d->cacheValid[i]) {
            continue;
        }
#if REG_RAW_VALUES
        idx[n] = i;
#endif
        n++;
    }

    ESP_LOGI(EXAMPLE_TAG, "%s SNAPSHOT BEGIN", ifn);

    if (n == 0) {
        ESP_LOGI(EXAMPLE_TAG, "%s SNAPSHOT: no cached registers", ifn);
        ESP_LOGI(EXAMPLE_TAG, "%s SNAPSHOT END", ifn);
        return;
    }

    printSnapshotDecoded(d);

#if REG_RAW_VALUES
    for (int i = 0; i < n - 1; i++) {
        for (int j = i + 1; j < n; j++) {
            if (d->cacheAddr[idx[j]] < d->cacheAddr[idx[i]]) {
                int t = idx[i];
                idx[i] = idx[j];
                idx[j] = t;
            }
        }
    }

    ESP_LOGI(EXAMPLE_TAG, "%s RAW REGISTER DUMP:", ifn);
    for (int i = 0; i < n; i++) {
        int k = idx[i];
        ESP_LOGI(EXAMPLE_TAG,
                 "%s reg[0x%04X] = 0x%04X (%u)",
                 ifn,
                 (unsigned)d->cacheAddr[k],
                 (unsigned)d->cacheVal[k],
                 (unsigned)d->cacheVal[k]);
    }
#endif

    ESP_LOGI(EXAMPLE_TAG, "%s SNAPSHOT END", ifn);
}


