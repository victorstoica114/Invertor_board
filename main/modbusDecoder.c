#include "modbusDecoder.h"
#include "config.h"
#include "Growatt_regs.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

#ifndef EXAMPLE_TAG
#define EXAMPLE_TAG "SNIFFER_BRIDGE"
#endif

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
}

static void printResp03(modbusDecoder_t *d, const uint8_t *frame, int len, bool crcOk)
{
    if (len < 5) {
        MODBUS_RUNTIME_LOGI( "RESP on %s: too short (len=%d)", d->ifName, len);
        return;
    }

    uint8_t slave = frame[0];
    uint8_t func = frame[1];

    if (func != 0x03) {
        MODBUS_RUNTIME_LOGI( "RESP on %s: func=0x%02X (not 0x03)", d->ifName, func);
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
    if (d->lastReqValid && d->lastReqSlave == slave && d->lastReqFunc == func) {
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

static void decodeFrame(modbusDecoder_t *d, const uint8_t *f, int len)
{
    if (len < 4) {
        printFrameGeneric(d, f, len, false);
        return;
    }

    bool crcOk = modbusCheckCrc(f, len);
    uint8_t func = f[1];

    if (func == 0x03 && len == 8) {
        printReq03(d, f, len, crcOk);
        return;
    }

    if (func == 0x03 && len >= 5) {
        printResp03(d, f, len, crcOk);
        return;
    }

    printFrameGeneric(d, f, len, crcOk);
}

void modbusDecoderInit(modbusDecoder_t *d, const char *ifName, uint32_t gapUs)
{
    memset(d, 0, sizeof(*d));
    d->ifName = ifName;
    d->gapUs = gapUs;
}

static void finalizeIfAny(modbusDecoder_t *d)
{
    if (d->len > 0) {
        decodeFrame(d, d->buf, d->len);
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

    for (int i = 0; i < len; i++) {
        if (d->len >= sizeof(d->buf)) {
            ESP_LOGW(EXAMPLE_TAG, "Decoder on %s overflow, dropping frame", d->ifName);
            d->len = 0;
        }
        d->buf[d->len++] = data[i];
    }

    d->lastByteUs = rxUs;
    d->haveLastByte = true;
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

static bool cacheGetReg(const modbusDecoder_t *d, uint16_t addr, uint16_t *valOut)
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

static void printSnapshotDecoded(modbusDecoder_t *d)
{
    const char *ifn = snapshotIfName(d);

    uint16_t r0001 = 0;
    uint16_t r0002 = 0;
    uint16_t r0003 = 0;
    uint16_t r0004 = 0;
    if (cacheGetReg(d, GROWATT_MB_REG_INFO_0001, &r0001) &&
        cacheGetReg(d, GROWATT_MB_REG_INFO_0002, &r0002) &&
        cacheGetReg(d, GROWATT_MB_REG_INFO_0003, &r0003) &&
        cacheGetReg(d, GROWATT_MB_REG_INFO_0004, &r0004)) {
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

    if (cacheGetReg(d, GROWATT_MB_REG_SOC_PCT, &soc) &&
        cacheGetReg(d, GROWATT_MB_REG_PACK_V_CV, &packCv) &&
        cacheGetReg(d, GROWATT_MB_REG_TEMP_C, &temp) &&
        cacheGetReg(d, GROWATT_MB_REG_CELL_MAX_MV, &cmaxMv) &&
        cacheGetReg(d, GROWATT_MB_REG_CELL_MIN_MV, &cminMv) &&
        cacheGetReg(d, GROWATT_MB_REG_CELL_MAX_IDX, &cmaxIdx) &&
        cacheGetReg(d, GROWATT_MB_REG_CELL_MIN_IDX, &cminIdx)) {
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

    if (cacheGetReg(d, GROWATT_MB_REG_REMAIN_CAP_CAH, &remainCapCah) &&
        cacheGetReg(d, GROWATT_MB_REG_FULL_CAP_CAH, &fullCapCah)) {
        ESP_LOGI(EXAMPLE_TAG,
                 "%s BMS-CAP: RM %.2fAh | FCC %.2fAh",
                 ifn,
                 (double)remainCapCah / 100.0,
                 (double)fullCapCah / 100.0);
    }

    if (cacheGetReg(d, GROWATT_MB_REG_CV_TARGET_CV, &cvTargetCv) &&
        cacheGetReg(d, GROWATT_MB_REG_SOH_PCT, &soh)) {
        ESP_LOGI(EXAMPLE_TAG,
                 "%s BMS-EXT: CVtarget %.2fV | SOH %u%%",
                 ifn,
                 (double)cvTargetCv / 100.0,
                 (unsigned)soh);
    }

    for (int i = 0; i < 16; i++) {
        uint16_t addr = (uint16_t)(GROWATT_MB_REG_CELL_BASE + i);
        uint16_t mv = 0;
        if (!cacheGetReg(d, addr, &mv)) {
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
        if (cacheGetReg(d, GROWATT_MB_REG_MAIN_RAW_0013, &r0013) && cacheGetReg(d, GROWATT_MB_REG_CELL_N(13), &cell13)) {
            ESP_LOGI(EXAMPLE_TAG,
                     "%s reg[0x0013] = 0x%04X (%u)  |  Cell13 @0x007C = %.3f V (%u mV)",
                     ifn,
                     (unsigned)r0013,
                     (unsigned)r0013,
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



