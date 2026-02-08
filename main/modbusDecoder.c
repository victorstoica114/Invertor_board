#include "modbusDecoder.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_log.h"

#ifndef EXAMPLE_TAG
#define EXAMPLE_TAG "SNIFFER_BRIDGE"
#endif

// ---------- Helpers ----------
static uint16_t be16(const uint8_t *p)
{
    return (uint16_t)((p[0] << 8) | p[1]);
}

// CRC16 Modbus (poly 0xA001), init 0xFFFF
static uint16_t modbusCrc16(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFFu;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1u) crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            else         crc = (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

// CRC în cadru Modbus RTU e little-endian: [CRCLo CRCHi]
static bool modbusCheckCrc(const uint8_t *frame, int len)
{
    if (len < 3) return false;
    uint16_t got = (uint16_t)(frame[len - 2] | (frame[len - 1] << 8));
    uint16_t calc = modbusCrc16(frame, len - 2);
    return got == calc;
}

static void dumpHexBrief(const uint8_t *buf, int len, char *out, int outSize)
{
    int pos = 0;
    int max = len;
    if (max > 64) max = 64;
    for (int i = 0; i < max; i++) {
        pos += snprintf(&out[pos], outSize - pos, "%02X ", buf[i]);
        if (pos >= outSize) break;
    }
    if (pos > 0) out[pos - 1] = 0;
    else out[0] = 0;
}

static void printReq03(modbusDecoder_t *d, const uint8_t *f, int len, bool crcOk)
{
    // Request func 0x03: [slave][03][startHi][startLo][countHi][countLo][crcLo][crcHi]
    if (len < 8) return;

    uint8_t slave = f[0];
    uint8_t func  = f[1];
    uint16_t start = be16(&f[2]);
    uint16_t count = be16(&f[4]);

    ESP_LOGI(EXAMPLE_TAG,
             "REQ on %s: slave=%u func=0x%02X start=0x%04X count=0x%04X crc=%s",
             d->ifName, (unsigned)slave, (unsigned)func,
             (unsigned)start, (unsigned)count, crcOk ? "OK" : "BAD");

    // Salvăm ultima cerere pt corelare cu răspunsul
    d->lastReqValid = true;
    d->lastReqSlave = slave;
    d->lastReqFunc  = func;
    d->lastReqStart = start;
    d->lastReqCount = count;
    d->lastReqUs    = d->lastByteUs;
}

static void printResp03(modbusDecoder_t *d,
                        const uint8_t *frame,
                        int len,
                        bool crcOk)
{
    // Modbus RTU response for func=0x03:
    // [0]=slave [1]=func [2]=byteCount [3..]=data (2*regs) [..]=CRClo CRChi
    if (len < 5) {
        ESP_LOGI(EXAMPLE_TAG, "RESP on %s: too short (len=%d)", d->ifName, len);
        return;
    }

    uint8_t slave = frame[0];
    uint8_t func  = frame[1];

    if (func != 0x03) {
        ESP_LOGI(EXAMPLE_TAG, "RESP on %s: func=0x%02X (not 0x03)", d->ifName, func);
        return;
    }

    uint8_t byteCount = frame[2];
    int dataLen = (int)byteCount;

    // Basic sanity:
    // total length should be: 3 + dataLen + 2(CRC)
    if (3 + dataLen + 2 > len) {
        ESP_LOGI(EXAMPLE_TAG, "RESP on %s: length mismatch (len=%d byteCount=%u)",
                 d->ifName, len, (unsigned)byteCount);
        return;
    }

    int regCount = dataLen / 2;
    const uint8_t *data = &frame[3];

    // Determine base start address from last request (if correlates)
    uint16_t startBase = 0xFFFF;
    if (d->lastReqValid && d->lastReqSlave == slave && d->lastReqFunc == func) {
        startBase = d->lastReqStart;
        ESP_LOGI(EXAMPLE_TAG,
                 "RESP on %s: slave=%u func=0x%02X start=0x%04X regs[%d] crc=%s",
                 d->ifName, (unsigned)slave, (unsigned)func,
                 (unsigned)startBase, regCount, crcOk ? "OK" : "BAD");
    } else {
        ESP_LOGI(EXAMPLE_TAG,
                 "RESP on %s: slave=%u func=0x%02X regs[%d] crc=%s",
                 d->ifName, (unsigned)slave, (unsigned)func,
                 regCount, crcOk ? "OK" : "BAD");
    }

    // If CRC bad, still print generic values (optional)
    // Dacă vrei să nu mai vezi gunoi, poți pune "if (!crcOk) return;" aici.
    // if (!crcOk) return;

    // Pretty decode for known ranges
    bool isCellBlock = (startBase != 0xFFFF && startBase == 0x0070);
    bool isMainBlock = (startBase != 0xFFFF && startBase == 0x0010);

    // For cell block: compute stats
    uint16_t minMv = 0xFFFF, maxMv = 0;
    uint32_t sumMv = 0;
    uint16_t minCell = 0, maxCell = 0;

    for (int i = 0; i < regCount; i++) {
        uint16_t v = be16(&data[i * 2]);
        uint16_t addr = (startBase != 0xFFFF) ? (uint16_t)(startBase + i) : 0xFFFF;

        if (isCellBlock) {
            // Treat as mV
            uint16_t cellIndex = (uint16_t)(i + 1);

            // stats
            if (v < minMv) { minMv = v; minCell = cellIndex; }
            if (v > maxMv) { maxMv = v; maxCell = cellIndex; }
            sumMv += v;

            ESP_LOGI(EXAMPLE_TAG,
                     "  cell[%02u] @0x%04X = %u mV (%.3f V)",
                     (unsigned)cellIndex, (unsigned)addr,
                     (unsigned)v, (double)v / 1000.0);
            continue;
        }

        if (isMainBlock) {
            // Aici punem câteva corelări confirmate de capturi:
            // addr 0x0015 = 100 -> SOC%
            // addr 0x0016 = 6976 -> Pack Voltage in cV => 69.76V
            // addr 0x0018 = 25 -> Temp °C
            if (addr == 0x0015) {
                ESP_LOGI(EXAMPLE_TAG,
                         "  reg[0x%04X] SOC = %u %% (raw=0x%04X)",
                         (unsigned)addr, (unsigned)v, (unsigned)v);
                continue;
            }
            if (addr == 0x0016) {
                ESP_LOGI(EXAMPLE_TAG,
                         "  reg[0x%04X] PackVoltage = %.2f V (raw=%u cV)",
                         (unsigned)addr, (double)v / 100.0, (unsigned)v);
                continue;
            }
            if (addr == 0x0018) {
                ESP_LOGI(EXAMPLE_TAG,
                         "  reg[0x%04X] Temp = %d C (raw=0x%04X)",
                         (unsigned)addr, (int16_t)v, (unsigned)v);
                continue;
            }

            // generic with address
            ESP_LOGI(EXAMPLE_TAG,
                     "  reg[0x%04X] = 0x%04X (%u)",
                     (unsigned)addr, (unsigned)v, (unsigned)v);
            continue;
        }

        // Fallback generic if we don't know startBase
        if (addr != 0xFFFF) {
            ESP_LOGI(EXAMPLE_TAG,
                     "  reg[0x%04X] = 0x%04X (%u)",
                     (unsigned)addr, (unsigned)v, (unsigned)v);
        } else {
            ESP_LOGI(EXAMPLE_TAG,
                     "  reg[%02d] = 0x%04X (%u)",
                     i, (unsigned)v, (unsigned)v);
        }
    }

    if (isCellBlock && regCount > 0) {
        double avgV = (double)sumMv / (double)regCount / 1000.0;
        double minV = (double)minMv / 1000.0;
        double maxV = (double)maxMv / 1000.0;
        double deltaV = (double)(maxMv - minMv) / 1000.0;

        ESP_LOGI(EXAMPLE_TAG,
                 "  Cells: min=cell%u %.3fV  max=cell%u %.3fV  delta=%.3fV  avg=%.3fV",
                 (unsigned)minCell, minV,
                 (unsigned)maxCell, maxV,
                 deltaV, avgV);
    }
}

static void printFrameGeneric(modbusDecoder_t *d, const uint8_t *f, int len, bool crcOk)
{
    char hex[3 * 64 + 1];
    dumpHexBrief(f, len, hex, sizeof(hex));
    ESP_LOGI(EXAMPLE_TAG,
             "FRAME on %s: len=%d crc=%s HEX(first64)=[%s]%s",
             d->ifName, len, crcOk ? "OK" : "BAD", hex,
             (len > 64) ? " ..." : "");
}

static void decodeFrame(modbusDecoder_t *d, const uint8_t *f, int len)
{
    if (len < 4) {
        // prea scurt să fie Modbus util
        printFrameGeneric(d, f, len, false);
        return;
    }

    bool crcOk = modbusCheckCrc(f, len);

    uint8_t slave = f[0];
    uint8_t func  = f[1];

    // Heuristic: cerere 0x03 are fix 8 bytes
    if (func == 0x03 && len == 8) {
        printReq03(d, f, len, crcOk);
        return;
    }

    // răspuns 0x03: len >= 5, byteCount în f[2]
    if (func == 0x03 && len >= 5) {
        printResp03(d, f, len, crcOk);
        return;
    }

    // alt func / alt tip: afișăm generic
    (void)slave;
    printFrameGeneric(d, f, len, crcOk);
}

// ---------- Public API ----------
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
    if (len <= 0) return;

    // Dacă avem gap față de ultimul byte, închidem cadrul anterior.
    if (d->haveLastByte) {
        int64_t delta = rxUs - d->lastByteUs;
        if (delta > (int64_t)d->gapUs) {
            finalizeIfAny(d);
        }
    }

    // Append bytes
    for (int i = 0; i < len; i++) {
        if (d->len >= sizeof(d->buf)) {
            // overflow => aruncăm cadrul curent
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
