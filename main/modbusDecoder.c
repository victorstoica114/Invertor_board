#include "modbusDecoder.h"

#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_timer.h"

#ifndef EXAMPLE_TAG
#define EXAMPLE_TAG "SNIFFER_BRIDGE"
#endif

// ---------------- CRC16 Modbus (poly 0xA001, init 0xFFFF) ----------------
static uint16_t modbusCrc16(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1) crc = (uint16_t)((crc >> 1) ^ 0xA001);
            else         crc = (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

static inline uint16_t be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static void dumpHex(const char *prefix, const uint8_t *buf, int len)
{
    const int maxHex = 64;
    int n = (len < maxHex) ? len : maxHex;

    char hex[3 * maxHex + 1];
    int pos = 0;
    for (int i = 0; i < n; i++) {
        pos += snprintf(&hex[pos], sizeof(hex) - pos, "%02X ", buf[i]);
        if (pos >= (int)sizeof(hex)) break;
    }
    if (pos > 0) hex[pos - 1] = 0; else hex[0] = 0;

    if (len > maxHex) {
        ESP_LOGI(EXAMPLE_TAG, "%s len=%d HEX(first%d)=[%s] ...", prefix, len, maxHex, hex);
    } else {
        ESP_LOGI(EXAMPLE_TAG, "%s len=%d HEX=[%s]", prefix, len, hex);
    }
}

// Forward decl: tu o ai în alt fișier (sau o pui aici)
extern void printResp03(modbusDecoder_t *d, const uint8_t *frame, int len, bool crcOk);

// ---------------- Frame parsing helpers ----------------
static bool looksLikeReq(const uint8_t *f, int len)
{
    // Modbus RTU request typical lengths:
    // 0x03/0x04 read: 8 bytes
    // 0x06 write single: 8 bytes
    // 0x10 write multi: 9+ bytes
    if (len < 4) return false;
    uint8_t func = f[1];
    if (func == 0x03 || func == 0x04 || func == 0x06) return (len == 8);
    if (func == 0x10) return (len >= 9);
    return false;
}

static bool looksLikeResp(const uint8_t *f, int len)
{
    // Response for 0x03/0x04: slave func byteCount data... crc (>=5)
    if (len < 5) return false;
    uint8_t func = f[1];
    if (func == 0x03 || func == 0x04) {
        uint8_t bc = f[2];
        return (3 + (int)bc + 2) == len;
    }
    // Response for 0x06: echo request (8)
    if (func == 0x06) return (len == 8);
    // Response for 0x10: 8 bytes (addr + count echoed)
    if (func == 0x10) return (len == 8);
    return false;
}

static void printReq(modbusDecoder_t *d, const uint8_t *f, int len, bool crcOk)
{
    uint8_t slave = f[0];
    uint8_t func  = f[1];

    if (func == 0x03 || func == 0x04) {
        uint16_t start = be16(&f[2]);
        uint16_t count = be16(&f[4]);

        // Save for correlating response
        d->lastReqValid = true;
        d->lastReqSlave = slave;
        d->lastReqFunc  = func;
        d->lastReqStart = start;
        d->lastReqCount = count;

        ESP_LOGI(EXAMPLE_TAG,
                 "REQ on %s: slave=%u func=0x%02X start=0x%04X count=0x%04X crc=%s",
                 d->ifName, (unsigned)slave, (unsigned)func,
                 (unsigned)start, (unsigned)count,
                 crcOk ? "OK" : "BAD");
        return;
    }

    if (func == 0x06) {
        uint16_t addr = be16(&f[2]);
        uint16_t val  = be16(&f[4]);
        ESP_LOGI(EXAMPLE_TAG,
                 "REQ on %s: slave=%u func=0x%02X addr=0x%04X val=0x%04X crc=%s",
                 d->ifName, (unsigned)slave, (unsigned)func,
                 (unsigned)addr, (unsigned)val,
                 crcOk ? "OK" : "BAD");
        return;
    }

    if (func == 0x10) {
        uint16_t start = be16(&f[2]);
        uint16_t count = be16(&f[4]);
        uint8_t  bc    = f[6];
        ESP_LOGI(EXAMPLE_TAG,
                 "REQ on %s: slave=%u func=0x%02X start=0x%04X count=0x%04X bytes=%u crc=%s",
                 d->ifName, (unsigned)slave, (unsigned)func,
                 (unsigned)start, (unsigned)count, (unsigned)bc,
                 crcOk ? "OK" : "BAD");
        return;
    }

    // Fallback
    ESP_LOGI(EXAMPLE_TAG, "REQ on %s: slave=%u func=0x%02X crc=%s",
             d->ifName, (unsigned)slave, (unsigned)func, crcOk ? "OK" : "BAD");
}

static void printResp(modbusDecoder_t *d, const uint8_t *f, int len, bool crcOk)
{
    uint8_t func = f[1];

    if (func == 0x03) {
        printResp03(d, f, len, crcOk);
        return;
    }

    // You can add similar for 0x04
    if (func == 0x06) {
        uint16_t addr = be16(&f[2]);
        uint16_t val  = be16(&f[4]);
        ESP_LOGI(EXAMPLE_TAG,
                 "RESP on %s: slave=%u func=0x%02X addr=0x%04X val=0x%04X crc=%s",
                 d->ifName, (unsigned)f[0], (unsigned)func,
                 (unsigned)addr, (unsigned)val,
                 crcOk ? "OK" : "BAD");
        return;
    }

    if (func == 0x10) {
        uint16_t start = be16(&f[2]);
        uint16_t count = be16(&f[4]);
        ESP_LOGI(EXAMPLE_TAG,
                 "RESP on %s: slave=%u func=0x%02X start=0x%04X count=0x%04X crc=%s",
                 d->ifName, (unsigned)f[0], (unsigned)func,
                 (unsigned)start, (unsigned)count,
                 crcOk ? "OK" : "BAD");
        return;
    }

    ESP_LOGI(EXAMPLE_TAG, "RESP on %s: slave=%u func=0x%02X len=%d crc=%s",
             d->ifName, (unsigned)f[0], (unsigned)func, len, crcOk ? "OK" : "BAD");
}

// Try to finalize a frame in d->buf[0..len-1]
static void tryParseFrame(modbusDecoder_t *d)
{
    if (d->len < 4) return;

    // CRC is last 2 bytes: [len-2]=CRCLo, [len-1]=CRCHi
    uint16_t rxCrc = (uint16_t)(d->buf[d->len - 2] | ((uint16_t)d->buf[d->len - 1] << 8));
    uint16_t calc  = modbusCrc16(d->buf, (int)d->len - 2);
    bool crcOk = (rxCrc == calc);

    // Classify
    bool req  = looksLikeReq(d->buf, d->len);
    bool resp = looksLikeResp(d->buf, d->len);

    if (req) {
        printReq(d, d->buf, d->len, crcOk);
    } else if (resp) {
        printResp(d, d->buf, d->len, crcOk);
    } else {
        // “Ciudate” / fragmente / alt protocol / delimitare greșită
        char pfx[64];
        snprintf(pfx, sizeof(pfx), "FRAME on %s: crc=%s", d->ifName, crcOk ? "OK" : "BAD");
        dumpHex(pfx, d->buf, d->len);
    }

    d->len = 0;
}

// ---------------- Public API ----------------
void modbusDecoderInit(modbusDecoder_t *d, const char *ifName, uint32_t baudrate)
{
    memset(d, 0, sizeof(*d));
    d->ifName = ifName;

    // 3.5 char times ≈ 3.5 * (start+8data+parity+stop)= ~ 3.5*10 bits
    // time/char = 10/baud seconds => charUs = 10e6/baud
    // gapUs ≈ 3.5 * charUs
    uint32_t charUs = (uint32_t)(10000000UL / (baudrate ? baudrate : 9600));
    d->gapUs = (uint32_t)(charUs * 35UL / 10UL); // 3.5 chars

    // Mulți adaptoare “rup” cadrele la timpi mai mari; poți mări ușor:
    // d->gapUs += 1500;

    d->len = 0;
    d->haveLastByte = false;
    d->lastReqValid = false;
}

void modbusDecoderFeed(modbusDecoder_t *d, const uint8_t *data, int len)
{
    int64_t rxUs = esp_timer_get_time();

    if (d->haveLastByte) {
        int64_t delta = rxUs - d->lastByteUs;
        if (delta > (int64_t)d->gapUs) {
            // gap => finalize previous frame
            if (d->len > 0) {
                tryParseFrame(d);
            }
        }
    }

    for (int i = 0; i < len; i++) {
        if (d->len >= sizeof(d->buf)) {
            // overflow => drop current
            ESP_LOGW(EXAMPLE_TAG, "Decoder overflow on %s (dropping)", d->ifName);
            d->len = 0;
        }
        d->buf[d->len++] = data[i];
    }

    d->lastByteUs = rxUs;
    d->haveLastByte = true;
}

void modbusDecoderFlush(modbusDecoder_t *d)
{
    if (d->len > 0) {
        tryParseFrame(d);
    }
    d->haveLastByte = false;
}
