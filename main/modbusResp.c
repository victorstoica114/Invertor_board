#include "modbusDecoder.h"
#include "esp_log.h"
#include <stdint.h>

#define TAG "SNIFFER_BRIDGE"

// Această implementare presupune că printResp03 primește cadrul RTU complet:
// [slave][func=0x03][byteCount][data...][crcLo][crcHi]
// Dacă semnătura ta diferă, păstrează corpul și aliniază doar argumentele.
void printResp03(const char *ifName, const uint8_t *frame, int len, bool crcOk)
{
    if (len < 5) {
        ESP_LOGI(TAG, "RESP on %s: func=0x03 (too short) len=%d", ifName, len);
        return;
    }

    uint8_t slave = frame[0];
    uint8_t func  = frame[1];

    if (func != 0x03) {
        ESP_LOGI(TAG, "RESP on %s: slave=%u func=0x%02X (not 0x03)", ifName, slave, func);
        return;
    }

    uint8_t byteCount = frame[2];
    int expectedLen = 3 + byteCount + 2; // hdr(3) + data + crc(2)

    if (len != expectedLen) {
        ESP_LOGI(TAG,
                 "RESP on %s: slave=%u func=0x03 byteCount=%u len=%d expected=%d crc=%s",
                 ifName, (unsigned)slave, (unsigned)byteCount, len, expectedLen, crcOk ? "OK" : "BAD");
        // continuăm oricum, să vedem ce avem
    }

    int regCount = byteCount / 2;
    ESP_LOGI(TAG, "RESP on %s: slave=%u func=0x03 regs[%d] crc=%s",
             ifName, (unsigned)slave, regCount, crcOk ? "OK" : "BAD");

    const uint8_t *p = &frame[3];
    for (int i = 0; i < regCount; i++) {
        uint16_t v = ((uint16_t)p[2*i] << 8) | (uint16_t)p[2*i + 1];
        ESP_LOGI(TAG, "  reg[%02d] = 0x%04X (%u)", i, (unsigned)v, (unsigned)v);
    }
}
