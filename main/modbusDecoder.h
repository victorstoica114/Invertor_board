#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *ifName;

    // Accumulator frame-by-frame
    uint8_t  buf[512];
    uint16_t len;

    // Gap detection (Modbus RTU: 3.5 char times)
    uint32_t gapUs;
    int64_t  lastByteUs;
    bool     haveLastByte;

    // Last request (for correlating response start addr)
    bool     lastReqValid;
    uint8_t  lastReqSlave;
    uint8_t  lastReqFunc;
    uint16_t lastReqStart;
    uint16_t lastReqCount;
} modbusDecoder_t;

void modbusDecoderInit(modbusDecoder_t *d, const char *ifName, uint32_t baudrate);
void modbusDecoderFeed(modbusDecoder_t *d, const uint8_t *data, int len);
void modbusDecoderFlush(modbusDecoder_t *d);

#ifdef __cplusplus
}
#endif
