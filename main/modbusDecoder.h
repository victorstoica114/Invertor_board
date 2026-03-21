#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MODBUS_DECODER_SNAPSHOT_PRINT_PERIOD_MS 5000
#define MODBUS_DECODER_CACHE_MAX_REGS 256

typedef struct {
    const char *ifName;
    uint32_t    gapUs;

    uint8_t     buf[256];
    uint16_t    len;

    int64_t     lastByteUs;
    bool        haveLastByte;

    bool        lastReqValid;
    uint8_t     lastReqSlave;
    uint8_t     lastReqFunc;
    uint16_t    lastReqStart;
    uint16_t    lastReqCount;
    int64_t     lastReqUs;

    uint16_t    cacheAddr[MODBUS_DECODER_CACHE_MAX_REGS];
    uint16_t    cacheVal[MODBUS_DECODER_CACHE_MAX_REGS];
    uint8_t     cacheValid[MODBUS_DECODER_CACHE_MAX_REGS];
    int64_t     cacheTsUs[MODBUS_DECODER_CACHE_MAX_REGS];
} modbusDecoder_t;

void modbusDecoderInit(modbusDecoder_t *d, const char *ifName, uint32_t gapUs);
void modbusDecoderFeed(modbusDecoder_t *d, const uint8_t *data, int len, int64_t rxUs);
void modbusDecoderFlush(modbusDecoder_t *d);
void modbusDecoderPrintSnapshot(modbusDecoder_t *d);
bool modbusDecoderGetReg(const modbusDecoder_t *d, uint16_t addr, uint16_t *valOut);
bool modbusDecoderHasFreshData(const modbusDecoder_t *d, uint32_t maxAgeMs);

#ifdef __cplusplus
}
#endif
