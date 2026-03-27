#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MODBUS_DECODER_SNAPSHOT_PRINT_PERIOD_MS 5000
#define MODBUS_DECODER_CACHE_MAX_REGS 256
#define MODBUS_DECODER_REQ_QUEUE_LEN 8

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
    uint8_t     reqQSlave[MODBUS_DECODER_REQ_QUEUE_LEN];
    uint8_t     reqQFunc[MODBUS_DECODER_REQ_QUEUE_LEN];
    uint16_t    reqQStart[MODBUS_DECODER_REQ_QUEUE_LEN];
    uint16_t    reqQCount[MODBUS_DECODER_REQ_QUEUE_LEN];
    int64_t     reqQTsUs[MODBUS_DECODER_REQ_QUEUE_LEN];
    uint8_t     reqQHead;
    uint8_t     reqQSize;

    uint16_t    cacheAddr[MODBUS_DECODER_CACHE_MAX_REGS];
    uint16_t    cacheVal[MODBUS_DECODER_CACHE_MAX_REGS];
    uint8_t     cacheValid[MODBUS_DECODER_CACHE_MAX_REGS];
    int64_t     cacheTsUs[MODBUS_DECODER_CACHE_MAX_REGS];
} modbusDecoder_t;

void modbusDecoderInit(modbusDecoder_t *d, const char *ifName, uint32_t gapUs);
void modbusDecoderFeed(modbusDecoder_t *d, const uint8_t *data, int len, int64_t rxUs);
void modbusDecoderFlush(modbusDecoder_t *d);
void modbusDecoderPrintSnapshot(modbusDecoder_t *d);
bool modbusDecoderGetCachedReg(const modbusDecoder_t *d, uint16_t addr, uint16_t *valOut);
void modbusDecoderRecordRequest(modbusDecoder_t *d,
                                uint8_t slave,
                                uint8_t func,
                                uint16_t start,
                                uint16_t count,
                                int64_t tsUs);

#ifdef __cplusplus
}
#endif
