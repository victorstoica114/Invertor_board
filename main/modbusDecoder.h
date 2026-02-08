#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *ifName;      // "RS485_1" / "RS485_2" etc.
    uint32_t    gapUs;       // prag gap pentru delimitare cadru (ex: 5000 us)

    // buffer cadru curent
    uint8_t     buf[256];
    uint16_t    len;

    // timing pentru gap
    int64_t     lastByteUs;
    bool        haveLastByte;

    // ultima cerere (pt corelare cu raspuns)
    bool        lastReqValid;
    uint8_t     lastReqSlave;
    uint8_t     lastReqFunc;
    uint16_t    lastReqStart;
    uint16_t    lastReqCount;
    int64_t     lastReqUs;
} modbusDecoder_t;

void modbusDecoderInit(modbusDecoder_t *d, const char *ifName, uint32_t gapUs);

/**
 * Feed bytes receptionați pe o interfață.
 * rxUs = timestamp în microsecunde (esp_timer_get_time()).
 */
void modbusDecoderFeed(modbusDecoder_t *d, const uint8_t *data, int len, int64_t rxUs);

/**
 * Forțează închiderea cadrului curent (dacă vrei, de ex. la stop).
 */
void modbusDecoderFlush(modbusDecoder_t *d);

#ifdef __cplusplus
}
#endif
