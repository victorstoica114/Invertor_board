#include "protocols/jkbms_modbus/jkbms_modbus_freshness.h"

#include "config.h"

bool jkbmsModbusDecoderCacheFresh(const modbusDecoder_t *decoder,
                                  int64_t nowUs,
                                  int64_t *newestCacheUsOut)
{
    const int64_t newestCacheUs = modbusDecoderGetNewestCacheTsUs(decoder);
    const int64_t maxAgeUs = (int64_t)BRIDGE_SOURCE_STALE_MS * 1000LL;
    int64_t ageUs = 0;

    if (newestCacheUsOut != NULL) {
        *newestCacheUsOut = newestCacheUs;
    }

    if (newestCacheUs <= 0) {
        return false;
    }

    ageUs = (nowUs >= newestCacheUs) ? (nowUs - newestCacheUs) : 0;
    return ageUs <= maxAgeUs;
}
