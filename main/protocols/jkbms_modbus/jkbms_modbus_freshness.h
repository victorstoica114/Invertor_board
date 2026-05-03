#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "decoders/modbusDecoder.h"

#ifdef __cplusplus
extern "C" {
#endif

bool jkbmsModbusDecoderCacheFresh(const modbusDecoder_t *decoder,
                                  int64_t nowUs,
                                  int64_t *newestCacheUsOut);

#ifdef __cplusplus
}
#endif
