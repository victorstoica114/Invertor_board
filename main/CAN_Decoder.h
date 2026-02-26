#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/twai.h"

#ifdef __cplusplus
extern "C" {
#endif

/* true = log raw CAN frame + decoded values; false = decoded values only */
extern bool g_canDecoderShowRawFrames;

void canDecoderOnFrame(const char *ifname, const twai_message_t *m);

#ifdef __cplusplus
}
#endif
