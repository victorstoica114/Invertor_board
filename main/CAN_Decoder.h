#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/twai.h"

#ifdef __cplusplus
extern "C" {
#endif

/* true = log raw CAN frame + decoded values; false = decoded values only */
extern bool g_canDecoderShowRawFrames;

/* Periodic aggregated print interval (ms) for cached BMS state */
#define CAN_DECODER_SNAPSHOT_PRINT_PERIOD_MS 5000

void canDecoderOnFrame(const char *ifname, const twai_message_t *m);
void canDecoderPrintCachedSnapshot(const char *ifname);

#ifdef __cplusplus
}
#endif
