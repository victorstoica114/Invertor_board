#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/twai.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Periodic aggregated print interval (ms) for cached BMS state */
#define CAN_DECODER_SNAPSHOT_PRINT_PERIOD_MS 5000

void canDecoderOnFrame(const char *ifname, const twai_message_t *m);
void canDecoderPrintCachedSnapshot(const char *ifname);
bool canDecoderTryGetSocPct(const char *ifname, uint8_t *socOut);
bool canDecoderHasFreshData(const char *ifname, uint32_t maxAgeMs);
void canDecoderResetCaches(void);
bool canDecoderGetGrowattAlertText(const char *ifname,
                                   char *protectionsOut,
                                   uint32_t protectionsOutSize,
                                   char *alarmsOut,
                                   uint32_t alarmsOutSize,
                                   char *warningsOut,
                                   uint32_t warningsOutSize);

#ifdef __cplusplus
}
#endif
