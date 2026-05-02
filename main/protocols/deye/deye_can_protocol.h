#pragma once

#include <stddef.h>

#include "../pylon/pylon_can_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

void deyeCanDecodeSnapshot(const char *ifname, const pylon_can_frame_t *cache, size_t count);

#ifdef __cplusplus
}
#endif
