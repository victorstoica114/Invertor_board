#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WORKING_MODE_BRIDGE = 0,
    WORKING_MODE_FORWARD = 1,
    WORKING_MODE_SNIFFER = 2,
} working_mode_t;

const char *workingModeToStr(working_mode_t mode);
esp_err_t workingModesStart(working_mode_t mode);

#ifdef __cplusplus
}
#endif
