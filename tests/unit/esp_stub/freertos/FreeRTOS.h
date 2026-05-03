#pragma once

typedef int TickType_t;

#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))

#include "freertos/portmacro.h"
