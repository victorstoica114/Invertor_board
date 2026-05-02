#pragma once

#include <stdio.h>

#include "freertos/FreeRTOS.h"

// Minimal logging stubs for host-based unit tests.
#define ESP_LOGI(tag, fmt, ...) ((void)0)
#define ESP_LOGW(tag, fmt, ...) ((void)0)
#define ESP_LOGE(tag, fmt, ...) ((void)0)
