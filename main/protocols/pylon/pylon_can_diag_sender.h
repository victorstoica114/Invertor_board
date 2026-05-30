#pragma once

#include "driver/twai.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t pylonCanDiagSenderStart(twai_handle_t bus, const char *ifName, TaskHandle_t *outTask);

#ifdef __cplusplus
}
#endif
