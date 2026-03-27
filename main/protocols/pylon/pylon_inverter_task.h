#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t pylonInverterTaskStart(QueueHandle_t inQueue);
esp_err_t pylonInverterTaskStop(void);

#ifdef __cplusplus
}
#endif
