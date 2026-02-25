// main.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "config.h"
#include "bridge.h"

void app_main(void)
{
    ESP_LOGI(EXAMPLE_TAG, "Booting sniffer/bridge...");

    xTaskCreate(led_blink_task, "led_blink", 2048, NULL, 1, NULL);

    rs485Init();
    canInit();

    rs485BridgeEnable();
    canBridgeEnable();

    ESP_LOGI(EXAMPLE_TAG, "Sniffer/bridge running.");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
