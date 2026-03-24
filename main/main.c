#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "config.h"
#include "orchestrator/orchestrator.h"

void app_main(void)
{
    ESP_LOGI(EXAMPLE_TAG, "Booting sniffer/bridge...");

    xTaskCreate(led_blink_task, "led_blink", 2048, NULL, 1, NULL);

    rs485Init();
    canInit();

    esp_err_t err = orchestratorStart(ACTIVE_BMS_PROTOCOL, ACTIVE_INVERTER_PROTOCOL);
    if (err != ESP_OK) {
        ESP_LOGE(EXAMPLE_TAG, "Orchestrator failed to start (err=0x%x)", (unsigned)err);
    }

    ESP_LOGI(EXAMPLE_TAG, "Task-based protocol bridge running.");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
