#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "modes/mode_manager.h"
#include "Drivers/can_driver.h"
#include "Drivers/rs485_driver.h"
#include "Web_interface/web_interface.h"
#include "config.h"
#include "runtime_settings.h"

static working_mode_t runtimeModeToWorkingMode(uint8_t runtimeMode)
{
    switch (runtimeMode) {
        case MODE_SNIFFER:
            return WORKING_MODE_SNIFFER;
        case MODE_FORWARD:
            return WORKING_MODE_FORWARD;
        case MODE_BRIDGE:
            return WORKING_MODE_BRIDGE;
        default:
            return (working_mode_t)ACTIVE_WORKING_MODE;
    }
}

void app_main(void)
{
    ESP_LOGI(EXAMPLE_TAG, "Booting sniffer/bridge...");

    xTaskCreate(led_blink_task, "led_blink", 2048, NULL, 1, NULL);

    rs485Init();
    canInit();

    bridge_runtime_settings_t settings = runtimeSettingsGet();
    const working_mode_t mode = runtimeModeToWorkingMode(settings.mode);
    esp_err_t err = workingModesStart(mode);
    if (err != ESP_OK) {
        ESP_LOGE(EXAMPLE_TAG,
                 "Failed to start mode=%s (err=0x%x)",
                 workingModeToStr(mode),
                 (unsigned)err);
    } else {
        ESP_LOGI(EXAMPLE_TAG, "Working mode started: %s", workingModeToStr(mode));
    }

    webInterfaceStartTask();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
