#include "Drivers/CAN/can_driver.h"

#include "config.h"

#include "esp_log.h"

static twai_handle_t s_canBus0;
static twai_handle_t s_canBus1;

static void canResetBus(twai_handle_t handle, const char *name)
{
    twai_status_info_t status = {0};
    bool wasRunning = false;

    if (handle == NULL) {
        return;
    }

    if (twai_get_status_info_v2(handle, &status) == ESP_OK) {
        wasRunning = (status.state == TWAI_STATE_RUNNING);
    }

    if (wasRunning) {
        esp_err_t stopErr = twai_stop_v2(handle);
        if (stopErr != ESP_OK) {
            ESP_LOGW(EXAMPLE_TAG,
                     "%s stop during reset failed (err=0x%x)",
                     name,
                     (unsigned)stopErr);
        }
    }

    (void)twai_clear_transmit_queue_v2(handle);
    (void)twai_clear_receive_queue_v2(handle);

    if (wasRunning) {
        esp_err_t startErr = twai_start_v2(handle);
        if (startErr != ESP_OK) {
            ESP_LOGW(EXAMPLE_TAG,
                     "%s start during reset failed (err=0x%x)",
                     name,
                     (unsigned)startErr);
        } else {
            ESP_LOGI(EXAMPLE_TAG, "%s queues/state reset", name);
        }
    }
}

void canInit(void)
{
    twai_general_config_t gConfig =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN1_TX, CAN1_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t tConfig = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t fConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    gConfig.controller_id = 0;
    ESP_ERROR_CHECK(twai_driver_install_v2(&gConfig, &tConfig, &fConfig, &s_canBus0));
    ESP_ERROR_CHECK(twai_start_v2(s_canBus0));
    ESP_LOGI(EXAMPLE_TAG, "CAN1 started");

    gConfig.controller_id = 1;
    gConfig.tx_io = CAN2_TX;
    gConfig.rx_io = CAN2_RX;
    ESP_ERROR_CHECK(twai_driver_install_v2(&gConfig, &tConfig, &fConfig, &s_canBus1));
    ESP_ERROR_CHECK(twai_start_v2(s_canBus1));
    ESP_LOGI(EXAMPLE_TAG, "CAN2 started");
}

twai_handle_t canGetBus0(void)
{
    return s_canBus0;
}

twai_handle_t canGetBus1(void)
{
    return s_canBus1;
}

void canResetBuses(void)
{
    canResetBus(s_canBus0, "CAN1");
    canResetBus(s_canBus1, "CAN2");
}
