#include "Drivers/can_driver.h"

#include "config.h"

#include "esp_log.h"

static twai_handle_t s_twaiBus0; /* CAN1 */
static twai_handle_t s_twaiBus1; /* CAN2 */

twai_handle_t canGetBus0(void) { return s_twaiBus0; }
twai_handle_t canGetBus1(void) { return s_twaiBus1; }

void canInit(void)
{
    twai_general_config_t gConfig =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN1_TX, CAN1_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t tConfig = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t fConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    /* Controller 0 => CAN1 */
    gConfig.controller_id = 0;
    ESP_ERROR_CHECK(twai_driver_install_v2(&gConfig, &tConfig, &fConfig, &s_twaiBus0));
    ESP_ERROR_CHECK(twai_start_v2(s_twaiBus0));
    ESP_LOGI(EXAMPLE_TAG, "CAN1 started");

    /* Controller 1 => CAN2 */
    gConfig.controller_id = 1;
    gConfig.tx_io = CAN2_TX;
    gConfig.rx_io = CAN2_RX;

    ESP_ERROR_CHECK(twai_driver_install_v2(&gConfig, &tConfig, &fConfig, &s_twaiBus1));
    ESP_ERROR_CHECK(twai_start_v2(s_twaiBus1));
    ESP_LOGI(EXAMPLE_TAG, "CAN2 started");
}
