#include "Drivers/can_driver.h"

#include "config.h"
#include "runtime_settings.h"

#include "esp_log.h"

static twai_handle_t s_twaiBus0; /* CAN1 */
static twai_handle_t s_twaiBus1; /* CAN2 */

twai_handle_t canGetBus0(void) { return s_twaiBus0; }
twai_handle_t canGetBus1(void) { return s_twaiBus1; }

static uint32_t canBitrateForPort(uint8_t port, const bridge_runtime_settings_t *settings)
{
    uint32_t bitrate = CAN_DEFAULT_BITRATE;
    bool haveBitrate = false;

    if (settings == NULL) {
        return bitrate;
    }

    if ((settings->bms_line == LINE_CAN) && (settings->bms_port == port)) {
        bitrate = bridgeProtocolCanBitrate(settings->bms_protocol);
        haveBitrate = true;
    }

    if ((settings->inverter_line == LINE_CAN) && (settings->inverter_port == port)) {
        uint32_t inverterBitrate = bridgeProtocolCanBitrate(settings->inverter_protocol);
        if (haveBitrate && inverterBitrate != bitrate) {
            ESP_LOGW(EXAMPLE_TAG,
                     "CAN%u requested with mixed bitrates (%u and %u); using inverter side",
                     (unsigned)port,
                     (unsigned)bitrate,
                     (unsigned)inverterBitrate);
        }
        bitrate = inverterBitrate;
    }

    return bitrate;
}

static twai_timing_config_t canTimingForBitrate(uint32_t bitrate)
{
    switch (bitrate) {
        case CAN_JKBMS_250K_BITRATE:
            return (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();
        case CAN_DEFAULT_BITRATE:
        default:
            return (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
    }
}

static void canStopOne(twai_handle_t *handle)
{
    if (handle == NULL || *handle == NULL) {
        return;
    }

    (void)twai_stop_v2(*handle);
    (void)twai_driver_uninstall_v2(*handle);
    *handle = NULL;
}

static void canSetupOne(uint8_t port, int txPin, int rxPin, uint32_t bitrate, twai_handle_t *outHandle)
{
    twai_general_config_t gConfig =
        TWAI_GENERAL_CONFIG_DEFAULT(txPin, rxPin, TWAI_MODE_NORMAL);
    twai_timing_config_t tConfig = canTimingForBitrate(bitrate);
    twai_filter_config_t fConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    gConfig.controller_id = (int)(port - 1u);
    ESP_ERROR_CHECK(twai_driver_install_v2(&gConfig, &tConfig, &fConfig, outHandle));
    ESP_ERROR_CHECK(twai_start_v2(*outHandle));
    ESP_LOGI(EXAMPLE_TAG, "CAN%u started (%u bit/s)", (unsigned)port, (unsigned)bitrate);
}

void canReinit(void)
{
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    uint32_t bitrate1 = canBitrateForPort(1u, &settings);
    uint32_t bitrate2 = canBitrateForPort(2u, &settings);

    canStopOne(&s_twaiBus0);
    canStopOne(&s_twaiBus1);

    canSetupOne(1u, CAN1_TX, CAN1_RX, bitrate1, &s_twaiBus0);
    canSetupOne(2u, CAN2_TX, CAN2_RX, bitrate2, &s_twaiBus1);
}

void canInit(void)
{
    canReinit();
}
