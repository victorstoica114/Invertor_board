#include "Drivers/can_driver.h"

#include "config.h"
#include "runtime_settings.h"

#include "esp_err.h"
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
        case 50000u:
            return (twai_timing_config_t)TWAI_TIMING_CONFIG_50KBITS();
        case 100000u:
            return (twai_timing_config_t)TWAI_TIMING_CONFIG_100KBITS();
        case 125000u:
            return (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS();
        case CAN_JKBMS_250K_BITRATE:
            return (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();
        case CAN_DEFAULT_BITRATE:
            return (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
        case 800000u:
            return (twai_timing_config_t)TWAI_TIMING_CONFIG_800KBITS();
        case 1000000u:
            return (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();
        default:
            ESP_LOGW(EXAMPLE_TAG,
                     "Unsupported CAN bitrate %u requested; using 500000 bit/s",
                     (unsigned)bitrate);
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

static esp_err_t canSetupOne(uint8_t port,
                             int txPin,
                             int rxPin,
                             uint32_t bitrate,
                             twai_mode_t mode,
                             twai_handle_t *outHandle)
{
    twai_general_config_t gConfig =
        TWAI_GENERAL_CONFIG_DEFAULT(txPin, rxPin, mode);
    twai_timing_config_t tConfig = canTimingForBitrate(bitrate);
    twai_filter_config_t fConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    esp_err_t err = ESP_OK;

    gConfig.controller_id = (int)(port - 1u);
    err = twai_driver_install_v2(&gConfig, &tConfig, &fConfig, outHandle);
    if (err != ESP_OK) {
        ESP_LOGE(EXAMPLE_TAG,
                 "CAN%u driver install failed (%u bit/s): err=0x%x",
                 (unsigned)port,
                 (unsigned)bitrate,
                 (unsigned)err);
        return err;
    }

    err = twai_start_v2(*outHandle);
    if (err != ESP_OK) {
        ESP_LOGE(EXAMPLE_TAG,
                 "CAN%u start failed (%u bit/s): err=0x%x",
                 (unsigned)port,
                 (unsigned)bitrate,
                 (unsigned)err);
        (void)twai_driver_uninstall_v2(*outHandle);
        *outHandle = NULL;
        return err;
    }

    ESP_LOGI(EXAMPLE_TAG, "CAN%u started (%u bit/s)", (unsigned)port, (unsigned)bitrate);
    return ESP_OK;
}

void canReinit(void)
{
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    uint32_t bitrate1 = canBitrateForPort(1u, &settings);
    uint32_t bitrate2 = canBitrateForPort(2u, &settings);

    canStopOne(&s_twaiBus0);
    canStopOne(&s_twaiBus1);

    ESP_ERROR_CHECK(canSetupOne(1u, CAN1_TX, CAN1_RX, bitrate1, TWAI_MODE_NORMAL, &s_twaiBus0));
    ESP_ERROR_CHECK(canSetupOne(2u, CAN2_TX, CAN2_RX, bitrate2, TWAI_MODE_NORMAL, &s_twaiBus1));
}

esp_err_t canReinitPort(uint8_t port, uint32_t bitrate)
{
    return canReinitPortMode(port, bitrate, TWAI_MODE_NORMAL);
}

esp_err_t canReinitPortMode(uint8_t port, uint32_t bitrate, twai_mode_t mode)
{
    if (port == 2u) {
        canStopOne(&s_twaiBus1);
        return canSetupOne(2u, CAN2_TX, CAN2_RX, bitrate, mode, &s_twaiBus1);
    }

    canStopOne(&s_twaiBus0);
    return canSetupOne(1u, CAN1_TX, CAN1_RX, bitrate, mode, &s_twaiBus0);
}

void canInit(void)
{
    canReinit();
}
