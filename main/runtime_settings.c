#include "runtime_settings.h"

#include <string.h>

#include "config.h"

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#define SETTINGS_TAG "RUNTIME_SETTINGS"
#define SETTINGS_NS  "bridge_cfg"

static bridge_runtime_settings_t s_runtimeSettings;
static bool s_runtimeSettingsInit;

static bridge_runtime_settings_t defaultSettings(void)
{
    bridge_runtime_settings_t s = {
        .mode = SYSTEM_MODE,
        .bms_line = BMS_line,
        .inverter_line = Inverter_line,
        .bms_protocol = BMS_protocol,
        .inverter_protocol = Inverter_protocol,
        .bms_port = BMS_PORT,
        .inverter_port = Inverter_PORT,
        .web_port = WEB_INTERFACE_PORT,
        .wifi_ssid = WIFI_STA_SSID,
        .wifi_password = WIFI_STA_PASSWORD,
    };
    return s;
}

static bool validateSettings(const bridge_runtime_settings_t *s)
{
    if (s == NULL) {
        return false;
    }
    if (s->mode < MODE_SNIFFER || s->mode > MODE_BRIDGE) {
        return false;
    }
    if (s->bms_line < LINE_CAN || s->bms_line > LINE_RS485) {
        return false;
    }
    if (s->inverter_line < LINE_CAN || s->inverter_line > LINE_RS485) {
        return false;
    }
    if (s->bms_protocol < PROTOCOL_CAN_GROWATT || s->bms_protocol > PROTOCOL_ID_MAX) {
        return false;
    }
    if (s->inverter_protocol < PROTOCOL_CAN_GROWATT || s->inverter_protocol > PROTOCOL_ID_MAX) {
        return false;
    }
    if (s->bms_port < 1u || s->bms_port > 2u) {
        return false;
    }
    if (s->inverter_port < 1u || s->inverter_port > 2u) {
        return false;
    }
    if (s->web_port == 0u) {
        return false;
    }
    if (s->wifi_ssid[0] == '\0') {
        return false;
    }
    if ((s->bms_line == LINE_CAN) &&
        (s->bms_protocol != PROTOCOL_CAN_GROWATT) &&
        (s->bms_protocol != PROTOCOL_CAN_PYLON) &&
        (s->bms_protocol != PROTOCOL_CAN_DEYE) &&
        (s->bms_protocol != PROTOCOL_CAN_GOODWE) &&
        (s->bms_protocol != PROTOCOL_CAN_SOFAR) &&
        (s->bms_protocol != PROTOCOL_CAN_SMA) &&
        (s->bms_protocol != PROTOCOL_CAN_VICTRON)) {
        return false;
    }
    if ((s->inverter_line == LINE_CAN) &&
        (s->inverter_protocol != PROTOCOL_CAN_GROWATT) &&
        (s->inverter_protocol != PROTOCOL_CAN_PYLON) &&
        (s->inverter_protocol != PROTOCOL_CAN_DEYE) &&
        (s->inverter_protocol != PROTOCOL_CAN_GOODWE) &&
        (s->inverter_protocol != PROTOCOL_CAN_SOFAR) &&
        (s->inverter_protocol != PROTOCOL_CAN_SMA) &&
        (s->inverter_protocol != PROTOCOL_CAN_VICTRON)) {
        return false;
    }
    if ((s->bms_line == LINE_RS485) &&
        (s->bms_protocol != PROTOCOL_RS485_GROWATT) &&
        !bridgeProtocolIsRs485Pylon(s->bms_protocol) &&
        !bridgeProtocolIsRs485JkbmsModbus(s->bms_protocol) &&
        (s->bms_protocol != PROTOCOL_RS485_PACE) &&
        (s->bms_protocol != PROTOCOL_RS485_JKBMS_NATIVE) &&
        (s->bms_protocol != PROTOCOL_RS485_VOLTRONIC) &&
        (s->bms_protocol != PROTOCOL_RS485_CHINA_TOWER) &&
        (s->bms_protocol != PROTOCOL_RS485_WOW)) {
        return false;
    }
    if ((s->inverter_line == LINE_RS485) &&
        (s->inverter_protocol != PROTOCOL_RS485_GROWATT) &&
        !bridgeProtocolIsRs485Pylon(s->inverter_protocol)) {
        return false;
    }
    return true;
}

void runtimeSettingsInit(void)
{
    if (s_runtimeSettingsInit) {
        return;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    s_runtimeSettings = defaultSettings();

    nvs_handle_t nvs = 0;
    err = nvs_open(SETTINGS_NS, NVS_READONLY, &nvs);
    if (err == ESP_OK) {
        bridge_runtime_settings_t loaded = s_runtimeSettings;
        uint8_t u8val = 0;
        size_t len = 0;

        if (nvs_get_u8(nvs, "mode", &u8val) == ESP_OK) {
            loaded.mode = u8val;
        }
        if (nvs_get_u8(nvs, "bline", &u8val) == ESP_OK) {
            loaded.bms_line = u8val;
        }
        if (nvs_get_u8(nvs, "iline", &u8val) == ESP_OK) {
            loaded.inverter_line = u8val;
        }
        if (nvs_get_u8(nvs, "bprot", &u8val) == ESP_OK) {
            loaded.bms_protocol = u8val;
        }
        if (nvs_get_u8(nvs, "iprot", &u8val) == ESP_OK) {
            loaded.inverter_protocol = u8val;
        }
        if (nvs_get_u8(nvs, "bport", &u8val) == ESP_OK) {
            loaded.bms_port = u8val;
        }
        if (nvs_get_u8(nvs, "iport", &u8val) == ESP_OK) {
            loaded.inverter_port = u8val;
        }
        (void)nvs_get_u16(nvs, "wport", &loaded.web_port);

        len = sizeof(loaded.wifi_ssid);
        (void)nvs_get_str(nvs, "ssid", loaded.wifi_ssid, &len);

        len = sizeof(loaded.wifi_password);
        (void)nvs_get_str(nvs, "pass", loaded.wifi_password, &len);

        if (validateSettings(&loaded)) {
            s_runtimeSettings = loaded;
        } else {
            ESP_LOGW(SETTINGS_TAG, "Stored runtime settings invalid, using defaults");
        }
        nvs_close(nvs);
    }

    s_runtimeSettingsInit = true;
}

bridge_runtime_settings_t runtimeSettingsGet(void)
{
    if (!s_runtimeSettingsInit) {
        runtimeSettingsInit();
    }
    return s_runtimeSettings;
}

bool runtimeSettingsSave(const bridge_runtime_settings_t *settings)
{
    if (!validateSettings(settings)) {
        return false;
    }
    if (!s_runtimeSettingsInit) {
        runtimeSettingsInit();
    }

    nvs_handle_t nvs = 0;
    if (nvs_open(SETTINGS_NS, NVS_READWRITE, &nvs) != ESP_OK) {
        return false;
    }

    bool ok = true;
    ok &= (nvs_set_u8(nvs, "mode", settings->mode) == ESP_OK);
    ok &= (nvs_set_u8(nvs, "bline", settings->bms_line) == ESP_OK);
    ok &= (nvs_set_u8(nvs, "iline", settings->inverter_line) == ESP_OK);
    ok &= (nvs_set_u8(nvs, "bprot", settings->bms_protocol) == ESP_OK);
    ok &= (nvs_set_u8(nvs, "iprot", settings->inverter_protocol) == ESP_OK);
    ok &= (nvs_set_u8(nvs, "bport", settings->bms_port) == ESP_OK);
    ok &= (nvs_set_u8(nvs, "iport", settings->inverter_port) == ESP_OK);
    ok &= (nvs_set_u16(nvs, "wport", settings->web_port) == ESP_OK);
    ok &= (nvs_set_str(nvs, "ssid", settings->wifi_ssid) == ESP_OK);
    ok &= (nvs_set_str(nvs, "pass", settings->wifi_password) == ESP_OK);
    ok &= (nvs_commit(nvs) == ESP_OK);
    nvs_close(nvs);

    if (ok) {
        s_runtimeSettings = *settings;
    }

    return ok;
}
