#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t mode;
    uint8_t bms_line;
    uint8_t inverter_line;
    uint8_t bms_protocol;
    uint8_t inverter_protocol;
    uint8_t bms_port;
    uint8_t inverter_port;
    uint16_t web_port;
    char wifi_ssid[33];
    char wifi_password[65];
} bridge_runtime_settings_t;

#ifdef __cplusplus
extern "C" {
#endif

void runtimeSettingsInit(void);
bridge_runtime_settings_t runtimeSettingsGet(void);
bool runtimeSettingsSave(const bridge_runtime_settings_t *settings);

#ifdef __cplusplus
}
#endif
