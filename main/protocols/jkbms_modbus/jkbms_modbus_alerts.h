#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t jkbmsModbusNormalizeAlarmBits(uint32_t alarmBits);

bool jkbmsModbusAlarmBitsAreValidated(uint32_t alarmBits);

void jkbmsModbusFormatAlertFields(uint32_t alarmBits,
                                  char *protectionsOut,
                                  size_t protectionsOutSize,
                                  char *alarmsOut,
                                  size_t alarmsOutSize,
                                  char *warningsOut,
                                  size_t warningsOutSize);

#ifdef __cplusplus
}
#endif
