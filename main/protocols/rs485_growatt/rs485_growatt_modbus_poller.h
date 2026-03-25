#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uart_port_t uart;
    gpio_num_t dirPin;
    uint8_t slaveAddr;
    size_t pollIndex;
    int64_t lastPollUs;
    bool lastReqValid;
    uint8_t lastReqSlave;
    uint8_t lastReqFunc;
    uint16_t lastReqStart;
    uint16_t lastReqCount;
    int64_t lastReqUs;
} rs485_growatt_modbus_poller_t;

void rs485GrowattModbusPollerInit(rs485_growatt_modbus_poller_t *poller,
                                  uart_port_t uart,
                                  gpio_num_t dirPin,
                                  uint8_t slaveAddr);

esp_err_t rs485GrowattModbusPollerTick(rs485_growatt_modbus_poller_t *poller,
                                       int64_t nowUs,
                                       uint32_t periodMs);

#ifdef __cplusplus
}
#endif
