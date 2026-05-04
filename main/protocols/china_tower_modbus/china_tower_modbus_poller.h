#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

typedef struct {
    uart_port_t uart;
    gpio_num_t dirPin;
    uint8_t slaveAddr;
    uint8_t pollIndex;
    int64_t lastPollUs;

    bool lastReqValid;
    uint8_t lastReqSlave;
    uint8_t lastReqFunc;
    uint16_t lastReqStart;
    uint16_t lastReqCount;
    int64_t lastReqUs;
} china_tower_modbus_poller_t;

void chinaTowerModbusPollerInit(china_tower_modbus_poller_t *poller,
                                uart_port_t uart,
                                gpio_num_t dirPin,
                                uint8_t slaveAddr);

esp_err_t chinaTowerModbusPollerTick(china_tower_modbus_poller_t *poller,
                                     int64_t nowUs,
                                     uint32_t periodMs);
