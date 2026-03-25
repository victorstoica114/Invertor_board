#include "protocols/jkbms_modbus/jkbms_modbus_poller.h"

#include <string.h>

#include "Drivers/rs485_driver.h"
#include "freertos/FreeRTOS.h"
#include "protocols/jkbms_modbus/jkbms_modbus_register_map.h"

static uint16_t jkbmsModbusCrc16(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFFu;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}

static int jkbmsBuildReadHoldingRegsReq(uint8_t slave,
                                        uint16_t start,
                                        uint16_t count,
                                        uint8_t *out,
                                        size_t outCap)
{
    if (out == NULL || outCap < 8u || count == 0u || count > 125u) {
        return 0;
    }

    out[0] = slave;
    out[1] = 0x03u;
    out[2] = (uint8_t)((start >> 8) & 0xFFu);
    out[3] = (uint8_t)(start & 0xFFu);
    out[4] = (uint8_t)((count >> 8) & 0xFFu);
    out[5] = (uint8_t)(count & 0xFFu);

    const uint16_t crc = jkbmsModbusCrc16(out, 6);
    out[6] = (uint8_t)(crc & 0xFFu);
    out[7] = (uint8_t)((crc >> 8) & 0xFFu);
    return 8;
}

void jkbmsModbusPollerInit(jkbms_modbus_poller_t *poller,
                           uart_port_t uart,
                           gpio_num_t dirPin,
                           uint8_t slaveAddr)
{
    if (poller == NULL) {
        return;
    }

    memset(poller, 0, sizeof(*poller));
    poller->uart = uart;
    poller->dirPin = dirPin;
    poller->slaveAddr = slaveAddr;
}

esp_err_t jkbmsModbusPollerTick(jkbms_modbus_poller_t *poller,
                                int64_t nowUs,
                                uint32_t periodMs)
{
    if (poller == NULL || g_jkbmsModbusPollBlocksCount == 0u) {
        return ESP_ERR_INVALID_ARG;
    }

    const int64_t periodUs = (int64_t)periodMs * 1000LL;
    if (poller->lastPollUs != 0 && (nowUs - poller->lastPollUs) < periodUs) {
        return ESP_OK;
    }

    const jkbms_modbus_poll_block_t *block = &g_jkbmsModbusPollBlocks[poller->pollIndex];
    uint8_t req[8];
    int reqLen = jkbmsBuildReadHoldingRegsReq(poller->slaveAddr,
                                              block->start,
                                              block->count,
                                              req,
                                              sizeof(req));
    if (reqLen <= 0) {
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t err = rs485WriteBytes(poller->uart,
                                    poller->dirPin,
                                    req,
                                    reqLen,
                                    pdMS_TO_TICKS(30));
    if (err == ESP_OK) {
        poller->pollIndex = (poller->pollIndex + 1u) % g_jkbmsModbusPollBlocksCount;
        poller->lastPollUs = nowUs;
    }
    return err;
}
