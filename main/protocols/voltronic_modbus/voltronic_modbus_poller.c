#include "protocols/voltronic_modbus/voltronic_modbus_poller.h"

#include <string.h>

#include "Drivers/rs485_driver.h"
#include "freertos/FreeRTOS.h"
#include "protocols/voltronic_modbus/voltronic_modbus_registers_map.h"

static uint16_t voltronicModbusCrc16(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFFu;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if ((crc & 1u) != 0u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}

static int voltronicBuildReadRegsReq(uint8_t slave,
                                     uint8_t functionCode,
                                     uint16_t start,
                                     uint16_t count,
                                     uint8_t frameOrder,
                                     uint8_t *out,
                                     size_t outCap)
{
    if (out == NULL || outCap < 8u || count == 0u || count > 125u ||
        (functionCode != VOLTRONIC_MB_READ_HOLDING_REGS &&
         functionCode != VOLTRONIC_MB_READ_INPUT_REGS)) {
        return 0;
    }

    if (frameOrder == VOLTRONIC_MB_FRAME_FUNCTION_FIRST) {
        out[0] = functionCode;
        out[1] = slave;
    } else {
        out[0] = slave;
        out[1] = functionCode;
    }
    out[2] = (uint8_t)((start >> 8) & 0xFFu);
    out[3] = (uint8_t)(start & 0xFFu);
    out[4] = (uint8_t)((count >> 8) & 0xFFu);
    out[5] = (uint8_t)(count & 0xFFu);

    const uint16_t crc = voltronicModbusCrc16(out, 6);
    out[6] = (uint8_t)(crc & 0xFFu);
    out[7] = (uint8_t)((crc >> 8) & 0xFFu);
    return 8;
}

void voltronicModbusPollerInit(voltronic_modbus_poller_t *poller,
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

esp_err_t voltronicModbusPollerTick(voltronic_modbus_poller_t *poller,
                                    int64_t nowUs,
                                    uint32_t periodMs)
{
    if (poller == NULL || g_voltronicModbusPollBlocksCount == 0u) {
        return ESP_ERR_INVALID_ARG;
    }

    const int64_t periodUs = (int64_t)periodMs * 1000LL;
    if (poller->lastPollUs != 0 && (nowUs - poller->lastPollUs) < periodUs) {
        return ESP_OK;
    }

    const voltronic_modbus_poll_block_t *block =
        &g_voltronicModbusPollBlocks[poller->pollIndex];
    uint8_t req[8];
    int reqLen = voltronicBuildReadRegsReq(poller->slaveAddr,
                                           block->functionCode,
                                           block->start,
                                           block->count,
                                           block->frameOrder,
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
        poller->lastReqValid = true;
        poller->lastReqSlave = poller->slaveAddr;
        poller->lastReqFunc = block->functionCode;
        poller->lastReqStart = block->start;
        poller->lastReqCount = block->count;
        poller->lastReqFrameOrder = block->frameOrder;
        poller->lastReqLen = (uint8_t)reqLen;
        memcpy(poller->lastReqFrame, req, (size_t)reqLen);
        poller->lastReqUs = nowUs;
        poller->pollIndex = (poller->pollIndex + 1u) % g_voltronicModbusPollBlocksCount;
        poller->lastPollUs = nowUs;
    }

    return err;
}
