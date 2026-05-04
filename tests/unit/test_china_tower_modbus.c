#include "unity.h"

#include <stdint.h>
#include <string.h>

#include "decoders/modbusDecoder.h"
#include "protocols/china_tower_modbus/china_tower_modbus_bms_task.h"
#include "protocols/china_tower_modbus/china_tower_modbus_poller.h"
#include "protocols/china_tower_modbus/china_tower_modbus_registers_map.h"

void setUp(void)
{
}

void tearDown(void)
{
}

static uint16_t crc16(const uint8_t *data, int len)
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

static int buildResp03(uint8_t slave,
                       const uint16_t *regs,
                       uint8_t count,
                       uint8_t *out,
                       int outCap)
{
    int len = 3 + ((int)count * 2) + 2;
    TEST_ASSERT_TRUE(len <= outCap);

    out[0] = slave;
    out[1] = 0x03u;
    out[2] = (uint8_t)(count * 2u);
    for (uint8_t i = 0; i < count; i++) {
        out[3 + (i * 2)] = (uint8_t)((regs[i] >> 8) & 0xFFu);
        out[4 + (i * 2)] = (uint8_t)(regs[i] & 0xFFu);
    }
    uint16_t crc = crc16(out, len - 2);
    out[len - 2] = (uint8_t)(crc & 0xFFu);
    out[len - 1] = (uint8_t)((crc >> 8) & 0xFFu);
    return len;
}

static void feedChinaTowerResponse(modbusDecoder_t *decoder,
                                   uint16_t start,
                                   const uint16_t *regs,
                                   uint8_t count)
{
    uint8_t frame[80];
    int len = buildResp03(0x01u, regs, count, frame, sizeof(frame));

    modbusDecoderRecordRequest(decoder, 0x01u, 0x03u, start, count, 1000);
    modbusDecoderFeed(decoder, frame, len, 2000);
    modbusDecoderFlush(decoder);
}

void test_china_tower_modbus_decodes_runtime_cells_and_temps(void)
{
    modbusDecoder_t decoder;
    bms_decoded_packet_t packet;

    uint16_t summary[13] = {
        7270u,
        16u,
        100u,
        4000u,
        100u,
        0u,
        26u,
        27u,
        29u,
        4528u,
        4559u,
        4618u,
        4509u,
    };
    uint16_t cells[16] = {
        4528u, 4559u, 4618u, 4509u,
        4517u, 4526u, 4557u, 4531u,
        4591u, 4574u, 4578u, 4476u,
        4519u, 4519u, 4519u, 4584u,
    };
    uint16_t flags[3] = {0x0001u, 0x0002u, 0x0004u};

    modbusDecoderInit(&decoder, "CHINA_TOWER_TEST", 5000);
    feedChinaTowerResponse(&decoder, CHINA_TOWER_MB_REG_PACK_VOLTAGE_10MV, summary, 13u);
    feedChinaTowerResponse(&decoder, CHINA_TOWER_MB_REG_CELL01_MV, cells, 16u);
    feedChinaTowerResponse(&decoder, CHINA_TOWER_MB_REG_WARNING_FLAGS, flags, 3u);

    TEST_ASSERT_TRUE(chinaTowerModbusBuildDecodedPacket(&decoder, 8u, &packet));
    TEST_ASSERT_EQUAL_UINT8(PROTOCOL_ID_CHINA_TOWER, packet.sourceProtocol);
    TEST_ASSERT_EQUAL_UINT32(8u, packet.sequence);
    TEST_ASSERT_TRUE(packet.hasSoc);
    TEST_ASSERT_EQUAL_UINT8(100u, packet.socPct);
    TEST_ASSERT_TRUE(packet.hasPackVoltageCv);
    TEST_ASSERT_EQUAL_UINT16(7270u, packet.packVoltageCv);
    TEST_ASSERT_TRUE(packet.hasCellExtremes);
    TEST_ASSERT_EQUAL_UINT16(4476u, packet.minCellMv);
    TEST_ASSERT_EQUAL_UINT8(12u, packet.minCellIndex);
    TEST_ASSERT_EQUAL_UINT16(4618u, packet.maxCellMv);
    TEST_ASSERT_EQUAL_UINT8(3u, packet.maxCellIndex);
    TEST_ASSERT_EQUAL_UINT8(16u, packet.cellCount);
    TEST_ASSERT_EQUAL_UINT16(4528u, packet.cellMv[0]);
    TEST_ASSERT_EQUAL_UINT16(4476u, packet.cellMv[11]);
    TEST_ASSERT_EQUAL_UINT16(4584u, packet.cellMv[15]);
    TEST_ASSERT_TRUE(packet.hasTemperatureC);
    TEST_ASSERT_EQUAL_UINT8(3u, packet.tempCount);
    TEST_ASSERT_EQUAL_INT16(260, packet.tempDeciC[0]);
    TEST_ASSERT_EQUAL_INT16(270, packet.tempDeciC[1]);
    TEST_ASSERT_EQUAL_INT16(290, packet.tempDeciC[2]);
    TEST_ASSERT_TRUE(packet.hasWarningFlags);
    TEST_ASSERT_EQUAL_UINT16(0x0001u, packet.warningFlags);
    TEST_ASSERT_TRUE(packet.hasProtectionFlags);
    TEST_ASSERT_EQUAL_UINT16(0x0002u, packet.protectionFlags);
    TEST_ASSERT_TRUE(packet.hasStatusFlags);
    TEST_ASSERT_EQUAL_UINT16(0x0004u, packet.statusFlags);
    TEST_ASSERT_FALSE(packet.hasBalanceFlags);
}

void test_china_tower_modbus_register_map_matches_initial_poll_plan(void)
{
    TEST_ASSERT_EQUAL_UINT32(3u, (uint32_t)g_chinaTowerModbusPollBlocksCount);

    TEST_ASSERT_EQUAL_UINT16(CHINA_TOWER_MB_REG_PACK_VOLTAGE_10MV,
                             g_chinaTowerModbusPollBlocks[0].start);
    TEST_ASSERT_EQUAL_UINT16(13u, g_chinaTowerModbusPollBlocks[0].count);
    TEST_ASSERT_EQUAL_UINT16(CHINA_TOWER_MB_REG_CELL_COUNT,
                             CHINA_TOWER_MB_REG_PACK_VOLTAGE_10MV + 1u);
    TEST_ASSERT_EQUAL_UINT16(CHINA_TOWER_MB_REG_SOC_PCT,
                             CHINA_TOWER_MB_REG_PACK_VOLTAGE_10MV + 2u);

    TEST_ASSERT_EQUAL_UINT16(CHINA_TOWER_MB_REG_CELL01_MV,
                             g_chinaTowerModbusPollBlocks[1].start);
    TEST_ASSERT_EQUAL_UINT16(16u, g_chinaTowerModbusPollBlocks[1].count);
    TEST_ASSERT_EQUAL_UINT16(CHINA_TOWER_MB_REG_WARNING_FLAGS,
                             g_chinaTowerModbusPollBlocks[2].start);
    TEST_ASSERT_EQUAL_UINT16(3u, g_chinaTowerModbusPollBlocks[2].count);
    TEST_ASSERT_EQUAL_UINT16(CHINA_TOWER_MB_REG_CELL01_MV + 15u,
                             CHINA_TOWER_MB_REG_CELL16_MV);
    TEST_ASSERT_EQUAL_UINT16(CHINA_TOWER_MB_REG_CELL01_MV + 22u,
                             0x001Fu);
    TEST_ASSERT_EQUAL_UINT16(CHINA_TOWER_MB_REG_TEMP1_C,
                             CHINA_TOWER_MB_REG_PACK_VOLTAGE_10MV + 6u);
    TEST_ASSERT_EQUAL_UINT16(CHINA_TOWER_MB_REG_TEMP2_C,
                             CHINA_TOWER_MB_REG_PACK_VOLTAGE_10MV + 7u);
    TEST_ASSERT_EQUAL_UINT16(CHINA_TOWER_MB_REG_MOS_TEMP_C,
                             CHINA_TOWER_MB_REG_PACK_VOLTAGE_10MV + 8u);
    TEST_ASSERT_EQUAL_UINT16(CHINA_TOWER_MB_REG_CELL16_MV + 1u,
                             CHINA_TOWER_MB_REG_WARNING_FLAGS);
    TEST_ASSERT_EQUAL_UINT16(CHINA_TOWER_MB_REG_WARNING_FLAGS + 1u,
                             CHINA_TOWER_MB_REG_PROTECTION_FLAGS);
    TEST_ASSERT_EQUAL_UINT16(CHINA_TOWER_MB_REG_WARNING_FLAGS + 2u,
                             CHINA_TOWER_MB_REG_STATUS_FLAGS);
}

void test_china_tower_modbus_poller_cycles_runtime_and_cell_blocks(void)
{
    china_tower_modbus_poller_t poller;

    chinaTowerModbusPollerInit(&poller, 0, 0, 0x01u);

    TEST_ASSERT_EQUAL(ESP_OK, chinaTowerModbusPollerTick(&poller, 1000000LL, 250u));
    TEST_ASSERT_TRUE(poller.lastReqValid);
    TEST_ASSERT_EQUAL_UINT8(0x01u, poller.lastReqSlave);
    TEST_ASSERT_EQUAL_UINT8(0x03u, poller.lastReqFunc);
    TEST_ASSERT_EQUAL_UINT16(CHINA_TOWER_MB_REG_PACK_VOLTAGE_10MV, poller.lastReqStart);
    TEST_ASSERT_EQUAL_UINT16(13u, poller.lastReqCount);

    TEST_ASSERT_EQUAL(ESP_OK, chinaTowerModbusPollerTick(&poller, 1100000LL, 250u));
    TEST_ASSERT_EQUAL_UINT16(CHINA_TOWER_MB_REG_PACK_VOLTAGE_10MV, poller.lastReqStart);

    TEST_ASSERT_EQUAL(ESP_OK, chinaTowerModbusPollerTick(&poller, 1300000LL, 250u));
    TEST_ASSERT_EQUAL_UINT16(CHINA_TOWER_MB_REG_CELL01_MV, poller.lastReqStart);
    TEST_ASSERT_EQUAL_UINT16(16u, poller.lastReqCount);

    TEST_ASSERT_EQUAL(ESP_OK, chinaTowerModbusPollerTick(&poller, 1600000LL, 250u));
    TEST_ASSERT_EQUAL_UINT16(CHINA_TOWER_MB_REG_WARNING_FLAGS, poller.lastReqStart);
    TEST_ASSERT_EQUAL_UINT16(3u, poller.lastReqCount);

    TEST_ASSERT_EQUAL(ESP_OK, chinaTowerModbusPollerTick(&poller, 1900000LL, 250u));
    TEST_ASSERT_EQUAL_UINT16(CHINA_TOWER_MB_REG_PACK_VOLTAGE_10MV, poller.lastReqStart);
}

void test_china_tower_modbus_accepts_high_byte_soc_layout(void)
{
    modbusDecoder_t decoder;
    bms_decoded_packet_t packet;
    uint16_t summary[13] = {
        0u, 5120u, 0x5500u, 0x6300u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u,
    };

    modbusDecoderInit(&decoder, "CHINA_TOWER_PCT", 5000);
    feedChinaTowerResponse(&decoder, CHINA_TOWER_MB_REG_PACK_VOLTAGE_10MV, summary, 13u);

    TEST_ASSERT_TRUE(chinaTowerModbusBuildDecodedPacket(&decoder, 1u, &packet));
    TEST_ASSERT_TRUE(packet.hasSoc);
    TEST_ASSERT_EQUAL_UINT8(85u, packet.socPct);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_china_tower_modbus_decodes_runtime_cells_and_temps);
    RUN_TEST(test_china_tower_modbus_register_map_matches_initial_poll_plan);
    RUN_TEST(test_china_tower_modbus_poller_cycles_runtime_and_cell_blocks);
    RUN_TEST(test_china_tower_modbus_accepts_high_byte_soc_layout);

    return UNITY_END();
}
