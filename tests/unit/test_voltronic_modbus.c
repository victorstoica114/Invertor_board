#include "unity.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "decoders/modbusDecoder.h"
#include "protocols/voltronic_modbus/voltronic_modbus_bms_task.h"
#include "protocols/voltronic_modbus/voltronic_modbus_poller.h"
#include "protocols/voltronic_modbus/voltronic_modbus_registers_map.h"

extern uint8_t g_rs485StubLastWrite[256];
extern int g_rs485StubLastWriteLen;
void rs485StubResetLastWrite(void);

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
            if ((crc & 1u) != 0u) {
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

    out[0] = 0x03u;
    out[1] = slave;
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

static int buildWordCountResp03(uint8_t slave,
                                const uint16_t *regs,
                                uint8_t count,
                                uint8_t *out,
                                int outCap)
{
    int len = 4 + ((int)count * 2) + 2;
    TEST_ASSERT_TRUE(len <= outCap);

    out[0] = slave;
    out[1] = 0x03u;
    out[2] = 0x00u;
    out[3] = count;
    for (uint8_t i = 0; i < count; i++) {
        out[4 + (i * 2)] = (uint8_t)((regs[i] >> 8) & 0xFFu);
        out[5 + (i * 2)] = (uint8_t)(regs[i] & 0xFFu);
    }
    uint16_t crc = crc16(out, len - 2);
    out[len - 2] = (uint8_t)(crc & 0xFFu);
    out[len - 1] = (uint8_t)((crc >> 8) & 0xFFu);
    return len;
}

static int buildWideByteCountResp03(uint8_t slave,
                                    const uint16_t *regs,
                                    uint8_t count,
                                    uint8_t *out,
                                    int outCap)
{
    int len = 4 + ((int)count * 2) + 2;
    TEST_ASSERT_TRUE(len <= outCap);

    out[0] = slave;
    out[1] = 0x03u;
    out[2] = 0x00u;
    out[3] = (uint8_t)(count * 2u);
    for (uint8_t i = 0; i < count; i++) {
        out[4 + (i * 2)] = (uint8_t)((regs[i] >> 8) & 0xFFu);
        out[5 + (i * 2)] = (uint8_t)(regs[i] & 0xFFu);
    }
    uint16_t crc = crc16(out, len - 2);
    out[len - 2] = (uint8_t)(crc & 0xFFu);
    out[len - 1] = (uint8_t)((crc >> 8) & 0xFFu);
    return len;
}

static void feedVoltronicResponse(modbusDecoder_t *decoder,
                                  uint16_t start,
                                  const uint16_t *regs,
                                  uint8_t count)
{
    uint8_t frame[120];
    int len = buildResp03(VOLTRONIC_MODBUS_DEFAULT_SLAVE_ADDR,
                          regs,
                          count,
                          frame,
                          sizeof(frame));

    modbusDecoderRecordRequest(decoder,
                               VOLTRONIC_MODBUS_DEFAULT_SLAVE_ADDR,
                               0x03u,
                               start,
                               count,
                               1000);
    modbusDecoderFeed(decoder, frame, len, 2000);
    modbusDecoderFlush(decoder);
}

static void feedVoltronicWordCountResponse(modbusDecoder_t *decoder,
                                           uint16_t start,
                                           const uint16_t *regs,
                                           uint8_t count)
{
    uint8_t frame[128];
    int len = buildWordCountResp03(VOLTRONIC_MODBUS_DEFAULT_SLAVE_ADDR,
                                   regs,
                                   count,
                                   frame,
                                   sizeof(frame));

    modbusDecoderRecordRequest(decoder,
                               VOLTRONIC_MODBUS_DEFAULT_SLAVE_ADDR,
                               0x03u,
                               start,
                               count,
                               1000);
    modbusDecoderFeed(decoder, frame, len, 2000);
    modbusDecoderFlush(decoder);
}

static void feedVoltronicWideByteCountResponse(modbusDecoder_t *decoder,
                                               uint16_t start,
                                               const uint16_t *regs,
                                               uint8_t count,
                                               int64_t requestUs)
{
    uint8_t frame[128];
    int len = buildWideByteCountResp03(VOLTRONIC_MODBUS_DEFAULT_SLAVE_ADDR,
                                       regs,
                                       count,
                                       frame,
                                       sizeof(frame));

    modbusDecoderRecordRequest(decoder,
                               VOLTRONIC_MODBUS_DEFAULT_SLAVE_ADDR,
                               0x03u,
                               start,
                               count,
                               requestUs);
    modbusDecoderFeed(decoder, frame, len, requestUs + 1000);
    modbusDecoderFlush(decoder);
}

static void setReg(uint16_t *regs, uint16_t start, uint16_t reg, uint16_t value)
{
    regs[reg - start] = value;
}

void test_voltronic_modbus_decodes_status_warnings_limits(void)
{
    enum {
        STATUS_COUNT = VOLTRONIC_MB_REG_STATUS_END - VOLTRONIC_MB_REG_STATUS_START + 1u,
        WARNING_COUNT = VOLTRONIC_MB_REG_WARNING_END - VOLTRONIC_MB_REG_WARNING_START + 1u,
        LIMITS_COUNT = VOLTRONIC_MB_REG_LIMITS_END - VOLTRONIC_MB_REG_LIMITS_START + 1u,
    };
    static const uint16_t cells[16] = {
        4528u, 4559u, 4618u, 4509u,
        4517u, 4526u, 4557u, 4531u,
        4593u, 4576u, 4578u, 4476u,
        4521u, 4520u, 4520u, 4584u,
    };
    static const uint16_t tempsDeciK[6] = {
        3001u, 3006u, 2996u, 3011u, 2986u, 3004u,
    };
    uint16_t statusRegs[STATUS_COUNT] = {0};
    uint16_t warningRegs[WARNING_COUNT] = {0};
    uint16_t limitsRegs[LIMITS_COUNT] = {0};
    modbusDecoder_t decoder;
    voltronic_modbus_snapshot_t snapshot;
    bms_decoded_packet_t packet;

    setReg(statusRegs, VOLTRONIC_MB_REG_STATUS_START, VOLTRONIC_MB_REG_CELL_COUNT, 16u);
    for (uint8_t i = 0u; i < 16u; i++) {
        setReg(statusRegs, VOLTRONIC_MB_REG_STATUS_START, (uint16_t)(VOLTRONIC_MB_REG_CELL01 + i), cells[i]);
    }
    setReg(statusRegs, VOLTRONIC_MB_REG_STATUS_START, VOLTRONIC_MB_REG_TEMP_COUNT, 6u);
    for (uint8_t i = 0u; i < 6u; i++) {
        setReg(statusRegs, VOLTRONIC_MB_REG_STATUS_START, (uint16_t)(VOLTRONIC_MB_REG_TEMP01_DECIK + i), tempsDeciK[i]);
    }
    setReg(statusRegs, VOLTRONIC_MB_REG_STATUS_START, VOLTRONIC_MB_REG_CHARGE_CURRENT_DA, 1u);
    setReg(statusRegs, VOLTRONIC_MB_REG_STATUS_START, VOLTRONIC_MB_REG_DISCHARGE_CURRENT_DA, 0u);
    setReg(statusRegs, VOLTRONIC_MB_REG_STATUS_START, VOLTRONIC_MB_REG_MODULE_VOLTAGE_DV, 727u);
    setReg(statusRegs, VOLTRONIC_MB_REG_STATUS_START, VOLTRONIC_MB_REG_SOC_PCT, 100u);
    setReg(statusRegs, VOLTRONIC_MB_REG_STATUS_START, VOLTRONIC_MB_REG_TOTAL_CAPACITY_MAH, 0u);
    setReg(statusRegs, VOLTRONIC_MB_REG_STATUS_START, VOLTRONIC_MB_REG_TOTAL_CAPACITY_MAH + 1u, 40000u);
    setReg(statusRegs, VOLTRONIC_MB_REG_STATUS_START, VOLTRONIC_MB_REG_CHARGE_ALARM, 0x0001u);
    setReg(statusRegs, VOLTRONIC_MB_REG_STATUS_START, VOLTRONIC_MB_REG_DISCHARGE_ALARM, 0x0008u);
    setReg(statusRegs, VOLTRONIC_MB_REG_STATUS_START, VOLTRONIC_MB_REG_CHARGE_PROTECT, 0x0400u);
    setReg(statusRegs, VOLTRONIC_MB_REG_STATUS_START, VOLTRONIC_MB_REG_CHARGE_PROTECT2, 0x0002u);
    setReg(statusRegs, VOLTRONIC_MB_REG_STATUS_START, VOLTRONIC_MB_REG_DISCHARGE_PROTECT, 0x0800u);
    setReg(statusRegs, VOLTRONIC_MB_REG_STATUS_START, VOLTRONIC_MB_REG_BMS_STATE, 0x0055u);
    setReg(statusRegs, VOLTRONIC_MB_REG_STATUS_START, VOLTRONIC_MB_REG_DESIGN_CAPACITY_MAH, 0u);
    setReg(statusRegs, VOLTRONIC_MB_REG_STATUS_START, VOLTRONIC_MB_REG_DESIGN_CAPACITY_MAH + 1u, 50000u);

    setReg(warningRegs, VOLTRONIC_MB_REG_WARNING_START, VOLTRONIC_MB_REG_WARNING_CELL_COUNT, 16u);
    setReg(warningRegs, VOLTRONIC_MB_REG_WARNING_START, VOLTRONIC_MB_REG_CELL_STATE_PAIR01, 0x0200u);
    setReg(warningRegs, VOLTRONIC_MB_REG_WARNING_START, VOLTRONIC_MB_REG_WARNING_TEMP_COUNT, 6u);
    setReg(warningRegs, VOLTRONIC_MB_REG_WARNING_START, VOLTRONIC_MB_REG_TEMP_STATE_PAIR01, 0x0003u);
    setReg(warningRegs, VOLTRONIC_MB_REG_WARNING_START, VOLTRONIC_MB_REG_MODULE_CHG_V_STATE, 2u);
    setReg(warningRegs, VOLTRONIC_MB_REG_WARNING_START, VOLTRONIC_MB_REG_MODULE_CHG_I_STATE, 3u);

    setReg(limitsRegs, VOLTRONIC_MB_REG_LIMITS_START, VOLTRONIC_MB_REG_CHARGE_V_LIMIT_DV, 730u);
    setReg(limitsRegs, VOLTRONIC_MB_REG_LIMITS_START, VOLTRONIC_MB_REG_DISCHARGE_V_LIMIT_DV, 600u);
    setReg(limitsRegs, VOLTRONIC_MB_REG_LIMITS_START, VOLTRONIC_MB_REG_CHARGE_I_LIMIT_DA, 500u);
    setReg(limitsRegs, VOLTRONIC_MB_REG_LIMITS_START, VOLTRONIC_MB_REG_DISCHARGE_I_LIMIT_DA, 800u);
    setReg(limitsRegs, VOLTRONIC_MB_REG_LIMITS_START, VOLTRONIC_MB_REG_CHG_DSG_STATUS,
           VOLTRONIC_MB_STATUS_CHARGE_ENABLE | VOLTRONIC_MB_STATUS_DISCHARGE_ENABLE);
    setReg(limitsRegs, VOLTRONIC_MB_REG_LIMITS_START, VOLTRONIC_MB_REG_REMAIN_CAPACITY_MAH, 0u);
    setReg(limitsRegs, VOLTRONIC_MB_REG_LIMITS_START, VOLTRONIC_MB_REG_REMAIN_CAPACITY_MAH + 1u, 40000u);

    modbusDecoderInit(&decoder, "VOLTRONIC_TEST", 5000);
    feedVoltronicResponse(&decoder, VOLTRONIC_MB_REG_STATUS_START, statusRegs, STATUS_COUNT);
    feedVoltronicResponse(&decoder, VOLTRONIC_MB_REG_WARNING_START, warningRegs, WARNING_COUNT);
    feedVoltronicResponse(&decoder, VOLTRONIC_MB_REG_LIMITS_START, limitsRegs, LIMITS_COUNT);

    TEST_ASSERT_TRUE(voltronicModbusBuildDecodedSnapshot(&decoder, 7000, &snapshot));
    TEST_ASSERT_TRUE(snapshot.valid);
    TEST_ASSERT_TRUE(snapshot.hasSoc);
    TEST_ASSERT_EQUAL_UINT8(100u, snapshot.socPct);
    TEST_ASSERT_TRUE(snapshot.hasPackVoltage);
    TEST_ASSERT_EQUAL_UINT16(727u, (uint16_t)(snapshot.packVoltageV * 10.0f + 0.5f));
    TEST_ASSERT_TRUE(snapshot.hasPackCurrent);
    TEST_ASSERT_EQUAL_UINT16(1u, (uint16_t)(snapshot.packCurrentA * 10.0f + 0.5f));
    TEST_ASSERT_TRUE(snapshot.hasFullMah);
    TEST_ASSERT_EQUAL_UINT32(40000u, snapshot.fullMah);
    TEST_ASSERT_TRUE(snapshot.hasRemainMah);
    TEST_ASSERT_EQUAL_UINT32(40000u, snapshot.remainMah);
    TEST_ASSERT_TRUE(snapshot.hasDesignMah);
    TEST_ASSERT_EQUAL_UINT32(50000u, snapshot.designMah);

    TEST_ASSERT_TRUE(snapshot.hasCellExtremes);
    TEST_ASSERT_EQUAL_UINT8(16u, snapshot.cellCount);
    TEST_ASSERT_EQUAL_UINT16(4476u, snapshot.minCellMv);
    TEST_ASSERT_EQUAL_UINT8(12u, snapshot.minCellIndex);
    TEST_ASSERT_EQUAL_UINT16(4618u, snapshot.maxCellMv);
    TEST_ASSERT_EQUAL_UINT8(3u, snapshot.maxCellIndex);
    TEST_ASSERT_EQUAL_UINT16(4528u, snapshot.cellMv[0]);
    TEST_ASSERT_EQUAL_UINT16(4476u, snapshot.cellMv[11]);
    TEST_ASSERT_EQUAL_UINT16(4584u, snapshot.cellMv[15]);

    TEST_ASSERT_EQUAL_UINT8(6u, snapshot.tempCount);
    TEST_ASSERT_EQUAL_INT16(270, snapshot.tempDeciC[0]);
    TEST_ASSERT_EQUAL_INT16(275, snapshot.tempDeciC[1]);
    TEST_ASSERT_EQUAL_INT16(265, snapshot.tempDeciC[2]);
    TEST_ASSERT_EQUAL_INT16(280, snapshot.tempDeciC[3]);
    TEST_ASSERT_EQUAL_INT16(255, snapshot.tempDeciC[4]);
    TEST_ASSERT_EQUAL_INT16(273, snapshot.tempDeciC[5]);

    TEST_ASSERT_TRUE(snapshot.hasAlarmRegisters);
    TEST_ASSERT_EQUAL_UINT16(0x0001u, snapshot.chargeAlarm);
    TEST_ASSERT_EQUAL_UINT16(0x0008u, snapshot.dischargeAlarm);
    TEST_ASSERT_EQUAL_UINT16(0x0400u, snapshot.chargeProtect);
    TEST_ASSERT_EQUAL_UINT16(0x0002u, snapshot.chargeProtect2);
    TEST_ASSERT_EQUAL_UINT16(0x0800u, snapshot.dischargeProtect);
    TEST_ASSERT_EQUAL_UINT16(0x0055u, snapshot.bmsState);
    TEST_ASSERT_EQUAL_UINT8(2u, snapshot.cellState[0]);
    TEST_ASSERT_EQUAL_UINT8(0u, snapshot.cellState[1]);
    TEST_ASSERT_EQUAL_UINT8(3u, snapshot.tempState[1]);
    TEST_ASSERT_EQUAL_UINT8(2u, snapshot.moduleState[0]);
    TEST_ASSERT_EQUAL_UINT8(3u, snapshot.moduleState[4]);

    TEST_ASSERT_TRUE(snapshot.hasChargeLimits);
    TEST_ASSERT_EQUAL_UINT16(730u, (uint16_t)(snapshot.chargeVoltageLimitV * 10.0f + 0.5f));
    TEST_ASSERT_EQUAL_UINT16(600u, (uint16_t)(snapshot.dischargeVoltageLimitV * 10.0f + 0.5f));
    TEST_ASSERT_EQUAL_UINT16(500u, (uint16_t)(snapshot.chargeCurrentLimitA * 10.0f + 0.5f));
    TEST_ASSERT_EQUAL_UINT16(800u, (uint16_t)(snapshot.dischargeCurrentLimitA * 10.0f + 0.5f));
    TEST_ASSERT_TRUE(snapshot.hasStatusFlags);
    TEST_ASSERT_EQUAL_UINT16(0x00C0u, snapshot.statusFlags);
    TEST_ASSERT_TRUE(snapshot.chargeEnabled);
    TEST_ASSERT_TRUE(snapshot.dischargeEnabled);

    TEST_ASSERT_TRUE(voltronicModbusBuildDecodedPacket(&snapshot, 42u, &packet));
    TEST_ASSERT_EQUAL_UINT8(PROTOCOL_ID_VOLTRONIC, packet.sourceProtocol);
    TEST_ASSERT_EQUAL_UINT32(42u, packet.sequence);
    TEST_ASSERT_TRUE(packet.hasSoc);
    TEST_ASSERT_EQUAL_UINT8(100u, packet.socPct);
    TEST_ASSERT_TRUE(packet.hasPackVoltageCv);
    TEST_ASSERT_EQUAL_UINT16(7270u, packet.packVoltageCv);
    TEST_ASSERT_TRUE(packet.hasCellExtremes);
    TEST_ASSERT_EQUAL_UINT16(4476u, packet.minCellMv);
    TEST_ASSERT_EQUAL_UINT16(4618u, packet.maxCellMv);
    TEST_ASSERT_EQUAL_UINT8(16u, packet.cellCount);
    TEST_ASSERT_EQUAL_UINT16(4528u, packet.cellMv[0]);
    TEST_ASSERT_EQUAL_UINT16(4476u, packet.cellMv[11]);
    TEST_ASSERT_TRUE(packet.hasTemperatureC);
    TEST_ASSERT_EQUAL_UINT8(6u, packet.tempCount);
    TEST_ASSERT_EQUAL_INT16(270, packet.tempDeciC[0]);
    TEST_ASSERT_EQUAL_INT16(273, packet.tempDeciC[5]);
    TEST_ASSERT_EQUAL_INT16(26, packet.temperatureC);
    TEST_ASSERT_TRUE(packet.hasWarningFlags);
    TEST_ASSERT_EQUAL_UINT16(0x0009u, packet.warningFlags);
    TEST_ASSERT_TRUE(packet.hasProtectionFlags);
    TEST_ASSERT_EQUAL_UINT16(0x0C00u, packet.protectionFlags);
    TEST_ASSERT_TRUE(packet.hasStatusFlags);
    TEST_ASSERT_EQUAL_UINT16(0x00C0u, packet.statusFlags);
}

void test_voltronic_modbus_decodes_jk_vendor_compat_map(void)
{
    enum {
        CELL_BLOCK_COUNT = 16u,
        JK_STATUS_COUNT = 5u,
        JK_RUNTIME_COUNT = 48u,
    };
    static const uint16_t cells[16] = {
        4528u, 4559u, 4618u, 4509u,
        4517u, 4526u, 4557u, 4531u,
        4593u, 4576u, 4578u, 4476u,
        4521u, 4520u, 4520u, 4584u,
    };
    uint16_t cellBlock0[CELL_BLOCK_COUNT] = {0};
    uint16_t cellBlock1[CELL_BLOCK_COUNT] = {0};
    uint16_t statusRegs[JK_STATUS_COUNT] = {0};
    uint16_t runtimeRegs[JK_RUNTIME_COUNT] = {0};
    modbusDecoder_t decoder;
    voltronic_modbus_snapshot_t snapshot;
    bms_decoded_packet_t packet;

    for (uint8_t i = 0u; i < 8u; i++) {
        cellBlock0[i * 2u] = cells[i];
        cellBlock1[i * 2u] = cells[i + 8u];
    }

    setReg(statusRegs, VOLTRONIC_JK_REG_STATUS_START, VOLTRONIC_JK_REG_CELL_COUNT, 16u);
    setReg(statusRegs, VOLTRONIC_JK_REG_STATUS_START, VOLTRONIC_JK_REG_CHARGE_MOS, 0u);
    setReg(statusRegs, VOLTRONIC_JK_REG_STATUS_START, VOLTRONIC_JK_REG_DISCHARGE_MOS, 1u);
    setReg(statusRegs, VOLTRONIC_JK_REG_STATUS_START, VOLTRONIC_JK_REG_RATED_CAPACITY_MAH, 40000u);

    setReg(runtimeRegs, VOLTRONIC_JK_REG_MOS_TEMP_DECIC, VOLTRONIC_JK_REG_MOS_TEMP_DECIC, 270u);
    setReg(runtimeRegs, VOLTRONIC_JK_REG_MOS_TEMP_DECIC, VOLTRONIC_JK_REG_PACK_VOLTAGE_CV, 7270u);
    setReg(runtimeRegs, VOLTRONIC_JK_REG_MOS_TEMP_DECIC,
           (uint16_t)(VOLTRONIC_JK_REG_PACK_VOLTAGE_CV + 2u), 10u);
    setReg(runtimeRegs, VOLTRONIC_JK_REG_MOS_TEMP_DECIC, VOLTRONIC_JK_REG_TEMP1_DECIC, 284u);
    setReg(runtimeRegs, VOLTRONIC_JK_REG_MOS_TEMP_DECIC, VOLTRONIC_JK_REG_TEMP2_DECIC, 277u);
    setReg(runtimeRegs, VOLTRONIC_JK_REG_MOS_TEMP_DECIC, VOLTRONIC_JK_REG_ALARM_U32, 0x2342u);
    setReg(runtimeRegs, VOLTRONIC_JK_REG_MOS_TEMP_DECIC,
           (uint16_t)(VOLTRONIC_JK_REG_ALARM_U32 + 1u), 0x6400u);
    setReg(runtimeRegs, VOLTRONIC_JK_REG_MOS_TEMP_DECIC, VOLTRONIC_JK_REG_BALANCE_SOC_U8X2, 0x0064u);
    setReg(runtimeRegs, VOLTRONIC_JK_REG_MOS_TEMP_DECIC, VOLTRONIC_JK_REG_REMAIN_MAH_I32, 0u);
    setReg(runtimeRegs, VOLTRONIC_JK_REG_MOS_TEMP_DECIC,
           (uint16_t)(VOLTRONIC_JK_REG_REMAIN_MAH_I32 + 1u), 40000u);
    setReg(runtimeRegs, VOLTRONIC_JK_REG_MOS_TEMP_DECIC, VOLTRONIC_JK_REG_FULL_MAH_U32, 0u);
    setReg(runtimeRegs, VOLTRONIC_JK_REG_MOS_TEMP_DECIC,
           (uint16_t)(VOLTRONIC_JK_REG_FULL_MAH_U32 + 1u), 40000u);

    modbusDecoderInit(&decoder, "VOLTRONIC_JK_VENDOR", 5000);
    feedVoltronicWordCountResponse(&decoder, VOLTRONIC_JK_REG_CELL01_MV, cellBlock0, CELL_BLOCK_COUNT);
    feedVoltronicWordCountResponse(&decoder,
                                   (uint16_t)(VOLTRONIC_JK_REG_CELL01_MV + 0x0010u),
                                   cellBlock1,
                                   CELL_BLOCK_COUNT);
    feedVoltronicWordCountResponse(&decoder, VOLTRONIC_JK_REG_STATUS_START, statusRegs, JK_STATUS_COUNT);
    feedVoltronicWordCountResponse(&decoder, VOLTRONIC_JK_REG_MOS_TEMP_DECIC, runtimeRegs, JK_RUNTIME_COUNT);

    TEST_ASSERT_TRUE(voltronicModbusBuildDecodedSnapshot(&decoder, 9000, &snapshot));
    TEST_ASSERT_TRUE(snapshot.valid);
    TEST_ASSERT_TRUE(snapshot.hasSoc);
    TEST_ASSERT_EQUAL_UINT8(100u, snapshot.socPct);
    TEST_ASSERT_TRUE(snapshot.hasPackVoltage);
    TEST_ASSERT_EQUAL_UINT16(7270u, (uint16_t)(snapshot.packVoltageV * 100.0f + 0.5f));
    TEST_ASSERT_TRUE(snapshot.hasPackCurrent);
    TEST_ASSERT_EQUAL_UINT16(10u, (uint16_t)(snapshot.packCurrentA * 100.0f + 0.5f));
    TEST_ASSERT_EQUAL_UINT8(16u, snapshot.cellCount);
    TEST_ASSERT_TRUE(snapshot.hasCellExtremes);
    TEST_ASSERT_EQUAL_UINT16(4476u, snapshot.minCellMv);
    TEST_ASSERT_EQUAL_UINT8(12u, snapshot.minCellIndex);
    TEST_ASSERT_EQUAL_UINT16(4618u, snapshot.maxCellMv);
    TEST_ASSERT_EQUAL_UINT8(3u, snapshot.maxCellIndex);
    TEST_ASSERT_EQUAL_UINT8(3u, snapshot.tempCount);
    TEST_ASSERT_EQUAL_INT16(270, snapshot.tempDeciC[0]);
    TEST_ASSERT_EQUAL_INT16(284, snapshot.tempDeciC[1]);
    TEST_ASSERT_EQUAL_INT16(277, snapshot.tempDeciC[2]);
    TEST_ASSERT_TRUE(snapshot.hasStatusFlags);
    TEST_ASSERT_FALSE(snapshot.chargeEnabled);
    TEST_ASSERT_TRUE(snapshot.dischargeEnabled);
    TEST_ASSERT_TRUE(snapshot.hasAlarmRegisters);
    TEST_ASSERT_EQUAL_UINT16(0x2342u, snapshot.chargeAlarm);
    TEST_ASSERT_EQUAL_UINT16(0x6400u, snapshot.dischargeAlarm);

    TEST_ASSERT_TRUE(voltronicModbusBuildDecodedPacket(&snapshot, 77u, &packet));
    TEST_ASSERT_TRUE(packet.hasSoc);
    TEST_ASSERT_EQUAL_UINT8(100u, packet.socPct);
    TEST_ASSERT_TRUE(packet.hasPackVoltageCv);
    TEST_ASSERT_EQUAL_UINT16(7270u, packet.packVoltageCv);
    TEST_ASSERT_TRUE(packet.hasTemperatureC);
    TEST_ASSERT_EQUAL_UINT8(3u, packet.tempCount);
    TEST_ASSERT_EQUAL_INT16(27, packet.temperatureC);
}

void test_voltronic_modbus_maps_fixed_status_word_count_response(void)
{
    enum {
        STATUS_COUNT = VOLTRONIC_MB_REG_STATUS_END - VOLTRONIC_MB_REG_STATUS_START + 1u,
    };
    static const uint16_t cells[16] = {
        45u, 46u, 46u, 45u,
        45u, 45u, 46u, 45u,
        46u, 46u, 46u, 44u,
        45u, 45u, 45u, 46u,
    };
    uint16_t statusRegs[STATUS_COUNT] = {0};
    modbusDecoder_t decoder;
    voltronic_modbus_snapshot_t snapshot;

    setReg(statusRegs, VOLTRONIC_MB_REG_STATUS_START, VOLTRONIC_MB_REG_CELL_COUNT, 16u);
    for (uint8_t i = 0u; i < 16u; i++) {
        setReg(statusRegs, VOLTRONIC_MB_REG_STATUS_START, (uint16_t)(VOLTRONIC_MB_REG_CELL01 + i), cells[i]);
    }
    setReg(statusRegs, VOLTRONIC_MB_REG_STATUS_START, VOLTRONIC_MB_REG_SOC_PCT, 100u);

    modbusDecoderInit(&decoder, "VOLTRONIC_FIXED_WORD_COUNT", 5000);
    feedVoltronicWordCountResponse(&decoder,
                                   VOLTRONIC_JK_REG_MOS_TEMP_DECIC,
                                   statusRegs,
                                   STATUS_COUNT);

    TEST_ASSERT_TRUE(voltronicModbusBuildDecodedSnapshot(&decoder, 11000, &snapshot));
    TEST_ASSERT_TRUE(snapshot.valid);
    TEST_ASSERT_TRUE(snapshot.hasSoc);
    TEST_ASSERT_EQUAL_UINT8(100u, snapshot.socPct);
    TEST_ASSERT_EQUAL_UINT8(16u, snapshot.cellCount);
    TEST_ASSERT_TRUE(snapshot.hasCellExtremes);
    TEST_ASSERT_EQUAL_UINT16(4400u, snapshot.minCellMv);
    TEST_ASSERT_EQUAL_UINT8(12u, snapshot.minCellIndex);
    TEST_ASSERT_EQUAL_UINT16(4600u, snapshot.maxCellMv);
    TEST_ASSERT_EQUAL_UINT8(2u, snapshot.maxCellIndex);
    TEST_ASSERT_TRUE(snapshot.hasPackVoltage);
    TEST_ASSERT_EQUAL_UINT16(726u, (uint16_t)(snapshot.packVoltageV * 10.0f + 0.5f));
}

void test_voltronic_modbus_decodes_seplos_wide_byte_count_single_regs(void)
{
    modbusDecoder_t decoder;
    voltronic_modbus_snapshot_t snapshot;
    bms_decoded_packet_t packet;
    int64_t requestUs = 1000;

    modbusDecoderInit(&decoder, "VOLTRONIC_SEPLOS_WIDE_BYTE", 5000);

    feedVoltronicWideByteCountResponse(&decoder, VOLTRONIC_MB_REG_CHARGE_CURRENT_DA,
                                       (const uint16_t[]){0u}, 1u, requestUs += 10000);
    feedVoltronicWideByteCountResponse(&decoder, VOLTRONIC_MB_REG_DISCHARGE_CURRENT_DA,
                                       (const uint16_t[]){0u}, 1u, requestUs += 10000);
    feedVoltronicWideByteCountResponse(&decoder, VOLTRONIC_MB_REG_MODULE_VOLTAGE_DV,
                                       (const uint16_t[]){249u}, 1u, requestUs += 10000);
    feedVoltronicWideByteCountResponse(&decoder, VOLTRONIC_MB_REG_SOC_PCT,
                                       (const uint16_t[]){2u}, 1u, requestUs += 10000);
    feedVoltronicWideByteCountResponse(&decoder, VOLTRONIC_MB_REG_TOTAL_CAPACITY_MAH,
                                       (const uint16_t[]){0x0003u}, 1u, requestUs += 10000);
    feedVoltronicWideByteCountResponse(&decoder, (uint16_t)(VOLTRONIC_MB_REG_TOTAL_CAPACITY_MAH + 1u),
                                       (const uint16_t[]){0x0D40u}, 1u, requestUs += 10000);
    feedVoltronicWideByteCountResponse(&decoder, VOLTRONIC_MB_REG_CHARGE_V_LIMIT_DV,
                                       (const uint16_t[]){576u}, 1u, requestUs += 10000);
    feedVoltronicWideByteCountResponse(&decoder, VOLTRONIC_MB_REG_DISCHARGE_V_LIMIT_DV,
                                       (const uint16_t[]){104u}, 1u, requestUs += 10000);
    feedVoltronicWideByteCountResponse(&decoder, VOLTRONIC_MB_REG_CHARGE_I_LIMIT_DA,
                                       (const uint16_t[]){1800u}, 1u, requestUs += 10000);
    feedVoltronicWideByteCountResponse(&decoder, VOLTRONIC_MB_REG_DISCHARGE_I_LIMIT_DA,
                                       (const uint16_t[]){1800u}, 1u, requestUs += 10000);
    feedVoltronicWideByteCountResponse(&decoder, VOLTRONIC_MB_REG_CHG_DSG_STATUS,
                                       (const uint16_t[]){0x00F0u}, 1u, requestUs += 10000);

    TEST_ASSERT_TRUE(voltronicModbusBuildDecodedSnapshot(&decoder, requestUs + 2000, &snapshot));
    TEST_ASSERT_TRUE(snapshot.valid);
    TEST_ASSERT_TRUE(snapshot.hasSoc);
    TEST_ASSERT_EQUAL_UINT8(2u, snapshot.socPct);
    TEST_ASSERT_TRUE(snapshot.hasPackVoltage);
    TEST_ASSERT_EQUAL_UINT16(249u, (uint16_t)(snapshot.packVoltageV * 10.0f + 0.5f));
    TEST_ASSERT_TRUE(snapshot.hasFullMah);
    TEST_ASSERT_EQUAL_UINT32(200000u, snapshot.fullMah);
    TEST_ASSERT_TRUE(snapshot.hasChargeLimits);
    TEST_ASSERT_EQUAL_UINT16(576u, (uint16_t)(snapshot.chargeVoltageLimitV * 10.0f + 0.5f));
    TEST_ASSERT_EQUAL_UINT16(104u, (uint16_t)(snapshot.dischargeVoltageLimitV * 10.0f + 0.5f));
    TEST_ASSERT_EQUAL_UINT16(1800u, (uint16_t)(snapshot.chargeCurrentLimitA * 10.0f + 0.5f));
    TEST_ASSERT_EQUAL_UINT16(1800u, (uint16_t)(snapshot.dischargeCurrentLimitA * 10.0f + 0.5f));
    TEST_ASSERT_TRUE(snapshot.hasStatusFlags);
    TEST_ASSERT_EQUAL_UINT16(0x00F0u, snapshot.statusFlags);
    TEST_ASSERT_TRUE(snapshot.chargeEnabled);
    TEST_ASSERT_TRUE(snapshot.dischargeEnabled);

    TEST_ASSERT_TRUE(voltronicModbusBuildDecodedPacket(&snapshot, 88u, &packet));
    TEST_ASSERT_TRUE(packet.hasSoc);
    TEST_ASSERT_EQUAL_UINT8(2u, packet.socPct);
    TEST_ASSERT_TRUE(packet.hasPackVoltageCv);
    TEST_ASSERT_EQUAL_UINT16(2490u, packet.packVoltageCv);
    TEST_ASSERT_TRUE(packet.hasStatusFlags);
    TEST_ASSERT_EQUAL_UINT16(0x00F0u, packet.statusFlags);
}

void test_voltronic_modbus_register_map_matches_poll_plan(void)
{
    TEST_ASSERT_EQUAL_UINT32(12u, (uint32_t)g_voltronicModbusPollBlocksCount);

    TEST_ASSERT_EQUAL_UINT16(VOLTRONIC_MB_REG_PROTOCOL_TYPE, g_voltronicModbusPollBlocks[0].start);
    TEST_ASSERT_EQUAL_UINT16(1u, g_voltronicModbusPollBlocks[0].count);
    TEST_ASSERT_EQUAL_UINT8(VOLTRONIC_MB_FRAME_CLASSIC, g_voltronicModbusPollBlocks[0].frameOrder);
    TEST_ASSERT_EQUAL_UINT8(VOLTRONIC_MB_READ_HOLDING_REGS, g_voltronicModbusPollBlocks[0].functionCode);
    TEST_ASSERT_EQUAL_UINT16(VOLTRONIC_MB_REG_CHARGE_CURRENT_DA, g_voltronicModbusPollBlocks[1].start);
    TEST_ASSERT_EQUAL_UINT8(VOLTRONIC_MB_READ_HOLDING_REGS, g_voltronicModbusPollBlocks[1].functionCode);
    TEST_ASSERT_EQUAL_UINT16(VOLTRONIC_MB_REG_DISCHARGE_CURRENT_DA, g_voltronicModbusPollBlocks[2].start);
    TEST_ASSERT_EQUAL_UINT8(VOLTRONIC_MB_READ_HOLDING_REGS, g_voltronicModbusPollBlocks[2].functionCode);
    TEST_ASSERT_EQUAL_UINT16(VOLTRONIC_MB_REG_MODULE_VOLTAGE_DV, g_voltronicModbusPollBlocks[3].start);
    TEST_ASSERT_EQUAL_UINT8(VOLTRONIC_MB_FRAME_CLASSIC, g_voltronicModbusPollBlocks[3].frameOrder);
    TEST_ASSERT_EQUAL_UINT8(VOLTRONIC_MB_READ_HOLDING_REGS, g_voltronicModbusPollBlocks[3].functionCode);
    TEST_ASSERT_EQUAL_UINT16(VOLTRONIC_MB_REG_SOC_PCT, g_voltronicModbusPollBlocks[4].start);
    TEST_ASSERT_EQUAL_UINT16(VOLTRONIC_MB_REG_TOTAL_CAPACITY_MAH, g_voltronicModbusPollBlocks[5].start);
    TEST_ASSERT_EQUAL_UINT16((VOLTRONIC_MB_REG_TOTAL_CAPACITY_MAH + 1u), g_voltronicModbusPollBlocks[6].start);
    TEST_ASSERT_EQUAL_UINT16(VOLTRONIC_MB_REG_LIMITS_START, g_voltronicModbusPollBlocks[7].start);
    TEST_ASSERT_EQUAL_UINT16(VOLTRONIC_MB_REG_CHG_DSG_STATUS, g_voltronicModbusPollBlocks[11].start);

    TEST_ASSERT_EQUAL_UINT16(0x0010u, VOLTRONIC_MB_REG_CELL_COUNT);
    TEST_ASSERT_EQUAL_UINT16(0x0025u, VOLTRONIC_MB_REG_TEMP_COUNT);
    TEST_ASSERT_EQUAL_UINT16(0x0032u, VOLTRONIC_MB_REG_MODULE_VOLTAGE_DV);
    TEST_ASSERT_EQUAL_UINT16(0x0040u, VOLTRONIC_MB_REG_WARNING_CELL_COUNT);
    TEST_ASSERT_EQUAL_UINT16(0x0074u, VOLTRONIC_MB_REG_CHG_DSG_STATUS);
    TEST_ASSERT_EQUAL_UINT16(0x106Cu, VOLTRONIC_JK_REG_STATUS_START);
    TEST_ASSERT_EQUAL_UINT16(0x1290u, VOLTRONIC_JK_REG_PACK_VOLTAGE_CV);
}

void test_voltronic_modbus_poller_cycles_public_then_jk_blocks(void)
{
    voltronic_modbus_poller_t poller;

    voltronicModbusPollerInit(&poller, 0, 0, VOLTRONIC_MODBUS_DEFAULT_SLAVE_ADDR);
    rs485StubResetLastWrite();

    TEST_ASSERT_EQUAL(ESP_OK, voltronicModbusPollerTick(&poller, 1000000LL, 250u));
    TEST_ASSERT_TRUE(poller.lastReqValid);
    TEST_ASSERT_EQUAL(8, g_rs485StubLastWriteLen);
    TEST_ASSERT_EQUAL_UINT8(VOLTRONIC_MODBUS_DEFAULT_SLAVE_ADDR, g_rs485StubLastWrite[0]);
    TEST_ASSERT_EQUAL_UINT8(0x03u, g_rs485StubLastWrite[1]);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(VOLTRONIC_MB_REG_PROTOCOL_TYPE >> 8), g_rs485StubLastWrite[2]);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(VOLTRONIC_MB_REG_PROTOCOL_TYPE & 0xFFu), g_rs485StubLastWrite[3]);
    TEST_ASSERT_EQUAL_UINT8(VOLTRONIC_MODBUS_DEFAULT_SLAVE_ADDR, poller.lastReqSlave);
    TEST_ASSERT_EQUAL_UINT8(VOLTRONIC_MB_READ_HOLDING_REGS, poller.lastReqFunc);
    TEST_ASSERT_EQUAL_UINT8(VOLTRONIC_MB_FRAME_CLASSIC, poller.lastReqFrameOrder);
    TEST_ASSERT_EQUAL_UINT16(VOLTRONIC_MB_REG_PROTOCOL_TYPE, poller.lastReqStart);
    TEST_ASSERT_EQUAL_UINT16(1u, poller.lastReqCount);

    TEST_ASSERT_EQUAL(ESP_OK, voltronicModbusPollerTick(&poller, 1100000LL, 250u));
    TEST_ASSERT_EQUAL_UINT16(VOLTRONIC_MB_REG_PROTOCOL_TYPE, poller.lastReqStart);

    TEST_ASSERT_EQUAL(ESP_OK, voltronicModbusPollerTick(&poller, 1300000LL, 250u));
    TEST_ASSERT_EQUAL_UINT16(VOLTRONIC_MB_REG_CHARGE_CURRENT_DA, poller.lastReqStart);
    TEST_ASSERT_EQUAL_UINT8(VOLTRONIC_MB_READ_HOLDING_REGS, poller.lastReqFunc);
    TEST_ASSERT_EQUAL_UINT16(1u, poller.lastReqCount);

    TEST_ASSERT_EQUAL(ESP_OK, voltronicModbusPollerTick(&poller, 1600000LL, 250u));
    TEST_ASSERT_EQUAL_UINT16(VOLTRONIC_MB_REG_DISCHARGE_CURRENT_DA, poller.lastReqStart);
    TEST_ASSERT_EQUAL_UINT8(VOLTRONIC_MB_READ_HOLDING_REGS, poller.lastReqFunc);
    TEST_ASSERT_EQUAL_UINT16(1u, poller.lastReqCount);

    TEST_ASSERT_EQUAL(ESP_OK, voltronicModbusPollerTick(&poller, 1900000LL, 250u));
    TEST_ASSERT_EQUAL_UINT16(VOLTRONIC_MB_REG_MODULE_VOLTAGE_DV, poller.lastReqStart);
    TEST_ASSERT_EQUAL_UINT8(VOLTRONIC_MB_FRAME_CLASSIC, poller.lastReqFrameOrder);
    TEST_ASSERT_EQUAL_UINT8(VOLTRONIC_MB_READ_HOLDING_REGS, poller.lastReqFunc);
    TEST_ASSERT_EQUAL_UINT16(1u, poller.lastReqCount);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_voltronic_modbus_decodes_status_warnings_limits);
    RUN_TEST(test_voltronic_modbus_decodes_jk_vendor_compat_map);
    RUN_TEST(test_voltronic_modbus_maps_fixed_status_word_count_response);
    RUN_TEST(test_voltronic_modbus_decodes_seplos_wide_byte_count_single_regs);
    RUN_TEST(test_voltronic_modbus_register_map_matches_poll_plan);
    RUN_TEST(test_voltronic_modbus_poller_cycles_public_then_jk_blocks);

    return UNITY_END();
}
