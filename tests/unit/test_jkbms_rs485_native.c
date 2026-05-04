#include "unity.h"

#include <string.h>

#include "protocols/jkbms_rs485/jkbms_rs485_native.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_jkbms_native_builds_read_all_request(void)
{
    uint8_t frame[JKBMS_RS485_NATIVE_READ_ALL_REQUEST_LEN] = {0};
    const uint8_t expected[JKBMS_RS485_NATIVE_READ_ALL_REQUEST_LEN] = {
        0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00,
        0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x68, 0x00, 0x00, 0x01, 0x29,
    };

    size_t len = jkbmsRs485NativeBuildReadAllRequest(frame, sizeof(frame));

    TEST_ASSERT_EQUAL_UINT32(JKBMS_RS485_NATIVE_READ_ALL_REQUEST_LEN, len);
    for (size_t i = 0u; i < sizeof(expected); i++) {
        TEST_ASSERT_EQUAL_UINT8(expected[i], frame[i]);
    }
}

void test_jkbms_native_decodes_summary_cells_temps_and_flags(void)
{
    const uint8_t frame[] = {
        0x4E, 0x57, 0x00, 0x3A, 0x00, 0x00, 0x00, 0x00,
        0x06, 0x00, 0x01,
        0x79, 0x06, 0x01, 0x11, 0xB0, 0x02, 0x11, 0xC0,
        0x80, 0x00, 0x1D,
        0x81, 0x00, 0x1C,
        0x82, 0x00, 0x1E,
        0x83, 0x1C, 0x6C,
        0x84, 0x80, 0x0A,
        0x85, 0x64,
        0x86, 0x03,
        0x87, 0x03, 0xBD,
        0x8A, 0x00, 0x02,
        0x8B, 0x24, 0x04,
        0x8C, 0x00, 0x03,
        0xAA, 0x00, 0x00, 0x00, 0x28,
        0x68, 0x00, 0x00, 0x00, 0x00,
    };
    jkbms_rs485_native_snapshot_t snapshot = {0};

    TEST_ASSERT_TRUE(jkbmsRs485NativeDecodeFrame(frame, sizeof(frame), &snapshot));

    TEST_ASSERT_TRUE(snapshot.valid);
    TEST_ASSERT_TRUE(snapshot.hasPackVoltageMv);
    TEST_ASSERT_EQUAL_UINT32(72760u, snapshot.packVoltageMv);
    TEST_ASSERT_TRUE(snapshot.hasPackCurrentMa);
    TEST_ASSERT_EQUAL(100, snapshot.packCurrentMa);
    TEST_ASSERT_TRUE(snapshot.hasPackPowerMw);
    TEST_ASSERT_EQUAL(7276, snapshot.packPowerMw);
    TEST_ASSERT_TRUE(snapshot.hasSoc);
    TEST_ASSERT_EQUAL_UINT8(100u, snapshot.socPct);
    TEST_ASSERT_TRUE(snapshot.hasSoh);
    TEST_ASSERT_EQUAL_UINT8(100u, snapshot.sohPct);
    TEST_ASSERT_TRUE(snapshot.hasCycles);
    TEST_ASSERT_EQUAL_UINT32(957u, snapshot.cycles);
    TEST_ASSERT_TRUE(snapshot.hasFullMah);
    TEST_ASSERT_EQUAL_UINT32(40000u, snapshot.fullMah);

    TEST_ASSERT_TRUE(snapshot.hasTempMosC);
    TEST_ASSERT_EQUAL_INT16(29, snapshot.tempMosC);
    TEST_ASSERT_TRUE(snapshot.hasTempBat1C);
    TEST_ASSERT_EQUAL_INT16(30, snapshot.tempBat1C);
    TEST_ASSERT_TRUE(snapshot.hasTempBat2C);
    TEST_ASSERT_EQUAL_INT16(28, snapshot.tempBat2C);
    TEST_ASSERT_TRUE(snapshot.hasTempSensorCount);
    TEST_ASSERT_EQUAL_UINT8(3u, snapshot.tempSensorCount);

    TEST_ASSERT_EQUAL_UINT8(2u, snapshot.cellCount);
    TEST_ASSERT_EQUAL_UINT16(4528u, snapshot.cellMv[0]);
    TEST_ASSERT_EQUAL_UINT16(4544u, snapshot.cellMv[1]);
    TEST_ASSERT_TRUE(snapshot.hasCellExtremes);
    TEST_ASSERT_EQUAL_UINT16(4528u, snapshot.minCellMv);
    TEST_ASSERT_EQUAL_UINT16(4544u, snapshot.maxCellMv);
    TEST_ASSERT_EQUAL_UINT8(1u, snapshot.minCellIndex);
    TEST_ASSERT_EQUAL_UINT8(2u, snapshot.maxCellIndex);
    TEST_ASSERT_TRUE(snapshot.hasCellAvgMv);
    TEST_ASSERT_EQUAL_UINT16(4536u, snapshot.cellAvgMv);
    TEST_ASSERT_TRUE(snapshot.hasCellDiffMaxMv);
    TEST_ASSERT_EQUAL_UINT16(16u, snapshot.cellDiffMaxMv);

    TEST_ASSERT_TRUE(snapshot.hasAlarmBits);
    TEST_ASSERT_EQUAL_UINT16(0x0424u, snapshot.alarmBits);
    TEST_ASSERT_TRUE(snapshot.hasStatusFlags);
    TEST_ASSERT_EQUAL_UINT16(0x0003u, snapshot.statusFlags);
    TEST_ASSERT_TRUE(snapshot.chargeEnabled);
    TEST_ASSERT_TRUE(snapshot.dischargeEnabled);
    TEST_ASSERT_FALSE(snapshot.balanceActive);
}

void test_jkbms_native_formats_known_and_unknown_alert_bits(void)
{
    char protections[160] = {0};
    char alarms[160] = {0};
    char warnings[160] = {0};

    jkbmsRs485NativeFormatAlertFields(0xC425u,
                                      protections,
                                      sizeof(protections),
                                      alarms,
                                      sizeof(alarms),
                                      warnings,
                                      sizeof(warnings));

    TEST_ASSERT_TRUE(strstr(protections, "Charge voltage high") != NULL);
    TEST_ASSERT_TRUE(strstr(protections, "Charge current high") != NULL);
    TEST_ASSERT_TRUE(strstr(protections, "Pack voltage high") != NULL);
    TEST_ASSERT_TRUE(strstr(alarms, "SOC low") != NULL);
    TEST_ASSERT_TRUE(strstr(alarms, "Unknown bit 14") != NULL);
    TEST_ASSERT_TRUE(strstr(alarms, "Unknown bit 15") != NULL);
    TEST_ASSERT_EQUAL_STRING("SOC low", warnings);
}

void test_jkbms_native_rejects_bad_start_and_incomplete_frame(void)
{
    const uint8_t badStart[] = {0x00, 0x57, 0x00, 0x0B, 0x00, 0x00, 0x00,
                                0x00, 0x06, 0x00, 0x01, 0x68, 0x00};
    const uint8_t incomplete[] = {0x4E, 0x57, 0x00, 0x3A, 0x00, 0x00,
                                  0x00, 0x00, 0x06, 0x00, 0x01, 0x85};
    jkbms_rs485_native_snapshot_t snapshot = {0};

    TEST_ASSERT_FALSE(jkbmsRs485NativeDecodeFrame(badStart, sizeof(badStart), &snapshot));
    TEST_ASSERT_FALSE(jkbmsRs485NativeDecodeFrame(incomplete, sizeof(incomplete), &snapshot));
}

void test_jkbms_native_accepts_vendor_tail_fields_after_b8(void)
{
    const uint8_t frame[] = {
        0x4E, 0x57, 0x00, 0x17, 0x00, 0x00, 0x00, 0x00,
        0x06, 0x00, 0x01,
        0xB8, 0x00,
        0xB9, 0x00, 0x00, 0x01, 0x18,
        0x85, 0x50,
        0x68, 0x00, 0x00, 0x00, 0x00,
    };
    jkbms_rs485_native_snapshot_t snapshot = {0};

    TEST_ASSERT_TRUE(jkbmsRs485NativeDecodeFrame(frame, sizeof(frame), &snapshot));
    TEST_ASSERT_TRUE(snapshot.hasSoc);
    TEST_ASSERT_EQUAL_UINT8(80u, snapshot.socPct);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_jkbms_native_builds_read_all_request);
    RUN_TEST(test_jkbms_native_decodes_summary_cells_temps_and_flags);
    RUN_TEST(test_jkbms_native_formats_known_and_unknown_alert_bits);
    RUN_TEST(test_jkbms_native_rejects_bad_start_and_incomplete_frame);
    RUN_TEST(test_jkbms_native_accepts_vendor_tail_fields_after_b8);

    return UNITY_END();
}
