#include "unity.h"

#include <string.h>

#include "protocols/seplos_rs485/seplos_rs485_protocol.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_seplos_builds_default_telemetry_and_alarm_requests(void)
{
    uint8_t frame[32] = {0};
    size_t len = seplosRs485BuildRequest(SEPLOS_RS485_CID2_TELEMETRY,
                                         0x00u,
                                         SEPLOS_RS485_PROTOCOL_VERSION,
                                         frame,
                                         sizeof(frame));

    TEST_ASSERT_EQUAL_UINT32(strlen("~20004642E00200FD37\r"), len);
    TEST_ASSERT_EQUAL_STRING("~20004642E00200FD37\r", (const char *)frame);

    memset(frame, 0, sizeof(frame));
    len = seplosRs485BuildRequest(SEPLOS_RS485_CID2_ALARMS,
                                  0x00u,
                                  SEPLOS_RS485_PROTOCOL_VERSION,
                                  frame,
                                  sizeof(frame));

    TEST_ASSERT_EQUAL_UINT32(strlen("~20004644E00200FD35\r"), len);
    TEST_ASSERT_EQUAL_STRING("~20004644E00200FD35\r", (const char *)frame);
}

void test_seplos_builds_pack_one_len_checked_request(void)
{
    uint8_t frame[32] = {0};
    size_t len = seplosRs485BuildRequestWithStyle(SEPLOS_RS485_CID2_TELEMETRY,
                                                  0x00u,
                                                  0x01u,
                                                  SEPLOS_RS485_PROTOCOL_VERSION,
                                                  SEPLOS_RS485_REQUEST_STYLE_LEN_CHECK,
                                                  frame,
                                                  sizeof(frame));

    TEST_ASSERT_EQUAL_UINT32(strlen("~20004642E00201FD36\r"), len);
    TEST_ASSERT_EQUAL_STRING("~20004642E00201FD36\r", (const char *)frame);
}

void test_seplos_builds_vendor_style_simple_length_requests(void)
{
    uint8_t frame[32] = {0};
    size_t len = seplosRs485BuildRequestWithStyle(SEPLOS_RS485_CID2_TELEMETRY,
                                                  0x01u,
                                                  0x01u,
                                                  SEPLOS_RS485_PROTOCOL_VERSION,
                                                  SEPLOS_RS485_REQUEST_STYLE_SIMPLE_LEN_NIBBLE,
                                                  frame,
                                                  sizeof(frame));

    TEST_ASSERT_EQUAL_UINT32(strlen("~2001464200021FD7A\r"), len);
    TEST_ASSERT_EQUAL_STRING("~2001464200021FD7A\r", (const char *)frame);

    memset(frame, 0, sizeof(frame));
    len = seplosRs485BuildRequestWithStyle(SEPLOS_RS485_CID2_ALARMS,
                                           0x01u,
                                           0x01u,
                                           SEPLOS_RS485_PROTOCOL_VERSION,
                                           SEPLOS_RS485_REQUEST_STYLE_SIMPLE_LEN_NIBBLE,
                                           frame,
                                           sizeof(frame));

    TEST_ASSERT_EQUAL_UINT32(strlen("~2001464400021FD78\r"), len);
    TEST_ASSERT_EQUAL_STRING("~2001464400021FD78\r", (const char *)frame);
}

void test_seplos_accepts_simple_length_response_field(void)
{
    static const uint8_t raw[] = "~20004600000200FD52\r";
    seplos_rs485_frame_t frame = {0};

    TEST_ASSERT_TRUE(seplosRs485DecodeFrame(raw, strlen((const char *)raw), &frame));
    TEST_ASSERT_EQUAL_UINT16(0x0002u, frame.lengthField);
    TEST_ASSERT_EQUAL_UINT32(1u, frame.infoLen);
    TEST_ASSERT_EQUAL_UINT8(0x00u, frame.info[0]);
}

void test_seplos_decodes_telemetry_sample(void)
{
    static const uint8_t raw[] =
        "~2000460010960001100CD70CE90CF40CD60CEF0CE50CE10CDC0CE90CF00CE80CEF0CEA0CDA0CDE0CD8060BA60BA00B970BA60BA50BA2FD5C14A0344E0A426803134650004603E8149F0000000000000000DC6C\r";
    seplos_rs485_frame_t frame = {0};
    seplos_rs485_snapshot_t snapshot = {0};

    TEST_ASSERT_TRUE(seplosRs485DecodeFrame(raw, strlen((const char *)raw), &frame));
    TEST_ASSERT_EQUAL_UINT8(0x20u, frame.version);
    TEST_ASSERT_EQUAL_UINT8(0x00u, frame.address);
    TEST_ASSERT_EQUAL_UINT8(SEPLOS_RS485_CID1_BMS, frame.cid1);
    TEST_ASSERT_EQUAL_UINT8(0x00u, frame.cid2);
    TEST_ASSERT_EQUAL_UINT32(75u, frame.infoLen);

    TEST_ASSERT_TRUE(seplosRs485DecodeTelemetryInfo(frame.info, frame.infoLen, &snapshot));
    TEST_ASSERT_TRUE(snapshot.valid);
    TEST_ASSERT_TRUE(snapshot.hasTelemetry);
    TEST_ASSERT_EQUAL_UINT8(16u, snapshot.cellCount);
    TEST_ASSERT_EQUAL_UINT16(3287u, snapshot.cellMv[0]);
    TEST_ASSERT_EQUAL_UINT16(3316u, snapshot.cellMv[2]);
    TEST_ASSERT_EQUAL_UINT16(3288u, snapshot.cellMv[15]);
    TEST_ASSERT_TRUE(snapshot.hasCellExtremes);
    TEST_ASSERT_EQUAL_UINT16(3316u, snapshot.maxCellMv);
    TEST_ASSERT_EQUAL_UINT8(3u, snapshot.maxCellIndex);
    TEST_ASSERT_EQUAL_UINT16(3286u, snapshot.minCellMv);
    TEST_ASSERT_EQUAL_UINT8(4u, snapshot.minCellIndex);
    TEST_ASSERT_TRUE(snapshot.hasCellAvgMv);
    TEST_ASSERT_EQUAL_UINT16(3300u, snapshot.cellAvgMv);
    TEST_ASSERT_TRUE(snapshot.hasCellDiffMv);
    TEST_ASSERT_EQUAL_UINT16(30u, snapshot.cellDiffMv);

    TEST_ASSERT_EQUAL_UINT8(6u, snapshot.tempCount);
    TEST_ASSERT_EQUAL_INT16(298, snapshot.tempDeciC[0]);
    TEST_ASSERT_EQUAL_INT16(297, snapshot.tempDeciC[2]);
    TEST_ASSERT_EQUAL_INT16(298, snapshot.tempDeciC[5]);

    TEST_ASSERT_TRUE(snapshot.hasPackCurrentCa);
    TEST_ASSERT_EQUAL_INT16(-676, snapshot.packCurrentCa);
    TEST_ASSERT_TRUE(snapshot.hasPackVoltageCv);
    TEST_ASSERT_EQUAL_UINT16(5280u, snapshot.packVoltageCv);
    TEST_ASSERT_TRUE(snapshot.hasRemainingCapacityCah);
    TEST_ASSERT_EQUAL_UINT16(13390u, snapshot.remainingCapacityCah);
    TEST_ASSERT_TRUE(snapshot.hasFullCapacityCah);
    TEST_ASSERT_EQUAL_UINT16(17000u, snapshot.fullCapacityCah);
    TEST_ASSERT_TRUE(snapshot.hasSocDeciPct);
    TEST_ASSERT_EQUAL_UINT16(787u, snapshot.socDeciPct);
    TEST_ASSERT_TRUE(snapshot.hasRatedCapacityCah);
    TEST_ASSERT_EQUAL_UINT16(18000u, snapshot.ratedCapacityCah);
    TEST_ASSERT_TRUE(snapshot.hasCycles);
    TEST_ASSERT_EQUAL_UINT16(70u, snapshot.cycles);
    TEST_ASSERT_TRUE(snapshot.hasSohDeciPct);
    TEST_ASSERT_EQUAL_UINT16(1000u, snapshot.sohDeciPct);
    TEST_ASSERT_TRUE(snapshot.hasPortVoltageCv);
    TEST_ASSERT_EQUAL_UINT16(5279u, snapshot.portVoltageCv);
}

void test_seplos_decodes_alarm_sample_and_formats_raw_flags(void)
{
    static const uint8_t raw[] =
        "~20004600A06000010F000000000000000000000000000000060000000000000000140000000000000300000200000000000000000002EB74\r";
    seplos_rs485_frame_t frame = {0};
    seplos_rs485_snapshot_t snapshot = {0};
    char protections[128] = {0};
    char alarms[128] = {0};
    char warnings[128] = {0};

    TEST_ASSERT_TRUE(seplosRs485DecodeFrame(raw, strlen((const char *)raw), &frame));
    TEST_ASSERT_EQUAL_UINT32(48u, frame.infoLen);
    TEST_ASSERT_TRUE(seplosRs485DecodeAlarmInfo(frame.info, frame.infoLen, &snapshot));

    TEST_ASSERT_TRUE(snapshot.valid);
    TEST_ASSERT_TRUE(snapshot.hasAlarms);
    TEST_ASSERT_EQUAL_UINT8(15u, frame.info[2]);
    TEST_ASSERT_EQUAL_UINT8(0x03u, snapshot.powerStatus);
    TEST_ASSERT_EQUAL_UINT8(0x02u, snapshot.systemStatus);
    TEST_ASSERT_FALSE(snapshot.dischargeEnabled);
    TEST_ASSERT_TRUE(snapshot.chargeEnabled);
    TEST_ASSERT_FALSE(snapshot.sleepMode);

    seplosRs485FormatAlertFields(&snapshot,
                                 protections,
                                 sizeof(protections),
                                 alarms,
                                 sizeof(alarms),
                                 warnings,
                                 sizeof(warnings));

    TEST_ASSERT_TRUE(strstr(protections, "Custom=0x14") != NULL);
    TEST_ASSERT_EQUAL_STRING("", alarms);
    TEST_ASSERT_EQUAL_STRING("", warnings);
}

void test_seplos_rejects_bad_checksum(void)
{
    static const uint8_t raw[] = "~20004642E00200FD38\r";
    seplos_rs485_frame_t frame = {0};

    TEST_ASSERT_FALSE(seplosRs485DecodeFrame(raw, strlen((const char *)raw), &frame));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_seplos_builds_default_telemetry_and_alarm_requests);
    RUN_TEST(test_seplos_builds_pack_one_len_checked_request);
    RUN_TEST(test_seplos_builds_vendor_style_simple_length_requests);
    RUN_TEST(test_seplos_accepts_simple_length_response_field);
    RUN_TEST(test_seplos_decodes_telemetry_sample);
    RUN_TEST(test_seplos_decodes_alarm_sample_and_formats_raw_flags);
    RUN_TEST(test_seplos_rejects_bad_checksum);

    return UNITY_END();
}
