#include "unity.h"

#include <string.h>

#include "protocols/jkbms_modbus/jkbms_modbus_alerts.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_jkbms_alerts_decode_live_alarm_raw_sample(void)
{
    char protections[512];
    char alarms[512];
    char warnings[512];

    TEST_ASSERT_EQUAL_UINT32(0x00644523u, jkbmsModbusNormalizeAlarmBits(0x23456400u));

    jkbmsModbusFormatAlertFields(0x23456400u,
                                 protections,
                                 sizeof(protections),
                                 alarms,
                                 sizeof(alarms),
                                 warnings,
                                 sizeof(warnings));

    TEST_ASSERT_EQUAL_STRING("Balance wire resistance fault, MOS overtemperature protection, Pack overvoltage protection, Charge overtemperature protection, Internal communication fault, Discharge short-circuit protection",
                             protections);
    TEST_ASSERT_TRUE(strstr(alarms, "Balance wire resistance fault") != NULL);
    TEST_ASSERT_TRUE(strstr(alarms, "Pack overvoltage protection") != NULL);
    TEST_ASSERT_TRUE(strstr(alarms, "Charge overtemperature protection") != NULL);
    TEST_ASSERT_TRUE(strstr(alarms, "GPS disconnected") != NULL);
    TEST_ASSERT_TRUE(strstr(alarms, "Battery overtemperature alarm") != NULL);
    TEST_ASSERT_TRUE(strstr(alarms, "Temperature sensor anomaly") != NULL);
    TEST_ASSERT_TRUE(strstr(alarms, "Unknown") == NULL);
    TEST_ASSERT_TRUE(strstr(warnings, "GPS disconnected") != NULL);
    TEST_ASSERT_TRUE(strstr(warnings, "Battery overtemperature alarm") != NULL);
    TEST_ASSERT_TRUE(strstr(warnings, "Temperature sensor anomaly") != NULL);
    TEST_ASSERT_TRUE(strstr(warnings, "Unknown") == NULL);
}

void test_jkbms_alerts_keep_documented_bit_order(void)
{
    TEST_ASSERT_EQUAL_UINT32(0x00006400u, jkbmsModbusNormalizeAlarmBits(0x00006400u));
}

void test_jkbms_alerts_zero_bits_clear_outputs(void)
{
    char protections[32] = "old";
    char alarms[32] = "old";
    char warnings[32] = "old";

    jkbmsModbusFormatAlertFields(0u,
                                 protections,
                                 sizeof(protections),
                                 alarms,
                                 sizeof(alarms),
                                 warnings,
                                 sizeof(warnings));

    TEST_ASSERT_EQUAL_STRING("", protections);
    TEST_ASSERT_EQUAL_STRING("", alarms);
    TEST_ASSERT_EQUAL_STRING("", warnings);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_jkbms_alerts_decode_live_alarm_raw_sample);
    RUN_TEST(test_jkbms_alerts_keep_documented_bit_order);
    RUN_TEST(test_jkbms_alerts_zero_bits_clear_outputs);
    return UNITY_END();
}
