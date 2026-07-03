#include "unity.h"

#include <string.h>

#include "protocols/jkbms_modbus/jkbms_modbus_alerts.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_jkbms_alerts_keep_live_nonzero_candidate_unvalidated(void)
{
    TEST_ASSERT_FALSE(jkbmsModbusAlarmBitsAreValidated(0x23436400u));
    TEST_ASSERT_FALSE(jkbmsModbusAlarmBitsAreValidated(0x23456400u));
    TEST_ASSERT_TRUE(jkbmsModbusAlarmBitsAreValidated(0u));
}

void test_jkbms_alerts_format_validated_synthetic_bits(void)
{
    char protections[512];
    char alarms[512];
    char warnings[512];

    const uint32_t alarmBits = (1u << 5) | (1u << 8) | (1u << 18);

    jkbmsModbusFormatAlertFields(alarmBits,
                                 protections,
                                 sizeof(protections),
                                 alarms,
                                 sizeof(alarms),
                                 warnings,
                                 sizeof(warnings));

    TEST_ASSERT_EQUAL_STRING("Pack overvoltage protection, Charge overtemperature protection",
                             protections);
    TEST_ASSERT_TRUE(strstr(alarms, "Pack overvoltage protection") != NULL);
    TEST_ASSERT_TRUE(strstr(alarms, "Charge overtemperature protection") != NULL);
    TEST_ASSERT_TRUE(strstr(alarms, "GPS disconnected") != NULL);
    TEST_ASSERT_TRUE(strstr(alarms, "Unknown") == NULL);
    TEST_ASSERT_TRUE(strstr(warnings, "GPS disconnected") != NULL);
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
    RUN_TEST(test_jkbms_alerts_keep_live_nonzero_candidate_unvalidated);
    RUN_TEST(test_jkbms_alerts_format_validated_synthetic_bits);
    RUN_TEST(test_jkbms_alerts_keep_documented_bit_order);
    RUN_TEST(test_jkbms_alerts_zero_bits_clear_outputs);
    return UNITY_END();
}
