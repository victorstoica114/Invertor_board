#include "unity.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "config.h"
#include "protocols/common/battery_model.h"
#include "protocols/pylon/pylon_rs485_bridge.h"
#include "runtime_settings.h"

extern bridge_runtime_settings_t g_pylonBridgeTestSettings;
extern battery_model_t g_pylonBridgeTestModel;
extern bool g_pylonBridgeTestModelValid;
void pylonBridgeStubReset(void);

void setUp(void)
{
    pylonBridgeStubReset();
}

void tearDown(void)
{
}

static void configureSyntheticRoute(uint8_t bmsProtocol)
{
    g_pylonBridgeTestSettings.mode = MODE_BRIDGE;
    g_pylonBridgeTestSettings.bms_line = LINE_RS485;
    g_pylonBridgeTestSettings.inverter_line = LINE_RS485;
    g_pylonBridgeTestSettings.bms_protocol = bmsProtocol;
    g_pylonBridgeTestSettings.inverter_protocol = PROTOCOL_RS485_PYLON;
    g_pylonBridgeTestSettings.bms_port = 1u;
    g_pylonBridgeTestSettings.inverter_port = 2u;
}

static void configureNativeCanRoute(uint8_t bmsProtocol)
{
    g_pylonBridgeTestSettings.mode = MODE_BRIDGE;
    g_pylonBridgeTestSettings.bms_line = LINE_CAN;
    g_pylonBridgeTestSettings.inverter_line = LINE_RS485;
    g_pylonBridgeTestSettings.bms_protocol = bmsProtocol;
    g_pylonBridgeTestSettings.inverter_protocol = PROTOCOL_RS485_PYLON;
    g_pylonBridgeTestSettings.bms_port = 1u;
    g_pylonBridgeTestSettings.inverter_port = 2u;
}

static void setValidModel(void)
{
    memset(&g_pylonBridgeTestModel, 0, sizeof(g_pylonBridgeTestModel));
    g_pylonBridgeTestModel.valid = true;
    g_pylonBridgeTestModel.updatedMs = 1000u;
    g_pylonBridgeTestModel.packVoltageV = 72.7f;
    g_pylonBridgeTestModel.packCurrentA = 12.3f;
    g_pylonBridgeTestModel.socPct = 87u;
    g_pylonBridgeTestModel.sohPct = 99u;
    g_pylonBridgeTestModel.cycleCount = 321u;
    g_pylonBridgeTestModel.cellMaxV = 4.618f;
    g_pylonBridgeTestModel.cellMinV = 4.476f;
    g_pylonBridgeTestModel.cellMaxIdx = 3u;
    g_pylonBridgeTestModel.cellMinIdx = 12u;
    g_pylonBridgeTestModel.temperaturesC[0] = 29.0f;
    g_pylonBridgeTestModel.temperaturesC[1] = 28.0f;
    g_pylonBridgeTestModel.temperaturesC[2] = 27.5f;
    g_pylonBridgeTestModel.temperaturesC[3] = 27.0f;
    g_pylonBridgeTestModel.temperaturesC[4] = 26.5f;
    g_pylonBridgeTestModelValid = true;
}

static uint8_t hexByteAt(const char *hex, int byteIndex)
{
    uint8_t value = 0u;

    for (int i = 0; i < 2; i++) {
        char c = hex[(byteIndex * 2) + i];
        uint8_t nibble = 0u;
        if (c >= '0' && c <= '9') {
            nibble = (uint8_t)(c - '0');
        } else if (c >= 'A' && c <= 'F') {
            nibble = (uint8_t)(10 + (c - 'A'));
        } else if (c >= 'a' && c <= 'f') {
            nibble = (uint8_t)(10 + (c - 'a'));
        } else {
            TEST_ASSERT_TRUE(false);
        }
        value = (uint8_t)((value << 4) | nibble);
    }

    return value;
}

static uint16_t hexBe16At(const char *hex, int byteIndex)
{
    return (uint16_t)(((uint16_t)hexByteAt(hex, byteIndex) << 8) |
                      (uint16_t)hexByteAt(hex, byteIndex + 1));
}

void test_pylon_synthetic_63_generic_sources_default_to_c0_and_ignore_balance(void)
{
    char info63[32] = {0};

    configureSyntheticRoute(PROTOCOL_RS485_WOW);
    setValidModel();
    g_pylonBridgeTestModel.balanceEnabled = true;

    TEST_ASSERT_TRUE(pylonRs485BridgeBuildSyntheticInfo63ForTest(info63, sizeof(info63)));
    TEST_ASSERT_EQUAL_STRING("05E0B1800000076CC0", info63);
    TEST_ASSERT_EQUAL_UINT8(0xC0u, hexByteAt(info63, 8));
}

void test_pylon_synthetic_63_uses_explicit_generic_charge_discharge_flags(void)
{
    char info63[32] = {0};

    configureSyntheticRoute(PROTOCOL_RS485_PACE);
    setValidModel();
    g_pylonBridgeTestModel.dischargeEnabled = true;

    TEST_ASSERT_TRUE(pylonRs485BridgeBuildSyntheticInfo63ForTest(info63, sizeof(info63)));
    TEST_ASSERT_EQUAL_UINT8(0x40u, hexByteAt(info63, 8));

    g_pylonBridgeTestModel.chargeEnabled = true;
    TEST_ASSERT_TRUE(pylonRs485BridgeBuildSyntheticInfo63ForTest(info63, sizeof(info63)));
    TEST_ASSERT_EQUAL_UINT8(0xC0u, hexByteAt(info63, 8));
}

void test_pylon_synthetic_63_native_sources_preserve_protocol_status_byte(void)
{
    char info63[32] = {0};

    configureNativeCanRoute(PROTOCOL_CAN_DEYE);
    setValidModel();
    g_pylonBridgeTestModel.protocolState = 0xA5u;

    TEST_ASSERT_TRUE(pylonRs485BridgeBuildSyntheticInfo63ForTest(info63, sizeof(info63)));
    TEST_ASSERT_EQUAL_UINT8(0xA5u, hexByteAt(info63, 8));
}

void test_pylon_synthetic_61_generic_sources_project_percentages_only(void)
{
    char info61[128] = {0};

    configureSyntheticRoute(PROTOCOL_RS485_WOW);
    setValidModel();

    TEST_ASSERT_TRUE(pylonRs485BridgeBuildSyntheticInfo61ForTest(info61, sizeof(info61)));
    TEST_ASSERT_EQUAL_UINT8(87u, hexByteAt(info61, 4));
    TEST_ASSERT_EQUAL_UINT8(99u, hexByteAt(info61, 9));
    TEST_ASSERT_EQUAL_UINT16(0x1265u, hexBe16At(info61, 11));
    TEST_ASSERT_EQUAL_UINT16(0x11D4u, hexBe16At(info61, 15));
}

void test_pylon_synthetic_payloads_reject_missing_model(void)
{
    char info61[128] = {0};
    char info63[32] = {0};

    configureSyntheticRoute(PROTOCOL_RS485_WOW);

    TEST_ASSERT_FALSE(pylonRs485BridgeBuildSyntheticInfo61ForTest(info61, sizeof(info61)));
    TEST_ASSERT_FALSE(pylonRs485BridgeBuildSyntheticInfo63ForTest(info63, sizeof(info63)));
}

void test_pylon_route_supports_115200_variants(void)
{
    bridge_runtime_settings_t settings = {0};

    settings.mode = MODE_BRIDGE;
    settings.bms_line = LINE_RS485;
    settings.bms_protocol = PROTOCOL_RS485_PYLON_115200;
    settings.bms_port = 1u;
    settings.inverter_line = LINE_RS485;
    settings.inverter_protocol = PROTOCOL_RS485_PYLON;
    settings.inverter_port = 2u;

    TEST_ASSERT_TRUE(pylonRs485BridgeSupportsRoute(&settings));

    settings.bms_line = LINE_CAN;
    settings.bms_protocol = PROTOCOL_CAN_PYLON;
    settings.inverter_protocol = PROTOCOL_RS485_PYLON_115200;

    TEST_ASSERT_TRUE(pylonRs485BridgeSupportsRoute(&settings));
}

void test_pylon_probe_in_bridge_mode_ignores_recent_inverter_traffic(void)
{
    const int64_t nowUs = 10000000LL;

    TEST_ASSERT_FALSE(pylonRs485BridgeProbeShouldWaitForQuietForTest(MODE_BRIDGE,
                                                                     nowUs,
                                                                     0,
                                                                     nowUs - 100000LL));
    TEST_ASSERT_TRUE(pylonRs485BridgeProbeShouldWaitForQuietForTest(MODE_BRIDGE,
                                                                    nowUs,
                                                                    nowUs - 100000LL,
                                                                    0));
}

void test_pylon_probe_in_forward_mode_waits_for_recent_inverter_traffic(void)
{
    const int64_t nowUs = 10000000LL;

    TEST_ASSERT_TRUE(pylonRs485BridgeProbeShouldWaitForQuietForTest(MODE_FORWARD,
                                                                    nowUs,
                                                                    0,
                                                                    nowUs - 100000LL));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_pylon_synthetic_63_generic_sources_default_to_c0_and_ignore_balance);
    RUN_TEST(test_pylon_synthetic_63_uses_explicit_generic_charge_discharge_flags);
    RUN_TEST(test_pylon_synthetic_63_native_sources_preserve_protocol_status_byte);
    RUN_TEST(test_pylon_synthetic_61_generic_sources_project_percentages_only);
    RUN_TEST(test_pylon_synthetic_payloads_reject_missing_model);
    RUN_TEST(test_pylon_route_supports_115200_variants);
    RUN_TEST(test_pylon_probe_in_bridge_mode_ignores_recent_inverter_traffic);
    RUN_TEST(test_pylon_probe_in_forward_mode_waits_for_recent_inverter_traffic);
    return UNITY_END();
}
