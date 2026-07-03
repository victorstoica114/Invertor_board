#include "protocols/jkbms_modbus/jkbms_modbus_alerts.h"

#include <stdio.h>
#include <string.h>

typedef struct {
    uint32_t mask;
    const char *name;
} jkbmsAlertFlag_t;

/*
 * Ji Kong BMS RS485 Modbus Universal Protocol V1.1 lists runtime register
 * 0x12A0 as an alarm/fault bitfield. On the live JK Modbus profile we also
 * see stable non-zero values here while the BMS display reports no alarms, so
 * callers must validate a raw candidate before publishing it as a real alarm.
 */
static const jkbmsAlertFlag_t kJkbmsAlarmFlags[] = {
    {1u << 0, "Balance wire resistance fault"},
    {1u << 1, "MOS overtemperature protection"},
    {1u << 2, "Cell count mismatch"},
    {1u << 3, "Current sensor fault"},
    {1u << 4, "Cell overvoltage protection"},
    {1u << 5, "Pack overvoltage protection"},
    {1u << 6, "Charge overcurrent protection"},
    {1u << 7, "Charge short-circuit protection"},
    {1u << 8, "Charge overtemperature protection"},
    {1u << 9, "Charge undertemperature protection"},
    {1u << 10, "Internal communication fault"},
    {1u << 11, "Cell undervoltage protection"},
    {1u << 12, "Pack undervoltage protection"},
    {1u << 13, "Discharge overcurrent protection"},
    {1u << 14, "Discharge short-circuit protection"},
    {1u << 15, "Discharge overtemperature protection"},
    {1u << 16, "Charge MOS fault"},
    {1u << 17, "Discharge MOS fault"},
    {1u << 18, "GPS disconnected"},
    {1u << 19, "Authorization password warning"},
    {1u << 20, "Discharge enable failed"},
    {1u << 21, "Battery overtemperature alarm"},
    {1u << 22, "Temperature sensor anomaly"},
    {1u << 23, "Parallel module anomaly"},
};

static uint32_t jkbmsKnownAlarmMask(void)
{
    uint32_t known = 0u;

    for (size_t i = 0u; i < (sizeof(kJkbmsAlarmFlags) / sizeof(kJkbmsAlarmFlags[0])); i++) {
        known |= kJkbmsAlarmFlags[i].mask;
    }

    return known;
}

static uint8_t countSetBits32(uint32_t bits)
{
    uint8_t count = 0u;

    while (bits != 0u) {
        count = (uint8_t)(count + (uint8_t)(bits & 1u));
        bits >>= 1;
    }

    return count;
}

static uint32_t reverseBytes32(uint32_t v)
{
    return ((v & 0x000000FFu) << 24) |
           ((v & 0x0000FF00u) << 8) |
           ((v & 0x00FF0000u) >> 8) |
           ((v & 0xFF000000u) >> 24);
}

uint32_t jkbmsModbusNormalizeAlarmBits(uint32_t alarmBits)
{
    const uint32_t known = jkbmsKnownAlarmMask();
    const uint32_t reversed = reverseBytes32(alarmBits);
    const uint8_t unknownRaw = countSetBits32(alarmBits & ~known);
    const uint8_t unknownReversed = countSetBits32(reversed & ~known);

    /*
     * The V1.1 alarm table defines bits 0..23 only. Some live Modbus replies
     * expose the same UINT32 in little-endian byte order, which otherwise turns
     * documented low bits into apparent unknown bits 24..31.
     */
    if (alarmBits != 0u && unknownReversed < unknownRaw) {
        return reversed;
    }

    return alarmBits;
}

bool jkbmsModbusAlarmBitsAreValidated(uint32_t alarmBits)
{
    /*
     * Zero is a useful confirmed state. Non-zero candidates need a capture
     * correlated with the JK display/app before we propagate them to the UI or
     * inverter-facing battery model. This prevents status/runtime words around
     * 0x12A0 from becoming false fault flags.
     */
    return alarmBits == 0u;
}

static void appendListItem(char *out, size_t outSize, const char *item)
{
    size_t used = 0u;
    int w = 0;

    if (out == NULL || outSize == 0u || item == NULL || item[0] == '\0') {
        return;
    }

    used = strlen(out);
    if (used >= (outSize - 1u)) {
        return;
    }

    w = snprintf(out + used, outSize - used, "%s%s", (used == 0u) ? "" : ", ", item);
    if (w <= 0) {
        return;
    }
    if ((size_t)w >= (outSize - used)) {
        out[outSize - 1u] = '\0';
    }
}

static void formatJkbmsAlertBits(uint32_t bits, char *out, size_t outSize)
{
    uint32_t known = 0u;

    if (out == NULL || outSize == 0u) {
        return;
    }

    out[0] = '\0';
    if (bits == 0u) {
        return;
    }

    for (size_t i = 0u; i < (sizeof(kJkbmsAlarmFlags) / sizeof(kJkbmsAlarmFlags[0])); i++) {
        known |= kJkbmsAlarmFlags[i].mask;
        if ((bits & kJkbmsAlarmFlags[i].mask) != 0u) {
            appendListItem(out, outSize, kJkbmsAlarmFlags[i].name);
        }
    }

    uint32_t unknown = bits & ~known;
    for (uint8_t i = 0u; i < 32u; i++) {
        uint32_t mask = 1u << i;
        if ((unknown & mask) != 0u) {
            char item[24];
            snprintf(item, sizeof(item), "Unknown bit %u", (unsigned)i);
            appendListItem(out, outSize, item);
        }
    }
}

void jkbmsModbusFormatAlertFields(uint32_t alarmBits,
                                  char *protectionsOut,
                                  size_t protectionsOutSize,
                                  char *alarmsOut,
                                  size_t alarmsOutSize,
                                  char *warningsOut,
                                  size_t warningsOutSize)
{
    const uint32_t decodedBits = jkbmsModbusNormalizeAlarmBits(alarmBits);

    /*
     * Only call this for alarm candidates that have already been validated.
     * Keep the legacy UI grouping: low word in Protections, high word in
     * Warnings, and the full bitfield in Alarms.
     */
    formatJkbmsAlertBits((decodedBits & 0x0000FFFFu),
                         protectionsOut,
                         protectionsOutSize);
    formatJkbmsAlertBits(decodedBits, alarmsOut, alarmsOutSize);
    formatJkbmsAlertBits((decodedBits & 0xFFFF0000u),
                         warningsOut,
                         warningsOutSize);
}
