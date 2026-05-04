#include "protocols/jkbms_rs485/jkbms_rs485_native.h"

#include <stdio.h>
#include <string.h>

#define JK_NATIVE_START_0 0x4Eu
#define JK_NATIVE_START_1 0x57u
#define JK_NATIVE_COMMAND_READ_ALL 0x06u
#define JK_NATIVE_END_ID 0x68u

#define JK_NATIVE_ID_CELL_VOLTAGES 0x79u
#define JK_NATIVE_ID_TUBE_TEMPERATURE 0x80u
#define JK_NATIVE_ID_BOX_TEMPERATURE 0x81u
#define JK_NATIVE_ID_BATTERY_TEMPERATURE 0x82u
#define JK_NATIVE_ID_TOTAL_VOLTAGE 0x83u
#define JK_NATIVE_ID_TOTAL_CURRENT 0x84u
#define JK_NATIVE_ID_SOC 0x85u
#define JK_NATIVE_ID_TEMP_SENSOR_COUNT 0x86u
#define JK_NATIVE_ID_CYCLE_COUNT 0x87u
#define JK_NATIVE_ID_STRING_COUNT 0x8Au
#define JK_NATIVE_ID_ALARMS 0x8Bu
#define JK_NATIVE_ID_STATUS 0x8Cu
#define JK_NATIVE_ID_RATED_CAPACITY 0xAAu

typedef struct {
    uint16_t mask;
    const char *name;
} jkNativeAlertName_t;

static const jkNativeAlertName_t kWarningNames[] = {
    {0x0001u, "SOC low"},
};

static const jkNativeAlertName_t kProtectionNames[] = {
    {0x0002u, "Module temperature high"},
    {0x0004u, "Charge voltage high"},
    {0x0008u, "Discharge voltage low"},
    {0x0010u, "Pack temperature high"},
    {0x0020u, "Charge current high"},
    {0x0040u, "Discharge current high"},
    {0x0080u, "Cell voltage delta high"},
    {0x0100u, "Enclosure temperature high"},
    {0x0200u, "Pack temperature low"},
    {0x0400u, "Pack voltage high"},
    {0x0800u, "Pack voltage low"},
};

static const jkNativeAlertName_t kAlarmNames[] = {
    {0x0001u, "SOC low"},
    {0x0002u, "Module temperature high"},
    {0x0004u, "Charge voltage high"},
    {0x0008u, "Discharge voltage low"},
    {0x0010u, "Pack temperature high"},
    {0x0020u, "Charge current high"},
    {0x0040u, "Discharge current high"},
    {0x0080u, "Cell voltage delta high"},
    {0x0100u, "Enclosure temperature high"},
    {0x0200u, "Pack temperature low"},
    {0x0400u, "Pack voltage high"},
    {0x0800u, "Pack voltage low"},
    {0x1000u, "Other fault 1"},
    {0x2000u, "Other fault 2"},
};

static uint16_t be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static uint32_t be32(const uint8_t *p)
{
    return (((uint32_t)p[0]) << 24) |
           (((uint32_t)p[1]) << 16) |
           (((uint32_t)p[2]) << 8) |
           (uint32_t)p[3];
}

static int16_t decodeTemperatureC(uint16_t raw)
{
    if (raw > 100u) {
        return (int16_t)(-(int32_t)(raw - 100u));
    }
    return (int16_t)raw;
}

static void appendListItem(char *out, size_t outSize, const char *item)
{
    size_t used = 0u;
    int written = 0;

    if (out == NULL || outSize == 0u || item == NULL || item[0] == '\0') {
        return;
    }

    used = strlen(out);
    if (used >= outSize - 1u) {
        return;
    }

    written = snprintf(out + used, outSize - used, "%s%s", (used == 0u) ? "" : ", ", item);
    if (written <= 0) {
        return;
    }
    if ((size_t)written >= outSize - used) {
        out[outSize - 1u] = '\0';
    }
}

static void formatAlertList(uint16_t bits,
                            const jkNativeAlertName_t *map,
                            size_t mapCount,
                            bool appendUnknown,
                            char *out,
                            size_t outSize)
{
    uint16_t known = 0u;

    if (out == NULL || outSize == 0u) {
        return;
    }

    out[0] = '\0';
    if (bits == 0u) {
        return;
    }

    for (size_t i = 0u; i < mapCount; i++) {
        known = (uint16_t)(known | map[i].mask);
        if ((bits & map[i].mask) != 0u) {
            appendListItem(out, outSize, map[i].name);
        }
    }

    if (!appendUnknown) {
        return;
    }

    const uint16_t unknown = (uint16_t)(bits & (uint16_t)(~known));
    for (uint8_t i = 0u; i < 16u; i++) {
        const uint16_t mask = (uint16_t)(1u << i);
        if ((unknown & mask) != 0u) {
            char item[24];
            snprintf(item, sizeof(item), "Unknown bit %u", (unsigned)i);
            appendListItem(out, outSize, item);
        }
    }
}

static int dataLengthForId(uint8_t id)
{
    switch (id) {
        case JK_NATIVE_ID_CELL_VOLTAGES:
            return -1;
        case JK_NATIVE_ID_TUBE_TEMPERATURE:
        case JK_NATIVE_ID_BOX_TEMPERATURE:
        case JK_NATIVE_ID_BATTERY_TEMPERATURE:
        case JK_NATIVE_ID_TOTAL_VOLTAGE:
        case JK_NATIVE_ID_TOTAL_CURRENT:
        case JK_NATIVE_ID_CYCLE_COUNT:
        case JK_NATIVE_ID_STRING_COUNT:
        case JK_NATIVE_ID_ALARMS:
        case JK_NATIVE_ID_STATUS:
        case 0x8Eu:
        case 0x8Fu:
        case 0x90u:
        case 0x91u:
        case 0x92u:
        case 0x93u:
        case 0x94u:
        case 0x95u:
        case 0x96u:
        case 0x97u:
        case 0x98u:
        case 0x99u:
        case 0x9Au:
        case 0x9Bu:
        case 0x9Cu:
        case 0x9Eu:
        case 0x9Fu:
        case 0xA0u:
        case 0xA1u:
        case 0xA2u:
        case 0xA3u:
        case 0xA4u:
        case 0xA5u:
        case 0xA6u:
        case 0xA7u:
        case 0xA8u:
        case 0xADu:
        case 0xB0u:
            return 2;
        case JK_NATIVE_ID_SOC:
        case JK_NATIVE_ID_TEMP_SENSOR_COUNT:
        case 0x9Du:
        case 0xA9u:
        case 0xABu:
        case 0xACu:
        case 0xAEu:
        case 0xAFu:
        case 0xB1u:
        case 0xB3u:
        case 0xB8u:
            return 1;
        case 0x89u:
        case JK_NATIVE_ID_RATED_CAPACITY:
        case 0xB5u:
        case 0xB6u:
        case 0xB9u:
            return 4;
        case 0xB2u:
            return 10;
        case 0xB4u:
            return 8;
        case 0xB7u:
            return 15;
        case 0xBAu:
            return 24;
        case 0xC0u:
            return 5;
        default:
            return -2;
    }
}

static void recomputeCellStats(jkbms_rs485_native_snapshot_t *snapshot)
{
    uint32_t sum = 0u;
    uint8_t counted = 0u;
    uint16_t minMv = 0xFFFFu;
    uint16_t maxMv = 0u;
    uint8_t minIdx = 0u;
    uint8_t maxIdx = 0u;

    if (snapshot == NULL || snapshot->cellCount == 0u) {
        return;
    }

    for (uint8_t i = 0u; i < snapshot->cellCount && i < JKBMS_RS485_NATIVE_MAX_CELLS; i++) {
        const uint16_t mv = snapshot->cellMv[i];
        if (mv == 0u) {
            continue;
        }

        sum += mv;
        counted++;
        if (mv < minMv) {
            minMv = mv;
            minIdx = (uint8_t)(i + 1u);
        }
        if (mv > maxMv) {
            maxMv = mv;
            maxIdx = (uint8_t)(i + 1u);
        }
    }

    if (counted == 0u) {
        return;
    }

    snapshot->hasCellAvgMv = true;
    snapshot->cellAvgMv = (uint16_t)((sum + (uint32_t)(counted / 2u)) / (uint32_t)counted);
    snapshot->hasCellExtremes = true;
    snapshot->minCellMv = minMv;
    snapshot->maxCellMv = maxMv;
    snapshot->minCellIndex = minIdx;
    snapshot->maxCellIndex = maxIdx;
    snapshot->hasCellDiffMaxMv = true;
    snapshot->cellDiffMaxMv = (uint16_t)(maxMv - minMv);
}

static void decodeCellVoltages(const uint8_t *data,
                               size_t dataLen,
                               jkbms_rs485_native_snapshot_t *snapshot)
{
    uint8_t maxCellSeen = 0u;

    if (data == NULL || snapshot == NULL || (dataLen % 3u) != 0u) {
        return;
    }

    for (size_t pos = 0u; pos + 2u < dataLen; pos += 3u) {
        const uint8_t cellNo = data[pos];
        const uint16_t mv = be16(&data[pos + 1u]);
        if (cellNo == 0u || cellNo > JKBMS_RS485_NATIVE_MAX_CELLS || mv == 0u) {
            continue;
        }

        snapshot->cellMv[cellNo - 1u] = mv;
        if (cellNo > maxCellSeen) {
            maxCellSeen = cellNo;
        }
    }

    if (maxCellSeen > snapshot->cellCount) {
        snapshot->cellCount = maxCellSeen;
    }
    recomputeCellStats(snapshot);
}

static void decodeEntry(uint8_t id,
                        const uint8_t *data,
                        size_t dataLen,
                        jkbms_rs485_native_snapshot_t *snapshot)
{
    uint16_t raw16 = 0u;
    uint32_t raw32 = 0u;

    if (data == NULL || snapshot == NULL) {
        return;
    }

    switch (id) {
        case JK_NATIVE_ID_CELL_VOLTAGES:
            decodeCellVoltages(data, dataLen, snapshot);
            break;

        case JK_NATIVE_ID_TUBE_TEMPERATURE:
            if (dataLen >= 2u) {
                snapshot->hasTempMosC = true;
                snapshot->tempMosC = decodeTemperatureC(be16(data));
            }
            break;

        case JK_NATIVE_ID_BOX_TEMPERATURE:
            if (dataLen >= 2u) {
                snapshot->hasTempBat2C = true;
                snapshot->tempBat2C = decodeTemperatureC(be16(data));
            }
            break;

        case JK_NATIVE_ID_BATTERY_TEMPERATURE:
            if (dataLen >= 2u) {
                snapshot->hasTempBat1C = true;
                snapshot->tempBat1C = decodeTemperatureC(be16(data));
            }
            break;

        case JK_NATIVE_ID_TOTAL_VOLTAGE:
            if (dataLen >= 2u) {
                snapshot->hasPackVoltageMv = true;
                snapshot->packVoltageMv = (uint32_t)be16(data) * 10u;
            }
            break;

        case JK_NATIVE_ID_TOTAL_CURRENT:
            if (dataLen >= 2u) {
                raw16 = be16(data);
                const int32_t magnitudeMa = (int32_t)(raw16 & 0x7FFFu) * 10;
                snapshot->hasPackCurrentMa = true;
                snapshot->packCurrentMa = ((raw16 & 0x8000u) != 0u) ? magnitudeMa : -magnitudeMa;
            }
            break;

        case JK_NATIVE_ID_SOC:
            if (dataLen >= 1u) {
                snapshot->hasSoc = true;
                snapshot->socPct = (data[0] > 100u) ? 100u : data[0];
            }
            break;

        case JK_NATIVE_ID_TEMP_SENSOR_COUNT:
            if (dataLen >= 1u) {
                snapshot->hasTempSensorCount = true;
                snapshot->tempSensorCount = data[0];
            }
            break;

        case JK_NATIVE_ID_CYCLE_COUNT:
            if (dataLen >= 2u) {
                snapshot->hasCycles = true;
                snapshot->cycles = be16(data);
            }
            break;

        case JK_NATIVE_ID_STRING_COUNT:
            if (dataLen >= 2u) {
                const uint16_t strings = be16(data);
                if (strings > 0u &&
                    strings <= JKBMS_RS485_NATIVE_MAX_CELLS &&
                    snapshot->cellCount == 0u) {
                    snapshot->cellCount = (uint8_t)strings;
                }
            }
            break;

        case JK_NATIVE_ID_ALARMS:
            if (dataLen >= 2u) {
                snapshot->hasAlarmBits = true;
                snapshot->alarmBits = (uint16_t)data[0] | (uint16_t)((uint16_t)data[1] << 8);
            }
            break;

        case JK_NATIVE_ID_STATUS:
            if (dataLen >= 2u) {
                raw16 = be16(data);
                const uint8_t flags = data[1];
                snapshot->hasStatusFlags = true;
                snapshot->statusFlags = raw16;
                snapshot->chargeEnabled = (flags & 0x01u) != 0u;
                snapshot->dischargeEnabled = (flags & 0x02u) != 0u;
                snapshot->balanceActive = (flags & 0x04u) != 0u;
            }
            break;

        case JK_NATIVE_ID_RATED_CAPACITY:
            if (dataLen >= 4u) {
                raw32 = be32(data);
                if (raw32 <= 100000u) {
                    snapshot->hasFullMah = true;
                    snapshot->fullMah = raw32 * 1000u;
                }
            }
            break;

        default:
            break;
    }
}

size_t jkbmsRs485NativeBuildReadAllRequest(uint8_t *out, size_t outSize)
{
    static const uint8_t prefix[] = {
        0x4Eu, 0x57u, 0x00u, 0x13u, 0x00u, 0x00u, 0x00u, 0x00u,
        0x06u, 0x03u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
        0x68u,
    };
    uint32_t sum = 0u;

    if (out == NULL || outSize < JKBMS_RS485_NATIVE_READ_ALL_REQUEST_LEN) {
        return 0u;
    }

    memset(out, 0, JKBMS_RS485_NATIVE_READ_ALL_REQUEST_LEN);
    memcpy(out, prefix, sizeof(prefix));

    for (size_t i = 0u; i < JKBMS_RS485_NATIVE_READ_ALL_REQUEST_LEN; i++) {
        sum += out[i];
    }

    out[17] = 0x00u;
    out[18] = 0x00u;
    out[19] = (uint8_t)((sum >> 8) & 0xFFu);
    out[20] = (uint8_t)(sum & 0xFFu);

    return JKBMS_RS485_NATIVE_READ_ALL_REQUEST_LEN;
}

bool jkbmsRs485NativeDecodeFrame(const uint8_t *frame,
                                 size_t frameLen,
                                 jkbms_rs485_native_snapshot_t *out)
{
    size_t expectedLen = 0u;
    size_t pos = 11u;
    jkbms_rs485_native_snapshot_t snapshot = {0};

    if (frame == NULL || out == NULL || frameLen < 13u) {
        return false;
    }
    if (frame[0] != JK_NATIVE_START_0 || frame[1] != JK_NATIVE_START_1) {
        return false;
    }
    if (frame[8] != JK_NATIVE_COMMAND_READ_ALL) {
        return false;
    }

    expectedLen = (size_t)be16(&frame[2]) + 2u;
    if (expectedLen < 13u || expectedLen > frameLen) {
        return false;
    }

    while (pos < expectedLen) {
        const uint8_t id = frame[pos++];
        int dataLen = 0;

        if (id == JK_NATIVE_END_ID) {
            break;
        }

        dataLen = dataLengthForId(id);
        if (dataLen == -1) {
            if (pos >= expectedLen) {
                return false;
            }
            dataLen = frame[pos++];
        } else if (dataLen < 0) {
            continue;
        }

        if ((size_t)dataLen > (expectedLen - pos)) {
            return false;
        }

        decodeEntry(id, &frame[pos], (size_t)dataLen, &snapshot);
        pos += (size_t)dataLen;
    }

    if (snapshot.hasPackVoltageMv && snapshot.hasPackCurrentMa) {
        snapshot.hasPackPowerMw = true;
        snapshot.packPowerMw = (int32_t)(((int64_t)snapshot.packVoltageMv *
                                          (int64_t)snapshot.packCurrentMa) /
                                         1000LL);
    }

    snapshot.valid = snapshot.hasSoc ||
                     snapshot.hasTempMosC ||
                     snapshot.hasTempBat1C ||
                     snapshot.hasTempBat2C ||
                     snapshot.hasPackVoltageMv ||
                     snapshot.hasPackCurrentMa ||
                     snapshot.hasFullMah ||
                     snapshot.hasCycles ||
                     snapshot.hasAlarmBits ||
                     snapshot.hasStatusFlags ||
                     (snapshot.hasCellExtremes && snapshot.cellCount > 0u);
    if (!snapshot.valid) {
        return false;
    }

    if (!snapshot.hasSoh && snapshot.hasSoc) {
        snapshot.hasSoh = true;
        snapshot.sohPct = 100u;
    }

    *out = snapshot;
    return true;
}

void jkbmsRs485NativeFormatAlertFields(uint16_t alarmBits,
                                       char *protections,
                                       size_t protectionsSize,
                                       char *alarms,
                                       size_t alarmsSize,
                                       char *warnings,
                                       size_t warningsSize)
{
    const uint16_t warningBits = (uint16_t)(alarmBits & 0x0001u);
    const uint16_t protectionBits = (uint16_t)(alarmBits & 0x0FFEu);

    formatAlertList(protectionBits,
                    kProtectionNames,
                    sizeof(kProtectionNames) / sizeof(kProtectionNames[0]),
                    false,
                    protections,
                    protectionsSize);
    formatAlertList(alarmBits,
                    kAlarmNames,
                    sizeof(kAlarmNames) / sizeof(kAlarmNames[0]),
                    true,
                    alarms,
                    alarmsSize);
    formatAlertList(warningBits,
                    kWarningNames,
                    sizeof(kWarningNames) / sizeof(kWarningNames[0]),
                    false,
                    warnings,
                    warningsSize);
}
