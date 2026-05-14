#include "protocols/seplos_rs485/seplos_rs485_protocol.h"

#include <stdio.h>
#include <string.h>

typedef struct {
    uint8_t mask;
    const char *name;
} seplosFlagName_t;

static const seplosFlagName_t kWarning1Names[] = {
    {0x01u, "Pack voltage sensor fault"},
    {0x02u, "Pack temperature sensor fault"},
    {0x04u, "Pack current sensor fault"},
    {0x08u, "Other fault"},
    {0x10u, "Cell voltage delta high"},
    {0x20u, "Charge breaker fault"},
    {0x40u, "Discharge breaker fault"},
    {0x80u, "Charge current high"},
};

static const seplosFlagName_t kWarning2Names[] = {
    {0x01u, "Cell voltage high warning"},
    {0x02u, "Cell voltage high alarm"},
    {0x04u, "Cell voltage low warning"},
    {0x08u, "Cell voltage low alarm"},
    {0x10u, "Pack voltage high warning"},
    {0x20u, "Pack voltage high alarm"},
    {0x40u, "Pack voltage low warning"},
    {0x80u, "Pack voltage low alarm"},
};

static const seplosFlagName_t kWarning3Names[] = {
    {0x01u, "Charge temperature high warning"},
    {0x02u, "Charge temperature high alarm"},
    {0x04u, "Charge temperature low warning"},
    {0x08u, "Charge temperature low alarm"},
    {0x10u, "Discharge temperature high warning"},
    {0x20u, "Discharge temperature high alarm"},
    {0x40u, "Discharge temperature low warning"},
    {0x80u, "Discharge temperature low alarm"},
};

static const seplosFlagName_t kWarning4Names[] = {
    {0x01u, "Environment temperature high warning"},
    {0x02u, "Environment temperature high alarm"},
    {0x04u, "Environment temperature low warning"},
    {0x08u, "Environment temperature low alarm"},
    {0x10u, "Pack temperature high warning"},
    {0x20u, "Pack temperature high alarm"},
};

static const seplosFlagName_t kWarning5Names[] = {
    {0x01u, "Charge current high warning"},
    {0x02u, "Charge current high alarm"},
    {0x04u, "Discharge current high warning"},
    {0x08u, "Discharge current high alarm"},
    {0x20u, "Short circuit alarm"},
    {0x40u, "Other alarm 5.6"},
    {0x80u, "Other alarm 5.7"},
};

static const seplosFlagName_t kWarning6Names[] = {
    {0x01u, "Charge voltage high alarm"},
    {0x04u, "SOC low warning"},
    {0x08u, "SOC low alarm"},
    {0x10u, "Other alarm 6.4"},
    {0x20u, "Other alarm 6.5"},
    {0x40u, "Other alarm 6.6"},
    {0x80u, "Other alarm 6.7"},
};

static int hexNibble(uint8_t ch)
{
    if (ch >= '0' && ch <= '9') return (int)(ch - '0');
    if (ch >= 'A' && ch <= 'F') return 10 + (int)(ch - 'A');
    if (ch >= 'a' && ch <= 'f') return 10 + (int)(ch - 'a');
    return -1;
}

static bool parseHexByte(const uint8_t *p, uint8_t *out)
{
    int hi = hexNibble(p[0]);
    int lo = hexNibble(p[1]);
    if (out == NULL || hi < 0 || lo < 0) {
        return false;
    }
    *out = (uint8_t)((hi << 4) | lo);
    return true;
}

static bool parseHexWord(const uint8_t *p, uint16_t *out)
{
    uint8_t hi = 0u;
    uint8_t lo = 0u;
    if (out == NULL || !parseHexByte(p, &hi) || !parseHexByte(p + 2u, &lo)) {
        return false;
    }
    *out = (uint16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
    return true;
}

static uint16_t be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static int16_t be16s(const uint8_t *p)
{
    return (int16_t)be16(p);
}

static uint16_t seplosChecksum(const char *body)
{
    uint32_t sum = 0u;

    if (body == NULL) {
        return 0u;
    }

    for (const char *p = body; *p != '\0'; p++) {
        sum += (uint8_t)(*p);
    }

    return (uint16_t)((~sum + 1u) & 0xFFFFu);
}

static uint16_t seplosLengthField(size_t infoAsciiLen)
{
    uint16_t lenid = (uint16_t)(infoAsciiLen & 0x0FFFu);
    uint16_t n0 = (uint16_t)((lenid >> 8) & 0x0Fu);
    uint16_t n1 = (uint16_t)((lenid >> 4) & 0x0Fu);
    uint16_t n2 = (uint16_t)(lenid & 0x0Fu);
    uint16_t sum = (uint16_t)((n0 + n1 + n2) & 0x0Fu);
    uint16_t lchk = (uint16_t)((~sum + 1u) & 0x0Fu);
    return (uint16_t)((lchk << 12) | lenid);
}

static bool lengthFieldValid(uint16_t field, size_t infoAsciiLen)
{
    if ((field & 0x0FFFu) != (uint16_t)(infoAsciiLen & 0x0FFFu)) {
        return false;
    }

    return field == seplosLengthField(infoAsciiLen) ||
           field == (uint16_t)infoAsciiLen;
}

size_t seplosRs485BuildRequestWithStyle(uint8_t cid2,
                                        uint8_t address,
                                        uint8_t requestInfo,
                                        uint8_t protocolVersion,
                                        seplos_rs485_request_style_t style,
                                        uint8_t *out,
                                        size_t outSize)
{
    char body[32];
    uint16_t checksum = 0u;
    int bodyLen = 0;
    int frameLen = 0;

    if (out == NULL || outSize == 0u) {
        return 0u;
    }

    if (style == SEPLOS_RS485_REQUEST_STYLE_SIMPLE_LEN_NIBBLE) {
        bodyLen = snprintf(body,
                           sizeof(body),
                           "%02X%02X%02X%02X0002%X",
                           (unsigned)protocolVersion,
                           (unsigned)address,
                           (unsigned)SEPLOS_RS485_CID1_BMS,
                           (unsigned)cid2,
                           (unsigned)(requestInfo & 0x0Fu));
    } else {
        const uint16_t length = seplosLengthField(2u);
        bodyLen = snprintf(body,
                           sizeof(body),
                           "%02X%02X%02X%02X%04X%02X",
                           (unsigned)protocolVersion,
                           (unsigned)address,
                           (unsigned)SEPLOS_RS485_CID1_BMS,
                           (unsigned)cid2,
                           (unsigned)length,
                           (unsigned)requestInfo);
    }
    if (bodyLen <= 0 || (size_t)bodyLen >= sizeof(body)) {
        return 0u;
    }

    checksum = seplosChecksum(body);
    frameLen = snprintf((char *)out, outSize, "~%s%04X\r", body, (unsigned)checksum);
    if (frameLen <= 0 || (size_t)frameLen >= outSize) {
        return 0u;
    }

    return (size_t)frameLen;
}

size_t seplosRs485BuildRequest(uint8_t cid2,
                               uint8_t address,
                               uint8_t protocolVersion,
                               uint8_t *out,
                               size_t outSize)
{
    return seplosRs485BuildRequestWithStyle(cid2,
                                            address,
                                            SEPLOS_RS485_DEFAULT_REQUEST_INFO,
                                            protocolVersion,
                                            SEPLOS_RS485_REQUEST_STYLE_LEN_CHECK,
                                            out,
                                            outSize);
}

bool seplosRs485DecodeFrame(const uint8_t *frame,
                            size_t frameLen,
                            seplos_rs485_frame_t *out)
{
    size_t bodyLen = 0u;
    size_t infoAsciiLen = 0u;
    size_t infoByteLen = 0u;
    uint16_t receivedChecksum = 0u;
    uint16_t expectedChecksum = 0u;
    char body[SEPLOS_RS485_MAX_FRAME_LEN];

    if (frame == NULL || out == NULL || frameLen < 18u ||
        frameLen >= SEPLOS_RS485_MAX_FRAME_LEN ||
        frame[0] != '~' || frame[frameLen - 1u] != '\r') {
        return false;
    }

    bodyLen = frameLen - 2u;
    if (bodyLen < 16u || bodyLen >= sizeof(body)) {
        return false;
    }

    for (size_t i = 1u; i < frameLen - 1u; i++) {
        if (hexNibble(frame[i]) < 0) {
            return false;
        }
    }

    memcpy(body, frame + 1u, bodyLen);
    body[bodyLen] = '\0';

    if (!parseHexWord((const uint8_t *)&body[bodyLen - 4u], &receivedChecksum)) {
        return false;
    }
    body[bodyLen - 4u] = '\0';
    expectedChecksum = seplosChecksum(body);
    if (receivedChecksum != expectedChecksum) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    if (!parseHexByte((const uint8_t *)&body[0], &out->version) ||
        !parseHexByte((const uint8_t *)&body[2], &out->address) ||
        !parseHexByte((const uint8_t *)&body[4], &out->cid1) ||
        !parseHexByte((const uint8_t *)&body[6], &out->cid2) ||
        !parseHexWord((const uint8_t *)&body[8], &out->lengthField)) {
        return false;
    }

    infoAsciiLen = (bodyLen - 4u) - 12u;
    if ((infoAsciiLen % 2u) != 0u || !lengthFieldValid(out->lengthField, infoAsciiLen)) {
        return false;
    }

    infoByteLen = infoAsciiLen / 2u;
    if (infoByteLen > sizeof(out->info)) {
        return false;
    }

    for (size_t i = 0u; i < infoByteLen; i++) {
        if (!parseHexByte((const uint8_t *)&body[12u + (i * 2u)], &out->info[i])) {
            return false;
        }
    }
    out->infoLen = infoByteLen;

    return out->cid1 == SEPLOS_RS485_CID1_BMS;
}

static void recomputeCells(seplos_rs485_snapshot_t *out)
{
    uint32_t sum = 0u;
    uint8_t counted = 0u;
    uint16_t minMv = UINT16_MAX;
    uint16_t maxMv = 0u;
    uint8_t minIdx = 0u;
    uint8_t maxIdx = 0u;

    if (out == NULL || out->cellCount == 0u) {
        return;
    }

    for (uint8_t i = 0u; i < out->cellCount && i < SEPLOS_RS485_MAX_CELLS; i++) {
        uint16_t mv = out->cellMv[i];
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

    out->hasCellExtremes = true;
    out->minCellMv = minMv;
    out->maxCellMv = maxMv;
    out->minCellIndex = minIdx;
    out->maxCellIndex = maxIdx;
    out->hasCellAvgMv = true;
    out->cellAvgMv = (uint16_t)((sum + (uint32_t)(counted / 2u)) / (uint32_t)counted);
    out->hasCellDiffMv = true;
    out->cellDiffMv = (uint16_t)(maxMv - minMv);
}

bool seplosRs485DecodeTelemetryInfo(const uint8_t *info,
                                    size_t infoLen,
                                    seplos_rs485_snapshot_t *out)
{
    size_t pos = 0u;
    uint8_t reportedCells = 0u;
    uint8_t reportedTemps = 0u;

    if (info == NULL || out == NULL || infoLen < 3u) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->dataFlag = info[pos++];
    out->commandGroup = info[pos++];
    reportedCells = info[pos++];
    if (reportedCells == 0u || reportedCells > SEPLOS_RS485_MAX_CELLS ||
        pos + ((size_t)reportedCells * 2u) > infoLen) {
        return false;
    }

    out->cellCount = reportedCells;
    for (uint8_t i = 0u; i < reportedCells; i++) {
        out->cellMv[i] = be16(&info[pos]);
        pos += 2u;
    }
    recomputeCells(out);

    if (pos >= infoLen) {
        return false;
    }
    reportedTemps = info[pos++];
    if (reportedTemps > SEPLOS_RS485_MAX_TEMPS ||
        pos + ((size_t)reportedTemps * 2u) > infoLen) {
        return false;
    }

    out->tempCount = reportedTemps;
    for (uint8_t i = 0u; i < reportedTemps; i++) {
        uint16_t centiC = be16(&info[pos]);
        out->tempDeciC[i] = (int16_t)((centiC + 5u) / 10u);
        pos += 2u;
    }

    if (pos + 19u > infoLen) {
        return false;
    }

    out->hasPackCurrentCa = true;
    out->packCurrentCa = be16s(&info[pos]);
    pos += 2u;

    out->hasPackVoltageCv = true;
    out->packVoltageCv = be16(&info[pos]);
    pos += 2u;

    out->hasRemainingCapacityCah = true;
    out->remainingCapacityCah = be16(&info[pos]);
    pos += 2u;

    pos += 1u; /* Custom info byte. */

    out->hasFullCapacityCah = true;
    out->fullCapacityCah = be16(&info[pos]);
    pos += 2u;

    out->hasSocDeciPct = true;
    out->socDeciPct = be16(&info[pos]);
    pos += 2u;

    out->hasRatedCapacityCah = true;
    out->ratedCapacityCah = be16(&info[pos]);
    pos += 2u;

    out->hasCycles = true;
    out->cycles = be16(&info[pos]);
    pos += 2u;

    out->hasSohDeciPct = true;
    out->sohDeciPct = be16(&info[pos]);
    pos += 2u;

    out->hasPortVoltageCv = true;
    out->portVoltageCv = be16(&info[pos]);

    if (out->hasPackVoltageCv && out->hasPackCurrentCa) {
        out->hasPackPowerW = true;
        out->packPowerW = (int32_t)(((int32_t)out->packVoltageCv *
                                     (int32_t)out->packCurrentCa) /
                                    10000);
    }

    out->hasTelemetry = true;
    out->valid = true;
    return true;
}

bool seplosRs485DecodeAlarmInfo(const uint8_t *info,
                                size_t infoLen,
                                seplos_rs485_snapshot_t *inOut)
{
    size_t pos = 0u;
    uint8_t reportedCells = 0u;
    uint8_t reportedTemps = 0u;

    if (info == NULL || inOut == NULL || infoLen < 3u) {
        return false;
    }

    inOut->dataFlag = info[pos++];
    inOut->commandGroup = info[pos++];
    reportedCells = info[pos++];
    if (reportedCells > SEPLOS_RS485_MAX_CELLS || pos + reportedCells > infoLen) {
        return false;
    }

    for (uint8_t i = 0u; i < reportedCells; i++) {
        inOut->cellAlarmFlags[i] = info[pos++];
    }

    if (pos >= infoLen) {
        return false;
    }
    reportedTemps = info[pos++];
    if (reportedTemps > SEPLOS_RS485_MAX_TEMPS || pos + reportedTemps > infoLen) {
        return false;
    }
    for (uint8_t i = 0u; i < reportedTemps; i++) {
        inOut->tempAlarmFlags[i] = info[pos++];
    }

    if (pos + 14u > infoLen) {
        return false;
    }

    inOut->currentAlarmFlags = info[pos++];
    inOut->voltageAlarmFlags = info[pos++];
    inOut->customAlarmFlags = info[pos++];

    for (uint8_t i = 0u; i < 6u; i++) {
        inOut->warningBytes[i] = info[pos++];
    }

    inOut->powerStatus = info[pos++];
    uint8_t balanceLow = info[pos++];
    uint8_t balanceHigh = info[pos++];
    inOut->balanceFlags = (uint16_t)balanceLow | (uint16_t)((uint16_t)balanceHigh << 8);

    inOut->systemStatus = info[pos++];
    inOut->dischargeEnabled = (inOut->systemStatus & 0x01u) != 0u;
    inOut->chargeEnabled = (inOut->systemStatus & 0x02u) != 0u;
    inOut->sleepMode = (inOut->systemStatus & 0x10u) != 0u;

    inOut->warningBytes[6] = info[pos++];
    inOut->warningBytes[7] = info[pos++];

    inOut->hasAlarms = true;
    inOut->valid = inOut->valid || inOut->hasTelemetry || inOut->hasAlarms;
    return true;
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

static void appendLowHighFlag(char *out,
                              size_t outSize,
                              uint8_t flags,
                              const char *label,
                              const char *low,
                              const char *high)
{
    char item[48];

    if (flags == 0u || label == NULL) {
        return;
    }
    if ((flags & 0x01u) != 0u) {
        (void)snprintf(item, sizeof(item), "%s %s", label, low);
        appendListItem(out, outSize, item);
    }
    if ((flags & 0x02u) != 0u) {
        (void)snprintf(item, sizeof(item), "%s %s", label, high);
        appendListItem(out, outSize, item);
    }
    if ((flags & (uint8_t)~0x03u) != 0u) {
        (void)snprintf(item, sizeof(item), "%s flag 0x%02X", label, (unsigned)flags);
        appendListItem(out, outSize, item);
    }
}

static void appendNamedFlags(uint8_t value,
                             const seplosFlagName_t *names,
                             size_t namesCount,
                             char *warnings,
                             size_t warningsSize,
                             char *alarms,
                             size_t alarmsSize)
{
    uint8_t known = 0u;

    if (value == 0u || names == NULL) {
        return;
    }

    for (size_t i = 0u; i < namesCount; i++) {
        known = (uint8_t)(known | names[i].mask);
        if ((value & names[i].mask) != 0u) {
            if (strstr(names[i].name, "alarm") != NULL ||
                strstr(names[i].name, "fault") != NULL) {
                appendListItem(alarms, alarmsSize, names[i].name);
            } else {
                appendListItem(warnings, warningsSize, names[i].name);
            }
        }
    }

    uint8_t unknown = (uint8_t)(value & (uint8_t)~known);
    for (uint8_t bit = 0u; bit < 8u; bit++) {
        uint8_t mask = (uint8_t)(1u << bit);
        if ((unknown & mask) != 0u) {
            char item[32];
            (void)snprintf(item, sizeof(item), "Unknown bit %u", (unsigned)bit);
            appendListItem(warnings, warningsSize, item);
        }
    }
}

void seplosRs485FormatAlertFields(const seplos_rs485_snapshot_t *snapshot,
                                  char *protections,
                                  size_t protectionsSize,
                                  char *alarms,
                                  size_t alarmsSize,
                                  char *warnings,
                                  size_t warningsSize)
{
    if (protections != NULL && protectionsSize > 0u) protections[0] = '\0';
    if (alarms != NULL && alarmsSize > 0u) alarms[0] = '\0';
    if (warnings != NULL && warningsSize > 0u) warnings[0] = '\0';

    if (snapshot == NULL || !snapshot->hasAlarms) {
        return;
    }

    for (uint8_t i = 0u; i < snapshot->cellCount && i < SEPLOS_RS485_MAX_CELLS; i++) {
        char label[16];
        (void)snprintf(label, sizeof(label), "Cell %u", (unsigned)(i + 1u));
        appendLowHighFlag(warnings,
                          warningsSize,
                          snapshot->cellAlarmFlags[i],
                          label,
                          "low",
                          "high");
    }

    for (uint8_t i = 0u; i < snapshot->tempCount && i < SEPLOS_RS485_MAX_TEMPS; i++) {
        char label[16];
        (void)snprintf(label, sizeof(label), "Temp %u", (unsigned)(i + 1u));
        appendLowHighFlag(warnings,
                          warningsSize,
                          snapshot->tempAlarmFlags[i],
                          label,
                          "low",
                          "high");
    }

    appendLowHighFlag(warnings,
                      warningsSize,
                      snapshot->currentAlarmFlags,
                      "Current",
                      "discharge high",
                      "charge high");
    appendLowHighFlag(warnings,
                      warningsSize,
                      snapshot->voltageAlarmFlags,
                      "Pack voltage",
                      "low",
                      "high");

    appendNamedFlags(snapshot->warningBytes[0],
                     kWarning1Names,
                     sizeof(kWarning1Names) / sizeof(kWarning1Names[0]),
                     warnings,
                     warningsSize,
                     alarms,
                     alarmsSize);
    appendNamedFlags(snapshot->warningBytes[1],
                     kWarning2Names,
                     sizeof(kWarning2Names) / sizeof(kWarning2Names[0]),
                     warnings,
                     warningsSize,
                     alarms,
                     alarmsSize);
    appendNamedFlags(snapshot->warningBytes[2],
                     kWarning3Names,
                     sizeof(kWarning3Names) / sizeof(kWarning3Names[0]),
                     warnings,
                     warningsSize,
                     alarms,
                     alarmsSize);
    appendNamedFlags(snapshot->warningBytes[3],
                     kWarning4Names,
                     sizeof(kWarning4Names) / sizeof(kWarning4Names[0]),
                     warnings,
                     warningsSize,
                     alarms,
                     alarmsSize);
    appendNamedFlags(snapshot->warningBytes[4],
                     kWarning5Names,
                     sizeof(kWarning5Names) / sizeof(kWarning5Names[0]),
                     warnings,
                     warningsSize,
                     alarms,
                     alarmsSize);
    appendNamedFlags(snapshot->warningBytes[5],
                     kWarning6Names,
                     sizeof(kWarning6Names) / sizeof(kWarning6Names[0]),
                     warnings,
                     warningsSize,
                     alarms,
                     alarmsSize);

    if (snapshot->warningBytes[6] != 0u) {
        char item[32];
        (void)snprintf(item, sizeof(item), "Warning7=0x%02X", (unsigned)snapshot->warningBytes[6]);
        appendListItem(warnings, warningsSize, item);
    }
    if (snapshot->warningBytes[7] != 0u) {
        char item[32];
        (void)snprintf(item, sizeof(item), "Warning8=0x%02X", (unsigned)snapshot->warningBytes[7]);
        appendListItem(warnings, warningsSize, item);
    }

    if (snapshot->customAlarmFlags != 0u) {
        char item[32];
        (void)snprintf(item, sizeof(item), "Custom=0x%02X", (unsigned)snapshot->customAlarmFlags);
        appendListItem(protections, protectionsSize, item);
    }
}
