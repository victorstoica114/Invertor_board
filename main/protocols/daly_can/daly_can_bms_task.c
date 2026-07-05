#include "protocols/daly_can/daly_can_bms_task.h"

#include <inttypes.h>
#include <limits.h>
#include <string.h>

#include "Drivers/can_driver.h"
#include "config.h"
#include "protocols/common/battery_model.h"
#include "runtime_settings.h"

#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DALY_CAN_HOST_ID 0x40u
#define DALY_CAN_MASTER_ID 0x80u
#define DALY_CAN_GPRS_ID 0x20u
#define DALY_CAN_ALT_HOST_ID 0x24u
#define DALY_CAN_FIELD_HOST_ID 0x92u
#define DALY_CAN_FIELD_BMS_ID 0x21u
#define DALY_CAN_FIELD_ALT_HOST_ID 0x21u
#define DALY_CAN_FIELD_ALT_BMS_ID 0x92u
#define DALY_CAN_FRAME_PRIO 0x18u
#define DALY_CAN_EMPTY_DATA {0, 0, 0, 0, 0, 0, 0, 0}
#define DALY_CAN_STATUS_LOG_INTERVAL_US 5000000LL
#define DALY_CAN_DECODE_LOG_INTERVAL_US 1000000LL
#define DALY_CAN_BITRATE_SWITCH_INTERVAL_US 7000000LL
#define DALY_CAN_OTHER_BUS_DIAG_INTERVAL_US 5000000LL
#define DALY_CAN_LOGGED_STD_ID_MAX 16u

static const uint8_t kDalyCanProbeTargets[] = {
    (uint8_t)DALY_CAN_BMS_ID, /* documented/default BMS ID */
    (uint8_t)DALY_CAN_FIELD_BMS_ID, /* observed on some Daly CAN packs */
    (uint8_t)DALY_CAN_FIELD_ALT_BMS_ID, /* inverse field addressing seen on LA captures */
};

typedef enum {
    DALY_CMD_RATED_CAPACITY_CELL_VOLTAGE = 0x50,
    DALY_CMD_BATTERY_TYPE_INFO = 0x53,
    DALY_CMD_MIN_MAX_PACK_VOLTAGE = 0x5A,
    DALY_CMD_MAX_DISCHARGE_CHARGE_CURRENT = 0x5B,
    DALY_CMD_VOUT_IOUT_SOC = 0x90,
    DALY_CMD_MIN_MAX_CELL_VOLTAGE = 0x91,
    DALY_CMD_MIN_MAX_TEMPERATURE = 0x92,
    DALY_CMD_MOS_STATUS = 0x93,
    DALY_CMD_STATUS_INFO = 0x94,
    DALY_CMD_CELL_VOLTAGES = 0x95,
    DALY_CMD_CELL_TEMPERATURES = 0x96,
    DALY_CMD_CELL_BALANCE_STATE = 0x97,
    DALY_CMD_FAILURE_CODES = 0x98,
} daly_can_cmd_t;

typedef enum {
    DALY_CAN_REQ_DLC8_ZERO = 0,
    DALY_CAN_REQ_DLC8_ZERO_HOST80,
    DALY_CAN_REQ_DLC8_ZERO_HOST20,
    DALY_CAN_REQ_DLC8_ZERO_HOST24,
    DALY_CAN_REQ_DLC8_ZERO_HOST92,
    DALY_CAN_REQ_DLC8_ZERO_HOST21,
    DALY_CAN_REQ_DLC0_HOST40,
    DALY_CAN_REQ_RTR_HOST40,
    DALY_CAN_REQ_VARIANT_COUNT,
} daly_can_request_variant_t;

typedef struct {
    QueueHandle_t outQueue;
    twai_handle_t bus;
    const char *ifName;
    uint8_t canPort;
    uint32_t bitrate;
    int64_t lastBitrateSwitchUs;
    bool allowBitrateAutoprobe;
    uint32_t sequence;
    int64_t lastFrameUs;
    int64_t lastPublishUs;
    uint8_t activeTargetId;
    uint8_t pendingTargetId;
    bool haveActiveTargetId;
    bool havePendingTargetId;
    bool havePreferredRequestVariant;
    daly_can_request_variant_t preferredRequestVariant;
    daly_rs485_snapshot_t snapshot;
} daly_can_task_ctx_t;

static daly_can_task_ctx_t g_dalyCanCtx;
static TaskHandle_t g_dalyCanTaskHandle;
static portMUX_TYPE g_latestMux = portMUX_INITIALIZER_UNLOCKED;
static bool g_haveLatestPacket;
static bms_decoded_packet_t g_latestPacket;
static bool g_haveLatestSnapshot;
static daly_rs485_snapshot_t g_latestSnapshot;
static int64_t g_lastStaleLogUs;
static int64_t g_lastDecodeLogUs;
static int64_t g_lastRxLogUs;
static int64_t g_lastTxErrorLogUs;
static int64_t g_lastTxOkLogUs;
static int64_t g_lastRecoveryUs;
static int64_t g_lastOtherBusDiagUs;
static uint32_t g_rawStdIdsLogged[DALY_CAN_LOGGED_STD_ID_MAX];
static uint8_t g_rawStdIdsLoggedCount;
static uint32_t g_rawExtFramesLogged;

static uint16_t getBe16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint32_t getBe32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) |
           (uint32_t)p[3];
}

static uint8_t clampPctFromDeci(uint16_t deciPct)
{
    uint16_t pct = (uint16_t)((deciPct + 5u) / 10u);
    return (uint8_t)((pct > 100u) ? 100u : pct);
}

static const char *dalyCanStateStr(twai_state_t state)
{
    switch (state) {
        case TWAI_STATE_STOPPED:
            return "STOPPED";
        case TWAI_STATE_RUNNING:
            return "RUNNING";
        case TWAI_STATE_BUS_OFF:
            return "BUS_OFF";
        case TWAI_STATE_RECOVERING:
            return "RECOVERING";
        default:
            return "UNKNOWN";
    }
}

static uint8_t dalyCanDefaultTargetId(void)
{
    return (uint8_t)DALY_CAN_BMS_ID;
}

static uint8_t dalyCanProbeTargetCount(void)
{
    return (uint8_t)(sizeof(kDalyCanProbeTargets) / sizeof(kDalyCanProbeTargets[0]));
}

static uint8_t dalyCanProbeTargetAt(uint8_t index)
{
    if (index >= dalyCanProbeTargetCount()) {
        return dalyCanDefaultTargetId();
    }
    return kDalyCanProbeTargets[index];
}

static twai_handle_t dalyCanBusForPort(uint8_t port)
{
    return (port == 2u) ? canGetBus1() : canGetBus0();
}

static const char *dalyCanIfNameForPort(uint8_t port)
{
    return (port == 2u) ? "CAN2" : "CAN1";
}

static uint32_t dalyCanNextProbeBitrate(uint32_t bitrate)
{
    static const uint32_t kProbeBitrates[] = {
        CAN_JKBMS_250K_BITRATE,
        CAN_DEFAULT_BITRATE,
        125000u,
        1000000u,
        800000u,
        100000u,
        50000u,
    };
    const size_t count = sizeof(kProbeBitrates) / sizeof(kProbeBitrates[0]);

    for (size_t i = 0u; i < count; i++) {
        if (kProbeBitrates[i] == bitrate) {
            return kProbeBitrates[(i + 1u) % count];
        }
    }

    return CAN_JKBMS_250K_BITRATE;
}

static void dalyCanResetProbeState(daly_can_task_ctx_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    ctx->activeTargetId = dalyCanDefaultTargetId();
    ctx->pendingTargetId = 0u;
    ctx->haveActiveTargetId = true;
    ctx->havePendingTargetId = false;
    ctx->havePreferredRequestVariant = false;
    ctx->preferredRequestVariant = DALY_CAN_REQ_DLC8_ZERO;
    g_lastTxErrorLogUs = 0;
    g_lastTxOkLogUs = 0;
    g_lastRxLogUs = 0;
    memset(g_rawStdIdsLogged, 0, sizeof(g_rawStdIdsLogged));
    g_rawStdIdsLoggedCount = 0u;
    g_rawExtFramesLogged = 0;
}

static bool dalyCanMaybeSwitchBitrate(daly_can_task_ctx_t *ctx, int64_t nowUs)
{
    if (ctx == NULL || !ctx->allowBitrateAutoprobe || ctx->canPort == 0u) {
        return false;
    }

    if (ctx->lastBitrateSwitchUs > 0 &&
        (nowUs - ctx->lastBitrateSwitchUs) < DALY_CAN_BITRATE_SWITCH_INTERVAL_US) {
        return false;
    }

    uint32_t oldBitrate = ctx->bitrate;
    uint32_t newBitrate = dalyCanNextProbeBitrate(oldBitrate);
    ctx->lastBitrateSwitchUs = nowUs;

    ESP_LOGW(EXAMPLE_TAG,
             "DALY CAN stale on %s: switching BMS CAN bitrate %u -> %u in normal mode and restarting request probe",
             ctx->ifName ? ctx->ifName : dalyCanIfNameForPort(ctx->canPort),
             (unsigned)oldBitrate,
             (unsigned)newBitrate);

    esp_err_t err = canReinitPortMode(ctx->canPort, newBitrate, TWAI_MODE_NORMAL);
    if (err != ESP_OK) {
        ESP_LOGW(EXAMPLE_TAG,
                 "DALY CAN bitrate switch failed on CAN%u: err=0x%x",
                 (unsigned)ctx->canPort,
                 (unsigned)err);
        return false;
    }

    ctx->bitrate = newBitrate;
    ctx->bus = dalyCanBusForPort(ctx->canPort);
    ctx->ifName = dalyCanIfNameForPort(ctx->canPort);
    dalyCanResetProbeState(ctx);
    if (ctx->bus != NULL) {
        (void)twai_clear_receive_queue_v2(ctx->bus);
        (void)twai_clear_transmit_queue_v2(ctx->bus);
    }
    return true;
}

static uint8_t dalyCanAlternateTargetId(uint8_t targetId)
{
    uint8_t count = dalyCanProbeTargetCount();
    for (uint8_t i = 0u; i < count; i++) {
        if (kDalyCanProbeTargets[i] == targetId) {
            return dalyCanProbeTargetAt((uint8_t)(i + 1u));
        }
    }
    return dalyCanDefaultTargetId();
}

static uint8_t dalyCanActiveTargetId(const daly_can_task_ctx_t *ctx)
{
    if (ctx != NULL && ctx->haveActiveTargetId) {
        return ctx->activeTargetId;
    }
    return dalyCanDefaultTargetId();
}

static const char *dalyCanRequestVariantName(daly_can_request_variant_t variant)
{
    switch (variant) {
        case DALY_CAN_REQ_DLC8_ZERO:
            return "host40-dlc8-zero";
        case DALY_CAN_REQ_DLC8_ZERO_HOST80:
            return "host80-dlc8-zero";
        case DALY_CAN_REQ_DLC8_ZERO_HOST20:
            return "host20-dlc8-zero";
        case DALY_CAN_REQ_DLC8_ZERO_HOST24:
            return "host24-dlc8-zero";
        case DALY_CAN_REQ_DLC8_ZERO_HOST92:
            return "host92-dlc8-zero";
        case DALY_CAN_REQ_DLC8_ZERO_HOST21:
            return "host21-dlc8-zero";
        case DALY_CAN_REQ_DLC0_HOST40:
            return "host40-dlc0";
        case DALY_CAN_REQ_RTR_HOST40:
            return "host40-rtr";
        default:
            return "unknown";
    }
}

static uint8_t dalyCanRequestHostId(daly_can_request_variant_t variant)
{
    if (variant == DALY_CAN_REQ_DLC8_ZERO_HOST80) {
        return DALY_CAN_MASTER_ID;
    }
    if (variant == DALY_CAN_REQ_DLC8_ZERO_HOST20) {
        return DALY_CAN_GPRS_ID;
    }
    if (variant == DALY_CAN_REQ_DLC8_ZERO_HOST24) {
        return DALY_CAN_ALT_HOST_ID;
    }
    if (variant == DALY_CAN_REQ_DLC8_ZERO_HOST92) {
        return DALY_CAN_FIELD_HOST_ID;
    }
    if (variant == DALY_CAN_REQ_DLC8_ZERO_HOST21) {
        return DALY_CAN_FIELD_ALT_HOST_ID;
    }
    return DALY_CAN_HOST_ID;
}

static bool dalyCanVariantUsesRtr(daly_can_request_variant_t variant)
{
    return variant == DALY_CAN_REQ_RTR_HOST40;
}

static bool dalyCanVariantUsesEmptyDlc(daly_can_request_variant_t variant)
{
    return (variant == DALY_CAN_REQ_DLC0_HOST40) ||
           (variant == DALY_CAN_REQ_RTR_HOST40);
}

static bool dalyCanSourceIsAllowed(const daly_can_task_ctx_t *ctx, uint8_t sourceId)
{
    if (sourceId == dalyCanDefaultTargetId() || sourceId == DALY_CAN_MASTER_ID) {
        return true;
    }
    for (uint8_t i = 0u; i < dalyCanProbeTargetCount(); i++) {
        if (sourceId == dalyCanProbeTargetAt(i)) {
            return true;
        }
    }
    if (ctx != NULL) {
        return (ctx->haveActiveTargetId && sourceId == ctx->activeTargetId) ||
               (ctx->havePendingTargetId && sourceId == ctx->pendingTargetId);
    }
    return false;
}

static bool dalyCanHostIsAllowed(uint8_t hostId)
{
    return hostId == DALY_CAN_HOST_ID ||
           hostId == DALY_CAN_MASTER_ID ||
           hostId == DALY_CAN_GPRS_ID ||
           hostId == DALY_CAN_ALT_HOST_ID ||
           hostId == DALY_CAN_FIELD_HOST_ID ||
           hostId == DALY_CAN_FIELD_ALT_HOST_ID ||
           hostId == 0x00u;
}

static uint32_t dalyCanRequestIdForVariant(uint8_t cmd,
                                           uint8_t targetId,
                                           daly_can_request_variant_t variant)
{
    uint8_t hostId = dalyCanRequestHostId(variant);
    return ((uint32_t)DALY_CAN_FRAME_PRIO << 24) |
           ((uint32_t)cmd << 16) |
           ((uint32_t)targetId << 8) |
           (uint32_t)hostId;
}

static uint32_t dalyCanRequestIdFor(uint8_t cmd, uint8_t targetId)
{
    return dalyCanRequestIdForVariant(cmd, targetId, DALY_CAN_REQ_DLC8_ZERO);
}

static uint32_t dalyCanResponseIdFor(uint8_t cmd, uint8_t sourceId)
{
    return ((uint32_t)DALY_CAN_FRAME_PRIO << 24) |
           ((uint32_t)cmd << 16) |
           ((uint32_t)DALY_CAN_HOST_ID << 8) |
           (uint32_t)sourceId;
}

static bool dalyCanResponseMatches(daly_can_task_ctx_t *ctx, uint32_t id, uint8_t cmd)
{
    uint8_t lowId = (uint8_t)(id & 0xFFu);
    uint8_t midId = (uint8_t)((id >> 8) & 0xFFu);
    bool classicResponse = dalyCanHostIsAllowed(midId) && dalyCanSourceIsAllowed(ctx, lowId);
    bool directResponse = dalyCanSourceIsAllowed(ctx, midId) && dalyCanHostIsAllowed(lowId);
    bool matches = (((id >> 24) & 0xFFu) == DALY_CAN_FRAME_PRIO) &&
                   (((id >> 16) & 0xFFu) == cmd) &&
                   (classicResponse || directResponse);
    if (matches && ctx != NULL) {
        ctx->activeTargetId = directResponse ? midId : lowId;
        ctx->haveActiveTargetId = true;
    }
    return matches;
}

static bool dalyCanLooksLikeResponse(daly_can_task_ctx_t *ctx, uint32_t id, uint8_t *cmdOut)
{
    uint8_t lowId = (uint8_t)(id & 0xFFu);
    uint8_t midId = (uint8_t)((id >> 8) & 0xFFu);
    bool classicResponse = dalyCanHostIsAllowed(midId) && dalyCanSourceIsAllowed(ctx, lowId);
    bool directResponse = dalyCanSourceIsAllowed(ctx, midId) && dalyCanHostIsAllowed(lowId);
    if (((id >> 24) & 0xFFu) != DALY_CAN_FRAME_PRIO ||
        (!classicResponse && !directResponse)) {
        return false;
    }

    if (ctx != NULL) {
        ctx->activeTargetId = directResponse ? midId : lowId;
        ctx->haveActiveTargetId = true;
    }
    if (cmdOut != NULL) {
        *cmdOut = (uint8_t)((id >> 16) & 0xFFu);
    }
    return true;
}

static bool dalyCanCommandKnown(uint8_t cmd)
{
    switch (cmd) {
        case DALY_CMD_RATED_CAPACITY_CELL_VOLTAGE:
        case DALY_CMD_BATTERY_TYPE_INFO:
        case DALY_CMD_MIN_MAX_PACK_VOLTAGE:
        case DALY_CMD_MAX_DISCHARGE_CHARGE_CURRENT:
        case DALY_CMD_VOUT_IOUT_SOC:
        case DALY_CMD_MIN_MAX_CELL_VOLTAGE:
        case DALY_CMD_MIN_MAX_TEMPERATURE:
        case DALY_CMD_MOS_STATUS:
        case DALY_CMD_STATUS_INFO:
        case DALY_CMD_CELL_VOLTAGES:
        case DALY_CMD_CELL_TEMPERATURES:
        case DALY_CMD_CELL_BALANCE_STATE:
        case DALY_CMD_FAILURE_CODES:
            return true;
        default:
            return false;
    }
}

static uint32_t dalyFailureMask(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24) |
           ((uint32_t)data[1] << 16) |
           ((uint32_t)data[2] << 8) |
           (uint32_t)data[3];
}

static void dalyCanDecodePayload(daly_can_task_ctx_t *ctx, uint8_t cmd, const uint8_t data[8])
{
    daly_rs485_snapshot_t *s = NULL;

    if (ctx == NULL || data == NULL || !dalyCanCommandKnown(cmd)) {
        return;
    }

    s = &ctx->snapshot;
    s->valid = true;
    s->timestampUs = esp_timer_get_time();
    ctx->lastFrameUs = s->timestampUs;

    switch (cmd) {
        case DALY_CMD_RATED_CAPACITY_CELL_VOLTAGE:
            s->hasCapacity = true;
            s->ratedCapacityMah = getBe32(&data[0]);
            break;

        case DALY_CMD_MAX_DISCHARGE_CHARGE_CURRENT:
            break;

        case DALY_CMD_VOUT_IOUT_SOC: {
            int32_t currentDeciA = (int32_t)getBe16(&data[4]) - 30000;
            s->hasPackVoltageCv = true;
            s->packVoltageCv = (uint16_t)(getBe16(&data[0]) * 10u);
            s->hasCurrentDeciA = true;
            if (currentDeciA < INT16_MIN) currentDeciA = INT16_MIN;
            if (currentDeciA > INT16_MAX) currentDeciA = INT16_MAX;
            s->currentDeciA = (int16_t)currentDeciA;
            s->hasSocDeciPct = true;
            s->socDeciPct = getBe16(&data[6]);
            break;
        }

        case DALY_CMD_MIN_MAX_CELL_VOLTAGE:
            s->hasCellExtremes = true;
            s->maxCellMv = getBe16(&data[0]);
            s->maxCellIndex = data[2];
            s->minCellMv = getBe16(&data[3]);
            s->minCellIndex = data[5];
            break;

        case DALY_CMD_MIN_MAX_TEMPERATURE:
            s->tempCount = (s->tempCount > 0u) ? s->tempCount : 2u;
            s->tempDeciC[0] = (int16_t)(((int16_t)data[0] - 40) * 10);
            s->tempDeciC[1] = (int16_t)(((int16_t)data[2] - 40) * 10);
            break;

        case DALY_CMD_MOS_STATUS:
            s->protocolState = data[0];
            s->chargeEnabled = data[1] == 1u;
            s->dischargeEnabled = data[2] == 1u;
            s->hasCapacity = true;
            s->remainingCapacityMah = getBe32(&data[4]);
            break;

        case DALY_CMD_STATUS_INFO:
            if (data[0] > 0u) {
                s->cellCount = (data[0] > BMS_DECODED_PACKET_MAX_CELLS)
                                   ? BMS_DECODED_PACKET_MAX_CELLS
                                   : data[0];
            }
            if (data[1] > 0u) {
                s->tempCount = (data[1] > BMS_DECODED_PACKET_MAX_TEMPS)
                                   ? BMS_DECODED_PACKET_MAX_TEMPS
                                   : data[1];
            }
            s->chargeEnabled = data[2] == 1u;
            s->dischargeEnabled = data[3] == 1u;
            s->hasCycles = true;
            s->cycles = getBe16(&data[5]);
            break;

        case DALY_CMD_CELL_VOLTAGES: {
            uint8_t frameNo = data[0];
            if (frameNo == 0xFFu) {
                break;
            }
            uint8_t firstCell = (uint8_t)(frameNo * 3u);
            for (uint8_t i = 0u; i < 3u; i++) {
                uint8_t cell = (uint8_t)(firstCell + i);
                uint16_t mv = getBe16(&data[1u + (uint8_t)(i * 2u)]);
                if (cell >= BMS_DECODED_PACKET_MAX_CELLS || mv < 1000u || mv > 6000u) {
                    continue;
                }
                s->cellMv[cell] = mv;
                if (s->cellCount < (uint8_t)(cell + 1u)) {
                    s->cellCount = (uint8_t)(cell + 1u);
                }
            }
            break;
        }

        case DALY_CMD_CELL_TEMPERATURES: {
            uint8_t frameNo = data[0];
            if (frameNo == 0xFFu) {
                break;
            }
            uint8_t firstTemp = (uint8_t)(frameNo * 7u);
            for (uint8_t i = 0u; i < 7u; i++) {
                uint8_t temp = (uint8_t)(firstTemp + i);
                if (temp >= BMS_DECODED_PACKET_MAX_TEMPS) {
                    break;
                }
                s->tempDeciC[temp] = (int16_t)(((int16_t)data[1u + i] - 40) * 10);
                if (s->tempCount < (uint8_t)(temp + 1u)) {
                    s->tempCount = (uint8_t)(temp + 1u);
                }
            }
            break;
        }

        case DALY_CMD_CELL_BALANCE_STATE:
            s->balanceEnabled = false;
            for (uint8_t i = 0u; i < 6u; i++) {
                if (data[i] != 0u) {
                    s->balanceEnabled = true;
                    break;
                }
            }
            break;

        case DALY_CMD_FAILURE_CODES:
            s->alarmMask = dalyFailureMask(data);
            s->warningMask = ((uint32_t)data[4] << 16) |
                             ((uint32_t)data[5] << 8) |
                             (uint32_t)data[6];
            break;

        default:
            break;
    }
}

static uint8_t dalyCanResponseFrameCount(uint8_t cmd, const daly_rs485_snapshot_t *snapshot)
{
    if (cmd == DALY_CMD_CELL_VOLTAGES) {
        uint8_t cells = (snapshot != NULL && snapshot->cellCount > 0u) ? snapshot->cellCount : 16u;
        return (uint8_t)((cells + 2u) / 3u);
    }
    if (cmd == DALY_CMD_CELL_TEMPERATURES) {
        uint8_t temps = (snapshot != NULL && snapshot->tempCount > 0u) ? snapshot->tempCount : 2u;
        return (uint8_t)((temps + 6u) / 7u);
    }
    return 1u;
}

static esp_err_t dalyCanTransmitRequest(twai_handle_t bus,
                                        uint8_t cmd,
                                        uint8_t targetId,
                                        daly_can_request_variant_t variant)
{
    static const uint8_t empty[8] = DALY_CAN_EMPTY_DATA;
    twai_message_t tx = {0};
    bool isRtr = dalyCanVariantUsesRtr(variant);

    if (bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    tx.identifier = dalyCanRequestIdForVariant(cmd, targetId, variant);
    tx.data_length_code = dalyCanVariantUsesEmptyDlc(variant) ? 0u : 8u;
    if (!isRtr && tx.data_length_code > 0u) {
        memcpy(tx.data, empty, sizeof(empty));
    }
    tx.extd = 1u;
    tx.rtr = isRtr ? 1u : 0u;
#ifdef TWAI_MSG_FLAG_EXTD
    tx.flags |= TWAI_MSG_FLAG_EXTD;
#endif
#ifdef TWAI_MSG_FLAG_SS
    tx.flags |= TWAI_MSG_FLAG_SS;
#endif
#ifdef TWAI_MSG_FLAG_RTR
    if (isRtr) {
        tx.flags |= TWAI_MSG_FLAG_RTR;
    }
#endif

    return twai_transmit_v2(bus, &tx, pdMS_TO_TICKS(DALY_CAN_TX_TIMEOUT_MS));
}

static bool dalyCanEnsureBusRunning(daly_can_task_ctx_t *ctx)
{
    twai_status_info_t status = {0};
    int64_t nowUs = esp_timer_get_time();

    if (ctx == NULL || ctx->bus == NULL) {
        return false;
    }

    esp_err_t err = twai_get_status_info_v2(ctx->bus, &status);
    if (err != ESP_OK) {
        if ((nowUs - g_lastTxErrorLogUs) >= DALY_CAN_STATUS_LOG_INTERVAL_US) {
            ESP_LOGW(EXAMPLE_TAG,
                     "DALY CAN status read failed on %s err=0x%x",
                     ctx->ifName ? ctx->ifName : "CAN",
                     (unsigned)err);
            g_lastTxErrorLogUs = nowUs;
        }
        return false;
    }

    if (status.state == TWAI_STATE_RUNNING) {
        return true;
    }

    if (status.state == TWAI_STATE_BUS_OFF) {
        if ((nowUs - g_lastRecoveryUs) >= 500000LL) {
            esp_err_t recErr = twai_initiate_recovery_v2(ctx->bus);
            if ((nowUs - g_lastTxErrorLogUs) >= DALY_CAN_STATUS_LOG_INTERVAL_US) {
                ESP_LOGW(EXAMPLE_TAG,
                         "DALY CAN bus-off on %s; recovery start err=0x%x txErr=%" PRIu32 " rxErr=%" PRIu32 " busErr=%" PRIu32,
                         ctx->ifName ? ctx->ifName : "CAN",
                         (unsigned)recErr,
                         status.tx_error_counter,
                         status.rx_error_counter,
                         status.bus_error_count);
                g_lastTxErrorLogUs = nowUs;
            }
            g_lastRecoveryUs = nowUs;
        }
        return false;
    }

    if (status.state == TWAI_STATE_STOPPED) {
        esp_err_t startErr = twai_start_v2(ctx->bus);
        if ((nowUs - g_lastTxErrorLogUs) >= DALY_CAN_STATUS_LOG_INTERVAL_US) {
            ESP_LOGW(EXAMPLE_TAG,
                     "DALY CAN restart on %s from STOPPED: err=0x%x",
                     ctx->ifName ? ctx->ifName : "CAN",
                     (unsigned)startErr);
            g_lastTxErrorLogUs = nowUs;
        }
        return startErr == ESP_OK;
    }

    if ((nowUs - g_lastTxErrorLogUs) >= DALY_CAN_STATUS_LOG_INTERVAL_US) {
        ESP_LOGW(EXAMPLE_TAG,
                 "DALY CAN not running on %s: state=%s txErr=%" PRIu32 " rxErr=%" PRIu32,
                 ctx->ifName ? ctx->ifName : "CAN",
                 dalyCanStateStr(status.state),
                 status.tx_error_counter,
                 status.rx_error_counter);
        g_lastTxErrorLogUs = nowUs;
    }
    return false;
}

static void dalyCanLogTxFailure(daly_can_task_ctx_t *ctx,
                                uint8_t cmd,
                                uint8_t targetId,
                                daly_can_request_variant_t variant,
                                esp_err_t err)
{
    twai_status_info_t status = {0};
    int64_t nowUs = esp_timer_get_time();

    if ((nowUs - g_lastTxErrorLogUs) < DALY_CAN_STATUS_LOG_INTERVAL_US) {
        return;
    }
    g_lastTxErrorLogUs = nowUs;

    if (ctx != NULL && ctx->bus != NULL &&
        twai_get_status_info_v2(ctx->bus, &status) == ESP_OK) {
        ESP_LOGW(EXAMPLE_TAG,
                 "DALY CAN TX failed on %s cmd=0x%02X id=0x%08" PRIX32 " target=0x%02X variant=%s err=0x%x state=%s txErr=%" PRIu32 " rxErr=%" PRIu32 " busErr=%" PRIu32,
                 ctx->ifName ? ctx->ifName : "CAN",
                 cmd,
                 dalyCanRequestIdForVariant(cmd, targetId, variant),
                 (unsigned)targetId,
                 dalyCanRequestVariantName(variant),
                 (unsigned)err,
                 dalyCanStateStr(status.state),
                 status.tx_error_counter,
                 status.rx_error_counter,
                 status.bus_error_count);
        return;
    }

    ESP_LOGW(EXAMPLE_TAG,
             "DALY CAN TX failed cmd=0x%02X id=0x%08" PRIX32 " target=0x%02X variant=%s err=0x%x",
             cmd,
             dalyCanRequestIdForVariant(cmd, targetId, variant),
             (unsigned)targetId,
             dalyCanRequestVariantName(variant),
             (unsigned)err);
}

static void dalyCanLogProbeTxOk(daly_can_task_ctx_t *ctx,
                                uint8_t cmd,
                                uint8_t targetId,
                                daly_can_request_variant_t variant)
{
    twai_status_info_t status = {0};
    int64_t nowUs = esp_timer_get_time();

    if (ctx == NULL || (nowUs - g_lastTxOkLogUs) < DALY_CAN_STATUS_LOG_INTERVAL_US) {
        return;
    }
    g_lastTxOkLogUs = nowUs;

    if (ctx->bus != NULL && twai_get_status_info_v2(ctx->bus, &status) == ESP_OK) {
        ESP_LOGI(EXAMPLE_TAG,
                 "DALY CAN probe TX queued on %s cmd=0x%02X id=0x%08" PRIX32 " target=0x%02X variant=%s state=%s txErr=%" PRIu32 " rxErr=%" PRIu32 " busErr=%" PRIu32,
                 ctx->ifName ? ctx->ifName : "CAN",
                 cmd,
                 dalyCanRequestIdForVariant(cmd, targetId, variant),
                 (unsigned)targetId,
                 dalyCanRequestVariantName(variant),
                 dalyCanStateStr(status.state),
                 status.tx_error_counter,
                 status.rx_error_counter,
                 status.bus_error_count);
        return;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "DALY CAN probe TX queued cmd=0x%02X id=0x%08" PRIX32 " target=0x%02X variant=%s",
             cmd,
             dalyCanRequestIdForVariant(cmd, targetId, variant),
             (unsigned)targetId,
             dalyCanRequestVariantName(variant));
}

static bool dalyCanShouldLogStdId(uint32_t id)
{
    for (uint8_t i = 0u; i < g_rawStdIdsLoggedCount; i++) {
        if (g_rawStdIdsLogged[i] == id) {
            return false;
        }
    }

    if (g_rawStdIdsLoggedCount < DALY_CAN_LOGGED_STD_ID_MAX) {
        g_rawStdIdsLogged[g_rawStdIdsLoggedCount++] = id;
        return true;
    }

    return false;
}

static void dalyCanLogIgnoredFrame(daly_can_task_ctx_t *ctx,
                                   const twai_message_t *rx,
                                   const char *reason)
{
    int64_t nowUs = esp_timer_get_time();
    uint8_t dlc = 0u;
    char dataHex[24u + 1u] = {0};
    size_t pos = 0u;
    bool shouldLog = false;

    if (ctx == NULL || rx == NULL) {
        return;
    }

    dlc = (rx->data_length_code > 8u) ? 8u : rx->data_length_code;
    for (uint8_t i = 0u; i < dlc && pos + 3u < sizeof(dataHex); i++) {
        static const char hex[] = "0123456789ABCDEF";
        dataHex[pos++] = hex[(rx->data[i] >> 4) & 0x0Fu];
        dataHex[pos++] = hex[rx->data[i] & 0x0Fu];
        if (i + 1u < dlc) {
            dataHex[pos++] = ' ';
        }
    }
    dataHex[pos] = '\0';

#ifdef TWAI_MSG_FLAG_EXTD
    if ((rx->flags & TWAI_MSG_FLAG_EXTD) == 0u) {
#endif
        if (rx->identifier < 0x400u) {
            shouldLog = dalyCanShouldLogStdId((uint32_t)rx->identifier);
        }
#ifdef TWAI_MSG_FLAG_EXTD
    } else if (g_rawExtFramesLogged < 8u) {
        g_rawExtFramesLogged++;
        shouldLog = true;
    }
#endif

    if (!shouldLog && (nowUs - g_lastRxLogUs) < DALY_CAN_STATUS_LOG_INTERVAL_US) {
        return;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "DALY CAN ignored frame on %s: ID=0x%08" PRIX32 " DLC=%u data=[%s]"
#ifdef TWAI_MSG_FLAG_EXTD
             " flags=0x%08" PRIX32
#endif
             " reason=%s",
             ctx->ifName ? ctx->ifName : "CAN",
             (uint32_t)rx->identifier,
             (unsigned)rx->data_length_code,
             dataHex,
#ifdef TWAI_MSG_FLAG_EXTD
             (uint32_t)rx->flags,
#endif
             (reason != NULL) ? reason : "unexpected-id");
    g_lastRxLogUs = nowUs;
}

static bool dalyCanFrameIsExtended(const twai_message_t *rx)
{
    if (rx == NULL) {
        return false;
    }
#ifdef TWAI_MSG_FLAG_EXTD
    return ((rx->flags & TWAI_MSG_FLAG_EXTD) != 0u) || (rx->extd != 0u);
#else
    return rx->extd != 0u;
#endif
}

static void dalyCanLogOtherBusDiagnostics(const daly_can_task_ctx_t *ctx, int64_t nowUs)
{
    uint8_t otherPort = 0u;
    twai_handle_t otherBus = NULL;
    uint8_t seen = 0u;
    uint8_t extSeen = 0u;
    uint8_t stdSeen = 0u;
    uint8_t dalyLikeSeen = 0u;

    if (ctx == NULL || ctx->canPort == 0u ||
        (nowUs - g_lastOtherBusDiagUs) < DALY_CAN_OTHER_BUS_DIAG_INTERVAL_US) {
        return;
    }
    g_lastOtherBusDiagUs = nowUs;

    otherPort = (ctx->canPort == 2u) ? 1u : 2u;
    otherBus = dalyCanBusForPort(otherPort);
    if (otherBus == NULL) {
        return;
    }

    for (uint8_t sample = 0u; sample < 12u; sample++) {
        twai_message_t rx = {0};
        esp_err_t err = twai_receive_v2(otherBus, &rx, 0);
        if (err != ESP_OK) {
            break;
        }

        uint8_t dlc = (rx.data_length_code > 8u) ? 8u : rx.data_length_code;
        char dataHex[24u + 1u] = {0};
        size_t pos = 0u;
        for (uint8_t i = 0u; i < dlc && pos + 3u < sizeof(dataHex); i++) {
            static const char hex[] = "0123456789ABCDEF";
            dataHex[pos++] = hex[(rx.data[i] >> 4) & 0x0Fu];
            dataHex[pos++] = hex[rx.data[i] & 0x0Fu];
            if (i + 1u < dlc) {
                dataHex[pos++] = ' ';
            }
        }
        dataHex[pos] = '\0';

        bool ext = dalyCanFrameIsExtended(&rx);
        uint8_t cmd = (uint8_t)(((uint32_t)rx.identifier >> 16) & 0xFFu);
        bool dalyLike = ext &&
                        ((((uint32_t)rx.identifier >> 24) & 0xFFu) == DALY_CAN_FRAME_PRIO) &&
                        dalyCanCommandKnown(cmd);
        seen++;
        if (ext) {
            extSeen++;
        } else {
            stdSeen++;
        }
        if (dalyLike) {
            dalyLikeSeen++;
        }

        if (sample < 4u || dalyLike) {
            ESP_LOGW(EXAMPLE_TAG,
                     "DALY CAN other-bus diag while %s is stale: %s saw ID=0x%08" PRIX32 " DLC=%u data=[%s] ext=%u daly_like=%u",
                     ctx->ifName ? ctx->ifName : "CAN",
                     dalyCanIfNameForPort(otherPort),
                     (uint32_t)rx.identifier,
                     (unsigned)rx.data_length_code,
                     dataHex,
                     ext ? 1u : 0u,
                     dalyLike ? 1u : 0u);
        }
    }

    ESP_LOGW(EXAMPLE_TAG,
             "DALY CAN other-bus diag summary while %s is stale: %s frames=%u ext=%u std=%u daly_like=%u",
             ctx->ifName ? ctx->ifName : "CAN",
             dalyCanIfNameForPort(otherPort),
             (unsigned)seen,
             (unsigned)extSeen,
             (unsigned)stdSeen,
             (unsigned)dalyLikeSeen);
}

static bool dalyCanHandleReceivedFrame(daly_can_task_ctx_t *ctx,
                                       const twai_message_t *rx,
                                       uint8_t expectedCmd,
                                       uint8_t *matched)
{
    uint8_t rxCmd = 0u;

    if (ctx == NULL || rx == NULL) {
        return false;
    }

#ifdef TWAI_MSG_FLAG_SELF
    if ((rx->flags & TWAI_MSG_FLAG_SELF) != 0u) {
        return false;
    }
#endif

    if (rx->data_length_code < 8u) {
        dalyCanLogIgnoredFrame(ctx, rx, "short-dlc");
        return false;
    }

    if (expectedCmd != 0u &&
        dalyCanResponseMatches(ctx, (uint32_t)rx->identifier, expectedCmd)) {
        dalyCanDecodePayload(ctx, expectedCmd, rx->data);
        if (matched != NULL) {
            (*matched)++;
        }
        return true;
    }

    if (dalyCanLooksLikeResponse(ctx, (uint32_t)rx->identifier, &rxCmd) &&
        dalyCanCommandKnown(rxCmd)) {
        dalyCanDecodePayload(ctx, rxCmd, rx->data);
        return true;
    }

    dalyCanLogIgnoredFrame(ctx, rx, expectedCmd != 0u ? "unexpected-response-id" : "raw-unmatched");
    return false;
}

static uint8_t dalyCanDrainPendingFrames(daly_can_task_ctx_t *ctx, const char *phase)
{
    uint8_t drained = 0u;
    uint8_t decoded = 0u;

    if (ctx == NULL || ctx->bus == NULL) {
        return 0u;
    }

    for (uint8_t i = 0u; i < 8u; i++) {
        twai_message_t rx = {0};
        if (twai_receive_v2(ctx->bus, &rx, 0) != ESP_OK) {
            break;
        }
        drained++;
        if (dalyCanHandleReceivedFrame(ctx, &rx, 0u, NULL)) {
            decoded++;
        }
    }

    if (drained > 0u) {
        int64_t nowUs = esp_timer_get_time();
        if ((nowUs - g_lastRxLogUs) >= DALY_CAN_STATUS_LOG_INTERVAL_US) {
            ESP_LOGI(EXAMPLE_TAG,
                     "DALY CAN drained %u pending frame(s) on %s phase=%s decoded=%u",
                     (unsigned)drained,
                     ctx->ifName ? ctx->ifName : "CAN",
                     phase ? phase : "-",
                     (unsigned)decoded);
            g_lastRxLogUs = nowUs;
        }
    }

    return drained;
}

static uint8_t dalyCanSniffOnlyWindow(daly_can_task_ctx_t *ctx, uint32_t windowMs, const char *phase)
{
    uint8_t seen = 0u;
    int64_t deadlineUs = 0;

    if (ctx == NULL || ctx->bus == NULL) {
        return 0u;
    }

    deadlineUs = esp_timer_get_time() + ((int64_t)windowMs * 1000LL);
    while (esp_timer_get_time() < deadlineUs) {
        twai_message_t rx = {0};
        esp_err_t err = twai_receive_v2(ctx->bus, &rx, pdMS_TO_TICKS(20));
        if (err != ESP_OK) {
            continue;
        }
        seen++;
        (void)dalyCanHandleReceivedFrame(ctx, &rx, 0u, NULL);
        if (seen >= 32u) {
            break;
        }
    }

    ESP_LOGI(EXAMPLE_TAG,
             "DALY CAN passive sniff on %s phase=%s window=%" PRIu32 "ms frames=%u",
             ctx->ifName ? ctx->ifName : "CAN",
             phase ? phase : "-",
             windowMs,
             (unsigned)seen);
    return seen;
}

static uint8_t dalyCanPollCommand(daly_can_task_ctx_t *ctx, uint8_t cmd)
{
    uint8_t matched = 0u;
    uint8_t expected = dalyCanResponseFrameCount(cmd, &ctx->snapshot);
    int64_t deadlineUs = 0;
    daly_can_request_variant_t variants[DALY_CAN_REQ_VARIANT_COUNT] = {
        DALY_CAN_REQ_DLC8_ZERO,
        DALY_CAN_REQ_DLC8_ZERO_HOST92,
        DALY_CAN_REQ_DLC8_ZERO_HOST21,
        DALY_CAN_REQ_DLC8_ZERO_HOST80,
        DALY_CAN_REQ_DLC8_ZERO_HOST20,
        DALY_CAN_REQ_DLC8_ZERO_HOST24,
        DALY_CAN_REQ_DLC0_HOST40,
        DALY_CAN_REQ_RTR_HOST40,
    };
    uint8_t variantCount = DALY_CAN_REQ_VARIANT_COUNT;
    uint8_t targetCount = 1u;

    if (ctx == NULL || ctx->bus == NULL) {
        return 0u;
    }

    if (!dalyCanEnsureBusRunning(ctx)) {
        return 0u;
    }

    int64_t nowUs = esp_timer_get_time();
    bool staleProbe = ctx->lastFrameUs <= 0 ||
                      (nowUs - ctx->lastFrameUs) > ((int64_t)DALY_CAN_SOURCE_STALE_MS * 1000LL);
    if (ctx->havePreferredRequestVariant) {
        variants[0] = ctx->preferredRequestVariant;
        variantCount = 1u;
    } else if (!staleProbe) {
        variantCount = 2u;
    } else {
        targetCount = dalyCanProbeTargetCount();
    }

    for (uint8_t targetIdx = 0u; targetIdx < targetCount && matched == 0u; targetIdx++) {
        uint8_t targetId = (targetCount > 1u) ? dalyCanProbeTargetAt(targetIdx) : dalyCanActiveTargetId(ctx);

        for (uint8_t variantIdx = 0u; variantIdx < variantCount && matched == 0u; variantIdx++) {
            daly_can_request_variant_t variant = variants[variantIdx];
            (void)dalyCanDrainPendingFrames(ctx, "pre-tx");
            ctx->pendingTargetId = targetId;
            ctx->havePendingTargetId = true;
            esp_err_t err = dalyCanTransmitRequest(ctx->bus, cmd, targetId, variant);
            if (err != ESP_OK) {
                ctx->havePendingTargetId = false;
                ctx->pendingTargetId = 0u;
                dalyCanLogTxFailure(ctx, cmd, targetId, variant, err);
                (void)twai_clear_transmit_queue_v2(ctx->bus);
                (void)dalyCanDrainPendingFrames(ctx, "tx-fail");
                if (err == ESP_ERR_INVALID_STATE) {
                    (void)dalyCanEnsureBusRunning(ctx);
                }
                continue;
            }
            if (staleProbe && cmd == DALY_CMD_VOUT_IOUT_SOC) {
                dalyCanLogProbeTxOk(ctx, cmd, targetId, variant);
            }

            deadlineUs = esp_timer_get_time() + ((int64_t)DALY_CAN_RESPONSE_TIMEOUT_MS * 1000LL);
            while (esp_timer_get_time() < deadlineUs && matched < expected) {
                twai_message_t rx = {0};
                uint8_t before = matched;
                err = twai_receive_v2(ctx->bus, &rx, pdMS_TO_TICKS(20));
                if (err != ESP_OK) {
                    continue;
                }
                (void)dalyCanHandleReceivedFrame(ctx, &rx, cmd, &matched);
                if (matched > before && !ctx->havePreferredRequestVariant) {
                    ctx->preferredRequestVariant = variant;
                    ctx->havePreferredRequestVariant = true;
                    ESP_LOGI(EXAMPLE_TAG,
                             "DALY CAN learned request route on %s: target=0x%02X variant=%s",
                             ctx->ifName ? ctx->ifName : "CAN",
                             (unsigned)targetId,
                             dalyCanRequestVariantName(variant));
                }
            }
            ctx->havePendingTargetId = false;
            ctx->pendingTargetId = 0u;
        }
    }

    return matched;
}

static void dalyCanUpdateExtremesFromCells(daly_rs485_snapshot_t *s)
{
    if (s == NULL || s->cellCount == 0u) {
        return;
    }

    uint16_t minMv = UINT16_MAX;
    uint16_t maxMv = 0u;
    uint8_t minIdx = 1u;
    uint8_t maxIdx = 1u;
    uint32_t sumMv = 0u;

    for (uint8_t i = 0u; i < s->cellCount; i++) {
        uint16_t mv = s->cellMv[i];
        if (mv == 0u) {
            continue;
        }
        sumMv += mv;
        if (mv < minMv) {
            minMv = mv;
            minIdx = (uint8_t)(i + 1u);
        }
        if (mv > maxMv) {
            maxMv = mv;
            maxIdx = (uint8_t)(i + 1u);
        }
    }

    if (maxMv == 0u || minMv == UINT16_MAX) {
        return;
    }

    s->hasCellExtremes = true;
    s->minCellMv = minMv;
    s->maxCellMv = maxMv;
    s->minCellIndex = minIdx;
    s->maxCellIndex = maxIdx;
    if (!s->hasPackVoltageCv) {
        s->hasPackVoltageCv = true;
        s->packVoltageCv = (uint16_t)((sumMv + 5u) / 10u);
    }
}

static void dalyCanPublishBatteryModel(const daly_rs485_snapshot_t *snapshot)
{
    battery_model_t model = {0};

    if (snapshot == NULL || !snapshot->valid) {
        return;
    }

    model.valid = true;
    model.updatedMs = (uint32_t)(esp_timer_get_time() / 1000LL);
    model.sohPct = 100u;

    if (snapshot->hasPackVoltageCv) {
        model.packVoltageV = (float)snapshot->packVoltageCv / 100.0f;
    }
    if (snapshot->hasCurrentDeciA) {
        model.packCurrentA = (float)snapshot->currentDeciA / 10.0f;
    }
    if (snapshot->hasSocDeciPct) {
        model.socPct = clampPctFromDeci(snapshot->socDeciPct);
    }
    if (snapshot->hasCycles) {
        model.cycleCount = snapshot->cycles;
    }
    if (snapshot->hasCellExtremes) {
        model.cellMaxV = (float)snapshot->maxCellMv / 1000.0f;
        model.cellMinV = (float)snapshot->minCellMv / 1000.0f;
        model.cellMaxIdx = snapshot->maxCellIndex;
        model.cellMinIdx = snapshot->minCellIndex;
        model.cellDeltaV = (float)(snapshot->maxCellMv - snapshot->minCellMv) / 1000.0f;
    }
    for (uint8_t i = 0u; i < UNIVERSAL_BATTERY_TEMP_SENSORS; i++) {
        model.temperaturesC[i] = -100.0f;
    }
    for (uint8_t i = 0u; i < snapshot->tempCount && i < UNIVERSAL_BATTERY_TEMP_SENSORS; i++) {
        model.temperaturesC[i] = (float)snapshot->tempDeciC[i] / 10.0f;
    }
    model.chargeEnabled = snapshot->chargeEnabled;
    model.dischargeEnabled = snapshot->dischargeEnabled;
    model.balanceEnabled = snapshot->balanceEnabled;
    model.alarmsMask = snapshot->alarmMask;
    model.warningsMask = snapshot->warningMask;
    model.protocolState =
        (snapshot->chargeEnabled ? 0x80u : 0u) |
        (snapshot->dischargeEnabled ? 0x40u : 0u) |
        (snapshot->balanceEnabled ? 0x20u : 0u);

    batteryModelSet(&model);
}

static void dalyCanStoreLatest(const daly_rs485_snapshot_t *snapshot,
                               const bms_decoded_packet_t *packet)
{
    portENTER_CRITICAL(&g_latestMux);
    if (snapshot != NULL) {
        g_latestSnapshot = *snapshot;
        g_haveLatestSnapshot = true;
    }
    if (packet != NULL) {
        g_latestPacket = *packet;
        g_haveLatestPacket = true;
    }
    portEXIT_CRITICAL(&g_latestMux);
}

static void dalyCanClearLatest(void)
{
    portENTER_CRITICAL(&g_latestMux);
    g_haveLatestSnapshot = false;
    g_haveLatestPacket = false;
    memset(&g_latestSnapshot, 0, sizeof(g_latestSnapshot));
    memset(&g_latestPacket, 0, sizeof(g_latestPacket));
    portEXIT_CRITICAL(&g_latestMux);
}

static void dalyCanLogSnapshot(const daly_rs485_snapshot_t *s)
{
    int64_t nowUs = esp_timer_get_time();

    if ((nowUs - g_lastDecodeLogUs) < DALY_CAN_DECODE_LOG_INTERVAL_US || s == NULL) {
        return;
    }
    g_lastDecodeLogUs = nowUs;

    ESP_LOGI(EXAMPLE_TAG,
             "DALY CAN decoded: soc=%u.%u%% pack=%.2fV current=%.1fA cells=%u tempCount=%u min=%.3fV#%u max=%.3fV#%u chg=%s dsg=%s bal=%s alarms=0x%08" PRIX32,
             s->hasSocDeciPct ? (unsigned)(s->socDeciPct / 10u) : 0u,
             s->hasSocDeciPct ? (unsigned)(s->socDeciPct % 10u) : 0u,
             s->hasPackVoltageCv ? ((double)s->packVoltageCv / 100.0) : 0.0,
             s->hasCurrentDeciA ? ((double)s->currentDeciA / 10.0) : 0.0,
             (unsigned)s->cellCount,
             (unsigned)s->tempCount,
             s->hasCellExtremes ? ((double)s->minCellMv / 1000.0) : 0.0,
             s->hasCellExtremes ? (unsigned)s->minCellIndex : 0u,
             s->hasCellExtremes ? ((double)s->maxCellMv / 1000.0) : 0.0,
             s->hasCellExtremes ? (unsigned)s->maxCellIndex : 0u,
             s->chargeEnabled ? "ON" : "OFF",
             s->dischargeEnabled ? "ON" : "OFF",
             s->balanceEnabled ? "ON" : "OFF",
             s->alarmMask);
}

static void dalyCanLogStaleStatus(daly_can_task_ctx_t *ctx, uint8_t matchedThisRound, int64_t nowUs)
{
    twai_status_info_t status = {0};

    if (ctx == NULL || (nowUs - g_lastStaleLogUs) < DALY_CAN_STATUS_LOG_INTERVAL_US) {
        return;
    }

    g_lastStaleLogUs = nowUs;
    if (ctx->bus != NULL && twai_get_status_info_v2(ctx->bus, &status) == ESP_OK) {
        ESP_LOGW(EXAMPLE_TAG,
             "DALY CAN source stale on %s: no valid frames (matched_round=%u active_target=0x%02X default=0x%02X alt=0x%02X state=%s txErr=%" PRIu32 " rxErr=%" PRIu32 " txFail=%" PRIu32 " rxMiss=%" PRIu32 " busErr=%" PRIu32 ")",
                 ctx->ifName ? ctx->ifName : "CAN",
                 (unsigned)matchedThisRound,
                 (unsigned)dalyCanActiveTargetId(ctx),
                 (unsigned)dalyCanDefaultTargetId(),
                 (unsigned)dalyCanAlternateTargetId(dalyCanDefaultTargetId()),
                 dalyCanStateStr(status.state),
                 status.tx_error_counter,
                 status.rx_error_counter,
                 status.tx_failed_count,
                 status.rx_missed_count,
                 status.bus_error_count);
        if (!ctx->havePreferredRequestVariant) {
            ESP_LOGW(EXAMPLE_TAG,
                     "DALY CAN has not learned a response yet; canonical poll cmd=0x%02X req=0x%08" PRIX32 " expected_rsp=0x%08" PRIX32 " variant=%s bitrate=%lu",
                     DALY_CMD_VOUT_IOUT_SOC,
                     dalyCanRequestIdFor(DALY_CMD_VOUT_IOUT_SOC, dalyCanDefaultTargetId()),
                     dalyCanResponseIdFor(DALY_CMD_VOUT_IOUT_SOC, dalyCanDefaultTargetId()),
                     dalyCanRequestVariantName(DALY_CAN_REQ_DLC8_ZERO),
                     (unsigned long)ctx->bitrate);
            ESP_LOGW(EXAMPLE_TAG,
                     "DALY CAN probe variants include host40/host92/host21/host80/host20/host24 DLC8 plus host40 DLC0/RTR; fixed bitrate unless explicitly restarted with another DALY_CAN protocol");
            ESP_LOGW(EXAMPLE_TAG,
                     "DALY CAN target probe on %s: count=%u target0=0x%02X target1=0x%02X target2=0x%02X",
                     ctx->ifName ? ctx->ifName : "CAN",
                     (unsigned)dalyCanProbeTargetCount(),
                     (unsigned)dalyCanProbeTargetAt(0u),
                     (unsigned)dalyCanProbeTargetAt(1u),
                     (unsigned)dalyCanProbeTargetAt(2u));
        }
        return;
    }

    ESP_LOGW(EXAMPLE_TAG,
             "DALY CAN source stale on %s: no valid Daly CAN frames yet (matched_round=%u active_target=0x%02X)",
             ctx->ifName ? ctx->ifName : "CAN",
             (unsigned)matchedThisRound,
             (unsigned)dalyCanActiveTargetId(ctx));
}

static void dalyCanTask(void *pv)
{
    daly_can_task_ctx_t *ctx = (daly_can_task_ctx_t *)pv;
    static const uint8_t initCmds[] = {
        DALY_CMD_RATED_CAPACITY_CELL_VOLTAGE,
        DALY_CMD_BATTERY_TYPE_INFO,
        DALY_CMD_MIN_MAX_PACK_VOLTAGE,
        DALY_CMD_MAX_DISCHARGE_CHARGE_CURRENT,
    };
    static const uint8_t runCmds[] = {
        DALY_CMD_VOUT_IOUT_SOC,
        DALY_CMD_MIN_MAX_CELL_VOLTAGE,
        DALY_CMD_MIN_MAX_TEMPERATURE,
        DALY_CMD_MOS_STATUS,
        DALY_CMD_STATUS_INFO,
        DALY_CMD_CELL_VOLTAGES,
        DALY_CMD_CELL_TEMPERATURES,
        DALY_CMD_CELL_BALANCE_STATE,
        DALY_CMD_FAILURE_CODES,
    };

    ESP_LOGI(EXAMPLE_TAG,
             "DALY CAN BMS task active on %s (target=0x%02X req_0x90=0x%08" PRIX32 " expected_rsp_0x90=0x%08" PRIX32 ")",
             ctx->ifName ? ctx->ifName : "CAN",
             (unsigned)dalyCanDefaultTargetId(),
             dalyCanRequestIdFor(DALY_CMD_VOUT_IOUT_SOC, dalyCanDefaultTargetId()),
             dalyCanResponseIdFor(DALY_CMD_VOUT_IOUT_SOC, dalyCanDefaultTargetId()));

    (void)dalyCanSniffOnlyWindow(ctx, 2000u, "startup");

    for (uint8_t i = 0u; i < (sizeof(initCmds) / sizeof(initCmds[0])); i++) {
        (void)dalyCanPollCommand(ctx, initCmds[i]);
        vTaskDelay(pdMS_TO_TICKS(DALY_CAN_QUERY_PERIOD_MS));
    }

    while (1) {
        uint8_t matchedThisRound = 0u;

        for (uint8_t i = 0u; i < (sizeof(runCmds) / sizeof(runCmds[0])); i++) {
            matchedThisRound = (uint8_t)(matchedThisRound + dalyCanPollCommand(ctx, runCmds[i]));
            vTaskDelay(pdMS_TO_TICKS(DALY_CAN_QUERY_PERIOD_MS));
        }

        if (ctx->snapshot.cellCount > 0u) {
            dalyCanUpdateExtremesFromCells(&ctx->snapshot);
        }

        int64_t nowUs = esp_timer_get_time();
        bool fresh = ctx->lastFrameUs > 0 &&
                     (nowUs - ctx->lastFrameUs) <= ((int64_t)DALY_CAN_SOURCE_STALE_MS * 1000LL);

        if (!fresh) {
            batteryModelClear();
            dalyCanClearLatest();
            dalyCanLogStaleStatus(ctx, matchedThisRound, nowUs);
            dalyCanLogOtherBusDiagnostics(ctx, nowUs);
            if (dalyCanMaybeSwitchBitrate(ctx, nowUs)) {
                (void)dalyCanSniffOnlyWindow(ctx, 700u, "bitrate-switch");
                continue;
            }
            (void)dalyCanSniffOnlyWindow(ctx, 500u, "stale");
            continue;
        }

        if ((nowUs - ctx->lastPublishUs) >= ((int64_t)DALY_CAN_PUBLISH_PERIOD_MS * 1000LL)) {
            bms_decoded_packet_t packet = {0};
            ctx->snapshot.sequence = ++ctx->sequence;
            if (dalyRs485BuildDecodedPacket(&ctx->snapshot, ctx->sequence, &packet)) {
                dalyCanPublishBatteryModel(&ctx->snapshot);
                dalyCanStoreLatest(&ctx->snapshot, &packet);
                dalyCanLogSnapshot(&ctx->snapshot);
                if (ctx->outQueue != NULL && xQueueOverwrite(ctx->outQueue, &packet) != pdPASS) {
                    ESP_LOGW(EXAMPLE_TAG, "DALY CAN output queue overwrite failed");
                }
            }
            ctx->lastPublishUs = nowUs;
        }
    }
}

esp_err_t dalyCanBmsTaskStart(QueueHandle_t outQueue,
                              const bridge_runtime_settings_t *settings)
{
    if (outQueue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (settings == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_dalyCanTaskHandle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t bmsPort = (settings->bms_port == 2u) ? 2u : 1u;
    uint32_t bitrate = bridgeProtocolCanBitrate(settings->bms_protocol);
    esp_err_t err = canReinitPortMode(bmsPort, bitrate, TWAI_MODE_NORMAL);
    if (err != ESP_OK) {
        ESP_LOGW(EXAMPLE_TAG,
                 "DALY CAN failed to put CAN%u in normal poll mode (%lu bit/s): err=0x%x",
                 (unsigned)bmsPort,
                 (unsigned long)bitrate,
                 (unsigned)err);
        return err;
    }

    twai_handle_t bus = (bmsPort == 2u) ? canGetBus1() : canGetBus0();
    if (bus == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&g_dalyCanCtx, 0, sizeof(g_dalyCanCtx));
    g_dalyCanCtx.outQueue = outQueue;
    g_dalyCanCtx.bus = bus;
    g_dalyCanCtx.ifName = dalyCanIfNameForPort(bmsPort);
    g_dalyCanCtx.canPort = bmsPort;
    g_dalyCanCtx.bitrate = bitrate;
    g_dalyCanCtx.lastBitrateSwitchUs = esp_timer_get_time();
    /* Keep the selected Daly profile stable; the UI exposes separate 250k and 500k choices. */
    g_dalyCanCtx.allowBitrateAutoprobe = false;
    g_dalyCanCtx.activeTargetId = dalyCanDefaultTargetId();
    g_dalyCanCtx.haveActiveTargetId = true;
    g_lastStaleLogUs = 0;
    g_lastDecodeLogUs = 0;
    g_lastRxLogUs = 0;
    g_lastTxErrorLogUs = 0;
    g_lastTxOkLogUs = 0;
    g_lastRecoveryUs = 0;
    g_lastOtherBusDiagUs = 0;
    memset(g_rawStdIdsLogged, 0, sizeof(g_rawStdIdsLogged));
    g_rawStdIdsLoggedCount = 0u;
    g_rawExtFramesLogged = 0;
    batteryModelClear();
    dalyCanClearLatest();
    (void)twai_clear_receive_queue_v2(bus);

    ESP_LOGI(EXAMPLE_TAG,
             "DALY CAN start settings: bms_port=%u bus=%s bms_protocol=%u bitrate=%lu mode=NORMAL auto_bitrate=%s inverter_port=%u inverter_protocol=%u",
             (unsigned)bmsPort,
             g_dalyCanCtx.ifName,
             (unsigned)settings->bms_protocol,
             (unsigned long)bitrate,
             g_dalyCanCtx.allowBitrateAutoprobe ? "YES" : "NO",
             (unsigned)settings->inverter_port,
             (unsigned)settings->inverter_protocol);

    BaseType_t ok = xTaskCreate(dalyCanTask,
                                "daly_can",
                                DALY_CAN_TASK_STACK,
                                &g_dalyCanCtx,
                                DALY_CAN_TASK_PRIORITY,
                                &g_dalyCanTaskHandle);
    if (ok != pdPASS) {
        g_dalyCanTaskHandle = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t dalyCanBmsTaskStop(void)
{
    if (g_dalyCanTaskHandle != NULL) {
        vTaskDelete(g_dalyCanTaskHandle);
        g_dalyCanTaskHandle = NULL;
    }
    memset(&g_dalyCanCtx, 0, sizeof(g_dalyCanCtx));
    dalyCanClearLatest();
    return ESP_OK;
}

bool dalyCanBmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket)
{
    bool hasPacket = false;

    if (outPacket == NULL) {
        return false;
    }

    portENTER_CRITICAL(&g_latestMux);
    hasPacket = g_haveLatestPacket;
    if (hasPacket) {
        *outPacket = g_latestPacket;
    }
    portEXIT_CRITICAL(&g_latestMux);

    return hasPacket;
}

bool dalyCanBmsTaskGetLatestSnapshot(daly_rs485_snapshot_t *outSnapshot)
{
    bool hasSnapshot = false;

    if (outSnapshot == NULL) {
        return false;
    }

    portENTER_CRITICAL(&g_latestMux);
    hasSnapshot = g_haveLatestSnapshot;
    if (hasSnapshot) {
        *outSnapshot = g_latestSnapshot;
    }
    portEXIT_CRITICAL(&g_latestMux);

    return hasSnapshot;
}
