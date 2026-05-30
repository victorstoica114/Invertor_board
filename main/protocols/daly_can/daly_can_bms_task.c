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
#define DALY_CAN_FRAME_PRIO 0x18u
#define DALY_CAN_EMPTY_DATA {0, 0, 0, 0, 0, 0, 0, 0}
#define DALY_CAN_STATUS_LOG_INTERVAL_US 5000000LL
#define DALY_CAN_DECODE_LOG_INTERVAL_US 1000000LL

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

typedef struct {
    QueueHandle_t outQueue;
    twai_handle_t bus;
    const char *ifName;
    uint32_t sequence;
    int64_t lastFrameUs;
    int64_t lastPublishUs;
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
static int64_t g_lastRecoveryUs;

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

static uint32_t dalyCanRequestId(uint8_t cmd)
{
    return ((uint32_t)DALY_CAN_FRAME_PRIO << 24) |
           ((uint32_t)cmd << 16) |
           ((uint32_t)DALY_CAN_BMS_ID << 8) |
           (uint32_t)DALY_CAN_HOST_ID;
}

static bool dalyCanResponseMatches(uint32_t id, uint8_t cmd)
{
    return (((id >> 24) & 0xFFu) == DALY_CAN_FRAME_PRIO) &&
           (((id >> 16) & 0xFFu) == cmd) &&
           (((id >> 8) & 0xFFu) == DALY_CAN_HOST_ID) &&
           ((id & 0xFFu) == DALY_CAN_BMS_ID);
}

static bool dalyCanLooksLikeResponse(uint32_t id, uint8_t *cmdOut)
{
    if (((id >> 24) & 0xFFu) != DALY_CAN_FRAME_PRIO ||
        ((id >> 8) & 0xFFu) != DALY_CAN_HOST_ID ||
        (id & 0xFFu) != DALY_CAN_BMS_ID) {
        return false;
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
            if (frameNo == 0u) {
                break;
            }
            uint8_t firstCell = (uint8_t)((frameNo - 1u) * 3u);
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
            if (frameNo == 0u) {
                break;
            }
            uint8_t firstTemp = (uint8_t)((frameNo - 1u) * 7u);
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

static esp_err_t dalyCanTransmitRequest(twai_handle_t bus, uint8_t cmd)
{
    static const uint8_t empty[8] = DALY_CAN_EMPTY_DATA;
    twai_message_t tx = {0};

    if (bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    tx.identifier = dalyCanRequestId(cmd);
    tx.data_length_code = 8u;
    memcpy(tx.data, empty, sizeof(empty));
#ifdef TWAI_MSG_FLAG_EXTD
    tx.flags = TWAI_MSG_FLAG_EXTD;
#endif
#ifdef TWAI_MSG_FLAG_SS
    tx.flags |= TWAI_MSG_FLAG_SS;
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

static void dalyCanLogTxFailure(daly_can_task_ctx_t *ctx, uint8_t cmd, esp_err_t err)
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
                 "DALY CAN TX failed on %s cmd=0x%02X id=0x%08" PRIX32 " err=0x%x state=%s txErr=%" PRIu32 " rxErr=%" PRIu32 " busErr=%" PRIu32,
                 ctx->ifName ? ctx->ifName : "CAN",
                 cmd,
                 dalyCanRequestId(cmd),
                 (unsigned)err,
                 dalyCanStateStr(status.state),
                 status.tx_error_counter,
                 status.rx_error_counter,
                 status.bus_error_count);
        return;
    }

    ESP_LOGW(EXAMPLE_TAG,
             "DALY CAN TX failed cmd=0x%02X id=0x%08" PRIX32 " err=0x%x",
             cmd,
             dalyCanRequestId(cmd),
             (unsigned)err);
}

static uint8_t dalyCanPollCommand(daly_can_task_ctx_t *ctx, uint8_t cmd)
{
    uint8_t matched = 0u;
    uint8_t expected = dalyCanResponseFrameCount(cmd, &ctx->snapshot);
    int64_t deadlineUs = 0;

    if (ctx == NULL || ctx->bus == NULL) {
        return 0u;
    }

    if (!dalyCanEnsureBusRunning(ctx)) {
        return 0u;
    }

    (void)twai_clear_receive_queue_v2(ctx->bus);
    esp_err_t err = dalyCanTransmitRequest(ctx->bus, cmd);
    if (err != ESP_OK) {
        dalyCanLogTxFailure(ctx, cmd, err);
        if (err == ESP_ERR_INVALID_STATE) {
            (void)dalyCanEnsureBusRunning(ctx);
        }
        return 0u;
    }

    deadlineUs = esp_timer_get_time() + ((int64_t)DALY_CAN_RESPONSE_TIMEOUT_MS * 1000LL);
    while (esp_timer_get_time() < deadlineUs && matched < expected) {
        twai_message_t rx = {0};
        err = twai_receive_v2(ctx->bus, &rx, pdMS_TO_TICKS(20));
        if (err != ESP_OK) {
            continue;
        }
#ifdef TWAI_MSG_FLAG_SELF
        if ((rx.flags & TWAI_MSG_FLAG_SELF) != 0u) {
            continue;
        }
#endif
        if (rx.data_length_code < 8u) {
            continue;
        }

        uint8_t rxCmd = 0u;
        if (dalyCanResponseMatches((uint32_t)rx.identifier, cmd)) {
            dalyCanDecodePayload(ctx, cmd, rx.data);
            matched++;
            continue;
        }

        if (dalyCanLooksLikeResponse((uint32_t)rx.identifier, &rxCmd) &&
            dalyCanCommandKnown(rxCmd)) {
            dalyCanDecodePayload(ctx, rxCmd, rx.data);
            continue;
        }

        int64_t nowUs = esp_timer_get_time();
        if ((nowUs - g_lastRxLogUs) >= DALY_CAN_STATUS_LOG_INTERVAL_US) {
            ESP_LOGI(EXAMPLE_TAG,
                     "DALY CAN ignored frame on %s: ID=0x%08" PRIX32 " DLC=%u",
                     ctx->ifName ? ctx->ifName : "CAN",
                     (uint32_t)rx.identifier,
                     (unsigned)rx.data_length_code);
            g_lastRxLogUs = nowUs;
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
                 "DALY CAN source stale on %s: no valid frames (matched_round=%u bms_id=%u state=%s txErr=%" PRIu32 " rxErr=%" PRIu32 " txFail=%" PRIu32 " rxMiss=%" PRIu32 " busErr=%" PRIu32 ")",
                 ctx->ifName ? ctx->ifName : "CAN",
                 (unsigned)matchedThisRound,
                 (unsigned)DALY_CAN_BMS_ID,
                 dalyCanStateStr(status.state),
                 status.tx_error_counter,
                 status.rx_error_counter,
                 status.tx_failed_count,
                 status.rx_missed_count,
                 status.bus_error_count);
        return;
    }

    ESP_LOGW(EXAMPLE_TAG,
             "DALY CAN source stale on %s: no valid Daly CAN frames yet (matched_round=%u bms_id=%u)",
             ctx->ifName ? ctx->ifName : "CAN",
             (unsigned)matchedThisRound,
             (unsigned)DALY_CAN_BMS_ID);
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
             "DALY CAN BMS task active on %s (bms_id=%u request_id_0x90=0x%08" PRIX32 ")",
             ctx->ifName ? ctx->ifName : "CAN",
             (unsigned)DALY_CAN_BMS_ID,
             dalyCanRequestId(DALY_CMD_VOUT_IOUT_SOC));

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

esp_err_t dalyCanBmsTaskStart(QueueHandle_t outQueue)
{
    if (outQueue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_dalyCanTaskHandle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    bridge_runtime_settings_t settings = runtimeSettingsGet();
    uint8_t bmsPort = (settings.bms_port == 2u) ? 2u : 1u;
    twai_handle_t bus = (bmsPort == 2u) ? canGetBus1() : canGetBus0();
    if (bus == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&g_dalyCanCtx, 0, sizeof(g_dalyCanCtx));
    g_dalyCanCtx.outQueue = outQueue;
    g_dalyCanCtx.bus = bus;
    g_dalyCanCtx.ifName = (bmsPort == 2u) ? "CAN2" : "CAN1";
    g_lastStaleLogUs = 0;
    g_lastDecodeLogUs = 0;
    g_lastRxLogUs = 0;
    g_lastTxErrorLogUs = 0;
    g_lastRecoveryUs = 0;
    batteryModelClear();
    dalyCanClearLatest();
    (void)twai_clear_receive_queue_v2(bus);

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
