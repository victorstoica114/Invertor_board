#include "protocols/pylon/pylon_can_diag_sender.h"

#include <inttypes.h>
#include <string.h>

#include "config.h"
#include "protocols/pylon/pylon_registers_map.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    twai_handle_t bus;
    const char *ifName;
    uint32_t txCallsOk;
    uint32_t txCallsFail;
} pylon_can_diag_sender_ctx_t;

static pylon_can_diag_sender_ctx_t s_diagCtx;
static TaskHandle_t s_diagTask;

static const char *twaiStateName(twai_state_t state)
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

static void putLe16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static esp_err_t sendFrame(pylon_can_diag_sender_ctx_t *ctx, uint32_t id, const uint8_t data[8])
{
    twai_message_t tx = {0};

    if (ctx == NULL || ctx->bus == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    tx.identifier = id;
    tx.data_length_code = 8u;
    memcpy(tx.data, data, 8u);
#ifdef TWAI_MSG_FLAG_SS
    tx.flags |= TWAI_MSG_FLAG_SS;
#endif

    esp_err_t err = twai_transmit_v2(ctx->bus, &tx, pdMS_TO_TICKS(50));
    if (err == ESP_OK) {
        ctx->txCallsOk++;
    } else {
        ctx->txCallsFail++;
    }
    return err;
}

static void sendPylon24vSet(pylon_can_diag_sender_ctx_t *ctx)
{
    uint8_t f351[8] = {0};
    uint8_t f355[8] = {0};
    uint8_t f356[8] = {0};
    const uint8_t f359[8] = {0x00u, 0x00u, 0x00u, 0x00u, 0x01u, 0x50u, 0x4Eu, 0x00u};
    const uint8_t f35c[8] = {0xC0u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u};
    const uint8_t f35e[8] = {'P', 'Y', 'L', 'O', 'N', ' ', ' ', ' '};
    uint8_t f373[8] = {0};

    /* 8S/24V LiFePO4-safe synthetic values for an inverter-side Pylon test. */
    putLe16(&f351[PYLON_CAN_351_OFF_CHG_VLIM_DV], 292u);  /* 29.2 V */
    putLe16(&f351[PYLON_CAN_351_OFF_CHG_ILIM_DA], 400u);  /* 40.0 A */
    putLe16(&f351[PYLON_CAN_351_OFF_DIS_ILIM_DA], 400u);  /* 40.0 A */
    putLe16(&f351[PYLON_CAN_351_OFF_DIS_VLIM_DV], 200u);  /* 20.0 V */

    putLe16(&f355[PYLON_CAN_355_OFF_SOC_PCT], 100u);
    putLe16(&f355[PYLON_CAN_355_OFF_SOH_PCT], 100u);

    putLe16(&f356[PYLON_CAN_356_OFF_PACK_V_CV], 2640u);   /* 26.40 V */
    putLe16(&f356[PYLON_CAN_356_OFF_PACK_I_DA], 0u);      /* 0.0 A */
    putLe16(&f356[PYLON_CAN_356_OFF_TEMP_DECIC], 250u);   /* 25.0 C */

    putLe16(&f373[PYLON_CAN_373_OFF_CELL_MIN_MV], 3290u);
    putLe16(&f373[PYLON_CAN_373_OFF_CELL_MAX_MV], 3300u);
    putLe16(&f373[PYLON_CAN_373_OFF_TEMP1_DECIC], 250u);
    putLe16(&f373[PYLON_CAN_373_OFF_TEMP2_DECIC], 250u);

    (void)sendFrame(ctx, PYLON_CAN_ID_LIMITS_351, f351);
    (void)sendFrame(ctx, PYLON_CAN_ID_SOC_SOH_355, f355);
    (void)sendFrame(ctx, PYLON_CAN_ID_PACK_356, f356);
    (void)sendFrame(ctx, PYLON_CAN_ID_MODULE_INFO_359, f359);
    (void)sendFrame(ctx, PYLON_CAN_ID_STATUS_35C, f35c);
    (void)sendFrame(ctx, PYLON_CAN_ID_ASCII_ID_35E, f35e);
    (void)sendFrame(ctx, PYLON_CAN_ID_CELL_TEMP_373, f373);
}

static void logStatus(const pylon_can_diag_sender_ctx_t *ctx)
{
    twai_status_info_t status = {0};

    if (ctx == NULL || ctx->bus == NULL) {
        return;
    }

    esp_err_t err = twai_get_status_info_v2(ctx->bus, &status);
    if (err != ESP_OK) {
        ESP_LOGW(EXAMPLE_TAG,
                 "EASUN Pylon 24V diag status read failed on %s (err=0x%x)",
                 ctx->ifName ? ctx->ifName : "CAN",
                 (unsigned)err);
        return;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "EASUN Pylon 24V diag TX on %s: txOk=%" PRIu32 " txCallFail=%" PRIu32 " state=%s txErr=%" PRIu32 " rxErr=%" PRIu32 " txFail=%" PRIu32 " busErr=%" PRIu32,
             ctx->ifName ? ctx->ifName : "CAN",
             ctx->txCallsOk,
             ctx->txCallsFail,
             twaiStateName(status.state),
             status.tx_error_counter,
             status.rx_error_counter,
             status.tx_failed_count,
             status.bus_error_count);
}

static void pylonCanDiagSenderTask(void *pv)
{
    pylon_can_diag_sender_ctx_t *ctx = (pylon_can_diag_sender_ctx_t *)pv;

    ESP_LOGW(EXAMPLE_TAG,
             "EASUN Pylon 24V diagnostic sender active on %s: synthetic 8S frames only, no JK 16S forwarding",
             ctx->ifName ? ctx->ifName : "CAN");

    while (1) {
        sendPylon24vSet(ctx);
        vTaskDelay(pdMS_TO_TICKS(80));
        logStatus(ctx);
        vTaskDelay(pdMS_TO_TICKS(EASUN_PYLON_24V_DIAG_TX_PERIOD_MS));
    }
}

esp_err_t pylonCanDiagSenderStart(twai_handle_t bus, const char *ifName, TaskHandle_t *outTask)
{
    if (bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_diagTask != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_diagCtx, 0, sizeof(s_diagCtx));
    s_diagCtx.bus = bus;
    s_diagCtx.ifName = ifName;

    BaseType_t ok = xTaskCreate(pylonCanDiagSenderTask,
                                "pylon24_diag",
                                EASUN_PYLON_24V_DIAG_TASK_STACK,
                                &s_diagCtx,
                                EASUN_PYLON_24V_DIAG_TASK_PRIORITY,
                                &s_diagTask);
    if (ok != pdPASS) {
        memset(&s_diagCtx, 0, sizeof(s_diagCtx));
        s_diagTask = NULL;
        return ESP_ERR_NO_MEM;
    }

    if (outTask != NULL) {
        *outTask = s_diagTask;
    }
    return ESP_OK;
}
