#include "rs485_can_bridge.h"

#include "config.h"
#include "Growatt_regs.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    modbusDecoder_t *src;
    twai_handle_t txBus;
    const char *txName;
    uint32_t txSetCount;
} rs485Can322Ctx_t;

static rs485Can322Ctx_t g_rsCanCtx;
static TaskHandle_t g_rsCanTaskHandle;

static inline void putBe16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)((v >> 8) & 0xFFu);
    p[1] = (uint8_t)(v & 0xFFu);
}

static inline void putLe16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static bool decoderGetCachedReg(const modbusDecoder_t *d, uint16_t addr, uint16_t *outVal)
{
    if (d == NULL) {
        return false;
    }

    for (int i = 0; i < MODBUS_DECODER_CACHE_MAX_REGS; i++) {
        if (!d->cacheValid[i]) {
            continue;
        }
        if (d->cacheAddr[i] != addr) {
            continue;
        }

        if (outVal != NULL) {
            *outVal = d->cacheVal[i];
        }
        return true;
    }

    return false;
}

static bool getRegOrFallback(const rs485Can322Ctx_t *ctx, uint16_t addr, uint16_t fallback, uint16_t *out)
{
    if (decoderGetCachedReg(ctx->src, addr, out)) {
        return true;
    }

#if RS485_CAN_BRIDGE_USE_FALLBACK
    if (out != NULL) {
        *out = fallback;
    }
    return true;
#else
    (void)fallback;
    return false;
#endif
}

static uint8_t clampPctU8(uint16_t v)
{
    if (v > 100u) {
        return 100u;
    }
    return (uint8_t)v;
}

static uint8_t clampCellIdxU8(uint16_t idx, uint8_t fallback)
{
    if (idx >= 1u && idx <= 16u) {
        return (uint8_t)idx;
    }
    return fallback;
}

static bool sendFrame(twai_handle_t txBus, uint32_t id, const uint8_t data[8], const char *txName)
{
    twai_message_t tx = {0};
    tx.identifier = id;
    tx.data_length_code = 8;
    memcpy(tx.data, data, 8);

    esp_err_t e = twai_transmit_v2(txBus, &tx, pdMS_TO_TICKS(20));
    if (e != ESP_OK) {
        ESP_LOGW(EXAMPLE_TAG,
                 "RS485->CAN 0x%03X TX failed on %s (err=0x%x)",
                 (unsigned)id,
                 txName,
                 (unsigned)e);
        return false;
    }

    return true;
}

static void rs485CanTelemetryTask(void *pv)
{
    rs485Can322Ctx_t *ctx = (rs485Can322Ctx_t *)pv;

    while (1) {
        uint16_t soc = 0;
        uint16_t tempCraw = 0;
        uint16_t packCv = 0;
        uint16_t packIAbsCa = 0;
        uint16_t soh = 0;
        uint16_t remain10mAh = 0;
        uint16_t fcc10mAh = 0;
        uint16_t cycles = 0;
        uint16_t cellMaxMv = 0;
        uint16_t cellMinMv = 0;
        uint16_t cellMaxIdx = 0;
        uint16_t cellMinIdx = 0;

        bool okSoc = getRegOrFallback(ctx, GROWATT_MB_REG_SOC_PCT, RS485_CAN_BRIDGE_FALLBACK_SOC_PCT, &soc);
        bool okTemp = getRegOrFallback(ctx, GROWATT_MB_REG_TEMP_C, (uint16_t)RS485_CAN_BRIDGE_FALLBACK_TEMP_C, &tempCraw);
        bool okPackV = getRegOrFallback(ctx, GROWATT_MB_REG_PACK_V_CV, RS485_CAN_BRIDGE_FALLBACK_PACK_V_CV, &packCv);
        bool okPackI = getRegOrFallback(ctx, GROWATT_MB_REG_PACK_I_ABS_CA_TENTATIVE, (uint16_t)(RS485_CAN_BRIDGE_FALLBACK_PACK_I_0P1 * 10), &packIAbsCa);
        bool okSoh = getRegOrFallback(ctx, GROWATT_MB_REG_SOH_PCT, RS485_CAN_BRIDGE_FALLBACK_SOH_PCT, &soh);
        bool okRm = getRegOrFallback(ctx, GROWATT_MB_REG_REMAIN_CAP_CAH, RS485_CAN_BRIDGE_FALLBACK_RM_10MAH, &remain10mAh);
        bool okFcc = getRegOrFallback(ctx, GROWATT_MB_REG_FULL_CAP_CAH, RS485_CAN_BRIDGE_FALLBACK_FCC_10MAH, &fcc10mAh);
        bool okCycle = getRegOrFallback(ctx, GROWATT_MB_REG_CYCLE_COUNT_TENTATIVE, RS485_CAN_BRIDGE_FALLBACK_CYCLE_COUNT, &cycles);
        bool okCmax = getRegOrFallback(ctx, GROWATT_MB_REG_CELL_MAX_MV, RS485_CAN_BRIDGE_FALLBACK_CELL_MAX_MV, &cellMaxMv);
        bool okCmin = getRegOrFallback(ctx, GROWATT_MB_REG_CELL_MIN_MV, RS485_CAN_BRIDGE_FALLBACK_CELL_MIN_MV, &cellMinMv);
        bool okCmaxIdx = getRegOrFallback(ctx, GROWATT_MB_REG_CELL_MAX_IDX, RS485_CAN_BRIDGE_FALLBACK_CELL_MAX_IDX, &cellMaxIdx);
        bool okCminIdx = getRegOrFallback(ctx, GROWATT_MB_REG_CELL_MIN_IDX, RS485_CAN_BRIDGE_FALLBACK_CELL_MIN_IDX, &cellMinIdx);

        int16_t tempC = (int16_t)tempCraw;
        int16_t tempDeci = (int16_t)(tempC * 10);

        int32_t i0p1 = (int32_t)(packIAbsCa / 10u);
        if (i0p1 > 32767) {
            i0p1 = 32767;
        }
        int16_t packI_0p1 = (int16_t)i0p1;

        uint8_t socPct = clampPctU8(soc);
        uint8_t sohPct = clampPctU8(soh);

        uint8_t cmaxIdx = clampCellIdxU8(cellMaxIdx, (uint8_t)RS485_CAN_BRIDGE_FALLBACK_CELL_MAX_IDX);
        uint8_t cminIdx = clampCellIdxU8(cellMinIdx, (uint8_t)RS485_CAN_BRIDGE_FALLBACK_CELL_MIN_IDX);
        if (cellMaxMv < cellMinMv) {
            uint16_t t = cellMaxMv;
            cellMaxMv = cellMinMv;
            cellMinMv = t;
        }
        uint16_t dVmv = (uint16_t)(cellMaxMv - cellMinMv);

        bool sent313 = false;
        bool sent314 = false;
        bool sent319 = false;
        bool sent322 = false;

        if (okPackV && okPackI && okTemp && okSoc && okSoh) {
            uint8_t d313[8] = {0};
            putBe16(&d313[0], packCv);
            putBe16(&d313[2], (uint16_t)packI_0p1);
            putBe16(&d313[4], (uint16_t)tempDeci);
            d313[6] = socPct;
            d313[7] = (uint8_t)(sohPct & 0x7Fu);
            sent313 = sendFrame(ctx->txBus, GROWATT_CAN_ID_313_V_I_SOC_SOH, d313, ctx->txName);
        }

        if (okRm && okFcc && okCycle && okCmax && okCmin) {
            uint8_t d314[8] = {0};
            putBe16(&d314[0], remain10mAh);
            putBe16(&d314[2], fcc10mAh);
            putBe16(&d314[4], dVmv);
            putBe16(&d314[6], cycles);
            sent314 = sendFrame(ctx->txBus, GROWATT_CAN_ID_314_RM_FCC_DV_CYCLES, d314, ctx->txName);
        }

        if (okCmax && okCmin && okCmaxIdx && okCminIdx) {
            uint8_t d319[8] = {0};
            putLe16(&d319[0], cellMaxMv);
            putLe16(&d319[2], cellMinMv);
            d319[4] = (uint8_t)RS485_CAN_BRIDGE_FALLBACK_FLAGS_319;
            d319[5] = cmaxIdx;
            d319[6] = cminIdx;
            d319[7] = (uint8_t)RS485_CAN_BRIDGE_FALLBACK_ADDR_319;
            sent319 = sendFrame(ctx->txBus, GROWATT_CAN_ID_319_CELL_REF_FLAGS, d319, ctx->txName);
        }

        if (okTemp && okSoc) {
            uint8_t d322[8] = {0};
            putBe16(&d322[0], (uint16_t)tempDeci);
            putBe16(&d322[2], (uint16_t)tempDeci);
            d322[4] = (uint8_t)RS485_CAN_BRIDGE_FALLBACK_TEMP_SENSOR_MAX;
            d322[5] = (uint8_t)RS485_CAN_BRIDGE_FALLBACK_TEMP_SENSOR_MIN;
            d322[6] = socPct;
            d322[7] = socPct;
            sent322 = sendFrame(ctx->txBus, GROWATT_CAN_ID_322_TEMP_SOC_MIN_MAX, d322, ctx->txName);
        }

        if (sent313 || sent314 || sent319 || sent322) {
            ctx->txSetCount++;
#if RS485_CAN_BRIDGE_TX_LOG_EVERY_N > 0
            if ((ctx->txSetCount % RS485_CAN_BRIDGE_TX_LOG_EVERY_N) == 0u) {
                ESP_LOGI(EXAMPLE_TAG,
                         "RS485->CAN TXSET on %s: 313=%c 314=%c 319=%c 322=%c | V=%.2fV SOC=%u%% T=%dC",
                         ctx->txName,
                         sent313 ? 'Y' : 'N',
                         sent314 ? 'Y' : 'N',
                         sent319 ? 'Y' : 'N',
                         sent322 ? 'Y' : 'N',
                         (double)packCv / 100.0,
                         (unsigned)socPct,
                         (int)tempC);
            }
#endif
        }

        vTaskDelay(pdMS_TO_TICKS(RS485_CAN_322_TX_PERIOD_MS));
    }
}

void rs485Can322BridgeEnable(modbusDecoder_t *srcDecoder, twai_handle_t txBus, const char *txName)
{
#if !RS485_CAN_322_TRANSLATOR_ENABLE
    ESP_LOGI(EXAMPLE_TAG, "RS485->CAN translator disabled by config");
    return;
#else
    if (g_rsCanTaskHandle != NULL) {
        ESP_LOGI(EXAMPLE_TAG, "RS485->CAN translator already running");
        return;
    }

    if (srcDecoder == NULL || txBus == NULL) {
        ESP_LOGW(EXAMPLE_TAG, "RS485->CAN translator not started: invalid source decoder or CAN bus");
        return;
    }

    memset(&g_rsCanCtx, 0, sizeof(g_rsCanCtx));
    g_rsCanCtx.src = srcDecoder;
    g_rsCanCtx.txBus = txBus;
    g_rsCanCtx.txName = (txName != NULL) ? txName : "CAN";

    xTaskCreate(rs485CanTelemetryTask,
                "rs485_to_can",
                4096,
                &g_rsCanCtx,
                8,
                &g_rsCanTaskHandle);

    ESP_LOGI(EXAMPLE_TAG,
             "RS485->CAN translator enabled (tx=%s, period=%dms, fallback=%s)",
             g_rsCanCtx.txName,
             RS485_CAN_322_TX_PERIOD_MS,
             RS485_CAN_BRIDGE_USE_FALLBACK ? "ON" : "OFF");
#endif
}
