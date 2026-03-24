#include "rs485_can_bridge.h"

#include "config.h"
#include "protocols/growatt/growatt_register_map.h"

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
} rs485Can322Ctx_t;

static rs485Can322Ctx_t g_rs485Can322Ctx;
static TaskHandle_t g_rs485Can322TaskHandle;

static inline void putBe16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)((v >> 8) & 0xFFu);
    p[1] = (uint8_t)(v & 0xFFu);
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

        if (outVal) {
            *outVal = d->cacheVal[i];
        }
        return true;
    }

    return false;
}

static void rs485Can322Task(void *pv)
{
    rs485Can322Ctx_t *ctx = (rs485Can322Ctx_t *)pv;

    while (1) {
        uint16_t soc = 0;
        uint16_t tempC = 0;

        bool hasSoc = decoderGetCachedReg(ctx->src, GROWATT_MB_REG_SOC_PCT, &soc);
        bool hasTemp = decoderGetCachedReg(ctx->src, GROWATT_MB_REG_TEMP_C, &tempC);

        if (hasSoc && hasTemp) {
            uint8_t socPct = (uint8_t)((soc > 100u) ? 100u : soc);
            int16_t tC = (int16_t)tempC;
            int16_t tDeci = (int16_t)(tC * 10);

            twai_message_t tx = {0};
            tx.identifier = GROWATT_CAN_ID_322_TEMP_SOC_MIN_MAX;
            tx.data_length_code = 8;

            putBe16(&tx.data[0], (uint16_t)tDeci);
            putBe16(&tx.data[2], (uint16_t)tDeci);
            tx.data[4] = 1u;
            tx.data[5] = 1u;
            tx.data[6] = socPct;
            tx.data[7] = socPct;

            esp_err_t e = twai_transmit_v2(ctx->txBus, &tx, pdMS_TO_TICKS(20));
            if (e != ESP_OK) {
                ESP_LOGW(EXAMPLE_TAG,
                         "RS485->CAN 0x322 TX failed on %s (err=0x%x)",
                         ctx->txName,
                         (unsigned)e);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(RS485_CAN_322_TX_PERIOD_MS));
    }
}

void rs485Can322BridgeEnable(modbusDecoder_t *srcDecoder, twai_handle_t txBus, const char *txName)
{
#if !RS485_CAN_322_TRANSLATOR_ENABLE
    ESP_LOGI(EXAMPLE_TAG, "RS485->CAN 0x322 translator disabled by config");
    return;
#else
    if (g_rs485Can322TaskHandle != NULL) {
        ESP_LOGI(EXAMPLE_TAG, "RS485->CAN 0x322 translator already running");
        return;
    }

    if (srcDecoder == NULL || txBus == NULL) {
        ESP_LOGW(EXAMPLE_TAG, "RS485->CAN 0x322 translator not started: invalid source decoder or CAN bus");
        return;
    }

    memset(&g_rs485Can322Ctx, 0, sizeof(g_rs485Can322Ctx));
    g_rs485Can322Ctx.src = srcDecoder;
    g_rs485Can322Ctx.txBus = txBus;
    g_rs485Can322Ctx.txName = (txName != NULL) ? txName : "CAN";

    xTaskCreate(rs485Can322Task,
                "rs485_to_can322",
                4096,
                &g_rs485Can322Ctx,
                8,
                &g_rs485Can322TaskHandle);

    ESP_LOGI(EXAMPLE_TAG,
             "RS485->CAN 0x322 translator enabled (tx=%s, period=%dms)",
             g_rs485Can322Ctx.txName,
             RS485_CAN_322_TX_PERIOD_MS);
#endif
}
