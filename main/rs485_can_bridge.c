#include "rs485_can_bridge.h"

#include "CAN_Decoder.h"
#include "Drivers/rs485_driver.h"
#include "config.h"
#include "protocols/growatt/growatt_register_map.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    modbusDecoder_t *src;
    twai_handle_t txBus;
    const char *txName;
} rs485Can322Ctx_t;

static rs485Can322Ctx_t g_rs485Can322Ctx;
static TaskHandle_t g_rs485Can322TaskHandle;

typedef struct {
    bool hasSoc;
    uint8_t socPct;
    bool hasSoh;
    uint8_t sohPct;
    bool hasTempC;
    int16_t tempC;
    bool hasPackCv;
    uint16_t packCv;
    bool hasCycles;
    uint16_t cycles;
    bool hasCellExtremes;
    uint16_t cellMaxMv;
    uint16_t cellMinMv;
    uint8_t cellMaxIdx;
    uint8_t cellMinIdx;
} canGrowattCache_t;

typedef struct {
    uart_port_t uart;
    gpio_num_t dirPin;
    const char *ifName;
    twai_handle_t srcCanBus;
    const char *srcCanIf;
    uint8_t slaveId;
    uint8_t fakeSocPct;
    canGrowattCache_t cache;
    uint32_t reqCount;
    uint32_t rspCount;
} canRs485GrowattCtx_t;

static canRs485GrowattCtx_t g_canRsGrowattCtx;
static TaskHandle_t g_canRsGrowattTaskHandle;

static inline void putBe16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)((v >> 8) & 0xFFu);
    p[1] = (uint8_t)(v & 0xFFu);
}

static inline uint16_t be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static inline int16_t be16s(const uint8_t *p)
{
    return (int16_t)be16(p);
}

static inline uint16_t le16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[1] << 8) | (uint16_t)p[0]);
}

static uint16_t crc16(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFFu;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}

static bool checkCrc(const uint8_t *frame, int len)
{
    if (frame == NULL || len < 4) {
        return false;
    }
    const uint16_t got = (uint16_t)(frame[len - 2] | ((uint16_t)frame[len - 1] << 8));
    const uint16_t calc = crc16(frame, len - 2);
    return got == calc;
}

static void putLe16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void cacheFromCanFrame(canRs485GrowattCtx_t *ctx, const twai_message_t *m)
{
    if (ctx == NULL || m == NULL || m->data_length_code != 8u) {
        return;
    }

    const uint32_t id = (uint32_t)m->identifier;
    const uint8_t *d = m->data;

    switch (id) {
        case GROWATT_CAN_ID_313_V_I_SOC_SOH: {
            ctx->cache.packCv = be16(&d[0]);
            ctx->cache.hasPackCv = true;

            const int16_t tDeci = be16s(&d[4]);
            ctx->cache.tempC = (int16_t)(tDeci / 10);
            ctx->cache.hasTempC = true;

            ctx->cache.socPct = (uint8_t)((d[6] > 100u) ? 100u : d[6]);
            ctx->cache.hasSoc = true;

            ctx->cache.sohPct = (uint8_t)(d[7] & 0x7Fu);
            if (ctx->cache.sohPct > 100u) {
                ctx->cache.sohPct = 100u;
            }
            ctx->cache.hasSoh = true;
            break;
        }
        case GROWATT_CAN_ID_314_RM_FCC_DV_CYCLES: {
            ctx->cache.cycles = be16(&d[6]);
            ctx->cache.hasCycles = true;
            break;
        }
        case GROWATT_CAN_ID_319_CELL_REF_FLAGS: {
            ctx->cache.cellMaxMv = le16(&d[0]);
            ctx->cache.cellMinMv = le16(&d[2]);
            ctx->cache.cellMaxIdx = d[5];
            ctx->cache.cellMinIdx = d[6];
            ctx->cache.hasCellExtremes = true;
            break;
        }
        case GROWATT_CAN_ID_322_TEMP_SOC_MIN_MAX: {
            const int16_t tDeci = be16s(&d[0]);
            ctx->cache.tempC = (int16_t)(tDeci / 10);
            ctx->cache.hasTempC = true;
            ctx->cache.socPct = (uint8_t)((d[6] > 100u) ? 100u : d[6]);
            ctx->cache.hasSoc = true;
            break;
        }
        default:
            break;
    }
}

static bool parseReadReq(const uint8_t *frame,
                         int len,
                         uint8_t slaveId,
                         uint8_t *funcOut,
                         uint16_t *startOut,
                         uint16_t *countOut)
{
    if (frame == NULL || len != 8 || !checkCrc(frame, len)) {
        return false;
    }

    if (frame[0] != slaveId) {
        return false;
    }

    const uint8_t func = frame[1];
    if (func != 0x03u && func != 0x04u) {
        return false;
    }

    const uint16_t start = be16(&frame[2]);
    const uint16_t count = be16(&frame[4]);
    if (count == 0u || count > 125u) {
        return false;
    }

    if (funcOut != NULL) {
        *funcOut = func;
    }
    if (startOut != NULL) {
        *startOut = start;
    }
    if (countOut != NULL) {
        *countOut = count;
    }
    return true;
}

static uint16_t fallbackCell(uint8_t idx)
{
    static const uint16_t k_cells[16] = {
        3450u, 3450u, 3451u, 3452u,
        3450u, 3450u, 3451u, 3452u,
        3449u, 3448u, 3449u, 3448u,
        3450u, 3451u, 3450u, 3451u
    };
    if (idx >= 16u) {
        return 3450u;
    }
    return k_cells[idx];
}

static uint16_t synthReg(const canRs485GrowattCtx_t *ctx, uint16_t addr)
{
    const uint16_t soc = ctx->cache.hasSoc ? ctx->cache.socPct : (uint16_t)ctx->fakeSocPct;
    const uint16_t soh = ctx->cache.hasSoh ? ctx->cache.sohPct : 100u;
    const uint16_t packCv = ctx->cache.hasPackCv ? ctx->cache.packCv : 5120u;
    const uint16_t tempC = (uint16_t)(ctx->cache.hasTempC ? ctx->cache.tempC : 25);
    const uint16_t cycles = ctx->cache.hasCycles ? ctx->cache.cycles : 0u;
    const uint16_t fullCap = 4000u; /* 40.00Ah in 0.01Ah units */
    const uint16_t remCap = (uint16_t)(((uint32_t)fullCap * (uint32_t)soc) / 100u);
    uint16_t cellMaxMv = ctx->cache.hasCellExtremes ? ctx->cache.cellMaxMv : 3452u;
    uint16_t cellMinMv = ctx->cache.hasCellExtremes ? ctx->cache.cellMinMv : 3448u;
    uint8_t cellMaxIdx = ctx->cache.hasCellExtremes ? ctx->cache.cellMaxIdx : 4u;
    uint8_t cellMinIdx = ctx->cache.hasCellExtremes ? ctx->cache.cellMinIdx : 10u;

    if (cellMaxIdx < 1u || cellMaxIdx > 16u) {
        cellMaxIdx = 4u;
    }
    if (cellMinIdx < 1u || cellMinIdx > 16u) {
        cellMinIdx = 10u;
    }
    if (cellMaxMv < cellMinMv) {
        uint16_t t = cellMaxMv;
        cellMaxMv = cellMinMv;
        cellMinMv = t;
    }

    switch (addr) {
        case GROWATT_MB_REG_INFO_0001:
            return 0x0001u;
        case GROWATT_MB_REG_INFO_0002:
            return 0x0010u;
        case GROWATT_MB_REG_INFO_0003:
            return 0x0001u;
        case GROWATT_MB_REG_INFO_0004:
            return 0x0000u;
        case GROWATT_MB_REG_STATUS_FLAGS:
            return 0x0000u;
        case GROWATT_MB_REG_SOC_PCT:
            return soc;
        case GROWATT_MB_REG_PACK_V_CV:
            return packCv;
        case GROWATT_MB_REG_PACK_I_ABS_CA_TENTATIVE:
            return 0u;
        case GROWATT_MB_REG_TEMP_C:
            return tempC;
        case GROWATT_MB_REG_CYCLE_COUNT_TENTATIVE:
            return cycles;
        case GROWATT_MB_REG_REMAIN_CAP_CAH:
            return remCap;
        case GROWATT_MB_REG_FULL_CAP_CAH:
            return fullCap;
        case GROWATT_MB_REG_SOH_PCT:
            return soh;
        case GROWATT_MB_REG_CV_TARGET_CV:
            return packCv;
        case GROWATT_MB_REG_ICHG_LIM_CA_TENTATIVE:
            return 0u;
        case GROWATT_MB_REG_IDIS_LIM_CA_TENTATIVE:
            return 0u;
        case GROWATT_MB_REG_CELL_MAX_MV:
            return cellMaxMv;
        case GROWATT_MB_REG_CELL_MIN_MV:
            return cellMinMv;
        case GROWATT_MB_REG_CELL_MAX_IDX:
            return cellMaxIdx;
        case GROWATT_MB_REG_CELL_MIN_IDX:
            return cellMinIdx;
        case GROWATT_MB_REG_CELL_EXTRA:
            return 0u;
        default:
            if (addr >= GROWATT_MB_REG_CELL_BASE && addr <= GROWATT_MB_REG_CELL_LAST) {
                uint8_t idx = (uint8_t)(addr - GROWATT_MB_REG_CELL_BASE);
                if (idx == (uint8_t)(cellMaxIdx - 1u)) {
                    return cellMaxMv;
                }
                if (idx == (uint8_t)(cellMinIdx - 1u)) {
                    return cellMinMv;
                }
                return fallbackCell(idx);
            }
            return 0u;
    }
}

static bool sendGrowattResponse(canRs485GrowattCtx_t *ctx, uint8_t func, uint16_t start, uint16_t count)
{
    if (ctx == NULL) {
        return false;
    }

    const int respLen = (int)(3u + (count * 2u) + 2u);
    if (respLen <= 0 || respLen > 256) {
        return false;
    }

    uint8_t resp[256] = {0};
    resp[0] = ctx->slaveId;
    resp[1] = func;
    resp[2] = (uint8_t)(count * 2u);

    for (uint16_t i = 0; i < count; i++) {
        uint16_t addr = (uint16_t)(start + i);
        uint16_t val = synthReg(ctx, addr);
        putBe16(&resp[3 + (i * 2u)], val);
    }

    uint16_t crc = crc16(resp, respLen - 2);
    putLe16(&resp[respLen - 2], crc);

    rs485SetDirection(ctx->dirPin, true);
    int written = uart_write_bytes(ctx->uart, (const char *)resp, respLen);
    if (written == respLen) {
        (void)uart_wait_tx_done(ctx->uart, pdMS_TO_TICKS(100));
    }
    rs485SetDirection(ctx->dirPin, false);

    return written == respLen;
}

static void logDecodedRegisterSnapshot(const canRs485GrowattCtx_t *ctx,
                                       uint16_t reqStart,
                                       uint16_t reqCount,
                                       bool sent)
{
    if (ctx == NULL) {
        return;
    }

    const uint16_t soc = synthReg(ctx, GROWATT_MB_REG_SOC_PCT);
    const uint16_t soh = synthReg(ctx, GROWATT_MB_REG_SOH_PCT);
    const int16_t tempC = (int16_t)synthReg(ctx, GROWATT_MB_REG_TEMP_C);
    const uint16_t packCv = synthReg(ctx, GROWATT_MB_REG_PACK_V_CV);
    const uint16_t cycles = synthReg(ctx, GROWATT_MB_REG_CYCLE_COUNT_TENTATIVE);
    const uint16_t remCap = synthReg(ctx, GROWATT_MB_REG_REMAIN_CAP_CAH);
    const uint16_t fullCap = synthReg(ctx, GROWATT_MB_REG_FULL_CAP_CAH);
    const uint16_t cMaxMv = synthReg(ctx, GROWATT_MB_REG_CELL_MAX_MV);
    const uint16_t cMinMv = synthReg(ctx, GROWATT_MB_REG_CELL_MIN_MV);
    const uint16_t cMaxIdx = synthReg(ctx, GROWATT_MB_REG_CELL_MAX_IDX);
    const uint16_t cMinIdx = synthReg(ctx, GROWATT_MB_REG_CELL_MIN_IDX);

    ESP_LOGI(EXAMPLE_TAG,
             "CAN->RS485 req#%u on %s start=0x%04X count=%u sent=%s | "
             "SOC=%u%% SOH=%u%% T=%dC Vpack=%.2fV Cycles=%u Rem/FCC=%.2f/%.2fAh "
             "Cmax=%.3fV(#%u) Cmin=%.3fV(#%u)",
             (unsigned)ctx->reqCount,
             ctx->ifName,
             (unsigned)reqStart,
             (unsigned)reqCount,
             sent ? "Y" : "N",
             (unsigned)soc,
             (unsigned)soh,
             (int)tempC,
             (double)packCv / 100.0,
             (unsigned)cycles,
             (double)remCap / 100.0,
             (double)fullCap / 100.0,
             (double)cMaxMv / 1000.0,
             (unsigned)cMaxIdx,
             (double)cMinMv / 1000.0,
             (unsigned)cMinIdx);
}

static void canRs485GrowattTask(void *pv)
{
    canRs485GrowattCtx_t *ctx = (canRs485GrowattCtx_t *)pv;
    uint8_t rxChunk[64];
    uint8_t frameBuf[256];
    uint16_t frameLen = 0u;
    bool frameHaveLastByte = false;
    int64_t frameLastByteUs = 0;

    while (1) {
        twai_message_t canMsg = {0};
        while (twai_receive_v2(ctx->srcCanBus, &canMsg, 0) == ESP_OK) {
#ifdef TWAI_MSG_FLAG_SELF
            if (canMsg.flags & TWAI_MSG_FLAG_SELF) {
                continue;
            }
#endif
            canDecoderOnFrame(ctx->srcCanIf, &canMsg);
            cacheFromCanFrame(ctx, &canMsg);
        }

        int len = uart_read_bytes(ctx->uart, rxChunk, sizeof(rxChunk), pdMS_TO_TICKS(2));
        int64_t nowUs = esp_timer_get_time();

        if (len > 0) {
            if (frameHaveLastByte &&
                ((nowUs - frameLastByteUs) > (int64_t)CAN_RS485_SOC_RX_GAP_US)) {
                frameLen = 0u;
                frameHaveLastByte = false;
            }

            if ((size_t)frameLen + (size_t)len > sizeof(frameBuf)) {
                frameLen = 0u;
                frameHaveLastByte = false;
            } else {
                memcpy(&frameBuf[frameLen], rxChunk, (size_t)len);
                frameLen = (uint16_t)(frameLen + len);
                frameLastByteUs = nowUs;
                frameHaveLastByte = true;
            }
        }

        if (frameHaveLastByte &&
            ((nowUs - frameLastByteUs) > (int64_t)CAN_RS485_SOC_RX_GAP_US)) {
            uint8_t func = 0u;
            uint16_t start = 0u;
            uint16_t count = 0u;
            bool sent = false;

            if (parseReadReq(frameBuf, frameLen, ctx->slaveId, &func, &start, &count)) {
                ctx->reqCount++;
                sent = sendGrowattResponse(ctx, func, start, count);
                if (sent) {
                    ctx->rspCount++;
                }
                if (ctx->reqCount <= 3u || (ctx->reqCount % 25u) == 0u) {
                    logDecodedRegisterSnapshot(ctx, start, count, sent);
                }
            }

            frameLen = 0u;
            frameHaveLastByte = false;
        }
    }
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

esp_err_t canRs485GrowattBridgeEnable(uart_port_t inverterUart,
                                      gpio_num_t inverterDir,
                                      const char *ifName,
                                      twai_handle_t srcCanBus,
                                      const char *srcCanIf)
{
#if !CAN_RS485_SOC_TRANSLATOR_ENABLE
    (void)inverterUart;
    (void)inverterDir;
    (void)ifName;
    (void)srcCanBus;
    (void)srcCanIf;
    ESP_LOGI(EXAMPLE_TAG, "CAN->RS485 Growatt translator disabled by config");
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (g_canRsGrowattTaskHandle != NULL) {
        ESP_LOGI(EXAMPLE_TAG, "CAN->RS485 Growatt translator already running");
        return ESP_OK;
    }
    if (srcCanBus == NULL) {
        ESP_LOGW(EXAMPLE_TAG, "CAN->RS485 Growatt translator not started: source CAN bus is null");
        return ESP_ERR_INVALID_ARG;
    }

    memset(&g_canRsGrowattCtx, 0, sizeof(g_canRsGrowattCtx));
    g_canRsGrowattCtx.uart = inverterUart;
    g_canRsGrowattCtx.dirPin = inverterDir;
    g_canRsGrowattCtx.ifName = (ifName != NULL) ? ifName : "RS485";
    g_canRsGrowattCtx.srcCanBus = srcCanBus;
    g_canRsGrowattCtx.srcCanIf = (srcCanIf != NULL) ? srcCanIf : "CAN1";
    g_canRsGrowattCtx.slaveId = (uint8_t)CAN_RS485_SOC_SLAVE_ID;
    g_canRsGrowattCtx.fakeSocPct =
        (uint8_t)((CAN_RS485_SOC_FAKE_PCT > 100u) ? 100u : CAN_RS485_SOC_FAKE_PCT);

    rs485SetDirection(g_canRsGrowattCtx.dirPin, false);

    /* UART0 is also used by the serial monitor/console, so avoid flush deadlocks there. */
    if (g_canRsGrowattCtx.uart != UART_NUM_0) {
        (void)uart_flush_input(g_canRsGrowattCtx.uart);
    } else {
        ESP_LOGW(EXAMPLE_TAG,
                 "CAN->RS485 translator on UART0: skip uart_flush_input to avoid console lock");
    }

    BaseType_t taskOk = xTaskCreate(canRs485GrowattTask,
                                    "can_to_rs485_gw",
                                    4096,
                                    &g_canRsGrowattCtx,
                                    9,
                                    &g_canRsGrowattTaskHandle);
    if (taskOk != pdPASS) {
        g_canRsGrowattTaskHandle = NULL;
        memset(&g_canRsGrowattCtx, 0, sizeof(g_canRsGrowattCtx));
        ESP_LOGE(EXAMPLE_TAG, "CAN->RS485 Growatt translator task create failed");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "CAN->RS485 Growatt translator enabled (if=%s src=%s slave=%u fallbackSOC=%u%%)",
             g_canRsGrowattCtx.ifName,
             g_canRsGrowattCtx.srcCanIf,
             (unsigned)g_canRsGrowattCtx.slaveId,
             (unsigned)g_canRsGrowattCtx.fakeSocPct);
    return ESP_OK;
#endif
}

void canRs485GrowattBridgeStop(void)
{
    if (g_canRsGrowattTaskHandle != NULL) {
        vTaskDelete(g_canRsGrowattTaskHandle);
        g_canRsGrowattTaskHandle = NULL;
    }
    if ((int)g_canRsGrowattCtx.dirPin >= 0) {
        rs485SetDirection(g_canRsGrowattCtx.dirPin, false);
    }
    memset(&g_canRsGrowattCtx, 0, sizeof(g_canRsGrowattCtx));
}
