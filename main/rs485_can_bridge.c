#include "rs485_can_bridge.h"

#include "CAN_Decoder.h"
#include "Drivers/rs485_driver.h"
#include "config.h"
<<<<<<< HEAD
#include "bridge.h"
#include "runtime_settings.h"
#include "BMS_Protocols/Growatt/growatt_modbus_map.h"
#include "CAN_Decoder.h"
=======
#include "protocols/growatt/growatt_register_map.h"
#include "protocols/jkbms_modbus/jkbms_modbus_bms_task.h"
>>>>>>> sniffer_V2

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
    uint32_t txSetCount;
} rs485Can322Ctx_t;

static rs485Can322Ctx_t g_rsCanCtx;
static TaskHandle_t g_rsCanTaskHandle;

typedef struct {
    uart_port_t uart;
    gpio_num_t dirPin;
    const char *ifName;
    const char *srcCanIf;
    uint8_t slaveId;
    uint8_t fakeSocPct;
    uint32_t reqCount;
    uint32_t rspCount;
} canRs485GrowattCtx_t;

static canRs485GrowattCtx_t g_canRsGrowattCtx;
static TaskHandle_t g_canRsGrowattTaskHandle;

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
    uint32_t rxBytes;
    uint32_t reqCount;
    uint32_t rspCount;
} canRs485GrowattCtx_t;

static canRs485GrowattCtx_t g_canRsGrowattCtx;
static TaskHandle_t g_canRsGrowattTaskHandle;

typedef struct {
    uart_port_t uart;
    gpio_num_t dirPin;
    const char *ifName;
    uint8_t slaveId;
    uint8_t fakeSocPct;
    uint32_t rxBytes;
    uint32_t reqCount;
    uint32_t rspCount;
} jkbmsRs485GrowattCtx_t;

static jkbmsRs485GrowattCtx_t g_jkbmsRsGrowattCtx;
static TaskHandle_t g_jkbmsRsGrowattTaskHandle;

static int drainUartRx(uart_port_t uart, int maxBytes)
{
    uint8_t tmp[64];
    int drained = 0;

    if (maxBytes <= 0) {
        return 0;
    }

    while (drained < maxBytes) {
        int toRead = (int)sizeof(tmp);
        if ((maxBytes - drained) < toRead) {
            toRead = maxBytes - drained;
        }

        int got = uart_read_bytes(uart, tmp, (uint32_t)toRead, 0);
        if (got <= 0) {
            break;
        }
        drained += got;
    }

    return drained;
}

static inline void putBe16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)((v >> 8) & 0xFFu);
    p[1] = (uint8_t)(v & 0xFFu);
}

<<<<<<< HEAD
static inline void putLe16(uint8_t *p, uint16_t v)
=======
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
>>>>>>> sniffer_V2
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

<<<<<<< HEAD
=======
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

typedef struct {
    uint16_t soc;
    uint16_t soh;
    uint16_t packCv;
    int16_t tempC;
    uint16_t cycles;
    uint16_t cellMaxMv;
    uint16_t cellMinMv;
    uint8_t cellMaxIdx;
    uint8_t cellMinIdx;
} growattSynthSnapshot_t;

static bool fillSnapshotFromJkbms(growattSynthSnapshot_t *snap, uint8_t fallbackSocPct)
{
    if (snap == NULL) {
        return false;
    }

    snap->soc = fallbackSocPct;
    snap->soh = 100u;
    snap->packCv = 5120u;
    snap->tempC = 25;
    snap->cycles = 0u;
    snap->cellMaxMv = 3452u;
    snap->cellMinMv = 3448u;
    snap->cellMaxIdx = 4u;
    snap->cellMinIdx = 10u;

    bms_decoded_packet_t pkt = {0};
    if (!jkbmsModbusBmsTaskGetLatestPacket(&pkt)) {
        return false;
    }
    if (pkt.timestampUs <= 0) {
        return false;
    }

    int64_t nowUs = esp_timer_get_time();
    int64_t ageUs = nowUs - pkt.timestampUs;
    if (ageUs < 0) {
        ageUs = 0;
    }
    if (ageUs > ((int64_t)BRIDGE_SOURCE_STALE_MS * 1000LL)) {
        return false;
    }

    if (pkt.hasSoc) {
        snap->soc = (pkt.socPct > 100u) ? 100u : pkt.socPct;
    }
    if (pkt.hasTemperatureC) {
        snap->tempC = pkt.temperatureC;
    }
    if (pkt.hasPackVoltageCv) {
        snap->packCv = pkt.packVoltageCv;
    }
    if (pkt.hasCellExtremes) {
        snap->cellMaxMv = pkt.maxCellMv;
        snap->cellMinMv = pkt.minCellMv;
        snap->cellMaxIdx = pkt.maxCellIndex;
        snap->cellMinIdx = pkt.minCellIndex;
    }
    return true;
}

static uint16_t synthRegFromSnapshot(const growattSynthSnapshot_t *snap, uint16_t addr)
{
    if (snap == NULL) {
        return 0u;
    }

    const uint16_t soc = snap->soc;
    const uint16_t soh = snap->soh;
    const uint16_t packCv = snap->packCv;
    const uint16_t tempC = (uint16_t)snap->tempC;
    const uint16_t cycles = snap->cycles;
    const uint16_t fullCap = 4000u; /* 40.00Ah in 0.01Ah units */
    const uint16_t remCap = (uint16_t)(((uint32_t)fullCap * (uint32_t)soc) / 100u);
    uint16_t cellMaxMv = snap->cellMaxMv;
    uint16_t cellMinMv = snap->cellMinMv;
    uint8_t cellMaxIdx = snap->cellMaxIdx;
    uint8_t cellMinIdx = snap->cellMinIdx;

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
    uint8_t streamBuf[256];
    uint16_t streamLen = 0u;
    TickType_t lastStatsTick = xTaskGetTickCount();

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
        if (len > 0) {
            ctx->rxBytes += (uint32_t)len;

            if ((size_t)streamLen + (size_t)len > sizeof(streamBuf)) {
                if (streamLen > 7u) {
                    const uint16_t keep = 7u;
                    memmove(streamBuf, &streamBuf[streamLen - keep], keep);
                    streamLen = keep;
                }
            }

            if ((size_t)streamLen + (size_t)len <= sizeof(streamBuf)) {
                memcpy(&streamBuf[streamLen], rxChunk, (size_t)len);
                streamLen = (uint16_t)(streamLen + len);
            }

            while (streamLen >= 8u) {
                bool found = false;
                for (uint16_t off = 0; off + 8u <= streamLen; off++) {
                    uint8_t func = 0u;
                    uint16_t start = 0u;
                    uint16_t count = 0u;
                    if (parseReadReq(&streamBuf[off], 8, ctx->slaveId, &func, &start, &count)) {
                        ctx->reqCount++;
                        bool sent = sendGrowattResponse(ctx, func, start, count);
                        if (sent) {
                            ctx->rspCount++;
                        }
                        if (ctx->reqCount <= 3u || (ctx->reqCount % 25u) == 0u) {
                            logDecodedRegisterSnapshot(ctx, start, count, sent);
                        }

                        const uint16_t consume = (uint16_t)(off + 8u);
                        if (consume < streamLen) {
                            memmove(streamBuf, &streamBuf[consume], streamLen - consume);
                        }
                        streamLen = (uint16_t)(streamLen - consume);
                        found = true;
                        break;
                    }
                }

                if (!found) {
                    if (streamLen > 7u) {
                        const uint16_t drop = (uint16_t)(streamLen - 7u);
                        memmove(streamBuf, &streamBuf[drop], 7u);
                        streamLen = 7u;
                    }
                    break;
                }
            }
        }

        if ((xTaskGetTickCount() - lastStatsTick) >= pdMS_TO_TICKS(5000)) {
            ESP_LOGI(EXAMPLE_TAG,
                     "CAN->RS485 %s stats: rxBytes=%u req=%u rsp=%u",
                     ctx->ifName,
                     (unsigned)ctx->rxBytes,
                     (unsigned)ctx->reqCount,
                     (unsigned)ctx->rspCount);
            ctx->rxBytes = 0u;
            lastStatsTick = xTaskGetTickCount();
        }
    }
}

static bool sendGrowattResponseFromSnapshot(const jkbmsRs485GrowattCtx_t *ctx,
                                            const growattSynthSnapshot_t *snap,
                                            uint8_t func,
                                            uint16_t start,
                                            uint16_t count)
{
    if (ctx == NULL || snap == NULL) {
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
        uint16_t val = synthRegFromSnapshot(snap, addr);
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

static void jkbmsRs485GrowattTask(void *pv)
{
    jkbmsRs485GrowattCtx_t *ctx = (jkbmsRs485GrowattCtx_t *)pv;
    uint8_t rxChunk[64];
    uint8_t streamBuf[256];
    uint16_t streamLen = 0u;
    TickType_t lastStatsTick = xTaskGetTickCount();

    while (1) {
        int len = uart_read_bytes(ctx->uart, rxChunk, sizeof(rxChunk), pdMS_TO_TICKS(2));
        if (len > 0) {
            ctx->rxBytes += (uint32_t)len;

            if ((size_t)streamLen + (size_t)len > sizeof(streamBuf)) {
                if (streamLen > 7u) {
                    const uint16_t keep = 7u;
                    memmove(streamBuf, &streamBuf[streamLen - keep], keep);
                    streamLen = keep;
                }
            }

            if ((size_t)streamLen + (size_t)len <= sizeof(streamBuf)) {
                memcpy(&streamBuf[streamLen], rxChunk, (size_t)len);
                streamLen = (uint16_t)(streamLen + len);
            }

            while (streamLen >= 8u) {
                bool found = false;
                for (uint16_t off = 0; off + 8u <= streamLen; off++) {
                    uint8_t func = 0u;
                    uint16_t start = 0u;
                    uint16_t count = 0u;
                    if (parseReadReq(&streamBuf[off], 8, ctx->slaveId, &func, &start, &count)) {
                        ctx->reqCount++;

                        growattSynthSnapshot_t snap = {0};
                        bool fresh = fillSnapshotFromJkbms(&snap, ctx->fakeSocPct);

                        bool sent = fresh && sendGrowattResponseFromSnapshot(ctx, &snap, func, start, count);
                        if (sent) {
                            ctx->rspCount++;
                        }

                        if (ctx->reqCount <= 3u || (ctx->reqCount % 25u) == 0u) {
                            ESP_LOGI(EXAMPLE_TAG,
                                     "JKBMS->RS485 req#%u on %s start=0x%04X count=%u sent=%s | "
                                     "SOC=%u%% T=%dC Vpack=%.2fV Cmax=%.3fV(#%u) Cmin=%.3fV(#%u)",
                                     (unsigned)ctx->reqCount,
                                     ctx->ifName,
                                     (unsigned)start,
                                     (unsigned)count,
                                     sent ? "Y" : "N",
                                     (unsigned)snap.soc,
                                     (int)snap.tempC,
                                     (double)snap.packCv / 100.0,
                                     (double)snap.cellMaxMv / 1000.0,
                                     (unsigned)snap.cellMaxIdx,
                                     (double)snap.cellMinMv / 1000.0,
                                     (unsigned)snap.cellMinIdx);
                        }

                        const uint16_t consume = (uint16_t)(off + 8u);
                        if (consume < streamLen) {
                            memmove(streamBuf, &streamBuf[consume], streamLen - consume);
                        }
                        streamLen = (uint16_t)(streamLen - consume);
                        found = true;
                        break;
                    }
                }

                if (!found) {
                    if (streamLen > 7u) {
                        const uint16_t drop = (uint16_t)(streamLen - 7u);
                        memmove(streamBuf, &streamBuf[drop], 7u);
                        streamLen = 7u;
                    }
                    break;
                }
            }
        }

        if ((xTaskGetTickCount() - lastStatsTick) >= pdMS_TO_TICKS(5000)) {
            ESP_LOGI(EXAMPLE_TAG,
                     "JKBMS->RS485 %s stats: rxBytes=%u req=%u rsp=%u",
                     ctx->ifName,
                     (unsigned)ctx->rxBytes,
                     (unsigned)ctx->reqCount,
                     (unsigned)ctx->rspCount);
            ctx->rxBytes = 0u;
            lastStatsTick = xTaskGetTickCount();
        }
    }
}
>>>>>>> sniffer_V2
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

static uint16_t canRsBe16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint16_t canRsCrc16(const uint8_t *data, int len)
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

static bool canRsCheckCrc(const uint8_t *frame, int len)
{
    if (len < 4) {
        return false;
    }
    uint16_t got = (uint16_t)(frame[len - 2] | ((uint16_t)frame[len - 1] << 8));
    uint16_t calc = canRsCrc16(frame, len - 2);
    return got == calc;
}

static void canRsSetTx(gpio_num_t dirPin, bool txEn)
{
    gpio_set_level(dirPin, txEn ? 1 : 0);
}

static bool canRsModelLooksUsable(const universal_battery_model_t *model)
{
    return model != NULL && model->valid;
}

static bool canRsModelFresh(const universal_battery_model_t *model)
{
    if (!canRsModelLooksUsable(model) || model->updatedMs == 0u) {
        return false;
    }

    uint32_t nowMs = (uint32_t)(esp_timer_get_time() / 1000LL);
    uint32_t ageMs = nowMs - model->updatedMs;
    return ageMs <= BRIDGE_SOURCE_STALE_MS;
}

static bool canRsSourceFresh(const canRs485GrowattCtx_t *ctx, const universal_battery_model_t *model)
{
    if (ctx != NULL && ctx->srcCanIf != NULL) {
        if (canDecoderHasFreshData(ctx->srcCanIf, BRIDGE_SOURCE_STALE_MS)) {
            return true;
        }
    }

    /* Fallback for routes that rely mainly on universal model propagation (e.g. Pylon path). */
    return canRsModelFresh(model);
}

static uint16_t canRsRoundScaled(float value, float scale, uint16_t fallback)
{
    float scaled = value * scale;
    int32_t rounded = 0;

    if (!(value == value)) {
        return fallback;
    }

    if (scaled < 0.0f) {
        return fallback;
    }

    rounded = (int32_t)(scaled + 0.5f);
    if (rounded < 0) {
        return fallback;
    }
    if (rounded > 65535) {
        return 65535u;
    }
    return (uint16_t)rounded;
}

static uint16_t canRsAbsScaled(float value, float scale, uint16_t fallback)
{
    if (value < 0.0f) {
        value = -value;
    }
    return canRsRoundScaled(value, scale, fallback);
}

static uint16_t canRsSocFromSources(const canRs485GrowattCtx_t *ctx,
                                    const universal_battery_model_t *model,
                                    bool *fromModel,
                                    bool *fromCan)
{
    uint8_t socPct = (ctx != NULL) ? ctx->fakeSocPct : 0u;
    bool socFromCan = false;
    bool socFromModel = false;

    if (fromModel != NULL) {
        *fromModel = false;
    }
    if (fromCan != NULL) {
        *fromCan = false;
    }

    if (model != NULL && model->valid && model->socPct <= 100u) {
        socPct = model->socPct;
        socFromModel = true;
    } else if (ctx != NULL && ctx->srcCanIf != NULL) {
        socFromCan = canDecoderTryGetSocPct(ctx->srcCanIf, &socPct);
    }

    if (fromModel != NULL) {
        *fromModel = socFromModel;
    }
    if (fromCan != NULL) {
        *fromCan = socFromCan;
    }
    return socPct;
}

static void canRsSynthCellRegs(const universal_battery_model_t *model, uint16_t cells[16])
{
    uint16_t maxMv = RS485_CAN_BRIDGE_FALLBACK_CELL_MAX_MV;
    uint16_t minMv = RS485_CAN_BRIDGE_FALLBACK_CELL_MIN_MV;
    uint8_t maxIdx = (uint8_t)RS485_CAN_BRIDGE_FALLBACK_CELL_MAX_IDX;
    uint8_t minIdx = (uint8_t)RS485_CAN_BRIDGE_FALLBACK_CELL_MIN_IDX;
    uint16_t avgMv = 0;

    if (model != NULL) {
        if (model->cellMaxV > 0.0f) {
            maxMv = canRsRoundScaled(model->cellMaxV, 1000.0f, maxMv);
        }
        if (model->cellMinV > 0.0f) {
            minMv = canRsRoundScaled(model->cellMinV, 1000.0f, minMv);
        }
        if (model->cellMaxIdx >= 1u && model->cellMaxIdx <= 16u) {
            maxIdx = model->cellMaxIdx;
        }
        if (model->cellMinIdx >= 1u && model->cellMinIdx <= 16u) {
            minIdx = model->cellMinIdx;
        }
    }

    if (maxMv < minMv) {
        uint16_t tmpMv = maxMv;
        uint8_t tmpIdx = maxIdx;
        maxMv = minMv;
        minMv = tmpMv;
        maxIdx = minIdx;
        minIdx = tmpIdx;
    }

    if (model != NULL && model->packVoltageV > 0.0f) {
        avgMv = canRsRoundScaled(model->packVoltageV / 16.0f, 1000.0f, 0u);
    }
    if (avgMv == 0u) {
        avgMv = (uint16_t)((maxMv + minMv) / 2u);
    }
    if (avgMv < minMv) {
        avgMv = minMv;
    }
    if (avgMv > maxMv) {
        avgMv = maxMv;
    }

    for (int i = 0; i < 16; i++) {
        cells[i] = avgMv;
    }

    if (maxIdx >= 1u && maxIdx <= 16u) {
        cells[maxIdx - 1u] = maxMv;
    }
    if (minIdx >= 1u && minIdx <= 16u) {
        cells[minIdx - 1u] = minMv;
    }
}

static uint16_t canRsSynthStatusFlags(const universal_battery_model_t *model)
{
    uint16_t status = 0u;

    if (model != NULL && model->protocolState != 0u) {
        status = (uint16_t)(model->protocolState & 0xFFFFu);
    }

    if (model != NULL) {
        if (model->balanceEnabled) {
            status |= 0x0020u;
        }
        if (model->dischargeEnabled) {
            status |= 0x0040u;
        }
        if (model->chargeEnabled) {
            status |= 0x0080u;
        }
    }

    return status;
}

static uint16_t canRsSynthGrowattReg(const universal_battery_model_t *model,
                                     uint16_t addr,
                                     uint16_t socPct,
                                     uint16_t fullCapCah,
                                     const uint16_t cells[16])
{
    bool modelOk = canRsModelLooksUsable(model);

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
            return canRsSynthStatusFlags(model);
        case GROWATT_MB_REG_SOC_PCT:
            return socPct;
        case GROWATT_MB_REG_PACK_V_CV:
            if (modelOk && model->packVoltageV > 0.0f) {
                return canRsRoundScaled(model->packVoltageV, 100.0f, RS485_CAN_BRIDGE_FALLBACK_PACK_V_CV);
            }
            return RS485_CAN_BRIDGE_FALLBACK_PACK_V_CV;
        case GROWATT_MB_REG_PACK_I_ABS_CA_TENTATIVE:
            if (modelOk) {
                return canRsAbsScaled(model->packCurrentA, 100.0f, (uint16_t)(RS485_CAN_BRIDGE_FALLBACK_PACK_I_0P1 * 10));
            }
            return (uint16_t)(RS485_CAN_BRIDGE_FALLBACK_PACK_I_0P1 * 10);
        case GROWATT_MB_REG_TEMP_C:
            if (modelOk) {
                return canRsRoundScaled(model->temperaturesC[0], 1.0f, (uint16_t)RS485_CAN_BRIDGE_FALLBACK_TEMP_C);
            }
            return (uint16_t)RS485_CAN_BRIDGE_FALLBACK_TEMP_C;
        case GROWATT_MB_REG_CYCLE_COUNT_TENTATIVE:
            if (modelOk && model->cycleCount > 0u) {
                return model->cycleCount;
            }
            return RS485_CAN_BRIDGE_FALLBACK_CYCLE_COUNT;
        case GROWATT_MB_REG_REMAIN_CAP_CAH:
            return (uint16_t)(((uint32_t)fullCapCah * (uint32_t)socPct) / 100u);
        case GROWATT_MB_REG_FULL_CAP_CAH:
            return fullCapCah;
        case GROWATT_MB_REG_SOH_PCT:
            if (modelOk && model->sohPct <= 100u) {
                return model->sohPct;
            }
            return RS485_CAN_BRIDGE_FALLBACK_SOH_PCT;
        case GROWATT_MB_REG_CV_TARGET_CV:
            if (modelOk && model->chargeVoltageLimitV > 0.0f) {
                return canRsRoundScaled(model->chargeVoltageLimitV, 100.0f, RS485_CAN_BRIDGE_FALLBACK_PACK_V_CV);
            }
            return canRsSynthGrowattReg(model, GROWATT_MB_REG_PACK_V_CV, socPct, fullCapCah, cells);
        case GROWATT_MB_REG_ICHG_LIM_CA_TENTATIVE:
            if (modelOk && model->chargeCurrentLimitA > 0.0f) {
                return canRsAbsScaled(model->chargeCurrentLimitA, 100.0f, 0u);
            }
            return 0u;
        case GROWATT_MB_REG_IDIS_LIM_CA_TENTATIVE:
            if (modelOk && model->dischargeCurrentLimitA > 0.0f) {
                return canRsAbsScaled(model->dischargeCurrentLimitA, 100.0f, 0u);
            }
            return 0u;
        case GROWATT_MB_REG_CELL_MAX_MV:
            return cells[clampCellIdxU8((modelOk ? model->cellMaxIdx : 0u), (uint8_t)RS485_CAN_BRIDGE_FALLBACK_CELL_MAX_IDX) - 1u];
        case GROWATT_MB_REG_CELL_MIN_MV:
            return cells[clampCellIdxU8((modelOk ? model->cellMinIdx : 0u), (uint8_t)RS485_CAN_BRIDGE_FALLBACK_CELL_MIN_IDX) - 1u];
        case GROWATT_MB_REG_CELL_MAX_IDX:
            return clampCellIdxU8((modelOk ? model->cellMaxIdx : 0u), (uint8_t)RS485_CAN_BRIDGE_FALLBACK_CELL_MAX_IDX);
        case GROWATT_MB_REG_CELL_MIN_IDX:
            return clampCellIdxU8((modelOk ? model->cellMinIdx : 0u), (uint8_t)RS485_CAN_BRIDGE_FALLBACK_CELL_MIN_IDX);
        case GROWATT_MB_REG_CELL_EXTRA:
            return 0u;
        default:
            if (addr >= GROWATT_MB_REG_CELL_BASE && addr <= GROWATT_MB_REG_CELL_LAST) {
                return cells[addr - GROWATT_MB_REG_CELL_BASE];
            }
            return 0u;
    }
}

static bool canRsParseReadReq(const uint8_t *frame,
                              int len,
                              uint8_t slaveId,
                              uint8_t *funcOut,
                              uint16_t *startOut,
                              uint16_t *countOut)
{
    if (frame == NULL || len != 8) {
        return false;
    }
    if (!canRsCheckCrc(frame, len)) {
        return false;
    }

    if (frame[0] != slaveId) {
        return false;
    }

    const uint8_t func = frame[1];
    if (func != 0x03u && func != 0x04u) {
        return false;
    }

    const uint16_t start = canRsBe16(&frame[2]);
    const uint16_t count = canRsBe16(&frame[4]);
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

static bool canRsSendGrowattResponse(canRs485GrowattCtx_t *ctx,
                                     uint8_t func,
                                     uint16_t start,
                                     uint16_t count,
                                     const universal_battery_model_t *model,
                                     uint16_t socPct)
{
    uint16_t fullCapCah = RS485_CAN_BRIDGE_FALLBACK_FCC_10MAH;
    uint16_t cells[16];

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

    canRsSynthCellRegs(model, cells);

    for (uint16_t i = 0; i < count; i++) {
        uint16_t addr = (uint16_t)(start + i);
        uint16_t val = canRsSynthGrowattReg(model, addr, socPct, fullCapCah, cells);
        putBe16(&resp[3 + (i * 2u)], val);
    }

    uint16_t crc = canRsCrc16(resp, respLen - 2);
    resp[respLen - 2] = (uint8_t)(crc & 0xFFu);
    resp[respLen - 1] = (uint8_t)((crc >> 8) & 0xFFu);

    canRsSetTx(ctx->dirPin, true);
    uart_write_bytes(ctx->uart, (const char *)resp, respLen);
    uart_wait_tx_done(ctx->uart, pdMS_TO_TICKS(100));
    canRsSetTx(ctx->dirPin, false);

    ctx->rspCount++;
    return true;
}

static void canRs485GrowattTask(void *pv)
{
    canRs485GrowattCtx_t *ctx = (canRs485GrowattCtx_t *)pv;
    uint8_t rxChunk[64];
    uint8_t frameBuf[256];
    uint16_t frameLen = 0;
    bool frameHaveLastByte = false;
    int64_t frameLastByteUs = 0;

    while (1) {
        int len = uart_read_bytes(ctx->uart, rxChunk, sizeof(rxChunk), pdMS_TO_TICKS(5));
        int64_t nowUs = esp_timer_get_time();

        if (len > 0) {
            if (frameHaveLastByte && ((nowUs - frameLastByteUs) > (int64_t)CAN_RS485_SOC_RX_GAP_US)) {
                frameLen = 0;
                frameHaveLastByte = false;
            }

            if ((size_t)frameLen + (size_t)len > sizeof(frameBuf)) {
                frameLen = 0;
                frameHaveLastByte = false;
            } else {
                memcpy(&frameBuf[frameLen], rxChunk, (size_t)len);
                frameLen = (uint16_t)(frameLen + len);
                frameLastByteUs = nowUs;
                frameHaveLastByte = true;
            }
        }

        if (frameHaveLastByte && ((nowUs - frameLastByteUs) > (int64_t)CAN_RS485_SOC_RX_GAP_US)) {
            uint8_t func = 0;
            uint16_t start = 0;
            uint16_t count = 0;
            bool sent = false;
            universal_battery_model_t model = {0};
            bool socFromModel = false;
            bool socFromCan = false;
            uint16_t socPct = 0;

            if (canRsParseReadReq(frameBuf, frameLen, ctx->slaveId, &func, &start, &count)) {
                bridgeGetUniversalBatteryModel(&model);
                ctx->reqCount++;
                if (canRsSourceFresh(ctx, &model)) {
                    socPct = canRsSocFromSources(ctx, &model, &socFromModel, &socFromCan);
                    sent = canRsSendGrowattResponse(ctx, func, start, count, &model, socPct);
                }

#if CAN_RS485_SOC_LOG_EVERY_N > 0
                if (ctx->reqCount <= 3u || (ctx->reqCount % CAN_RS485_SOC_LOG_EVERY_N) == 0u) {
                    ESP_LOGI(EXAMPLE_TAG,
                             "CAN->RS485 Growatt translator on %s: req start=0x%04X count=0x%04X sent=%s src=%s soc=%u pack=%.2fV",
                             ctx->ifName,
                             (unsigned)start,
                             (unsigned)count,
                             sent ? "Y" : "N",
                             sent ? (socFromModel ? "UNIVERSAL" : (socFromCan ? "CAN_SOC+FALLBACK" : "FALLBACK")) : "STALE_OR_MISSING",
                             (unsigned)socPct,
                             (double)model.packVoltageV);
                }
#endif
            }

            frameLen = 0;
            frameHaveLastByte = false;
        }
    }
}

void canRs485GrowattBridgeEnable(uart_port_t inverterUart,
                                 gpio_num_t inverterDir,
                                 const char *ifName,
                                 const char *srcCanIf)
{
#if !CAN_RS485_SOC_TRANSLATOR_ENABLE
    (void)inverterUart;
    (void)inverterDir;
    (void)ifName;
    (void)srcCanIf;
    ESP_LOGI(EXAMPLE_TAG, "CAN->RS485 Growatt translator disabled by config");
    return;
#else
    if (g_canRsGrowattTaskHandle != NULL) {
        ESP_LOGI(EXAMPLE_TAG, "CAN->RS485 Growatt translator already running");
        return;
    }

    memset(&g_canRsGrowattCtx, 0, sizeof(g_canRsGrowattCtx));
    g_canRsGrowattCtx.uart = inverterUart;
    g_canRsGrowattCtx.dirPin = inverterDir;
    g_canRsGrowattCtx.ifName = (ifName != NULL) ? ifName : "RS485";
    g_canRsGrowattCtx.srcCanIf = (srcCanIf != NULL) ? srcCanIf : "CAN1";
    g_canRsGrowattCtx.slaveId = (uint8_t)CAN_RS485_SOC_SLAVE_ID;
    g_canRsGrowattCtx.fakeSocPct = (uint8_t)((CAN_RS485_SOC_FAKE_PCT > 100u) ? 100u : CAN_RS485_SOC_FAKE_PCT);

    xTaskCreate(canRs485GrowattTask,
                "can_to_rs485_gw",
                4096,
                &g_canRsGrowattCtx,
                9,
                &g_canRsGrowattTaskHandle);

    ESP_LOGI(EXAMPLE_TAG,
             "CAN->RS485 Growatt translator enabled (if=%s src=%s slave=%u fallbackSOC=%u%%)",
             g_canRsGrowattCtx.ifName,
             g_canRsGrowattCtx.srcCanIf,
             (unsigned)g_canRsGrowattCtx.slaveId,
             (unsigned)g_canRsGrowattCtx.fakeSocPct);
#endif
}

void canRs485GrowattBridgeStop(void)
{
    if (g_canRsGrowattTaskHandle != NULL) {
        vTaskDelete(g_canRsGrowattTaskHandle);
        g_canRsGrowattTaskHandle = NULL;
    }
    memset(&g_canRsGrowattCtx, 0, sizeof(g_canRsGrowattCtx));
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

<<<<<<< HEAD
void rs485Can322BridgeStop(void)
{
    if (g_rsCanTaskHandle != NULL) {
        vTaskDelete(g_rsCanTaskHandle);
        g_rsCanTaskHandle = NULL;
    }
    memset(&g_rsCanCtx, 0, sizeof(g_rsCanCtx));
}


=======
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

    (void)drainUartRx(g_canRsGrowattCtx.uart, 4096);

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

esp_err_t jkbmsRs485GrowattBridgeEnable(uart_port_t inverterUart,
                                        gpio_num_t inverterDir,
                                        const char *ifName)
{
#if !CAN_RS485_SOC_TRANSLATOR_ENABLE
    (void)inverterUart;
    (void)inverterDir;
    (void)ifName;
    ESP_LOGI(EXAMPLE_TAG, "JKBMS->RS485 Growatt translator disabled by config");
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (g_jkbmsRsGrowattTaskHandle != NULL) {
        ESP_LOGI(EXAMPLE_TAG, "JKBMS->RS485 Growatt translator already running");
        return ESP_OK;
    }

    memset(&g_jkbmsRsGrowattCtx, 0, sizeof(g_jkbmsRsGrowattCtx));
    g_jkbmsRsGrowattCtx.uart = inverterUart;
    g_jkbmsRsGrowattCtx.dirPin = inverterDir;
    g_jkbmsRsGrowattCtx.ifName = (ifName != NULL) ? ifName : "RS485";
    g_jkbmsRsGrowattCtx.slaveId = (uint8_t)CAN_RS485_SOC_SLAVE_ID;
    g_jkbmsRsGrowattCtx.fakeSocPct =
        (uint8_t)((CAN_RS485_SOC_FAKE_PCT > 100u) ? 100u : CAN_RS485_SOC_FAKE_PCT);

    rs485SetDirection(g_jkbmsRsGrowattCtx.dirPin, false);
    (void)drainUartRx(g_jkbmsRsGrowattCtx.uart, 4096);

    BaseType_t taskOk = xTaskCreate(jkbmsRs485GrowattTask,
                                    "jkbms_to_rs485_gw",
                                    4096,
                                    &g_jkbmsRsGrowattCtx,
                                    9,
                                    &g_jkbmsRsGrowattTaskHandle);
    if (taskOk != pdPASS) {
        g_jkbmsRsGrowattTaskHandle = NULL;
        memset(&g_jkbmsRsGrowattCtx, 0, sizeof(g_jkbmsRsGrowattCtx));
        ESP_LOGE(EXAMPLE_TAG, "JKBMS->RS485 Growatt translator task create failed");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "JKBMS->RS485 Growatt translator enabled (if=%s slave=%u fallbackSOC=%u%%)",
             g_jkbmsRsGrowattCtx.ifName,
             (unsigned)g_jkbmsRsGrowattCtx.slaveId,
             (unsigned)g_jkbmsRsGrowattCtx.fakeSocPct);
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

    if (g_jkbmsRsGrowattTaskHandle != NULL) {
        vTaskDelete(g_jkbmsRsGrowattTaskHandle);
        g_jkbmsRsGrowattTaskHandle = NULL;
    }
    if ((int)g_jkbmsRsGrowattCtx.dirPin >= 0) {
        rs485SetDirection(g_jkbmsRsGrowattCtx.dirPin, false);
    }
    memset(&g_jkbmsRsGrowattCtx, 0, sizeof(g_jkbmsRsGrowattCtx));
}
>>>>>>> sniffer_V2

