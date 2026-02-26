#include "bridge.h"
#include "config.h"
#include "modbusDecoder.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"


/* ---------- Growatt CAN decode (based on protocol Rev_05 + RS485 cross-check) ---------- */
#define CAN_DECODE_ENABLE 1

static inline uint16_t can_be16(const uint8_t *p)
{
    return (uint16_t)((((uint16_t)p[0]) << 8) | (uint16_t)p[1]);
}

static inline int16_t can_be16s(const uint8_t *p)
{
    return (int16_t)can_be16(p);
}

static inline uint16_t can_le16(const uint8_t *p)
{
    return (uint16_t)((((uint16_t)p[1]) << 8) | (uint16_t)p[0]);
}

static const char *growattChemStr(uint8_t code)
{
    switch (code & 0x03u) {
    case 0u: return "LFP";
    case 1u: return "Ternary";
    case 2u: return "LTO";
    default: return "Reserved";
    }
}

static const char *growattModeStr(uint16_t status)
{
    switch (status & 0x0003u) {
    case 0u: return "soft_start";
    case 1u: return "standby";
    case 2u: return "charging";
    case 3u: return "discharging";
    default: return "?";
    }
}

static const char *growattOpModeStr(uint16_t status)
{
    switch ((status >> 8) & 0x03u) {
    case 0u: return "standalone";
    case 1u: return "parallel";
    case 2u: return "parallel_prep";
    default: return "reserved";
    }
}

static void decodeGrowattCanFrame(const char *ifname, const twai_message_t *m)
{
#if !CAN_DECODE_ENABLE
    (void)ifname;
    (void)m;
    return;
#else
    if (m->data_length_code != 8) return;
    const uint8_t *d = m->data;
    const uint32_t id = (uint32_t)m->identifier;

    switch (id) {
    case 0x311: {
        const int16_t cv_0p1  = can_be16s(&d[0]);
        const int16_t chg_0p1 = can_be16s(&d[2]);
        const int16_t dis_0p1 = can_be16s(&d[4]);
        const uint16_t st     = can_be16(&d[6]);
        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s 0x311: CV=%.1fV IchgLim=%.1fA IdisLim=%.1fA status=0x%04X mode=%s errValid=%u bal=%u sleep=%u outDis=%u outChg=%u termOpen=%u opMode=%s",
                 ifname,
                 (double)((float)cv_0p1 / 10.0f),
                 (double)((float)chg_0p1 / 10.0f),
                 (double)((float)dis_0p1 / 10.0f),
                 (unsigned)st,
                 growattModeStr(st),
                 (unsigned)((st >> 2) & 1u),
                 (unsigned)((st >> 3) & 1u),
                 (unsigned)((st >> 4) & 1u),
                 (unsigned)((st >> 5) & 1u),
                 (unsigned)((st >> 6) & 1u),
                 (unsigned)((st >> 7) & 1u),
                 growattOpModeStr(st));
        break;
    }

    case 0x312:
        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s 0x312: Prot1=0x%02X Prot2=0x%02X Alm1=0x%02X Alm2=0x%02X PackNo=%u PwrRed(H/L)=0x%02X%02X Rsv=0x%02X",
                 ifname,
                 (unsigned)d[0], (unsigned)d[1], (unsigned)d[2], (unsigned)d[3],
                 (unsigned)d[4], (unsigned)d[5], (unsigned)d[6], (unsigned)d[7]);
        break;

    case 0x313: {
        const int16_t v_0p01 = can_be16s(&d[0]);
        const int16_t i_0p1  = can_be16s(&d[2]);
        const int16_t t_0p1  = can_be16s(&d[4]);
        const uint8_t soh    = (uint8_t)(d[7] & 0x7Fu);
        const uint8_t lifeWarn = (uint8_t)((d[7] >> 7) & 0x01u);
        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s 0x313: V=%.2fV I=%+.1fA Tavg=%.1fC SOC=%u%% SOH=%u%% lifeWarn=%u",
                 ifname,
                 (double)((float)v_0p01 / 100.0f),
                 (double)((float)i_0p1 / 10.0f),
                 (double)((float)t_0p1 / 10.0f),
                 (unsigned)d[6], (unsigned)soh, (unsigned)lifeWarn);
        break;
    }

    case 0x314: {
        const uint16_t rm_10mAh  = can_be16(&d[0]);
        const uint16_t fcc_10mAh = can_be16(&d[2]);
        const uint16_t delta_mV  = can_be16(&d[4]);
        const uint16_t cycles    = can_be16(&d[6]);
        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s 0x314: RM=%.2fAh FCC=%.2fAh dV=%umV Cycles=%u",
                 ifname,
                 (double)((float)rm_10mAh / 100.0f),
                 (double)((float)fcc_10mAh / 100.0f),
                 (unsigned)delta_mV,
                 (unsigned)cycles);
        break;
    }

    case 0x315:
    case 0x316:
    case 0x317:
    case 0x318: {
        const unsigned base = (unsigned)((id - 0x315u) * 4u + 1u);
        uint16_t c[4];
        for (int i = 0; i < 4; i++) {
            uint16_t be = can_be16(&d[i * 2]);
            uint16_t le = can_le16(&d[i * 2]);
            bool beOk = (be >= 2000u && be <= 5000u);
            bool leOk = (le >= 2000u && le <= 5000u);
            c[i] = beOk ? be : (leOk ? le : be);
        }
        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s 0x%03X CELLS(opt): C%02u=%umV C%02u=%umV C%02u=%umV C%02u=%umV",
                 ifname, (unsigned)id,
                 base + 0u, (unsigned)c[0],
                 base + 1u, (unsigned)c[1],
                 base + 2u, (unsigned)c[2],
                 base + 3u, (unsigned)c[3]);
        break;
    }

    case 0x319: {
        const uint16_t vmax = can_le16(&d[0]);
        const uint16_t vmin = can_le16(&d[2]);
        const uint8_t flags = d[4];
        const uint8_t cmaxNo = d[5];
        const uint8_t cminNo = d[6];
        const uint8_t addr = d[7];
        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s 0x319: Cmax=%umV(C%u) Cmin=%umV(C%u) dV=%umV | type=%s flags=0x%02X chgEn=%u disEn=%u force1=%u force2=%u addr=%u",
                 ifname,
                 (unsigned)vmax, (unsigned)cmaxNo,
                 (unsigned)vmin, (unsigned)cminNo,
                 (unsigned)(vmax >= vmin ? (vmax - vmin) : 0u),
                 growattChemStr(flags),
                 (unsigned)flags,
                 (unsigned)((flags >> 2) & 1u),
                 (unsigned)((flags >> 3) & 1u),
                 (unsigned)((flags >> 4) & 1u),
                 (unsigned)((flags >> 5) & 1u),
                 (unsigned)addr);
        break;
    }

    case 0x320: {
        const char a = (d[0] >= 32 && d[0] <= 126) ? (char)d[0] : '?';
        const char b = (d[1] >= 32 && d[1] <= 126) ? (char)d[1] : '?';
        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s 0x320: maker='%c%c' hw=0x%02X swL=0x%02X swHext=0x%02X compat=0x%02X ext=0x%02X rsv=0x%02X",
                 ifname, a, b,
                 (unsigned)d[2], (unsigned)d[3], (unsigned)d[4], (unsigned)d[5], (unsigned)d[6], (unsigned)d[7]);
        break;
    }

    case 0x321: {
        bool allZero = true;
        for (int i = 0; i < 8; i++) if (d[i] != 0) { allZero = false; break; }
        if (allZero) {
            ESP_LOGI(EXAMPLE_TAG, "CAN-%s 0x321: remote-upgrade frame unused (all zero)", ifname);
        } else {
            ESP_LOGI(EXAMPLE_TAG,
                     "CAN-%s 0x321: updStatus=0x%02X progress=%u%% progStatus=0x%02X rsv=[%02X %02X %02X %02X %02X]",
                     ifname,
                     (unsigned)d[0], (unsigned)d[1], (unsigned)d[2],
                     (unsigned)d[3], (unsigned)d[4], (unsigned)d[5], (unsigned)d[6], (unsigned)d[7]);
        }
        break;
    }

    case 0x322: {
        const int16_t tMax_0p1 = can_be16s(&d[0]);
        const int16_t tMin_0p1 = can_be16s(&d[2]);
        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s 0x322: Tmax=%.1fC(U%u) Tmin=%.1fC(U%u) SOCmax=%u%% SOCmin=%u%%",
                 ifname,
                 (double)((float)tMax_0p1 / 10.0f), (unsigned)d[4],
                 (double)((float)tMin_0p1 / 10.0f), (unsigned)d[5],
                 (unsigned)d[6], (unsigned)d[7]);
        break;
    }

    case 0x323:
        ESP_LOGI(EXAMPLE_TAG,
                 "CAN-%s 0x323: cellCount=%u prot3=0x%02X prot4=0x%02X prot5=0x%02X warn3=0x%02X",
                 ifname,
                 (unsigned)d[0], (unsigned)d[4], (unsigned)d[5], (unsigned)d[6], (unsigned)d[7]);
        break;

    default:
        break;
    }
#endif
}

/* ---------- Helpers: log CAN ---------- */
static void logCanMsg(const char *ifname, const twai_message_t *m)
{
    char dataHex[3 * 8 + 1] = {0};
    int pos = 0;

    for (int i = 0; i < m->data_length_code && i < 8; i++) {
        pos += snprintf(&dataHex[pos], sizeof(dataHex) - pos, "%02X ", m->data[i]);
        if (pos >= (int)sizeof(dataHex)) break;
    }

    ESP_LOGI(EXAMPLE_TAG,
             "RX on %s: ID=0x%03" PRIX32 " DLC=%d DATA=[%s]",
             ifname,
             (uint32_t)m->identifier,
             m->data_length_code,
             dataHex);
}

/* ---------- CAN bridge task ---------- */
typedef struct {
    const char   *rxName;
    const char   *txName;
    twai_handle_t rxBus;
    twai_handle_t txBus;
} canBridgeCtx_t;

static void canBridgeTask(void *pv)
{
    canBridgeCtx_t *ctx = (canBridgeCtx_t*)pv;
    twai_message_t rx;

    while (1) {
        if (twai_receive_v2(ctx->rxBus, &rx, portMAX_DELAY) == ESP_OK) {

#ifdef TWAI_MSG_FLAG_SELF
            if (rx.flags & TWAI_MSG_FLAG_SELF) {
                continue;
            }
#endif
            logCanMsg(ctx->rxName, &rx);
            decodeGrowattCanFrame(ctx->rxName, &rx);

            esp_err_t e = twai_transmit_v2(ctx->txBus, &rx, pdMS_TO_TICKS(50));
            if (e != ESP_OK) {
                ESP_LOGW(EXAMPLE_TAG, "CAN forward %s -> %s failed (err=0x%x)",
                         ctx->rxName, ctx->txName, (unsigned)e);
            }
        }
    }
}

/* ---------- RS485 log (HEX only) ---------- */
static void logRs485Bytes(const char *ifname, const uint8_t *buf, int len)
{
    const int maxHexBytes = 64;
    int n = (len < maxHexBytes) ? len : maxHexBytes;

    char hex[3 * maxHexBytes + 1];
    int pos = 0;

    for (int i = 0; i < n; i++) {
        pos += snprintf(&hex[pos], sizeof(hex) - pos, "%02X ", buf[i]);
        if (pos >= (int)sizeof(hex)) break;
    }

    if (pos > 0) hex[pos - 1] = 0;
    else hex[0] = 0;

    if (len > maxHexBytes) {
        ESP_LOGI(EXAMPLE_TAG,
                 "RX on %s: len=%d HEX(first %d)=[%s] ...",
                 ifname, len, maxHexBytes, hex);
    } else {
        ESP_LOGI(EXAMPLE_TAG,
                 "RX on %s: len=%d HEX=[%s]",
                 ifname, len, hex);
    }
}

/* ---------- RS485 bridge ---------- */
typedef struct {
    const char *rxName;
    const char *txName;
    uart_port_t rxUart;
    uart_port_t txUart;
    gpio_num_t  txDirPin;
} rs485BridgeCtx_t;

static inline void rs485SetTx(gpio_num_t dirPin, bool txEn)
{
    gpio_set_level(dirPin, txEn ? 1 : 0); // 1=TX, 0=RX (cum ai avut)
}

static void rs485BridgeTask(void *pv)
{
    rs485BridgeCtx_t *ctx = (rs485BridgeCtx_t*)pv;
    uint8_t buf[RS485_BUF_SIZE];

    // Prag gap: 5000us (~5ms) e ok la 9600bps pt Modbus RTU
    static modbusDecoder_t dec1;
    static modbusDecoder_t dec2;
    modbusDecoder_t *dec = NULL;

    // alegem instanța după nume ca să nu alocăm dinamic
    if (strcmp(ctx->rxName, "RS485_1") == 0) {
        dec = &dec1;
        if (dec->ifName == NULL) modbusDecoderInit(dec, "RS485_1", 5000);
    } else {
        dec = &dec2;
        if (dec->ifName == NULL) modbusDecoderInit(dec, "RS485_2", 5000);
    }

    while (1) {
        // poți reduce la 5ms dacă vrei; decoderul oricum reface cadrele.
        int len = uart_read_bytes(ctx->rxUart, buf, RS485_BUF_SIZE, pdMS_TO_TICKS(5));
        if (len > 0) {
            int64_t nowUs = esp_timer_get_time();
            modbusDecoderFeed(dec, buf, len, nowUs);

            // forward ca înainte
            rs485SetTx(ctx->txDirPin, true);
            uart_write_bytes(ctx->txUart, (const char*)buf, len);
            uart_wait_tx_done(ctx->txUart, pdMS_TO_TICKS(100));
            rs485SetTx(ctx->txDirPin, false);
        } else {
            // dacă n-au venit bytes, verificăm dacă a trecut gap-ul și flush-uim
            if (dec->haveLastByte) {
                int64_t nowUs = esp_timer_get_time();
                if ((nowUs - dec->lastByteUs) > (int64_t)dec->gapUs) {
                    modbusDecoderFlush(dec);
                }
            }
        }
    }
}

/* ---------- Enable functions (create tasks) ---------- */
void canBridgeEnable(void)
{
    static canBridgeCtx_t can12;
    static canBridgeCtx_t can21;

    can12.rxName = "CAN1";
    can12.txName = "CAN2";
    can12.rxBus  = canGetBus0();
    can12.txBus  = canGetBus1();

    can21.rxName = "CAN2";
    can21.txName = "CAN1";
    can21.rxBus  = canGetBus1();
    can21.txBus  = canGetBus0();

    xTaskCreate(canBridgeTask, "can1_to_can2", 4096, &can12, 10, NULL);
    xTaskCreate(canBridgeTask, "can2_to_can1", 4096, &can21, 10, NULL);

    ESP_LOGI(EXAMPLE_TAG, "CAN bridge enabled (CAN1<->CAN2)");
}

void rs485BridgeEnable(void)
{
    static rs485BridgeCtx_t rs12;
    static rs485BridgeCtx_t rs21;

    rs12.rxName   = "RS485_1";
    rs12.txName   = "RS485_2";
    rs12.rxUart   = rs485GetUart1();
    rs12.txUart   = rs485GetUart2();
    rs12.txDirPin = rs485GetDir2();

    rs21.rxName   = "RS485_2";
    rs21.txName   = "RS485_1";
    rs21.rxUart   = rs485GetUart2();
    rs21.txUart   = rs485GetUart1();
    rs21.txDirPin = rs485GetDir1();

    xTaskCreate(rs485BridgeTask, "rs485_1_to_2", 4096, &rs12, 9, NULL);
    xTaskCreate(rs485BridgeTask, "rs485_2_to_1", 4096, &rs21, 9, NULL);

    ESP_LOGI(EXAMPLE_TAG, "RS485 bridge enabled (RS485_1<->RS485_2)");
}
