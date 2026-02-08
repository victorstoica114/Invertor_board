#include "bridge.h"
#include "config.h"
#include "modbusDecoder.h"

#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/twai.h"

#define TAG "SNIFFER_BRIDGE"

/* ===================== Helpers RS485 ===================== */

static inline void rs485SetTx(gpio_num_t dirGpio, bool txEnable)
{
    /* logica ta: 1=TX, 0=RX */
    gpio_set_level(dirGpio, txEnable ? 1 : 0);
}

static uint32_t rs485GapUsFromBaud(uint32_t baudrate)
{
    /* Modbus RTU: end-of-frame ~3.5 char times.
       Char time ~ (start+8data+parity?+stop) => aprox 11 bits.
       3.5 * 11 = 38.5 bits => 38.5/baud sec.
    */
    const uint32_t bits = 39; // rotunjit
    uint64_t us = (1000000ULL * (uint64_t)bits) / (uint64_t)baudrate;
    if (us < 2000) us = 2000;   // minim defensiv (2ms)
    if (us > 50000) us = 50000; // maxim defensiv
    return (uint32_t)us;
}

static void rs485ForwardFrame(uart_port_t txUart, gpio_num_t txDirGpio,
                              const uint8_t *data, int len, uint32_t baudrate)
{
    if (len <= 0) return;

    rs485SetTx(txDirGpio, true);

    /* Trimite tot cadrul dintr-un foc (fără gap-uri între bucăți) */
    uart_write_bytes(txUart, (const char *)data, len);

    /* Așteaptă să se golească TX FIFO (asta e cheia) */
    /* Timeout: suficient pentru len bytes la baudrate */
    uint32_t bitsPerChar = 11;
    uint32_t txTimeMs = (uint32_t)(((uint64_t)len * bitsPerChar * 1000ULL) / baudrate) + 10;
    uart_wait_tx_done(txUart, pdMS_TO_TICKS(txTimeMs));

    rs485SetTx(txDirGpio, false);
}

/* ===================== RS485 bridge task ===================== */

typedef struct {
    const char *ifName;
    uart_port_t rxUart;
    uart_port_t txUart;
    gpio_num_t  txDirGpio;
    uint32_t    baudrate;
    bool        decode;   // true => decodare Modbus pe RX
} rs485TaskCtx_t;

static void rs485BridgeTask(void *arg)
{
    const rs485TaskCtx_t *ctx = (const rs485TaskCtx_t *)arg;

    modbusDecoder_t dec;
    modbusDecoderInit(&dec, ctx->ifName, ctx->baudrate);

    const uint32_t gapUs = rs485GapUsFromBaud(ctx->baudrate);

    uint8_t rxBuf[256];

    /* Buffer de “frame” (cumulează până la gap) */
    uint8_t frameBuf[512];
    int frameLen = 0;

    int64_t lastByteUs = 0;

    while (1) {
        int len = uart_read_bytes(ctx->rxUart, rxBuf, sizeof(rxBuf), pdMS_TO_TICKS(10));
        int64_t nowUs = esp_timer_get_time();

        if (len > 0) {
            /* Dacă a trecut un gap mare înainte de bytes noi, finalizează frame-ul vechi */
            if (frameLen > 0 && lastByteUs != 0 && (nowUs - lastByteUs) > (int64_t)gapUs) {
                /* decode + forward frame complet */
                if (ctx->decode) {
                    modbusDecoderFeed(&dec, frameBuf, frameLen, nowUs);
                    modbusDecoderFlush(&dec);
                }
                rs485ForwardFrame(ctx->txUart, ctx->txDirGpio, frameBuf, frameLen, ctx->baudrate);
                frameLen = 0;
            }

            /* append la buffer */
            if (frameLen + len > (int)sizeof(frameBuf)) {
                ESP_LOGW(TAG, "%s: frameBuf overflow, drop (%d + %d)", ctx->ifName, frameLen, len);
                frameLen = 0;
            }

            memcpy(&frameBuf[frameLen], rxBuf, (size_t)len);
            frameLen += len;

            lastByteUs = nowUs;
        } else {
            /* N-ai primit bytes acum. Dacă ai un frame în lucru și s-a făcut gap, finalizează-l */
            if (frameLen > 0 && lastByteUs != 0 && (nowUs - lastByteUs) > (int64_t)gapUs) {
                if (ctx->decode) {
                    modbusDecoderFeed(&dec, frameBuf, frameLen, nowUs);
                    modbusDecoderFlush(&dec);
                }
                rs485ForwardFrame(ctx->txUart, ctx->txDirGpio, frameBuf, frameLen, ctx->baudrate);
                frameLen = 0;
                lastByteUs = 0;
            }
        }
    }
}

/* ===================== Public API ===================== */

static rs485TaskCtx_t gRsCtx21; // RS485_2 -> RS485_1
static rs485TaskCtx_t gRsCtx12; // RS485_1 -> RS485_2

void rs485BridgeEnable(void)
{
    /* Asigură RX by default */
    rs485SetTx(RS485_1_DIR, false);
    rs485SetTx(RS485_2_DIR, false);

    gRsCtx21 = (rs485TaskCtx_t){
        .ifName   = "RS485_2",
        .rxUart   = RS485_2_UART,
        .txUart   = RS485_1_UART,
        .txDirGpio= RS485_1_DIR,
        .baudrate = RS485_BAUDRATE,
        .decode   = true,  // decode pe RX RS485_2 (vei vedea REQ + cadre BAD)
    };

    gRsCtx12 = (rs485TaskCtx_t){
        .ifName   = "RS485_1",
        .rxUart   = RS485_1_UART,
        .txUart   = RS485_2_UART,
        .txDirGpio= RS485_2_DIR,
        .baudrate = RS485_BAUDRATE,
        .decode   = true,  // decode pe RX RS485_1 (vei vedea RESP)
    };

    xTaskCreate(rs485BridgeTask, "rs485_2_to_1", 4096, &gRsCtx21, 10, NULL);
    xTaskCreate(rs485BridgeTask, "rs485_1_to_2", 4096, &gRsCtx12, 10, NULL);

    ESP_LOGI(TAG, "RS485 bridge enabled (RS485_1<->RS485_2)");
}

/* ===================== CAN bridge (rămâne cum ai) ===================== */

static void canBridgeTask(void *arg)
{
    twai_handle_t *h = (twai_handle_t *)arg;

    while (1) {
        twai_message_t msg;
        esp_err_t err = twai_receive(*h, &msg, pdMS_TO_TICKS(1000));
        if (err == ESP_OK) {
            /* TODO: forward către cealaltă interfață dacă vrei */
        }
    }
}

void canBridgeEnable(void)
{
    /* Aici păstrează implementarea ta existentă.
       IMPORTANT: gCan1/gCan2 trebuie să fie defined o singură dată (în config.c),
       iar în config.h doar extern. */
    ESP_LOGI(TAG, "CAN bridge enabled (CAN1<->CAN2)");
}
