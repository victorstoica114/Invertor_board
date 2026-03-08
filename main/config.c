#include "config.h"

#include "esp_log.h"

/* CAN handles kept here (init/config module) */
static twai_handle_t twaiBus0; /* CAN1 */
static twai_handle_t twaiBus1; /* CAN2 */

/*
 * CAN forward exclusion list (CAN1 -> CAN2).
 * Remove entries to discover the minimal frame set needed by the inverter.
 */
const uint32_t g_can1ToCan2ExcludeIds[] = {
    GROWATT_CAN_ID_311_STATUS_LIMITS,
    GROWATT_CAN_ID_312_PROT_ALM,
    GROWATT_CAN_ID_313_V_I_SOC_SOH,
    GROWATT_CAN_ID_314_RM_FCC_DV_CYCLES,
    GROWATT_CAN_ID_315_CELL_GRP1,
    GROWATT_CAN_ID_316_CELL_GRP2,
    GROWATT_CAN_ID_317_CELL_GRP3,
    GROWATT_CAN_ID_318_CELL_GRP4,
    GROWATT_CAN_ID_319_CELL_REF_FLAGS,
    GROWATT_CAN_ID_320_MAKER_SW,
    GROWATT_CAN_ID_321_UPGRADE_INFO,
    GROWATT_CAN_ID_322_TEMP_SOC_MIN_MAX,
    GROWATT_CAN_ID_323_CELLCOUNT_PROT_WARN,
    GROWATT_CAN_ID_324_EXT1,
    GROWATT_CAN_ID_325_EXT2,
};

const size_t g_can1ToCan2ExcludeIdsCount =
    sizeof(g_can1ToCan2ExcludeIds) / sizeof(g_can1ToCan2ExcludeIds[0]);

/*
 * RS485 Modbus register exclusion list (forward path to inverter).
 * Requests that include these registers are filtered/sanitized per bridge logic.
 */
const uint16_t g_rs485ForwardExcludeRegs[] = {
    0x0001u, 0x0002u, 0x0003u, 0x0004u, 0x0005u, 0x0006u, 0x0007u,
    0x0008u, 0x0009u, 0x000Au, 0x000Bu, 0x000Cu, 0x000Du, 0x000Eu,
    0x000Fu,
    GROWATT_MB_REG_CELL08_MV, GROWATT_MB_REG_CELL09_MV, GROWATT_MB_REG_CELL10_MV,
    GROWATT_MB_REG_CELL11_MV, GROWATT_MB_REG_CELL12_MV, GROWATT_MB_REG_CELL13_MV,
    GROWATT_MB_REG_CELL14_MV, GROWATT_MB_REG_CELL15_MV, GROWATT_MB_REG_CELL16_MV,
    GROWATT_MB_REG_CELL_EXTRA,
};

const size_t g_rs485ForwardExcludeRegsCount =
    sizeof(g_rs485ForwardExcludeRegs) / sizeof(g_rs485ForwardExcludeRegs[0]);

twai_handle_t canGetBus0(void) { return twaiBus0; }
twai_handle_t canGetBus1(void) { return twaiBus1; }

uart_port_t rs485GetUart1(void) { return RS485_1_UART; }
uart_port_t rs485GetUart2(void) { return RS485_2_UART; }

gpio_num_t rs485GetDir1(void) { return (gpio_num_t)RS485_1_DIR; }
gpio_num_t rs485GetDir2(void) { return (gpio_num_t)RS485_2_DIR; }

/* ---------- LED BLINK 1 Hz ---------- */
void led_blink_task(void *pvParameters)
{
    (void)pvParameters;
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ---------- RS485 init ---------- */
void rs485Init(void)
{
    uart_config_t uartConfig = {
        .baud_rate = RS485_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* RS485_1 */
    ESP_ERROR_CHECK(uart_param_config(RS485_1_UART, &uartConfig));
    ESP_ERROR_CHECK(uart_set_pin(RS485_1_UART, RS485_1_TX, RS485_1_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(RS485_1_UART,
                                        RS485_BUF_SIZE, RS485_BUF_SIZE,
                                        0, NULL, 0));
    gpio_reset_pin(RS485_1_DIR);
    gpio_set_direction(RS485_1_DIR, GPIO_MODE_OUTPUT);
    gpio_set_level(RS485_1_DIR, 0); /* RX default */

    /* RS485_2 */
    ESP_ERROR_CHECK(uart_param_config(RS485_2_UART, &uartConfig));
    ESP_ERROR_CHECK(uart_set_pin(RS485_2_UART, RS485_2_TX, RS485_2_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(RS485_2_UART,
                                        RS485_BUF_SIZE, RS485_BUF_SIZE,
                                        0, NULL, 0));
    gpio_reset_pin(RS485_2_DIR);
    gpio_set_direction(RS485_2_DIR, GPIO_MODE_OUTPUT);
    gpio_set_level(RS485_2_DIR, 0); /* RX default */

    ESP_LOGI(EXAMPLE_TAG, "RS485_1 & RS485_2 initialized (%d 8N1)", RS485_BAUDRATE);
}

/* ---------- CAN init ---------- */
void canInit(void)
{
    twai_general_config_t gConfig =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN1_TX, CAN1_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t tConfig = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t fConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    /* Controller 0 => CAN1 */
    gConfig.controller_id = 0;
    ESP_ERROR_CHECK(twai_driver_install_v2(&gConfig, &tConfig, &fConfig, &twaiBus0));
    ESP_ERROR_CHECK(twai_start_v2(twaiBus0));
    ESP_LOGI(EXAMPLE_TAG, "CAN1 started");

    /* Controller 1 => CAN2 */
    gConfig.controller_id = 1;
    gConfig.tx_io = CAN2_TX;
    gConfig.rx_io = CAN2_RX;

    ESP_ERROR_CHECK(twai_driver_install_v2(&gConfig, &tConfig, &fConfig, &twaiBus1));
    ESP_ERROR_CHECK(twai_start_v2(twaiBus1));
    ESP_LOGI(EXAMPLE_TAG, "CAN2 started");
}
