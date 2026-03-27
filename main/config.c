#include "config.h"

#include "driver/gpio.h"
<<<<<<< HEAD
#include "esp_log.h"
=======
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
>>>>>>> sniffer_V2

/*
 * CAN forward exclusion list (CAN1 -> CAN2).
 * Remove entries to discover the minimal frame set needed by the inverter.
 */
const uint32_t g_can1ToCan2ExcludeIds[] = {
    GROWATT_CAN_ID_311_STATUS_LIMITS,
    GROWATT_CAN_ID_312_PROT_ALM,
    // GROWATT_CAN_ID_313_V_I_SOC_SOH,
    // GROWATT_CAN_ID_314_RM_FCC_DV_CYCLES,
    GROWATT_CAN_ID_315_CELL_GRP1,
    GROWATT_CAN_ID_316_CELL_GRP2,
    GROWATT_CAN_ID_317_CELL_GRP3,
    GROWATT_CAN_ID_318_CELL_GRP4,
    // GROWATT_CAN_ID_319_CELL_REF_FLAGS,
    GROWATT_CAN_ID_320_MAKER_SW,
    GROWATT_CAN_ID_321_UPGRADE_INFO,
    // GROWATT_CAN_ID_322_TEMP_SOC_MIN_MAX,
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
    GROWATT_MB_REG_INFO_0001,
    GROWATT_MB_REG_INFO_0002,
    GROWATT_MB_REG_INFO_0003,
    GROWATT_MB_REG_INFO_0004,
    0x0005u,
    0x0006u,
    0x0007u,
    0x0008u,
    0x0009u,
    0x000Au,
    0x000Bu,
    0x000Cu,
    0x000Du,
    0x000Eu,
    0x000Fu,
    GROWATT_MB_REG_MAIN_START,
    0x0011u,
    0x0012u,
    GROWATT_MB_REG_MAIN_RAW_0013,
    0x0014u,
    // GROWATT_MB_REG_SOC_PCT,
    GROWATT_MB_REG_PACK_V_CV,
    GROWATT_MB_REG_PACK_I_ABS_CA_TENTATIVE,
    GROWATT_MB_REG_TEMP_C,
    GROWATT_MB_REG_CYCLE_COUNT_TENTATIVE,
    GROWATT_MB_REG_REMAIN_CAP_CAH,
    GROWATT_MB_REG_FULL_CAP_CAH,
    0x001Cu,
    0x001Du,
    0x001Eu,
    0x001Fu,
    GROWATT_MB_REG_SOH_PCT,
    GROWATT_MB_REG_CV_TARGET_CV,
    GROWATT_MB_REG_ICHG_LIM_CA_TENTATIVE,
    GROWATT_MB_REG_IDIS_LIM_CA_TENTATIVE,
    0x0024u,
    GROWATT_MB_REG_CELL_MAX_MV,
    GROWATT_MB_REG_CELL_MIN_MV,
    GROWATT_MB_REG_CELL_MAX_IDX,
    GROWATT_MB_REG_CELL_MIN_IDX,
    0x0029u,
    GROWATT_MB_REG_MAIN_END,
    GROWATT_MB_REG_CELL01_MV,
    GROWATT_MB_REG_CELL02_MV,
    GROWATT_MB_REG_CELL03_MV,
    GROWATT_MB_REG_CELL04_MV,
    GROWATT_MB_REG_CELL05_MV,
    GROWATT_MB_REG_CELL06_MV,
    GROWATT_MB_REG_CELL07_MV,
    GROWATT_MB_REG_CELL08_MV,
    GROWATT_MB_REG_CELL09_MV,
    GROWATT_MB_REG_CELL10_MV,
    GROWATT_MB_REG_CELL11_MV,
    GROWATT_MB_REG_CELL12_MV,
    GROWATT_MB_REG_CELL13_MV,
    GROWATT_MB_REG_CELL14_MV,
    GROWATT_MB_REG_CELL15_MV,
    GROWATT_MB_REG_CELL16_MV,
    GROWATT_MB_REG_CELL_EXTRA,
};

const size_t g_rs485ForwardExcludeRegsCount =
    sizeof(g_rs485ForwardExcludeRegs) / sizeof(g_rs485ForwardExcludeRegs[0]);

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
<<<<<<< HEAD


=======
>>>>>>> sniffer_V2
