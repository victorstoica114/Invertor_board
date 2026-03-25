#include "protocols/growatt/growatt_bms_task.h"

#include "protocols/rs485_growatt/rs485_growatt_bms_task.h"

esp_err_t growattBmsTaskStart(QueueHandle_t outQueue)
{
    return rs485GrowattBmsTaskStart(outQueue);
}

esp_err_t growattBmsTaskStop(void)
{
    return rs485GrowattBmsTaskStop();
}

bool growattBmsTaskGetLatestPacket(bms_decoded_packet_t *outPacket)
{
    return rs485GrowattBmsTaskGetLatestPacket(outPacket);
}
