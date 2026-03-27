idf_component_register(
    SRCS
        "main.c"
        "Working_modes.c"
        "Web_interface/bridge_compat.c"
        "runtime_settings.c"
        "rs485_can_bridge.c"
        "Web_interface/web_interface.c"
        "config.c"
<<<<<<< HEAD
        "runtime_settings.c"
        "bridge.c"
        "Drivers/CAN/can_driver.c"
        "Drivers/RS485/rs485_driver.c"
        "Operation_Modes/can_forward_sniffer.c"
        "Operation_Modes/rs485_forward_sniffer.c"
        "CAN_Decoder.c"
        "modbusDecoder.c"
        "rs485_can_bridge.c"
        "BMS_Protocols/Pylon/pylon_rs485_bridge.c"
        "BMS_Protocols/Pylon/pylon_can_protocol.c"
        "BMS_Protocols/Deye/deye_can_protocol.c"
        "Web_interface/web_interface.c"
        # "modbusResp.c"
=======
        "Drivers/can_driver.c"
        "Drivers/rs485_driver.c"
        "CAN_Decoder.c"
        "modbusDecoder.c"
        "orchestrator/orchestrator.c"
        "protocols/growatt/growatt_register_map.c"
        "protocols/growatt/growatt_bms_task.c"
        "protocols/growatt/growatt_inverter_task.c"
        "protocols/jkbms_modbus/jkbms_modbus_register_map.c"
        "protocols/jkbms_modbus/jkbms_modbus_poller.c"
        "protocols/jkbms_modbus/jkbms_modbus_bms_task.c"
        "protocols/rs485_growatt/rs485_growatt_register_map.c"
        "protocols/rs485_growatt/rs485_growatt_modbus_poller.c"
        "protocols/rs485_growatt/rs485_growatt_bms_task.c"
        "protocols/pylon/pylon_bms_task.c"
        "protocols/pylon/pylon_inverter_task.c"
>>>>>>> sniffer_V2
    INCLUDE_DIRS
        "."
    REQUIRES
        driver
        esp_timer
        esp_http_server
        esp_wifi
<<<<<<< HEAD
=======
        esp_event
>>>>>>> sniffer_V2
        esp_netif
        nvs_flash
)
