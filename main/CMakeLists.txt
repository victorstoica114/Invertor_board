idf_component_register(
    SRCS
        "main.c"
        "config.c"
        "bridge.c"
        "CAN_Decoder.c"
        "modbusDecoder.c"
        # "modbusResp.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        driver
        esp_timer
)