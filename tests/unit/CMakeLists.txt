idf_component_register(
    SRC_DIRS "."
    INCLUDE_DIRS "."
    REQUIRES unity main
)

# Add test sources
target_sources(${COMPONENT_LIB} PRIVATE
    test_can_decoder.c
    test_jkbms_modbus_alerts.c
    test_jkbms_rs485_native.c
    test_modbus_decoder.c
    test_pace_modbus.c
    test_route_selection.c
    test_seplos_rs485.c
)
