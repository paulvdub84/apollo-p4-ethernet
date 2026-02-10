idf_component_register(
    SRCS
        "main.c"
        "mqtt.c"
        "cc1101.c"
        "mqtt_log.c"
    INCLUDE_DIRS
        "."
        "generated"
    PRIV_REQUIRES
        ethernet_init
        esp_event
        esp_netif
        nvs_flash
        esp_eth
        mqtt
        driver
        esp_http_server
        app_update
        esp_timer
        log
)

add_custom_command(
    OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/generated/build_info.h
    COMMAND ${CMAKE_SOURCE_DIR}/tools/gen_build_info.sh
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Generating build_info.h"
)

add_custom_target(gen_build_info ALL
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/generated/build_info.h
)

add_dependencies(${COMPONENT_LIB} gen_build_info)
