set(COMPONENT_SRCS
    api/core/src/vl53l0x_api.c
    api/core/src/vl53l0x_api_calibration.c
    api/core/src/vl53l0x_api_core.c
    api/core/src/vl53l0x_api_ranging.c
    api/core/src/vl53l0x_api_strings.c
    api/platform/src/vl53l0x_platform.c
    api/platform/src/vl53l0x_platform_log.c
)

set(COMPONENT_ADD_INCLUDEDIRS
        include
    api/core/inc
    api/platform/inc
)

set(COMPONENT_REQUIRES driver)

register_component()

target_compile_definitions(${COMPONENT_TARGET}
    PRIVATE USE_I2C_2V8=1
)

# -mlongcalls is Xtensa-only (ESP32/ESP32-S2/ESP32-S3)
if(IDF_TARGET STREQUAL "esp32" OR IDF_TARGET STREQUAL "esp32s2" OR IDF_TARGET STREQUAL "esp32s3")
    target_compile_options(${COMPONENT_LIB} PRIVATE -mlongcalls)
endif()