set(API_DIR Api)

set(API_SRCS
    ${API_DIR}/platform/src/vl53l0x_platform_log.c
    ${API_DIR}/platform/src/vl53l0x_platform.c
    ${API_DIR}/core/src/vl53l0x_api_calibration.c
    ${API_DIR}/core/src/vl53l0x_api_core.c
    ${API_DIR}/core/src/vl53l0x_api_ranging.c
    ${API_DIR}/core/src/vl53l0x_api_strings.c
    ${API_DIR}/core/src/vl53l0x_api.c
)

set(API_INCS
    ${API_DIR}/core/inc
    ${API_DIR}/platform/inc
)

idf_component_register(SRCS vl53l0x.c ${API_SRCS}
                    INCLUDE_DIRS "include" ${API_INCS}
                    REQUIRES driver)
