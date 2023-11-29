if(NOT DEFINED ENV{PICO_UROS_PATH})
    message(FATAL_ERROR "Must set PICO_UROS_PATH to the root of the micro_ros_raspberrypi_pico_sdk.")
else()
        message(STATUS "PICO_UROS_PATH set to $ENV{PICO_UROS_PATH}")
endif()

set(TOF_PATH $ENV{PICO_EXAMPLES_PATH}/i2c/vl53l5cx_i2c)

include_directories($ENV{PICO_UROS_PATH})
include_directories(${TOF_PATH})

project(tof8_imu_uros_usb C CXX ASM)

link_directories(libmicroros)

add_executable(tof8_imu_uros_usb
    tof8_imu_uros_usb.cpp
    $ENV{PICO_UROS_PATH}/pico_uart_transport.c
    ${TOF_PATH}/platform.c 
    ${TOF_PATH}/vl53l5cx_api.c
    ${TOF_PATH}/vl53l5cx_plugin_xtalk.c    
)   

target_link_directories(tof8_imu_uros_usb PUBLIC
    $ENV{PICO_UROS_PATH}/libmicroros
    )

target_link_libraries(tof8_imu_uros_usb
    pico_stdlib
    microros
    hardware_i2c
    pico_multicore
    pico_cyw43_arch_none # if using pico-w - need the led driver
)


target_include_directories(tof8_imu_uros_usb PUBLIC
    $ENV{PICO_UROS_PATH}/libmicroros/include

)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(tof8_imu_uros_usb 1)
pico_enable_stdio_uart(tof8_imu_uros_usb 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(tof8_imu_uros_usb)