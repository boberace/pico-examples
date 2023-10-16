if(NOT DEFINED ENV{PICO_UROS_PATH})
    message(FATAL_ERROR "Must set PICO_UROS_PATH to the root of the micro_ros_raspberrypi_pico_sdk.")
else()
        message(STATUS "PICO_UROS_PATH set to $ENV{PICO_UROS_PATH}")
endif()

set(IMU_PATH $ENV{PICO_EXAMPLES_PATH}/i2c/bno085_i2c)

include_directories($ENV{PICO_UROS_PATH})
include_directories(${IMU_PATH})

project(bno085_uros_usb C CXX ASM)

add_executable(bno085_uros_usb
    bno085_uros_usb.cpp
    $ENV{PICO_UROS_PATH}/pico_uart_transport.c
    ${IMU_PATH}/bno085_i2c.cpp
    ${IMU_PATH}/sh2.c
    ${IMU_PATH}/sh2_SensorValue.c
    ${IMU_PATH}/sh2_util.c
    ${IMU_PATH}/shtp.c
)   

target_link_directories(bno085_uros_usb PRIVATE
    $ENV{PICO_UROS_PATH}/libmicroros
    )

target_link_libraries(bno085_uros_usb
    pico_stdlib
    microros
    hardware_i2c
    pico_multicore
    pico_cyw43_arch_none # if using pico-w - need the led driver
)

target_include_directories(bno085_uros_usb PUBLIC
    $ENV{PICO_UROS_PATH}/libmicroros/include
)

# Configure Pico
pico_enable_stdio_usb(bno085_uros_usb 1)
pico_enable_stdio_uart(bno085_uros_usb 1)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(bno085_uros_usb)