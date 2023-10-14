if(NOT DEFINED ENV{PICO_UROS_PATH})
    message(FATAL_ERROR "Must set PICO_UROS_PATH to the root of the micro_ros_raspberrypi_pico_sdk.")
else()
        message(STATUS "PICO_UROS_PATH set to $ENV{PICO_UROS_PATH}")
endif()

include_directories($ENV{PICO_UROS_PATH})

project(usb_example_uros C CXX ASM)

add_executable(usb_example_uros
    usb_example_uros.c
    $ENV{PICO_UROS_PATH}/pico_uart_transport.c
)   

target_link_directories(usb_example_uros PRIVATE
    $ENV{PICO_UROS_PATH}/libmicroros
    )

target_link_libraries(usb_example_uros
    pico_stdlib
    microros
)

target_include_directories(usb_example_uros PUBLIC
    $ENV{PICO_UROS_PATH}/libmicroros/include
)

# Configure Pico
pico_enable_stdio_usb(usb_example_uros 1)
pico_enable_stdio_uart(usb_example_uros 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(usb_example_uros)