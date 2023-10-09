if(NOT DEFINED ENV{PICO_UROS_PATH})
    message(FATAL_ERROR "Must set PICO_UROS_PATH to the root of the micro_ros_raspberrypi_pico_sdk.")
else()
        message(STATUS "PICO_UROS_PATH set to $ENV{PICO_UROS_PATH}")
endif()

include_directories($ENV{PICO_UROS_PATH})

project(wifi_example_uros C CXX ASM)

add_executable(wifi_example_uros
    wifi_example_uros.c
    $ENV{PICO_UROS_PATH}/pico_wifi_transport.c
)   

target_compile_definitions(wifi_example_uros PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )

target_link_directories(wifi_example_uros PRIVATE
    $ENV{PICO_UROS_PATH}/libmicroros
    )

target_link_libraries(wifi_example_uros
    pico_stdlib
    microros
    pico_cyw43_arch_lwip_threadsafe_background
)

target_include_directories(wifi_example_uros PUBLIC
    $ENV{PICO_UROS_PATH}/libmicroros/include
)

# Configure Pico
pico_enable_stdio_usb(wifi_example_uros 1)
pico_enable_stdio_uart(wifi_example_uros 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(wifi_example_uros)

if (PICO_CYW43_SUPPORTED) # set by PICO_BOARD=pico_w
    if (NOT TARGET pico_cyw43_arch)
        message("Skipping Pico W examples as support is not available")
    else()
        message("+Creating Pico W projects")
        if (DEFINED ENV{WIFI_SSID} AND (NOT WIFI_SSID))
            set(WIFI_SSID $ENV{WIFI_SSID})
            message("Using WIFI_SSID from environment ('${WIFI_SSID}')")
        endif()

        if (DEFINED ENV{WIFI_PASSWORD} AND (NOT WIFI_PASSWORD))
            set(WIFI_PASSWORD $ENV{WIFI_PASSWORD})
            message("Using WIFI_PASSWORD from environment")
        endif()

        set(WIFI_SSID "${WIFI_SSID}" CACHE INTERNAL "WiFi SSID for examples")
        set(WIFI_PASSWORD "${WIFI_PASSWORD}" CACHE INTERNAL "WiFi password for examples")

    endif()
else()
    message("*** PICO_CYW43_SUPPORTED not set, skipping Pico W projects")
endif()