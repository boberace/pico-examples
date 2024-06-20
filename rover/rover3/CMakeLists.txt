message("Running generate_html_response python script")
execute_process(
    COMMAND python3 generate_html_response.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    RESULT_VARIABLE script_result
    OUTPUT_VARIABLE script_output
    ERROR_VARIABLE script_error
)

if(NOT script_result EQUAL 0)
    message(FATAL_ERROR "Python script failed with output: ${script_output} and error: ${script_error}")
endif()

add_executable(rover3_background
        rover3.cpp
        messages.c
        html_server.c
        mdns_picow.c
        pid.cpp
        )

pico_generate_pio_header(rover3_background ${CMAKE_CURRENT_LIST_DIR}/pin_encoder.pio)

target_compile_definitions(rover3_background PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )

target_include_directories(rover3_background PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )

target_link_libraries(rover3_background
        pico_cyw43_arch_lwip_threadsafe_background
        pico_stdlib
        pico_lwip_mdns
        pico_multicore
        hardware_pwm
        # hardware_pio
        )

pico_add_extra_outputs(rover3_background)

# pico_enable_stdio_usb(rover3_background 0)
# pico_enable_stdio_uart(rover3_background 1)
