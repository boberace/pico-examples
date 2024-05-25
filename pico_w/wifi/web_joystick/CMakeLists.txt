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

add_executable(web_joystick_background
        main.c
        mdns_picow.c
        )
target_compile_definitions(web_joystick_background PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )
target_include_directories(web_joystick_background PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )
target_link_libraries(web_joystick_background
        pico_cyw43_arch_lwip_threadsafe_background
        pico_lwip_http
        pico_stdlib
        pico_lwip_mdns
        )

pico_add_extra_outputs(web_joystick_background)

# pico_enable_stdio_usb(web_joystick_background 0)
# pico_enable_stdio_uart(web_joystick_background 1)


