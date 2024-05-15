add_executable(pico_w_webserver_background
        main.c
        mdns_picow.c
        )
target_compile_definitions(pico_w_webserver_background PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )
target_include_directories(pico_w_webserver_background PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )
target_link_libraries(pico_w_webserver_background
        pico_cyw43_arch_lwip_threadsafe_background
        pico_lwip_http
        pico_stdlib
        hardware_adc
        pico_lwip_mdns
        )

# pico_enable_stdio_usb(pico_w_webserver_background TRUE)
# pico_enable_stdio_uart(pico_w_webserver_background FALSE)

pico_add_extra_outputs(pico_w_webserver_background)

add_executable(pico_w_webserver_poll
        main.c
        mdns_picow.c
        )
target_compile_definitions(pico_w_webserver_poll PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )
target_include_directories(pico_w_webserver_poll PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )
target_link_libraries(pico_w_webserver_poll
        pico_cyw43_arch_lwip_poll
        pico_lwip_http
        pico_stdlib
        hardware_adc
        pico_lwip_mdns
        )
pico_add_extra_outputs(pico_w_webserver_poll)

# pico_enable_stdio_usb(pico_w_webserver_poll TRUE)
# pico_enable_stdio_uart(pico_w_webserver_poll FALSE)

message("Running makefsdata python script")
execute_process(COMMAND
    py makefsdata.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
)



