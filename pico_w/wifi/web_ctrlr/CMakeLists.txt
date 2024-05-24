message("Running makefsdata python script")
execute_process(COMMAND
    python3 makefsdata.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
)

add_executable(web_ctrlr_background
        main.c
        mdns_picow.c
        )
target_compile_definitions(web_ctrlr_background PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )
target_include_directories(web_ctrlr_background PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )
target_link_libraries(web_ctrlr_background
        pico_cyw43_arch_lwip_threadsafe_background
        pico_lwip_http
        pico_stdlib
        pico_lwip_mdns

        )

pico_add_extra_outputs(web_ctrlr_background)

# pico_enable_stdio_usb(web_ctrlr_background 0)
# pico_enable_stdio_uart(web_ctrlr_background 1)

add_executable(web_ctrlr_poll
        main.c
        mdns_picow.c
        )
target_compile_definitions(web_ctrlr_poll PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )
target_include_directories(web_ctrlr_poll PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )
target_link_libraries(web_ctrlr_poll
        pico_cyw43_arch_lwip_poll
        pico_lwip_http
        pico_stdlib
        pico_lwip_mdns

        )
pico_add_extra_outputs(web_ctrlr_poll)

# pico_enable_stdio_usb(web_ctrlr_poll 0)
# pico_enable_stdio_uart(web_ctrlr_poll 1)

