
add_executable(web_slider_background
        main.c
        # mdns_picow.c
        )
target_compile_definitions(web_slider_background PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )
target_include_directories(web_slider_background PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )
target_link_libraries(web_slider_background
        pico_cyw43_arch_lwip_threadsafe_background
        # pico_lwip_http
        pico_stdlib
        # pico_lwip_mdns

        )

pico_add_extra_outputs(web_slider_background)

# pico_enable_stdio_usb(web_slider_background 0)
# pico_enable_stdio_uart(web_slider_background 1)

add_executable(web_slider_poll
        main.c
        # mdns_picow.c
        )
target_compile_definitions(web_slider_poll PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )
target_include_directories(web_slider_poll PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )
target_link_libraries(web_slider_poll
        pico_cyw43_arch_lwip_poll
        # pico_lwip_http
        pico_stdlib
        # pico_lwip_mdns

        )
pico_add_extra_outputs(web_slider_poll)

# pico_enable_stdio_usb(web_slider_poll 0)
# pico_enable_stdio_uart(web_slider_poll 1)

