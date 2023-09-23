add_executable(picow_pickup_background
        picow_pickup.c
        mdns_picow.c
        pwp_tcp_server.c
        )
target_compile_definitions(picow_pickup_background PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )
target_include_directories(picow_pickup_background PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )
target_link_libraries(picow_pickup_background
        pico_cyw43_arch_lwip_threadsafe_background
        pico_stdlib
        pico_lwip_mdns
        )

pico_add_extra_outputs(picow_pickup_background)
pico_enable_stdio_usb(picow_pickup_background 1)
pico_enable_stdio_uart(picow_pickup_background 0)

add_executable(picow_pickup_poll
        picow_pickup.c
        mdns_picow.c
        pwp_tcp_server.c
        )
target_compile_definitions(picow_pickup_poll PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )
target_include_directories(picow_pickup_poll PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )
target_link_libraries(picow_pickup_poll
        pico_cyw43_arch_lwip_poll
        pico_stdlib
        pico_lwip_mdns
        )
pico_add_extra_outputs(picow_pickup_poll)
pico_enable_stdio_usb(picow_pickup_poll 1)
pico_enable_stdio_uart(picow_pickup_poll 0)