add_executable(mdns_example_background
        main.c
        mdns_picow.c
        )
target_compile_definitions(mdns_example_background PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )
target_include_directories(mdns_example_background PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )
target_link_libraries(mdns_example_background
        pico_cyw43_arch_lwip_threadsafe_background
        pico_stdlib
        pico_lwip_mdns
        )

pico_add_extra_outputs(mdns_example_background)
pico_enable_stdio_usb(mdns_example_background 1)
pico_enable_stdio_uart(mdns_example_background 0)

add_executable(mdns_example_poll
        main.c
        mdns_picow.c
        )
target_compile_definitions(mdns_example_poll PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )
target_include_directories(mdns_example_poll PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )
target_link_libraries(mdns_example_poll
        pico_cyw43_arch_lwip_poll
        pico_stdlib
        pico_lwip_mdns
        )
pico_add_extra_outputs(mdns_example_poll)
pico_enable_stdio_usb(mdns_example_poll 1)
pico_enable_stdio_uart(mdns_example_poll 0)