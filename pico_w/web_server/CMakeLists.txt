
# # add the HTML file to the project
# add_custom_command(
#     OUTPUT index.html
#     COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/index.html ${CMAKE_CURRENT_BINARY_DIR}/index.html
#     DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/index.html
# )

add_executable(picow_web_server_background
        main.c
        )
target_compile_definitions(picow_web_server_background PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )
target_include_directories(picow_web_server_background PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )
target_link_libraries(picow_web_server_background
        pico_cyw43_arch_lwip_threadsafe_background
        pico_stdlib
        pico_lwip_http
        )
pico_add_extra_outputs(picow_web_server_background)
example_auto_set_url(picow_web_server_background)
pico_enable_stdio_usb(picow_web_server_background 1)
pico_enable_stdio_uart(picow_web_server_background 0)

add_executable(picow_web_server_poll
        main.c
        )
target_compile_definitions(picow_web_server_poll PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )
target_include_directories(picow_web_server_poll PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )
target_link_libraries(picow_web_server_poll
        pico_cyw43_arch_lwip_poll
        pico_stdlib
        pico_lwip_http
        )
pico_add_extra_outputs(picow_web_server_poll)
example_auto_set_url(picow_web_server_poll)
pico_enable_stdio_usb(picow_web_server_poll 1)
pico_enable_stdio_uart(picow_web_server_poll 0)
