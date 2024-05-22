if (NOT TEST_TCP_SERVER_IP)
    message("Skipping tcp_client example as TEST_TCP_SERVER_IP is not defined")
else()
    add_executable(picow_tcpip_client_ws_background
                main.cpp
                ws.cpp
            )
    target_compile_definitions(picow_tcpip_client_ws_background PRIVATE
            WIFI_SSID=\"${WIFI_SSID}\"
            WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
            TEST_TCP_SERVER_IP=\"${TEST_TCP_SERVER_IP}\"
            )
    target_include_directories(picow_tcpip_client_ws_background PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}
            ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
            )
    target_link_libraries(picow_tcpip_client_ws_background
            pico_cyw43_arch_lwip_threadsafe_background
            pico_stdlib
            )

    pico_add_extra_outputs(picow_tcpip_client_ws_background)

    add_executable(picow_tcpip_client_ws_poll
                main.cpp
                ws.cpp
            )
    target_compile_definitions(picow_tcpip_client_ws_poll PRIVATE
            WIFI_SSID=\"${WIFI_SSID}\"
            WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
            TEST_TCP_SERVER_IP=\"${TEST_TCP_SERVER_IP}\"
            )
    target_include_directories(picow_tcpip_client_ws_poll PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}
            ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
            )
    target_link_libraries(picow_tcpip_client_ws_poll
            pico_cyw43_arch_lwip_poll
            pico_stdlib
            )
    pico_add_extra_outputs(picow_tcpip_client_ws_poll)
endif()
