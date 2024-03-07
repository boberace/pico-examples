add_executable(pio_blink_test)

# by default the header is generated into the build dir
pico_generate_pio_header(pio_blink_test ${CMAKE_CURRENT_LIST_DIR}/pin_blink.pio)
pico_generate_pio_header(pio_blink_test ${CMAKE_CURRENT_LIST_DIR}/pin_monitor.pio)

target_sources(pio_blink_test PRIVATE main.c)

target_link_libraries(pio_blink_test PRIVATE pico_stdlib hardware_pio hardware_dma)

pico_add_extra_outputs(pio_blink_test)

# add url via pico_set_program_url
example_auto_set_url(pio_blink_test)

# enable usb output, disable uart output
pico_enable_stdio_usb(pio_blink_test 1)
pico_enable_stdio_uart(pio_blink_test 0)