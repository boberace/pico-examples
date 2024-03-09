add_executable(scratch)

# by default the header is generated into the build dir
pico_generate_pio_header(scratch ${CMAKE_CURRENT_LIST_DIR}/pin_blink.pio)
pico_generate_pio_header(scratch ${CMAKE_CURRENT_LIST_DIR}/pin_monitor.pio)

target_sources(scratch PRIVATE main.c)

target_link_libraries(scratch PRIVATE pico_stdlib hardware_pio hardware_dma)

pico_add_extra_outputs(scratch)

# add url via pico_set_program_url
example_auto_set_url(scratch)

# enable usb output, disable uart output
pico_enable_stdio_usb(scratch 1)
pico_enable_stdio_uart(scratch 0)