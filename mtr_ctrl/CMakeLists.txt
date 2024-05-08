add_executable(mtr_ctrl_s1)

# by default the header is generated into the build dir
pico_generate_pio_header(mtr_ctrl_s1 ${CMAKE_CURRENT_LIST_DIR}/pin_blink.pio)
# pico_generate_pio_header(mtr_ctrl_s1 ${CMAKE_CURRENT_LIST_DIR}/pin_monitor.pio)

target_sources(mtr_ctrl_s1 PRIVATE mtr_ctrl_s1.c)

target_link_libraries(mtr_ctrl_s1 PRIVATE pico_stdlib hardware_pio hardware_dma)

pico_add_extra_outputs(mtr_ctrl_s1)

# add url via pico_set_program_url
example_auto_set_url(mtr_ctrl_s1)

# enable usb output, disable uart output
pico_enable_stdio_usb(mtr_ctrl_s1 1)
pico_enable_stdio_uart(mtr_ctrl_s1 0)


add_executable(mtr_ctrl_brushed)

# by default the header is generated into the build dir
pico_generate_pio_header(mtr_ctrl_brushed ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)

target_sources(mtr_ctrl_brushed PRIVATE mtr_ctrl_brushed.c)

target_link_libraries(mtr_ctrl_brushed PRIVATE pico_stdlib hardware_pio hardware_dma)

pico_add_extra_outputs(mtr_ctrl_brushed)

# add url via pico_set_program_url
example_auto_set_url(mtr_ctrl_brushed)

# enable usb output, disable uart output
pico_enable_stdio_usb(mtr_ctrl_brushed 1)
# pico_enable_stdio_uart(mtr_ctrl_brushed 0)