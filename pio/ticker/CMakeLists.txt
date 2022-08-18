add_executable(pio_ticker)

pico_generate_pio_header(pio_ticker ${CMAKE_CURRENT_LIST_DIR}/blink.pio)
pico_generate_pio_header(pio_ticker ${CMAKE_CURRENT_LIST_DIR}/hall.pio)

target_sources(pio_ticker PRIVATE 
                ticker.c)

target_link_libraries(pio_ticker PRIVATE 
                        pico_stdlib 
                        hardware_pio
                        hardware_irq
                        )

pico_add_extra_outputs(pio_ticker)

# add url via pico_set_program_url
example_auto_set_url(pio_ticker)

pico_enable_stdio_usb(pio_ticker 1)
pico_enable_stdio_uart(pio_ticker 0)