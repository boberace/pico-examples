add_executable(charlieplex)

pico_generate_pio_header(charlieplex ${CMAKE_CURRENT_LIST_DIR}/charlieplex.pio)

target_sources(charlieplex PRIVATE 
                charlieplex.c)

# pull in common dependencies
target_link_libraries(charlieplex PRIVATE
                        pico_stdlib
                        hardware_pio
                        hardware_dma
                        hardware_irq
                        )

# enable usb output, disable uart output
pico_enable_stdio_usb(charlieplex 1)
pico_enable_stdio_uart(charlieplex 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(charlieplex)

# add url via pico_set_program_url
example_auto_set_url(charlieplex)
