add_executable(picotuner
        main.cpp
        # ../findfun.cpp
        # cindex.cpp
        )
pico_generate_pio_header(picotuner ${CMAKE_CURRENT_LIST_DIR}/charlieplex.pio)
# pico_generate_pio_header(picotuner ${CMAKE_CURRENT_LIST_DIR}/blink.pio)
pico_generate_pio_header(picotuner ${CMAKE_CURRENT_LIST_DIR}/strobe.pio)

# pull in common dependencies
target_link_libraries(picotuner 
			pico_stdlib
                        hardware_pio
                        hardware_dma
                        pico_multicore
			)

# create map/bin/hex file etc.
pico_add_extra_outputs(picotuner)

# add url via pico_set_program_url
example_auto_set_url(picotuner)

pico_enable_stdio_usb(picotuner 1)
pico_enable_stdio_uart(picotuner 0)
