add_executable(instrutune
        instrutune.cpp
        ../findfun.cpp
        ssd1306.c
        cindex.cpp
        )

pico_generate_pio_header(instrutune ${CMAKE_CURRENT_LIST_DIR}/blink.pio)
pico_generate_pio_header(instrutune ${CMAKE_CURRENT_LIST_DIR}/strobe.pio)

# pull in common dependencies
target_link_libraries(instrutune 
			pico_stdlib
                        hardware_pio
                        hardware_dma
                        pico_multicore
                        hardware_i2c
			)

# create map/bin/hex file etc.
pico_add_extra_outputs(instrutune)

# add url via pico_set_program_url
example_auto_set_url(instrutune)

pico_enable_stdio_usb(instrutune 1)
pico_enable_stdio_uart(instrutune 0)
