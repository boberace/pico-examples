add_executable(siggen
        siggen.cpp
        ssd1306.c
        cindex.cpp
        )

# pull in common dependencies
target_link_libraries(siggen 
			pico_stdlib
                        pico_multicore
                        hardware_i2c
			)

# create map/bin/hex file etc.
pico_add_extra_outputs(siggen)

# add url via pico_set_program_url
example_auto_set_url(siggen)

pico_enable_stdio_usb(siggen 1)
pico_enable_stdio_uart(siggen 0)
