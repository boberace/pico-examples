add_executable(wave
        main.c
        )

# pull in common dependencies
target_link_libraries(wave pico_stdlib)

# create map/bin/hex file etc.
pico_add_extra_outputs(wave)

# add url via pico_set_program_url
example_auto_set_url(wave)

pico_enable_stdio_usb(wave 1)
pico_enable_stdio_uart(wave 0)
