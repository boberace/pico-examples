add_executable(charlieplex
        charlieplex.c
        )

# pull in common dependencies
target_link_libraries(charlieplex pico_stdlib)

# enable usb output, disable uart output
pico_enable_stdio_usb(charlieplex 1)
pico_enable_stdio_uart(charlieplex 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(charlieplex)

# add url via pico_set_program_url
example_auto_set_url(charlieplex)
