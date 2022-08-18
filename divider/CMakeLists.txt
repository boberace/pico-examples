add_executable(hello_divider
        hello_divider.c
        )

# pull in common dependencies
target_link_libraries(hello_divider pico_stdlib)

# create map/bin/hex file etc.
pico_add_extra_outputs(hello_divider)

# add url via pico_set_program_url
example_auto_set_url(hello_divider)

pico_enable_stdio_usb(hello_divider 1)
pico_enable_stdio_uart(hello_divider 0)