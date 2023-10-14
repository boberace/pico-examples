add_executable(scratch
        main.c
        )

# pull in common dependencies
target_link_libraries(scratch pico_stdlib)

# create map/bin/hex file etc.
pico_add_extra_outputs(scratch)

# add url via pico_set_program_url
example_auto_set_url(scratch)
pico_enable_stdio_usb(scratch 1)
pico_enable_stdio_uart(scratch 0)