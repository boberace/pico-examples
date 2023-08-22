add_executable(i2c_bus_test
        bus_test.c
        )

# pull in common dependencies and additional i2c hardware support
target_link_libraries(i2c_bus_test pico_stdlib hardware_i2c)

# create map/bin/hex file etc.
pico_add_extra_outputs(i2c_bus_test)

# add url via pico_set_program_url
example_auto_set_url(i2c_bus_test)

pico_enable_stdio_usb(i2c_bus_test 1)
pico_enable_stdio_uart(i2c_bus_test 0)
