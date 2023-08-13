add_executable(test_periph_i2c
    test_periph_i2c.cpp
    "../../cindex/cindex.cpp"
    )
target_link_libraries(test_periph_i2c
    pico_i2c_slave
    hardware_i2c
    pico_stdlib
    )
pico_add_extra_outputs(test_periph_i2c)
example_auto_set_url(test_periph_i2c)

pico_enable_stdio_usb(test_periph_i2c 1)
pico_enable_stdio_uart(test_periph_i2c 0)


