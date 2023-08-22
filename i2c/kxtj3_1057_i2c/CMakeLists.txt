add_executable(kxtj3_1057_i2c
        main.cpp
        kxtj3_1057_i2c.cpp
        )

# pull in common dependencies and additional i2c hardware support
target_link_libraries(kxtj3_1057_i2c pico_stdlib hardware_i2c)

# create map/bin/hex file etc.
pico_add_extra_outputs(kxtj3_1057_i2c)

# add url via pico_set_program_url
example_auto_set_url(kxtj3_1057_i2c)

# enable usb output, disable uart output
pico_enable_stdio_usb(kxtj3_1057_i2c 1)
pico_enable_stdio_uart(kxtj3_1057_i2c 0)
