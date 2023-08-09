add_executable(pca9685emu_i2c
    pca9685emu_i2c.cpp
    )
target_link_libraries(pca9685emu_i2c
    pico_i2c_slave
    hardware_i2c
    pico_stdlib
    )
pico_add_extra_outputs(pca9685emu_i2c)
example_auto_set_url(pca9685emu_i2c)

pico_enable_stdio_usb(slave_mem_i2c 0)
pico_enable_stdio_uart(slave_mem_i2c 1)
