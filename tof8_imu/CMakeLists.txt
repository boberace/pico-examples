add_executable(tof8_imu
        main.c
        )

# pull in common dependencies and additional i2c hardware support
target_link_libraries(tof8_imu 
        pico_stdlib 
        hardware_i2c
        )

# create map/bin/hex file etc.
pico_add_extra_outputs(tof8_imu)

# add url via pico_set_program_url
example_auto_set_url(tof8_imu)

pico_enable_stdio_usb(tof8_imu 1)
pico_enable_stdio_uart(tof8_imu 0)
