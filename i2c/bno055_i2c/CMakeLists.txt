add_executable(bno055_i2c)

target_sources(bno055_i2c PRIVATE 
        main.c
        bno055.c
        bno055_i2c.c
        )

target_link_libraries(bno055_i2c PRIVATE
        pico_stdlib
        pico_multicore
        hardware_pio
        hardware_i2c
        hardware_adc
        hardware_pwm
        hardware_irq
        )

pico_enable_stdio_usb(bno055_i2c 1)

pico_add_extra_outputs(bno055_i2c)

# add url via pico_set_program_url
example_auto_set_url(bno055_i2c)
