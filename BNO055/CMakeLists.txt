add_executable(BNO055)

target_sources(BNO055 PRIVATE 
        main.cpp
        bno055.c
        bno055_pico.cpp
        )

target_link_libraries(BNO055 PRIVATE
        pico_stdlib
        pico_multicore
        hardware_pio
        hardware_i2c
        hardware_adc
        hardware_pwm
        hardware_irq
        )

pico_enable_stdio_usb(BNO055 1)

pico_add_extra_outputs(BNO055)

# add url via pico_set_program_url
example_auto_set_url(BNO055)
