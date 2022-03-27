add_executable(spin_test)

pico_generate_pio_header(spin_test ${CMAKE_CURRENT_LIST_DIR}/hall.pio)

target_sources(spin_test PRIVATE 
        spin_test.cpp
        ssd1306.c
        bno055.c
        bno055_i2c.c
        )

target_link_libraries(spin_test PRIVATE
        pico_stdlib
        pico_multicore
        hardware_pio
        hardware_i2c
        hardware_adc
        hardware_pwm
        hardware_irq
        )

pico_enable_stdio_usb(spin_test 1)

pico_add_extra_outputs(spin_test)

# add url via pico_set_program_url
example_auto_set_url(spin_test)
