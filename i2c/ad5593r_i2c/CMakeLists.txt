add_executable(ad5593r_i2c)

target_sources(ad5593r_i2c PRIVATE 
        main.c
        ad5592r-base.c
        ad5593r.c
        no_os_i2c.c
        )

target_link_libraries(ad5593r_i2c PRIVATE
        pico_stdlib
        pico_multicore
        hardware_pio
        hardware_i2c
        hardware_pwm
        hardware_irq
        )

pico_enable_stdio_usb(ad5593r_i2c 1)

pico_add_extra_outputs(ad5593r_i2c)

# add url via pico_set_program_url
example_auto_set_url(ad5593r_i2c)
