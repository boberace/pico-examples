add_executable(maker_pi_mc)

pico_generate_pio_header(maker_pi_mc ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)

target_sources(maker_pi_mc PRIVATE 
                maker_pi_mc.cpp
                bmc.cpp
                pid.cpp
                i2c_slave.c)

target_link_libraries(maker_pi_mc PRIVATE
        pico_stdlib
        pico_multicore
        hardware_pio
        hardware_pwm
        pico_stdlib hardware_i2c
        )

pico_enable_stdio_usb(maker_pi_mc 1)
pico_enable_stdio_uart(maker_pi_mc 0)

pico_add_extra_outputs(maker_pi_mc)

# add url via pico_set_program_url
example_auto_set_url(maker_pi_mc)
