add_executable(pio_maker_pi_mc)

pico_generate_pio_header(pio_maker_pi_mc ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)

target_sources(pio_maker_pi_mc PRIVATE 
                maker_pi_mc.cpp
                bmc.cpp
                pid.cpp)

target_link_libraries(pio_maker_pi_mc PRIVATE
        pico_stdlib
        pico_multicore
        hardware_pio
        hardware_pwm
        )

pico_enable_stdio_usb(pio_maker_pi_mc 1)
pico_enable_stdio_uart(pio_maker_pi_mc 0)

pico_add_extra_outputs(pio_maker_pi_mc)

# add url via pico_set_program_url
example_auto_set_url(pio_maker_pi_mc)
