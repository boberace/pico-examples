add_executable(st_bno055)

pico_generate_pio_header(st_bno055 ${CMAKE_CURRENT_LIST_DIR}/hall.pio)

target_sources(st_bno055 PRIVATE 
        st_bno055.cpp
        ssd1306.c
        )

target_link_libraries(st_bno055 PRIVATE
        pico_stdlib
        pico_multicore
        hardware_pio
        hardware_i2c
        hardware_adc
        hardware_pwm
        hardware_irq
        )

pico_enable_stdio_usb(st_bno055 1)

pico_add_extra_outputs(st_bno055)

# add url via pico_set_program_url
example_auto_set_url(st_bno055)
