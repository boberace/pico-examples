add_executable(st_bno085)

pico_generate_pio_header(st_bno085 ${CMAKE_CURRENT_LIST_DIR}/hall.pio)

target_sources(st_bno085 PRIVATE 
        st_bno085.cpp
        ssd1306.c
        bno085_i2c.cpp
        sh2.c
        sh2_SensorValue.c
        sh2_util.c
        shtp.c
        )

target_link_libraries(st_bno085 PRIVATE
        pico_stdlib
        pico_multicore
        hardware_pio
        hardware_i2c
        hardware_adc
        hardware_pwm
        hardware_irq
        )

pico_enable_stdio_usb(st_bno085 1)

pico_add_extra_outputs(st_bno085)

# add url via pico_set_program_url
example_auto_set_url(st_bno085)
