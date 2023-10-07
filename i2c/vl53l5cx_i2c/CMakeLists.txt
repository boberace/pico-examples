add_executable(vl53l5cx_i2c)

target_sources(vl53l5cx_i2c PRIVATE 
        Example_1_ranging_basic.c
        platform.c 
        vl53l5cx_api.c
        # vl53l5cx_plugin_detection_thresholds.c
        # vl53l5cx_plugin_motion_indicator.c
        # vl53l5cx_plugin_xtalk.c
        )

target_link_libraries(vl53l5cx_i2c PRIVATE
        pico_stdlib
        hardware_i2c

        # pico_multicore
        # hardware_pio
        # hardware_pwm
        # hardware_irq
        )

pico_enable_stdio_usb(vl53l5cx_i2c 1)
pico_enable_stdio_uart(vl53l5cx_i2c 0)

pico_add_extra_outputs(vl53l5cx_i2c)

# add url via pico_set_program_url
example_auto_set_url(vl53l5cx_i2c)
