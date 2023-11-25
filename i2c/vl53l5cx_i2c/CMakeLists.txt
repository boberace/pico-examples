add_executable(vl53l5cx_i2c)

target_sources(vl53l5cx_i2c PRIVATE 
        Example_1_ranging_basic.c
        # Example_2_get_set_params.c
        # Example_3_ranging_modes.c
        # Example_4_power_modes.c
        # Example_5_multiple_targets_per_zone.c
        # Example_6_I2C_and_RAM_optimization.c
        # Example_7_calibrate_xtalk.c
        # Example_8_vizualize_xtalk_data.c

        platform.c 
        vl53l5cx_api.c
        vl53l5cx_plugin_xtalk.c
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
