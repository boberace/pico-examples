add_executable(m24c64_i2c)

target_sources(m24c64_i2c PRIVATE         
        main.c
        )

target_link_libraries(m24c64_i2c PRIVATE
        pico_stdlib
        hardware_i2c
        )

pico_enable_stdio_usb(m24c64_i2c 1)

pico_add_extra_outputs(m24c64_i2c)

# add url via pico_set_program_url
example_auto_set_url(m24c64_i2c)
