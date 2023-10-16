add_executable(bno085_i2c)

target_sources(bno085_i2c PRIVATE 
        main.cpp
        bno085_i2c.cpp
        sh2.c
        sh2_SensorValue.c
        sh2_util.c
        shtp.c
        )

target_link_libraries(bno085_i2c PRIVATE
        pico_stdlib
        hardware_i2c
        )

pico_enable_stdio_usb(bno085_i2c 0)

pico_add_extra_outputs(bno085_i2c)

# add url via pico_set_program_url
example_auto_set_url(bno085_i2c)
