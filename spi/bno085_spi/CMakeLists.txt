add_executable(bno085_spi)

target_sources(bno085_spi PRIVATE 
        main.cpp
        bno085_spi.cpp
        sh2.c
        sh2_SensorValue.c
        sh2_util.c
        shtp.c
        )
# pull in common dependencies and additional spi hardware support
target_link_libraries(bno085_spi 
        pico_stdlib 
        hardware_spi
        )

# create map/bin/hex file etc.
pico_add_extra_outputs(bno085_spi)

# add url via pico_set_program_url
example_auto_set_url(bno085_spi)

pico_enable_stdio_usb(bno085_spi 1)
pico_enable_stdio_uart(bno085_spi 0)