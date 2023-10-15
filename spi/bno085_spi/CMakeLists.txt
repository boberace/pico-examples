add_executable(bno085_spi_cpp)

target_sources(bno085_spi_cpp PRIVATE 
        main.cpp
        bno085_spi.cpp
        sh2.c
        sh2_SensorValue.c
        sh2_util.c
        shtp.c
        )
# pull in common dependencies and additional spi hardware support
target_link_libraries(bno085_spi_cpp 
        pico_stdlib 
        hardware_spi
        )

# create map/bin/hex file etc.
pico_add_extra_outputs(bno085_spi_cpp)

# add url via pico_set_program_url
example_auto_set_url(bno085_spi_cpp)

pico_enable_stdio_usb(bno085_spi_cpp 1)
pico_enable_stdio_uart(bno085_spi_cpp 0)
