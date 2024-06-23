add_executable(spi_sd
        spi_sd.cpp
        hw_config.c 
        )

add_subdirectory(lib/no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI build)

# pull in common dependencies and additional spi hardware support
target_link_libraries(spi_sd 
        pico_stdlib 
        FatFs_SPI
        )

# create map/bin/hex file etc.
pico_add_extra_outputs(spi_sd)

# add url via pico_set_program_url
example_auto_set_url(spi_sd)
