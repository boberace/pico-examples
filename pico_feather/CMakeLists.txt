add_executable(pico_feather
        main.c
        )

# pull in common dependencies and additional i2c hardware support
target_link_libraries(pico_feather 
        pico_stdlib 
        hardware_i2c
        hardware_adc
        hardware_dma
        )

# create map/bin/hex file etc.
pico_add_extra_outputs(pico_feather)

# add url via pico_set_program_url
example_auto_set_url(pico_feather)
