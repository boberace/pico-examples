add_executable(lis2dw12_i2c
        main.c
        lis2dw12_pico.c
        lis2dw12_reg.c
        )


target_link_libraries(lis2dw12_i2c pico_stdlib hardware_i2c)# pull in common dependencies and additional i2c hardware support


pico_add_extra_outputs(lis2dw12_i2c) # create map/bin/hex file etc.

# add url via pico_set_program_url
example_auto_set_url(lis2dw12_i2c)

pico_enable_stdio_usb(lis2dw12_i2c 1)
pico_enable_stdio_uart(lis2dw12_i2c 1)