set(ADCDAC_PATH $ENV{PICO_EXAMPLES_PATH}/i2c/ad5593r_i2c)
include_directories(${ADCDAC_PATH})

add_executable(scratch
        main.c
        ${ADCDAC_PATH}/ad5593r.c
        ${ADCDAC_PATH}/ad5592r-base.c
        ${ADCDAC_PATH}/ad5593r_pico.c
        ${ADCDAC_PATH}/ad5593r_pico_i2c.c
        )

# pull in common dependencies
target_link_libraries(scratch 
        pico_stdlib 
        hardware_i2c
        )

# create map/bin/hex file etc.
pico_add_extra_outputs(scratch)

# add url via pico_set_program_url
example_auto_set_url(scratch)
pico_enable_stdio_usb(scratch 1)
pico_enable_stdio_uart(scratch 0)