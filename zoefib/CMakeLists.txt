include_directories(../tmc_stepsticks)
add_executable(scratch)

pico_generate_pio_header(scratch ${CMAKE_CURRENT_LIST_DIR}/pwm.pio)
pico_generate_pio_header(scratch ${CMAKE_CURRENT_LIST_DIR}/swf.pio)

target_sources(scratch PRIVATE 
                main.c
                ../tmc_stepsticks/tmc/ic/TMC2208/TMC2208.c
                ../tmc_stepsticks/tmc/helpers/CRC.c
                ../tmc_stepsticks/tmc2208_pico.c
                )

target_link_libraries(scratch PRIVATE pico_stdlib hardware_pio)
pico_add_extra_outputs(scratch)

# add url via pico_set_program_url
example_auto_set_url(scratch)

pico_enable_stdio_usb(scratch 1)
pico_enable_stdio_uart(scratch 0)