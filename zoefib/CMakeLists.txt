include_directories(../tmc_stepsticks)
add_executable(zoefib)

pico_generate_pio_header(zoefib ${CMAKE_CURRENT_LIST_DIR}/pwm.pio)
pico_generate_pio_header(zoefib ${CMAKE_CURRENT_LIST_DIR}/swf.pio)

target_sources(zoefib PRIVATE 
                main.c
                ../tmc_stepsticks/tmc/ic/TMC2208/TMC2208.c
                ../tmc_stepsticks/tmc/helpers/CRC.c
                ../tmc_stepsticks/tmc2208_pico.c
                )

target_link_libraries(zoefib PRIVATE pico_stdlib hardware_pio)
pico_add_extra_outputs(zoefib)

# add url via pico_set_program_url
example_auto_set_url(zoefib)

pico_enable_stdio_usb(zoefib 1)
pico_enable_stdio_uart(zoefib 0)