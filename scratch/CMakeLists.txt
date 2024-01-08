add_executable(scratch_flash)

pico_generate_pio_header(scratch_flash ${CMAKE_CURRENT_LIST_DIR}/spi.pio)

target_sources(scratch_flash PRIVATE
        spi_flash.c
        pio_spi.c
        pio_spi.h
        )

target_link_libraries(scratch_flash PRIVATE pico_stdlib hardware_pio)
pico_add_extra_outputs(scratch_flash)

example_auto_set_url(scratch_flash)

add_executable(scratch_loopback)

pico_generate_pio_header(scratch_loopback ${CMAKE_CURRENT_LIST_DIR}/spi.pio)

target_sources(scratch_loopback PRIVATE
        spi_loopback.c
        pio_spi.c
        pio_spi.h
        )

target_link_libraries(scratch_loopback PRIVATE pico_stdlib hardware_pio)
pico_add_extra_outputs(scratch_loopback)

example_auto_set_url(scratch_loopback)
