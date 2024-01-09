add_executable(adc_spi_pio)

pico_generate_pio_header(adc_spi_pio ${CMAKE_CURRENT_LIST_DIR}/spi.pio)

target_sources(adc_spi_pio PRIVATE
        main.c
        pio_spi.c
        pio_spi.h
        )

target_link_libraries(adc_spi_pio PRIVATE pico_stdlib hardware_pio)
pico_add_extra_outputs(adc_spi_pio)

example_auto_set_url(adc_spi_pio)

pico_enable_stdio_usb(adc_spi_pio 0)
pico_enable_stdio_uart(adc_spi_pio 1)
