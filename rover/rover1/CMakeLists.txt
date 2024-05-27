add_executable(rover1
        rover1.c
        )

target_link_libraries(rover1
        pico_stdlib              # for core functionality
        pico_cyw43_arch_none     # we need Wifi to access the GPIO, but we don't need anything else
        hardware_pwm
        )

# create map/bin/hex file etc.
pico_add_extra_outputs(rover1)

# add url via pico_set_program_url
example_auto_set_url(rover1)

pico_enable_stdio_usb(rover1 1)
pico_enable_stdio_uart(rover1 0)