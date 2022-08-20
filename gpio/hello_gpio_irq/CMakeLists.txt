add_executable(hello_gpio_irq
        hello_gpio_irq.c
        )

# pull in common dependencies
target_link_libraries(hello_gpio_irq 
                        pico_stdlib
                        hardware_irq)

# create map/bin/hex file etc.
pico_add_extra_outputs(hello_gpio_irq)

# add url via pico_set_program_url
example_auto_set_url(hello_gpio_irq)

pico_enable_stdio_usb(hello_gpio_irq 1)
pico_enable_stdio_uart(hello_gpio_irq 0)