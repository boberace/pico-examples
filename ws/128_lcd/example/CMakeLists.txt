add_subdirectory(./lib/Config)
add_subdirectory(./lib/LCD)
add_subdirectory(./lib/Fonts)
add_subdirectory(./lib/GUI)
add_subdirectory(./lib/QMI8658)
add_subdirectory(./examples)

include_directories(./examples)
include_directories(./lib/Config)
include_directories(./lib/Fonts)
include_directories(./lib/GUI)
include_directories(./lib/LCD)
include_directories(./lib/QMI8658)

add_executable(example
        main.c
        )

# pull in common dependencies
target_link_libraries(example 
            examples 
            LCD QMI8658 
            GUI 
            Fonts 
            Config 
            pico_stdlib 
            hardware_spi 
            hardware_i2c
			)

# create map/bin/hex file etc.
pico_add_extra_outputs(example)

# add url via pico_set_program_url
example_auto_set_url(example)

pico_enable_stdio_usb(example 1)
pico_enable_stdio_uart(example 0)