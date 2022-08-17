add_executable(example_ff
        example_ff.cpp
        findfun.cpp
        )

# pull in common dependencies
target_link_libraries(example_ff 
			pico_stdlib
			)

# create map/bin/hex file etc.
pico_add_extra_outputs(example_ff)

# add url via pico_set_program_url
example_auto_set_url(example_ff)

pico_enable_stdio_usb(example_ff 1)
pico_enable_stdio_uart(example_ff 0)
