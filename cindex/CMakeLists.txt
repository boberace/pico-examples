add_executable(cindex
        hello_cindex.cpp
        cindex.cpp
        )

# pull in common dependencies
target_link_libraries(cindex 
			pico_stdlib
			)

# create map/bin/hex file etc.
pico_add_extra_outputs(cindex)

# add url via pico_set_program_url
example_auto_set_url(cindex)

pico_enable_stdio_usb(cindex 1)
pico_enable_stdio_uart(cindex 0)
