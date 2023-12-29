add_executable(scratch
        main.c
        )

target_link_libraries(scratch
        pico_stdlib              # for core functionality
        pico_cyw43_arch_none     # we need Wifi to access the GPIO, but we don't need anything else
        )

# create map/bin/hex file etc.
pico_add_extra_outputs(scratch)

# add url via pico_set_program_url
example_auto_set_url(scratch)
