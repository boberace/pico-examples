add_executable(findfun
        findfun.c
        )

# pull in common dependencies
target_link_libraries(findfun 
			pico_stdlib
			)

# create map/bin/hex file etc.
pico_add_extra_outputs(findfun)

# add url via pico_set_program_url
example_auto_set_url(findfun)
