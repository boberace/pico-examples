
message(STATUS "*************************Current source directory: ${CMAKE_CURRENT_SOURCE_DIR}")
include_directories(${CMAKE_CURRENT_SOURCE_DIR})


add_executable(tmc2208_test
        tmc2208_test.c
        tmc/ic/TMC2208/TMC2208.c
        tmc/helpers/CRC.c
        tmc2208_pico.c
        )

pico_generate_pio_header(tmc2208_test ${CMAKE_CURRENT_LIST_DIR}/squarewave.pio)

get_property(dirs TARGET tmc2208_test PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
        message(STATUS "Include directory: '${dir}'")
endforeach()


# pull in common dependencies
target_link_libraries(tmc2208_test 
                        pico_stdlib 
                        hardware_pio
)

# create map/bin/hex file etc.
pico_add_extra_outputs(tmc2208_test)

# add url via pico_set_program_url
example_auto_set_url(tmc2208_test)
pico_enable_stdio_usb(tmc2208_test 1)
pico_enable_stdio_uart(tmc2208_test 0)