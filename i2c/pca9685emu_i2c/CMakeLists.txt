if (NOT FREERTOS_KERNEL_PATH AND NOT DEFINED ENV{FREERTOS_KERNEL_PATH})
    message("Skipping Pico FreeRTOS examples as FREERTOS_KERNEL_PATH not defined")
else()
    add_executable(pca9685emu_i2c
        pca9685emu_i2c.cpp
        "../../cindex/cindex.cpp"
        )
    target_link_libraries(pca9685emu_i2c
        hardware_pwm
        pico_i2c_slave
        hardware_i2c
        pico_stdlib
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
        )
    target_include_directories(pca9685emu_i2c PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}
            )
    include(FreeRTOS_Kernel_import.cmake)
    pico_add_extra_outputs(pca9685emu_i2c)
    example_auto_set_url(pca9685emu_i2c)
    pico_enable_stdio_usb(pca9685emu_i2c 1)
    pico_enable_stdio_uart(pca9685emu_i2c 0)
endif()