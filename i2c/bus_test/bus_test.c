/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Sweep through all 7-bit I2C addresses, to see if any slaves are present on
// the I2C bus. Print out a table that looks like this:
//
// I2C Bus Scan
//   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
// 0
// 1       @
// 2
// 3             @
// 4
// 5
// 6
// 7
//
// E.g. if slave addresses 0x12 and 0x34 were acknowledged.

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"


#define I2C_PORT i2c1
#define I2C_SDA_PIN 26
#define I2C_SCL_PIN 27
#define I2C_SPEED 1000000

#define KXTJ3_ADDRESS 0x0E 
#define M24C64_ADDRESS 0x50 

int main() {
    stdio_init_all();
    sleep_ms(2000);
    printf("KXTJ3 M24C64 test\n");

    i2c_init(I2C_PORT, I2C_SPEED);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    uint8_t data;



    // Check communication with KXTJ3
    int result_kxtj3 = i2c_read_timeout_us(I2C_PORT, KXTJ3_ADDRESS, &data, 1, false, 10000);
    if (result_kxtj3 == PICO_ERROR_GENERIC) {
        printf("Failed to communicate with KXTJ3\n");
    } else {
        printf("Successfully communicated with KXTJ3\n");
    }

    // Check communication with M24C64
    int result_m24c64 = i2c_read_timeout_us(I2C_PORT, M24C64_ADDRESS, &data, 1, false, 10000);
    if (result_m24c64 == PICO_ERROR_GENERIC) {
        printf("Failed to communicate with M24C64\n");
    } else {
        printf("Successfully communicated with M24C64\n");
    }

    while (1) {
        // Keep the program running
        tight_loop_contents();
    }

    return 0;
}