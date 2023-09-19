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

#define I2C_SDA_PIN 18 
#define I2C_SCL_PIN 19
#define I2C i2c1 // depends on pin selection for either zero or one

static const uint8_t LP_GPIO[]={0,1,2,3,4,5,6,7};

// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

int main() {
    stdio_init_all();
    i2c_init(I2C, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(I2C_SDA_PIN, I2C_SCL_PIN, GPIO_FUNC_I2C));
    sleep_ms(2000);
    for(uint i=0; i<8; i++){
        gpio_init(LP_GPIO[i]);
        gpio_set_dir(LP_GPIO[i], GPIO_OUT);
        gpio_put(LP_GPIO[i], 0);
    }

    for(uint i=0; i<8; i++){
        gpio_put(LP_GPIO[i], 1);
        sleep_ms(1000);
        printf("\n Start GPIO %d.\n", LP_GPIO[i]);

        printf("\nI2C Bus Scan\n");
        printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

        for (int addr = 0; addr < (1 << 7); ++addr) {
            if (addr % 16 == 0) {
                printf("%02x ", addr);
            }

            // Perform a 1-byte dummy read from the probe address. If a slave
            // acknowledges this address, the function returns the number of bytes
            // transferred. If the address byte is ignored, the function returns
            // -1.

            // Skip over any reserved addresses.
            int ret;
            uint8_t rxdata;
            if (reserved_addr(addr))
                ret = PICO_ERROR_GENERIC;
            else
                // ret = i2c_read_blocking(I2C, addr, &rxdata, 1, false);
                ret = i2c_read_timeout_us(I2C, addr, &rxdata, 1, false, 1000);

            printf(ret < 0 ? "." : "@");
            printf(addr % 16 == 15 ? "\n" : "  ");
        }
        printf(" End GPIO %d.\n", LP_GPIO[i]);
        gpio_put(LP_GPIO[i], 0);
    }

    int counter = 0;

    while (1) {

        printf("\033[A\33[2K\rbus scan done, %i\n", counter);
        sleep_ms(1000);

        counter++;
    }

    return 0;

}
