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
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#define PIN_BNO085_SDA 16
#define PIN_BNO085_SCK  17
#define PIN_BNO085_INT  26
#define PIN_BNO085_RST  14

#define I2C_SDA_PIN PIN_BNO085_SDA //  26 //
#define I2C_SCL_PIN PIN_BNO085_SCK //  27 //
#define I2C i2c0 // depends on pin selection for either zero or one

#define PRINT_PROBE_UART // uncomment to print to uart

#ifdef PRINT_PROBE_UART

#define UART_A_ID uart1
#define UART_A_BAUD_RATE 115200
#define UART_A_TX_PIN 8
#define UART_A_RX_PIN 9

uint setup_uart() {
    uint uart_ret = uart_init(UART_A_ID, UART_A_BAUD_RATE);
    gpio_set_function(UART_A_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_A_RX_PIN, GPIO_FUNC_UART);
    return uart_ret;
}

#define BUFFER_SIZE 256
#define PRINT(...)                     \
    do {                               \
        char buffer[BUFFER_SIZE];      \
        snprintf(buffer, sizeof(buffer), __VA_ARGS__); \
        uart_puts(UART_A_ID, buffer);      \
    } while (0)
#else
#define PRINT(...) printf(__VA_ARGS__)
#endif


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

    #ifdef PRINT_PROBE_UART
    uint uart_ret = setup_uart();
    if(abs((int)(uart_ret - UART_A_BAUD_RATE)) > 0.02*UART_A_BAUD_RATE){
        PRINT("UART setup failed %d\r\n", uart_ret);
        return 1;
    };
    PRINT("UART setup baud rate %d\r\n", uart_ret);
    #endif

    sleep_ms(2000);
    PRINT("\nI2C Bus Scan\r\n");
    PRINT("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\r\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            PRINT("%02x ", addr);
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

        PRINT(ret < 0 ? "." : "@");
        PRINT(addr % 16 == 15 ? "\r\n" : "  ");
    }
    PRINT("Done.\r\n");

    int counter = 0;

    while (1) {

        PRINT("\033[A\33[2K\rbus scan done, %i\n", counter);
        sleep_ms(1000);

        counter++;
    }

    return 0;

}
