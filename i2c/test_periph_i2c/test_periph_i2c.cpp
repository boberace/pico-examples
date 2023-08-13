/*
 * Copyright (c) 2021 Valentin Milea <valentin.milea@gmail.com>
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * Modifed by others
 */

#include <hardware/i2c.h>
#include "hardware/clocks.h"
#include <pico/i2c_slave.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>
#include "../../cindex/cindex.h"

#define num_mem_bytes 256
#define DEBUG

static const uint I2C_SLAVE_ADDRESS = 0x40;
static const uint I2C_BAUDRATE = 100000; // 100 kHz
static const uint I2C_SLAVE_SDA_PIN = 18; // 4
static const uint I2C_SLAVE_SCL_PIN = 19; // 5
i2c_inst_t* I2CC = i2c1;

#ifdef DEBUG
char temp_arr[32][9];
uint8_t temp_counter = 0;
#endif

// The slave implements byte memory size num_mem_bytes. To write a series of bytes, the master first
// writes the memory address, followed by the data. The address is automatically incremented
// for each byte transferred, looping back to 0 upon reaching the end. Reading is done
// sequentially from the current memory address.
static struct
{
    uint8_t mem[num_mem_bytes] = {0};
    cindex mem_address = num_mem_bytes;
    bool mem_address_written;
    uint8_t num_bytes_written = 0;
} context;

// Our handler is called from the I2C ISR, so it must complete quickly. Blocking calls /
// printing to stdio may interfere with interrupt handling.
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
    case I2C_SLAVE_RECEIVE: // master has written some data
        if (!context.mem_address_written) {
            // writes always start with the memory address
            context.mem_address = i2c_read_byte_raw(i2c);
            context.mem_address_written = true;
                #ifdef DEBUG
                strcpy(temp_arr[temp_counter], "address\n");
                temp_counter++;
                #endif
        } else { 

            context.mem[context.mem_address++] = i2c_read_byte_raw(i2c);
            context.num_bytes_written++;
#ifdef DEBUG
                strcpy(temp_arr[temp_counter], "read\n");
                temp_counter++;
#endif
        }
        break;
    case I2C_SLAVE_REQUEST: // master is requesting data
        // load from memory
        i2c_write_byte_raw(i2c, context.mem[context.mem_address]);
        context.mem_address++;
#ifdef DEBUG
            strcpy(temp_arr[temp_counter], "write\n");
            temp_counter++;
#endif
        break;
    case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
        context.mem_address_written = false;
#ifdef DEBUG
            strcpy(temp_arr[temp_counter], "finish\n");
            temp_counter++;
#endif
        break;
    default:
#ifdef DEBUG
            strcpy(temp_arr[temp_counter], "unkown\n");
            temp_counter++;
#endif
        break;
    }
}

static void setup_slave() {
    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);

    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    i2c_init(I2CC, I2C_BAUDRATE);
    // configure I2CC for slave mode
    i2c_slave_init(I2CC, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
}

int main() {
    stdio_init_all();
    setup_slave();
    const uint LED_PIN = 25;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    uint32_t pus = time_us_32();
    while(1){
        uint32_t cus = time_us_32();
        if(cus - pus >= 500000){
            pus=cus;
            gpio_put(LED_PIN,!gpio_get(LED_PIN));
#ifdef DEBUG
        for(int i=0;i<32;i++){
            printf("%i %s",i,temp_arr[i]);
        }            
        printf("--------------------\n");
    uint32_t core_clock = clk_sys;

    printf("CPU Core Clock: %i Hz\n", clock_get_hz(clock_index(core_clock)));
#endif      
        }
    }
}
