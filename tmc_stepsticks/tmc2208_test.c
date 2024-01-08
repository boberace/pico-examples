/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"

#include "tmc2208_pico.h"
#include "tmc/helpers/Macros.h"
#include "tmc/ic/TMC2208/TMC2208_Fields.h"


#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "squarewave.pio.h"


#define PIN_LED 18
#define PIN_DIR 20
#define PIN_STEP 19


int main() {

    stdio_init_all();
    sleep_ms(2000);

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 0);
    // gpio_init(PIN_STEP);
    // gpio_set_dir(PIN_STEP, GPIO_OUT);
    // gpio_put(PIN_STEP, 0);
    // gpio_init(PIN_DIR);
    // gpio_set_dir(PIN_DIR, GPIO_OUT);
    // gpio_put(PIN_DIR, 0);

    tmc2208_pico_init();   

    tmc2208_write_register(TMC2208_GCONF, 0b0111000010);


    PIO pio = pio0;
    for (int i = 0; i < count_of(squarewave_program_instructions); ++i)
        pio->instr_mem[i] = squarewave_program_instructions[i];

    // 0xffff0000 [31:16] : INT (1): Effective frequency is sysclk/(int + frac/256)
    // 0x0000ff00 [15:8]  : FRAC (0): Fractional part of clock divisor
    uint32_t sys_clk_hz = clock_get_hz(clk_sys);
    float div = sys_clk_hz / (float)(256*4*200);
    uint16_t int_div = div;  
    uint8_t frac_div = ((div - (float)int_div) * (float)256); 

    printf("sys_clk_hz: %d, div: %f, int_div: %i, frac_div: %i\n\n\n", sys_clk_hz, div, int_div, frac_div);

    pio->sm[0].clkdiv = (uint32_t) ( (0xFFFF << 16) | (frac_div << 8)); 
    pio->sm[0].pinctrl =
            (1 << PIO_SM0_PINCTRL_SET_COUNT_LSB) |
            (PIN_STEP << PIO_SM0_PINCTRL_SET_BASE_LSB);
    gpio_set_function(PIN_STEP, GPIO_FUNC_PIO0);
    hw_set_bits(&pio->ctrl, 1 << (PIO_CTRL_SM_ENABLE_LSB + 0));
    

     for(int i=0xFFFF; i > int_div ; i = i >> 1){
        pio->sm[0].clkdiv = (uint32_t) (i << 16); 
        printf(".");
        sleep_ms(100);
    }

    pio->sm[0].clkdiv = (uint32_t) ( (int_div << 16) | (frac_div << 8)); 



    // for(int i=0; i < 1500000 ; i=i+1000){
    //     tmc2208_set_VACTUAL(i);
    //     sleep_ms(2);
    // }

    int counter = 0;

    while (1) {

        printf("\033[A\33[2K\rtmc2208 test, %i\n", counter);
        sleep_ms(1000);

        counter++;
    }

}



