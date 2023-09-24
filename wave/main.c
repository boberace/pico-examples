/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include <math.h>

volatile bool timer_fired = false;

#define SINE_WAVE_TABLE_LEN 2048
static uint8_t sine_wave_table[SINE_WAVE_TABLE_LEN];

#define SAMPLE_RATE 100
#define SAMPLE_MICROS (1000000/SAMPLE_RATE)

float base_freq = 1;
uint8_t counter = 0;
bool repeating_timer_callback(struct repeating_timer *t) {
    float seconds = (float)time_us_32()/1000000;
    float fseconds = (float)(seconds - (uint32_t)seconds);
    uint32_t aindex = (uint32_t) (SINE_WAVE_TABLE_LEN * fseconds * base_freq);
    // printf("%c%c", 0xFF, sine_wave_table[aindex] );
    // printf("%d\n", sine_wave_table[aindex] );
    // printf("%c%c%c", 0xFF, counter, sine_wave_table[aindex] );
    printf("%d %d %\n", counter, sine_wave_table[aindex] );
    counter++;
    counter%=SINE_WAVE_TABLE_LEN;
    return true;
}

int main() {
    stdio_init_all();

    for (int i = 0; i < SINE_WAVE_TABLE_LEN; i++) {
        sine_wave_table[i] = 126 * (cosf(i * 2 * (float) (M_PI / SINE_WAVE_TABLE_LEN))+1.0) + 1;
    }

    struct repeating_timer timer;
    add_repeating_timer_us(-SAMPLE_MICROS, repeating_timer_callback, NULL, &timer);
    while(true ){
        tight_loop_contents();
    }


    return 0;
}
