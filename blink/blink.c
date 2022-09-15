/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"

#define LED_16_ARRAY

int main() {
#ifndef LED_16_ARRAY
    #ifndef PICO_DEFAULT_LED_PIN
    #warning blink example requires a board with a regular LED
    #else
        const uint LED_PIN = PICO_DEFAULT_LED_PIN;
        gpio_init(LED_PIN);
        gpio_set_dir(LED_PIN, GPIO_OUT);
        while (true) {
            gpio_put(LED_PIN, 1);
            sleep_ms(250);
            gpio_put(LED_PIN, 0);
            sleep_ms(250);
        }
    #endif
#else

    for(auto i = 0; i < 16 ; ++i){
        gpio_init(i);
        gpio_set_dir(i, GPIO_OUT);
    }
        while(true){

            for(auto i = 0; i < 16 ; ++i){
                gpio_put(i, 1);
                sleep_ms(100);
            }
            for(auto i = 0; i < 16 ; ++i){
                gpio_put(i, 0);
                sleep_ms(100);
            }

        }

#endif


}
