/**
 * 
 * Charlieplexing is a multiplexing protocol that can drive many leds on a few pins.
 * It can drive only one led at a time (without fade) but can make nice displays if 
 * driving in rapid succession.  the amount of leds that can be driven from a 
 * number op pins (p) is p*(p-1) = #leds. This example uses six pins wich can drive
 * 6*5 = 30 leds.  the led refresh frequency is 30 leds * 30 frames per second = 900 hz
 * To light up an led you drive one pin high and one pin low with the rest turn to inputs.
 * checkout https://en.wikipedia.org/wiki/Charlieplexing
 */

#include "pico/stdlib.h"
#include <stdio.h>

#define NUM_PINS 6
#define NUM_LEDS NUM_PINS*(NUM_PINS-1)
uint pins[NUM_PINS] = {17, 18, 19, 20, 21, 22};
uint32_t pins_mask = 0; // used to generate a 32 bit pin mask
uint led_pins_index[NUM_LEDS][2]={ // led pins - number is index of pins array
    {0,1}, {1,0},
    {0,2}, {2,0}, {1,2}, {2,1}, 
    {0,3}, {3,0}, {1,3}, {3,1}, {2,3}, {3,2},
    {0,4}, {4,0}, {1,4}, {4,1}, {2,4}, {4,2}, {3,4}, {4,3},
    {0,5}, {5,0}, {1,5}, {5,1}, {2,5}, {5,2}, {3,5}, {5,3}, {4,5}, {5,4}
    };
uint led_pins[NUM_LEDS][2];
uint32_t led_out_masks[NUM_LEDS]; 
uint32_t led_hi_masks[NUM_LEDS]; 

int main() {

    stdio_init_all();

    for (uint i = 0; i < NUM_PINS; ++i){
        uint pin = pins[i];
        pins_mask |= (1 << pin);
        gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_12MA); // two 150 ohm resistors in series
        gpio_disable_pulls(pin);
    }

    for (uint i = 0; i < NUM_LEDS; ++i){

        led_pins[i][0] = pins[led_pins_index[i][0]];
        led_pins[i][1] = pins[led_pins_index[i][1]];

        uint a = led_pins[i][0];
        uint c = led_pins[i][1];

        led_out_masks[i] = (1 << a) | (1 << c); 
        led_hi_masks[i] = (1 << a); 
    }

    gpio_set_dir_in_masked(pins_mask);
    gpio_init_mask(pins_mask);

    uint led_counter = 0;
    while(true){

        // // printf("mask %b\n", pins_mask);
        // for (uint i = 0; i < NUM_LEDS; ++i){
        //     // printf("a %d b %d : a %d b %d\n", led_pins_index[i][0], led_pins_index[i][1],led_pins[i][0], led_pins[i][1]);
        //     printf(" %30b\n",led_hi_masks[i] );
        // }

        // printf("\n");
        // sleep_ms(3000);

        gpio_put_masked(pins_mask, 0);
        gpio_set_dir_in_masked(pins_mask);
        gpio_set_dir_out_masked(led_out_masks[led_counter]);
        gpio_put(led_pins[led_counter][0], 1);
        // gpio_put_masked(led_hi_masks[led_counter], 1);
        

        led_counter++;
        led_counter%=NUM_LEDS;
        sleep_ms(100);

        

    }

/*
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
*/

}
