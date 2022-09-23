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

#define FPS 50

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



uint32_t bALPHA[7] ={
0b000000000000000001011111110110, //A
0b000000000000000001011010111111, //B
0b000000000000000001010010111010, //C
0b000000000000000001010110111111, //D
0b000000000000000001101001111111, //E
0b000000000000000001101000110111, //F
0b000000000000000001000111111010  //G
};
uint32_t bNUMER[10] = {
0b111011111111100000000000000000, //0
0b011011100000000000000000000000, //1
0b111110111110100000000000000000, //2
0b111111111000100000000000000000, //3
0b011111100011000000000000000000, //4
0b110111111011100000000000000000, //5
0b100111111111100000000000000000, //6
0b111011100000100000000000000000, //7
0b111111111111100000000000000000, //8
0b111111100011100000000000000000  //9
};
uint32_t bSHARP = 0b000000000000000010000000000000;
uint32_t bFLAT = 0b000000000000000100000000000000;
uint32_t bDISPLAY = 0;
uint32_t led_micros = 1000000/(FPS*NUM_LEDS);

int main() {

    bDISPLAY = bALPHA[0] | bNUMER[0] | bSHARP;

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
    uint frame_counter = 0;
    uint alpha_counter = 0;
    uint numer_counter = 0;
    while(true){

        gpio_put_masked(pins_mask, 0);
        gpio_set_dir_in_masked(pins_mask);
        gpio_set_dir_out_masked(led_out_masks[led_counter]);
        gpio_put(led_pins[led_counter][0], (bDISPLAY & ( 1 << led_counter)));

        led_counter++;
        led_counter%=NUM_LEDS;
        if(led_counter == 0) {
            frame_counter++;

            if(frame_counter%FPS == 0){
                alpha_counter++;
                alpha_counter%=7;
                numer_counter++;
                numer_counter%=10;

                bDISPLAY = bALPHA[alpha_counter] | bNUMER[numer_counter];

                printf(" ac: %d, nc: %d\n", alpha_counter,numer_counter );
            }
        }

        sleep_us(led_micros);
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
