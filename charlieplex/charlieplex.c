/**
 * 
 * Charlieplexing is a multiplexing protocol that can drive many leds on a few pins.
 * It can drive only one led at a time (without fade) but can make nice displays if 
 * driving in rapid succession.  the amount of leds that can be driven from a 
 * number op pins (p) is p*(p-1) = #leds. This example uses six pins wich can drive
 * 6*5 = 30 leds.  the led refresh frequency is 30 leds * 30 frames per second = 900 hz
 * To light up an led you drive one pin high and one pin low with the rest turn to inputs.
 * checkout https://en.wikipedia.org/wiki/Charlieplexing
 * 
 * 
 * tode: set up another dma channel to replace irq callback
 * 
 */
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"

#include "charlieplex.pio.h"

PIO CPLEX_PIO = pio0;
uint CPLEX_SM = 0;

#define FPS 50

#define NUM_PINS 6
#define NUM_LEDS NUM_PINS*(NUM_PINS-1)
uint pins[NUM_PINS] = {17, 18, 19, 20, 21, 22};
uint32_t pins_mask = 0; // used to generate a 32 bit pin mask
uint led_pins_index[NUM_LEDS][2]={ // led pins - number is index of pins array set in pio
    {0,1}, {1,0},
    {0,2}, {2,0}, {1,2}, {2,1}, 
    {0,3}, {3,0}, {1,3}, {3,1}, {2,3}, {3,2},
    {0,4}, {4,0}, {1,4}, {4,1}, {2,4}, {4,2}, {3,4}, {4,3},
    {0,5}, {5,0}, {1,5}, {5,1}, {2,5}, {5,2}, {3,5}, {5,3}, {4,5}, {5,4}
    };

uint32_t led_out_masks[NUM_LEDS]; 
uint32_t led_hi_masks[NUM_LEDS]; 
uint32_t led_pio_masks[NUM_LEDS*2];

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
uint32_t bDISPLAY = 0xFFFFFFFF;
uint32_t led_freq = FPS*NUM_LEDS;
uint32_t led_micros = 1000000/(FPS*NUM_LEDS);

int dma_chan_cplex_leds;
int dma_chan_cplex_loop;

void dma_handler_cplex_leds() {
    dma_hw->ints0 = 1u << dma_chan_cplex_leds;
    dma_channel_set_read_addr(dma_chan_cplex_leds, &led_pio_masks, true);
}

int main() {

    stdio_init_all();

    for (uint i = 0; i < NUM_PINS; ++i){
        pins_mask |= (1 <<  pins[i]);
    }

    for (uint i = 0; i < NUM_LEDS; ++i){

        uint a = led_pins_index[i][0];
        uint c = led_pins_index[i][1];

        led_out_masks[i] = (1 << a) | (1 << c); 
        led_hi_masks[i] = (1 << a); 
    }

    for (uint i = 0; i < NUM_LEDS; ++i){

        led_pio_masks[2*i+0]=led_out_masks[i];
        led_pio_masks[2*i + 1] = (led_hi_masks[i] )* ((bDISPLAY & ( 1 << i)) > 0);

    }

    gpio_set_dir_in_masked(pins_mask);
    gpio_init_mask(pins_mask);

    uint offset_cplex = pio_add_program(CPLEX_PIO, &charlieplex_program);
    charlieplex_program_init(CPLEX_PIO, CPLEX_SM, offset_cplex, led_freq, pins[0], NUM_PINS);

    for (uint i = 0; i < NUM_PINS; ++i){
        uint pin = pins[i];
        gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_12MA); // two 150 o`hm resistors in series
        gpio_disable_pulls(pin);
    }

    dma_chan_cplex_leds = dma_claim_unused_channel(true);
    dma_chan_cplex_loop = dma_claim_unused_channel(true);

    dma_channel_config c_leds = dma_channel_get_default_config(dma_chan_cplex_leds);
    dma_channel_config c_loop = dma_channel_get_default_config(dma_chan_cplex_loop);    

    //config led dma
    channel_config_set_dreq(&c_leds, pio_get_dreq(CPLEX_PIO, CPLEX_SM, true));
    channel_config_set_chain_to(&c_leds, dma_chan_cplex_loop);
    channel_config_set_irq_quiet(&c_leds, true);

    dma_channel_configure(
        dma_chan_cplex_leds,
        &c_leds,
        &pio0_hw->txf[CPLEX_SM], 
        led_pio_masks,            
        NUM_LEDS*2, 
        false            
    );

    //config loop dma
    channel_config_set_read_increment(&c_loop, false);
    channel_config_set_chain_to(&c_loop, dma_chan_cplex_leds);
    channel_config_set_irq_quiet(&c_loop, true);

    uint32_t * p_led_pio_masks = led_pio_masks;
    dma_channel_configure(
        dma_chan_cplex_loop,
        &c_loop,
        &dma_hw->ch[dma_chan_cplex_leds].read_addr, 
        &p_led_pio_masks,            
        1, 
        false            
    );

    dma_start_channel_mask(1u << dma_chan_cplex_leds);    

    uint led_counter = 0;
    uint alpha_counter = 0;
    uint numer_counter = 0;
    uint previous_millis = 0;
    uint curr_millis = 0;
    while(true){

        curr_millis = to_ms_since_boot(get_absolute_time());
        if ((curr_millis - previous_millis) > 1000){
            previous_millis = curr_millis;

            for(uint i = 0; i < NUM_LEDS; ++i ){ 
                led_pio_masks[2*i + 1] = (led_hi_masks[i] )* ((bDISPLAY & ( 1 << i)) > 0);
            }

            bDISPLAY = bALPHA[alpha_counter] | bNUMER[numer_counter];

            alpha_counter++;
            alpha_counter%=7;
            numer_counter++;
            numer_counter%=10;
        }      

        led_counter++;
        led_counter%=NUM_LEDS;

    }
}

