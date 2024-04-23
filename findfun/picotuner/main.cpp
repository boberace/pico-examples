
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"

#include "charlieplex.pio.h"

PIO CPLEX_PIO = pio0;
uint CPLEX_SM = 0;

#define FPS 50

#define NUM_PINS 7
#define NUM_LEDS NUM_PINS*(NUM_PINS-1)
uint pins[NUM_PINS] = {16, 17, 18, 19, 20, 21, 22};
uint32_t pins_mask = 0; // used to generate a 32 bit pin mask for setting gpio
uint led_pins_index[NUM_LEDS][2]={ // led pins - number is index of pins array set in pio
    {0,1}, {1,0},
    {0,2}, {2,0}, {1,2}, {2,1}, 
    {0,3}, {3,0}, {1,3}, {3,1}, {2,3}, {3,2},
    {0,4}, {4,0}, {1,4}, {4,1}, {2,4}, {4,2}, {3,4}, {4,3},
    {0,5}, {5,0}, {1,5}, {5,1}, {2,5}, {5,2}, {3,5}, {5,3}, {4,5}, {5,4}, 
    {0,6}, {6,0}, {1,6}, {6,1}, {2,6}, {6,2}, {3,6}, {6,3}, {4,6}, {6,4}, {5,6}, {6,5}
    };

uint32_t led_out_masks[NUM_LEDS];   // for setting gpio direction - two bits high for each led entry
uint32_t led_hi_masks[NUM_LEDS];    // for setting gpio value - one bit high for each led entry
uint32_t led_pio_masks[NUM_LEDS*2]; // for auto pulling into pio isr - alternate gpio direction and gpio value

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
uint32_t bDISPLAY = 0x3FFFFFFF;  // first 30 bits are led status - set all high for led check
uint32_t led_freq = FPS*NUM_LEDS;
uint32_t led_micros = 1000000/(FPS*NUM_LEDS);

int dma_chan_cplex_leds;
int dma_chan_cplex_loop;



void charlieplex_program_init(PIO pio, uint sm, uint offset, float freq, uint base_pin, uint num_pins);

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
        led_pio_masks[2*i + 1] = (led_hi_masks[i] )*((bDISPLAY & ( 1 << i)) > 0);

    }

    gpio_set_dir_in_masked(pins_mask);
    gpio_init_mask(pins_mask);

    uint offset_cplex = pio_add_program(CPLEX_PIO, &charlieplex_program);
    charlieplex_program_init(CPLEX_PIO, CPLEX_SM, offset_cplex, led_freq, pins[0], NUM_PINS);

    for (uint i = 0; i < NUM_PINS; ++i){
        uint pin = pins[i];
        gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_2MA); 
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

// this is a raw helper function for use by the user which sets up the GPIO output, and configures the SM to output on a particular pin
void charlieplex_program_init(PIO pio, uint sm, uint offset, float freq, uint base_pin, uint num_pins) {
   pio_sm_config c = charlieplex_program_get_default_config(offset);

   sm_config_set_out_pins(&c, base_pin , num_pins);

   pio_sm_set_consecutive_pindirs(pio, sm, base_pin, num_pins, false);

   for (uint i = 0; i < num_pins; i++){
        pio_gpio_init(pio, i + base_pin);
   }

   sm_config_set_out_shift(&c, false, true, 32); 

   float div = (float)clock_get_hz(clk_sys) / (freq*34); // frequency of individual led * number of cycles per change of led
   sm_config_set_clkdiv(&c, div);

   pio_sm_init(pio, sm, offset, &c);
   pio_sm_set_enabled(pio, sm, true);
}
