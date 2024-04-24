
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"

#include <math.h>

#include "charlieplex.pio.h"
#include "strobe.pio.h"


// PINS 0 - 14 are LED strobe pins
#define LED_STROBE_BPIN 0
#define NUM_LEDS_STROBE 15
#define BUT_PIN 15
// PINS 16 - 22 are charlieplex pins
#define LED_PIN 25  // LED pin
#define ASIG_PIN 26 // analog signal pin
#define DSIG_PIN 27 // digital signal pin
#define POW_PIN 28  // power pin for signal amp


float con_pitch = 440.0;
uint midi_index = 45;

PIO CPLEX_PIO = pio0;   // charlieplex program pio
#define CPLEX_SM  0     // charlieplex program state machine

PIO PIO_STROBE = pio1;  // strobe program pio
#define STROBE_SM 0     // strobe program state machine

#define FPS 50

#define NUM_PINS_CPLEX 7
#define NUM_LEDS_CPLEX NUM_PINS_CPLEX*(NUM_PINS_CPLEX-1)
uint pins[NUM_PINS_CPLEX] = {16, 17, 18, 19, 20, 21, 22};
uint32_t pins_mask = 0; // initialized with pins array to generate a 32 bit pin mask for setting gpio

uint led_pins_index[NUM_LEDS_CPLEX][2]={ // led pins - number is index of pins array set in pio
{0,1}, 	{1,0}, 	{2,0}, 	{3,0}, 	{4,0}, 	{5,0}, 	{6,0}, 
{0,2}, 	{1,2}, 	{2,1}, 	{3,1}, 	{4,1}, 	{5,1}, 	{6,1}, 
{0,3}, 	{1,3}, 	{2,3}, 	{3,2}, 	{4,2}, 	{5,2}, 	{6,2}, 
{0,4}, 	{1,4}, 	{2,4}, 	{3,4}, 	{4,3}, 	{5,3}, 	{6,3}, 
{0,5}, 	{1,5}, 	{2,5}, 	{3,5}, 	{4,5}, 	{5,4}, 	{6,4}, 
{0,6}, 	{1,6}, 	{2,6}, 	{3,6}, 	{4,6}, 	{5,6}, 	{6,5}
};

uint32_t led_out_masks[NUM_LEDS_CPLEX];   // for setting gpio direction - two bits high for each led entry
uint32_t led_hi_masks[NUM_LEDS_CPLEX];    // for setting gpio value - one bit high for each led entry
uint32_t led_pio_masks[NUM_LEDS_CPLEX*2]; // for auto pulling into pio isr - alternate gpio direction and gpio value

uint64_t bNOTES[12] ={
0b000000001100001001000111100010010001001000,
0b000010001100001001000111100010010001001000,
0b000000011100001001000101000010010001110000,
0b000000001100001001000100000010010000110000,
0b000010001100001001000100000010010000110000,
0b000000011100001001000100100010010001110000,
0b000010011100001001000100100010010001110000,
0b000000011110001000000111000010000001111000,
0b000000011110001000000111000010000001000000,
0b000001011110001000000111000010000001000000,
0b000000001100001000000101100010010000110000,
0b000010001100001000000101100010010000110000
};

uint note_counter = 0;


uint64_t bSHARP = 0b000000000000000010000000000000;
uint64_t bFLAT = 0b000000000000000100000000000000;
uint64_t bDISPLAY = 0x3FFFFFFFFFF;  // first 42 bits are led status - set all high for led check
uint32_t led_freq = FPS*NUM_LEDS_CPLEX;
uint32_t led_micros = 1000000/(FPS*NUM_LEDS_CPLEX);

int dma_chan_cplex_leds;
int dma_chan_cplex_loop;

void gpio_callback(uint gpio, uint32_t events) {
    // "LEVEL_LOW",  // 0x1
    // "LEVEL_HIGH", // 0x2
    // "EDGE_FALL",  // 0x4
    // "EDGE_RISE"   // 0x8
    if(events == 0x8){
        note_counter++;
        note_counter%=12;
        gpio_put(LED_PIN, 1);
    } else if(events == 0x4){
        gpio_put(LED_PIN, 0);
    }
}

void charlieplex_program_init(PIO pio, uint sm, uint offset, float freq, uint base_pin, uint num_pins);
void strobe_leds_forever(PIO pio, uint sm, uint offset, uint sense_pin, uint base_led_pin, uint num_leds, float freq);

int main() {

    stdio_init_all();

    for (uint i = 0; i < NUM_PINS_CPLEX; ++i){
        pins_mask |= (1 <<  pins[i]);
    }

    for (uint i = 0; i < NUM_LEDS_CPLEX; ++i){

        uint a = led_pins_index[i][0];
        uint c = led_pins_index[i][1];

        led_out_masks[i] = (1 << a) | (1 << c); 
        led_hi_masks[i] = (1 << a); 
    }

    for (uint i = 0; i < NUM_LEDS_CPLEX; ++i){

        led_pio_masks[2*i + 0] = led_out_masks[i];
        led_pio_masks[2*i + 1] = (led_hi_masks[i] ); // set all leds high

    }

    gpio_set_dir_in_masked(pins_mask);
    gpio_init_mask(pins_mask);

    uint offset_cplex = pio_add_program(CPLEX_PIO, &charlieplex_program);
    charlieplex_program_init(CPLEX_PIO, CPLEX_SM, offset_cplex, led_freq, pins[0], NUM_PINS_CPLEX);

    // delete following
    float tf = con_pitch*(pow(2,((midi_index*100. + 0 -6900.)/1200.)));
    // end delete
    uint strobe_offset = pio_add_program(PIO_STROBE, &strobe_program);
    strobe_leds_forever(PIO_STROBE, STROBE_SM, strobe_offset, DSIG_PIN, LED_STROBE_BPIN, NUM_LEDS_STROBE,  tf);

    for (uint i = 0; i < NUM_PINS_CPLEX; ++i){
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
        NUM_LEDS_CPLEX*2, 
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

    gpio_set_irq_enabled_with_callback(BUT_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(POW_PIN);
    gpio_set_dir(POW_PIN, GPIO_OUT);
    gpio_put(POW_PIN, 1);

    uint led_counter = 0;
    uint previous_millis = 0;
    uint curr_millis = 0;
    while(true){

        curr_millis = to_ms_since_boot(get_absolute_time());
        if ((curr_millis - previous_millis) > 500){
            previous_millis = curr_millis;

            for(uint i = 0; i < NUM_LEDS_CPLEX; ++i ){ 
                // led_pio_masks[2*i + 1] = (led_hi_masks[i] )* (led_counter == i);
                led_pio_masks[2*i + 1] = (led_hi_masks[i] )* ((bDISPLAY & ( 1llu << i)) > 0);
            }
            bDISPLAY = 1 << led_counter;
            bDISPLAY = bNOTES[note_counter] ;

            // note_counter++;
            // note_counter%=12;
            // numer_counter++;
            // numer_counter%=10;

            led_counter++;
            led_counter%=NUM_LEDS_CPLEX;
        }      



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


// this is a raw helper function for use by the user which sets up the GPIO output, and configures the SM to output on a particular pin
void strobe_program_init(PIO pio, uint sm, uint offset, float target_freq, uint sense_pin, uint base_led_pin, uint num_leds ) {

   pio_gpio_init(pio, sense_pin);
   pio_sm_set_consecutive_pindirs(pio, sm, sense_pin, 1, false);

   for (auto i = base_led_pin; i < num_leds; i++){
    pio_gpio_init(pio, i);
   }
    pio_sm_set_consecutive_pindirs(pio, sm, 0, num_leds, true);

   pio_sm_config c = strobe_program_get_default_config(offset);
   sm_config_set_in_pins(&c, sense_pin);
   sm_config_set_out_pins(&c, 0 ,num_leds);
	
    sm_config_set_in_shift(&c, false, false, 32); // shift to left, autopull disabled

   float div = (float)clock_get_hz(clk_sys) / (num_leds*15*target_freq); // 16 leds * x cycles per sample in
   sm_config_set_clkdiv(&c, div);

   pio_sm_init(pio, sm, offset, &c);
}

void strobe_leds_forever(PIO pio, uint sm, uint offset, uint sense_pin, uint base_led_pin, uint num_leds, float freq) {
    strobe_program_init(pio, sm, offset, freq, sense_pin, base_led_pin, num_leds);
    pio_sm_set_enabled(pio, sm, true);

    printf("Srobing pins at %.2f Hz\n", freq);

        // set the frequency NUM_LEDS_STROBE times the target frequency 
        // takeout 5 cycles for processing between samples
    pio->txf[sm] = (clock_get_hz(clk_sys) / ( freq * num_leds)) - 5;
}