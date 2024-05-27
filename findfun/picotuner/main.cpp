
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

#define midi_to_note(midi) (midi + 3) % 12
#define midi_to_octave(midi) int(midi/12-1)

uint instrument_index = 0;

float con_pitch = 440.0;

const uint num_instruments = 2;

const uint num_guitar_tones = 6;
uint guitar_midi[num_guitar_tones]={64, 59, 55, 50, 45, 40};
const uint num_ukulele_tones = 4;
uint ukulele_midi[num_ukulele_tones]={69, 64, 60, 67};


typedef struct {
    uint8_t midi;
    uint8_t note;
    uint8_t octave;
    float freq;
} tone_t;

typedef struct {
    uint8_t num_tones;
    uint8_t tone_index = 0;
    tone_t tones[num_guitar_tones];
} instrument_t;

instrument_t instruments[2];


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
0b000000001100001001000111100010010001001000, //A
0b000010001100001001000111100010010001001000, //A#
0b000000011100001001000101000010010001110000, //B
0b000000001100001001000100000010010000110000, //C
0b000010001100001001000100000010010000110000, //C#
0b000000011100001001000100100010010001110000, //D
0b000010011100001001000100100010010001110000, //D#
0b000000011110001000000111000010000001111000, //E
0b000000011110001000000111000010000001000000, //F
0b000010011110001000000111000010000001000000, //F#
0b000000001100001000000101100010010000110000, //G
0b000010001100001000000101100010010000110000  //G#
};

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
        // tone_index++;
        // note_counter%=12;
        gpio_put(LED_PIN, 1);
    } else if(events == 0x4){
        gpio_put(LED_PIN, 0);
    }
}

void charlieplex_program_init(PIO pio, uint sm, uint offset, float freq, uint base_pin, uint num_pins);
void strobe_program_init(PIO pio, uint sm, uint offset, float target_freq, uint sense_pin, uint base_led_pin, uint num_leds );
float calc_strobe_div(float freq, uint num_leds);
void change_strobe_frequency(PIO pio, uint sm, float new_target_freq, uint num_leds);

int main() {

    stdio_init_all();
    sleep_ms(1000);    
    for(uint j = 0; j < num_guitar_tones; j++){
        uint m = guitar_midi[j];
        uint n = midi_to_note(guitar_midi[j]);
        uint o = midi_to_octave(guitar_midi[j]);
        float f = con_pitch*(pow(2,((m*100.0 -6900.)/1200.)));
        instruments[0].tones[j].midi = m;
        instruments[0].tones[j].note = n;
        instruments[0].tones[j].octave = o;
        instruments[0].tones[j].freq = f;
        printf("guitar midi %d, note %d, octave %d, freq %.2f\n", m,n,o,f);
    }

    for(uint j = 0; j < num_ukulele_tones; j++){
        uint m = ukulele_midi[j];
        uint n = midi_to_note(ukulele_midi[j]);
        uint o = midi_to_octave(ukulele_midi[j]);
        float f = con_pitch*(pow(2,((m*100.0 -6900.)/1200.)));
        instruments[1].tones[j].midi = m;
        instruments[1].tones[j].note = n;
        instruments[1].tones[j].octave = o;
        instruments[1].tones[j].freq = f;
        printf("ukulele midi %d, note %d, octave %d, freq %.2f\n", m,n,o,f);
    }

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


    uint strobe_offset = pio_add_program(PIO_STROBE, &strobe_program);
    strobe_program_init(PIO_STROBE, STROBE_SM, strobe_offset, 110.0, ASIG_PIN, LED_STROBE_BPIN, NUM_LEDS_STROBE);

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
    
    gpio_init(ASIG_PIN);
    gpio_set_dir(ASIG_PIN, GPIO_IN);

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
            bDISPLAY = bNOTES[instruments[instrument_index].tone_index] ;

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

   float div = calc_strobe_div(target_freq, num_leds);
   sm_config_set_clkdiv(&c, div);

   pio_sm_init(pio, sm, offset, &c);
   pio_sm_set_enabled(pio, sm, true);
}

float calc_strobe_div(float freq, uint num_leds){
    return (float)clock_get_hz(clk_sys) / (num_leds*15*freq); // num_leds * 15 cycles per sample in
}   

void change_strobe_frequency(PIO pio, uint sm, float new_target_freq, uint num_leds){
    float div = calc_strobe_div(new_target_freq, num_leds);
    pio_sm_set_clkdiv(pio, sm, div);
}


