#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include <string.h>
#include <math.h>
#include <vector>
#include "cindex.h"
#include "blink.pio.h"
#include "strobe.pio.h"
#include "../findfun.h"
#include "ssd1306.h"

// #define range_test // if defined; creates pure test signals that run through the midi range

#define I2C0_BUADRATE 400*1000

#define PIN_I2C_SDA 16u
#define PIN_I2C_SCL 17u
#define PIN_TEST 19u
#define PIN_SENSE2 22u
#define PIN_LED 25u
#define PIN_TRIG 26u
#define PIN_SENSE1 27u

#define SM_BLINK 0     // blink program state machine
PIO PIO_BLINK = pio0;  // blink program pio

#define SM_STROBE 0     // strobe program state machine
PIO PIO_STROBE = pio1;  // strobe program pio

#define pin_toggle(x) gpio_put(x, !gpio_get(x))

using std::vector;

ssd1306_t disp; // create oled display instance

float ConPitch = 440.0; // concert pitch (piano middle A : midi 69 : A4)
// sampling frequency (hz)  for generating integer period lengths 
float SF = 1000000; // same as the sample frequency of the edges sent    
float SP = 0.035;   // sampling period (seconds) 0.035 guitar
                    // needs to be long enough to capture two periods of lowest frequency 
int LMI = 38 ; // guitar 38 # piano 21
int HMI = 66 + 1; // guitar 66 # piano 96
int NCS = 20 ; // number of correlation shifts

int midi_idx = 0;
int midi_cent = 0;
int note_idx = 0;
int note_oct = 0;

char* note[12] = { "C ", "C#", "D ", "D# ", "E ", "F ", "F#", "G ", "G#", "A ", "Bb", "B "};

static char event_str[128];

#define NUM_LOOP_SAMPLES 500
cindex loop_idx(NUM_LOOP_SAMPLES);
uint64_t stamp_edges_ticks[NUM_LOOP_SAMPLES];

void setup_i2c0(void); 
void setup_oled(void);
void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, float freq);
void strobe_pins_forever(PIO pio, uint sm, uint offset, uint sense_pin, float freq);
void gpio_event_string(char *buf, uint32_t events);
void update_display(void);

int infer_midi_cent_idx = 0;

void core1_entry() {  
    findfun ff = findfun();
    ff.begin(   ConPitch,
                SF,
                LMI,
                HMI,
                NCS);

    uint32_t prev_millis = to_ms_since_boot(get_absolute_time());
    uint32_t curr_millis = to_ms_since_boot(get_absolute_time());

    while(1){  //core 1 loop ***  core 1 loop ***core 1 loop ***  core 1 loop ***  

        sleep_ms(1);

        curr_millis = to_ms_since_boot(get_absolute_time());
        if( curr_millis - prev_millis > SP*1000){ // grab new sample from buffer every sample period
            prev_millis = curr_millis;    

            int sample_end_idx = loop_idx -1; // mark end of sample

            cindex idx(NUM_LOOP_SAMPLES); // create circular index with top same as number of elements in circular buffer
            idx = sample_end_idx; // start cindex at end of sample

            uint64_t start_edge_ticks = stamp_edges_ticks[idx] - SP*SF;  // calculate how far back to grab samples

            while(stamp_edges_ticks[--idx] > start_edge_ticks); // find starting index closes to sample start edge

            uint64_t base_ticks = stamp_edges_ticks[idx]; // store base sample , used to normalize samples

            vector<int> sample_edges;  // create normalized sample edge vector and copy in from sample circular buffer

            while(++idx != sample_end_idx) sample_edges.push_back(stamp_edges_ticks[idx] - base_ticks);

        // send slice for detection;
        
        infer_midi_cent_idx = ff.find_midi_cent(sample_edges);

        }
    }

}


void gpio_callback(uint gpio, uint32_t events) {

    absolute_time_t t = get_absolute_time();
    uint64_t t_us = to_us_since_boot(t);

    stamp_edges_ticks[loop_idx] = t_us;
    pin_toggle(PIN_LED);
    loop_idx++; 

}

int main() {
    stdio_init_all();
    setup_i2c0(); 
    setup_oled();

    gpio_init(PIN_TEST);
    gpio_set_dir(PIN_TEST, GPIO_OUT);
    gpio_put(PIN_TEST, 1);

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    gpio_init(PIN_SENSE1);
    gpio_set_dir(PIN_SENSE1, GPIO_IN);

    uint blink_offset = pio_add_program(PIO_BLINK, &blink_program); 
    uint strobe_offset = pio_add_program(PIO_STROBE, &strobe_program);

    printf("Hello GPIO IRQ\n");
    gpio_set_irq_enabled_with_callback(PIN_SENSE1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    multicore_launch_core1(core1_entry);

    uint32_t prev_millis_display = to_ms_since_boot(get_absolute_time());
    uint32_t curr_millis_display = to_ms_since_boot(get_absolute_time());

    int frame_counter = 0;
    int idx_counter = HMI-LMI;

    // delete following
    int mi = 57;
    float tf = ConPitch*(pow(2,((mi*100. + 0 -6900.)/1200.)));
    float tfp1 = ConPitch*(pow(2,(((mi+1)*100. + 0 -6900.)/1200.)));
    float dtf = (tfp1-tf)*.01;

    blink_pin_forever(PIO_BLINK, SM_BLINK, blink_offset, PIN_TRIG, tf + dtf);
    strobe_pins_forever(PIO_STROBE, SM_STROBE, strobe_offset, PIN_SENSE2, tf);

    while(1){  // core 0 loop ***  core 0 loop ***  core 0 loop ***  core 0 loop ***  core 0 loop *** 

        sleep_ms(1);
        // display
        curr_millis_display = to_ms_since_boot(get_absolute_time());
        if( curr_millis_display - prev_millis_display > 100){
            prev_millis_display = curr_millis_display;

            update_display(); 

            printf("midi cent %d\n", infer_midi_cent_idx);

            int inf_midi_cent_idx_cpy = infer_midi_cent_idx;
            if(inf_midi_cent_idx_cpy > 0){
                midi_idx = round(inf_midi_cent_idx_cpy/100);
                midi_cent = inf_midi_cent_idx_cpy - midi_idx*100;
                note_idx = midi_idx % 12;
                note_oct = (midi_idx - note_idx)/12 - 1;
            }

#ifdef range_test
            if(frame_counter % 10 == 0){

                int midi_idx = idx_counter+LMI;
                float sample_freq = ConPitch*(pow(2,((midi_idx*100. + 0 -6900.)/1200.)));

                blink_pin_forever(PIO_BLINK, SM_BLINK, blink_offset, PIN_TRIG, sample_freq);
                strobe_pins_forever(PIO_STROBE, SM_STROBE, strobe_offset, PIN_SENSE2, sample_freq);

                idx_counter++;
                idx_counter%=(HMI - LMI);
            }
#endif


            frame_counter++;
        }
        
    }

    return 0;
}

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, float freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %.2f Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (clock_get_hz(clk_sys) / (2 * freq)) - 3;
}

void strobe_pins_forever(PIO pio, uint sm, uint offset, uint sense_pin, float freq) {
    strobe_program_init(pio, sm, offset, freq, sense_pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Srobing pins at %.2f Hz\n", freq);

        // set the frequency 16 times the target frequency (16 = num leds)
        // takeout 5 cycles for processing between samples
    pio->txf[sm] = (clock_get_hz(clk_sys) / ( freq * 16)) - 5;
}

static const char *gpio_irq_str[] = {
        "LEVEL_LOW",  // 0x1
        "LEVEL_HIGH", // 0x2
        "EDGE_FALL",  // 0x4
        "EDGE_RISE"   // 0x8
};

void gpio_event_string(char *buf, uint32_t events) {
    for (uint i = 0; i < 4; i++) {
        uint mask = (1 << i);
        if (events & mask) {
            // Copy this event string into the user string
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0') {
                *buf++ = *event_str++;
            }
            events &= ~mask;

            // If more events add ", "
            if (events) {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf++ = '\0';
}

void setup_i2c0(void){

    i2c_init(i2c0, I2C0_BUADRATE);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);
}


void setup_oled(void) {

    disp.external_vcc=false;
    ssd1306_init(&disp, 128, 32, 0x3C, i2c0);
    ssd1306_clear(&disp);
    ssd1306_draw_string(&disp, 8, 0, 1, (char*)"SSD1306");
    ssd1306_draw_string(&disp, 8, 16, 2, (char*)"DISPLAY");
    ssd1306_show(&disp);
}

void update_display(void){ 
    
    ssd1306_clear(&disp);

    char str[128];

    ssd1306_draw_string(&disp, 0, 0, 3, note[note_idx]);

    memset(str, 0, sizeof(char));
    sprintf(str, " %d:%d", note_oct, midi_cent);
    ssd1306_draw_string(&disp, 24, 0, 3, str);

    // memset(str, 0, sizeof(char));
    // sprintf(str, "%d, %d", midi_idx, midi_cent);
    // ssd1306_draw_string(&disp, 0, 16, 2, str);

    ssd1306_show(&disp);

}
