#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "../findfun.h"
#include <math.h>
#include <vector>
#include "hardware/pio.h"
#include "blink.pio.h"
#include "hardware/dma.h"
#include "pico/multicore.h"
#include "ssd1306.h"
#include <string.h>

// #define feather

#define I2C0_BUADRATE 400*1000
#define PIN_LED 25
#define PIN_TRIG 26

#define BUTTON_A  9
#define BUTTON_B  8
#define BUTTON_C  7

#ifdef feather
#define PIN_HALL 13   // pico 27 feather 13
#define PIN_I2C_SDA 2 // pico 16 feather 2
#define PIN_I2C_SCL 3 // pico 17 feather 3
#else
#define PIN_HALL 27   // pico 27 feather 13
#define PIN_I2C_SDA 16 // pico 16 feather 2
#define PIN_I2C_SCL 17 // pico 17 feather 3
#endif

#define pin_toggle(x) gpio_put(x, !gpio_get(x))

using std::vector;

ssd1306_t disp; // create oled display instance

float ConPitch = 440.0; // concert pitch (middle A)
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
int loop_idx = 0;
uint64_t stamp_ticks[NUM_LOOP_SAMPLES];

void setup_i2c0(void); 
void setup_i2c1(void);
void setup_oled(void);
void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, float freq);
void gpio_event_string(char *buf, uint32_t events);
void update_display(void);

int inf_midi_cent_idx = 0;


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
        if( curr_millis - prev_millis > SP*1000){
            prev_millis = curr_millis;

            // printf("\n\n\n %f\n", idx);            

            int top_idx = loop_idx -1;     
            int idx = top_idx;       

            uint64_t start_ticks = stamp_ticks[idx] - SP*SF;

            while(stamp_ticks[idx] > start_ticks){  

                if(--idx < 0){
                    idx = NUM_LOOP_SAMPLES;
                }

            }

            if(--idx < 0){
                idx = NUM_LOOP_SAMPLES;
            }

            uint64_t base_ticks = stamp_ticks[idx];

            int bot_idx = ++idx;

            vector<int> sample_edges;

            for(auto i = bot_idx; i < top_idx; ++i){
                sample_edges.push_back(stamp_ticks[i] - base_ticks);
            }            

            // for(auto se: sample_edges){
            //     printf("%d\n", se);
            // }

        inf_midi_cent_idx = ff.find_midi_cent(sample_edges);

        }
    }

}


void gpio_callback(uint gpio, uint32_t events) {

    absolute_time_t t = get_absolute_time();
    uint64_t t_us = to_us_since_boot(t);

    stamp_ticks[loop_idx] = t_us;
    pin_toggle(PIN_LED);
    
    // gpio_event_string(event_str, events);
    // printf("GPIO %d %s idx: %d time: %d \n", gpio, event_str, loop_idx, stamp_ticks[loop_idx] );

    loop_idx++;
    loop_idx %= NUM_LOOP_SAMPLES;   

}

int main() {
    stdio_init_all();

#ifdef feather
    setup_i2c1(); //pico 0 feather 1
#else
    setup_i2c0(); //pico 0 feather 1
#endif


    setup_oled();

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    gpio_init(PIN_HALL);
    gpio_set_dir(PIN_HALL, GPIO_IN);

    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program); 
    blink_pin_forever(pio, 0, offset, PIN_TRIG, 110);

    printf("Hello GPIO IRQ\n");
    gpio_set_irq_enabled_with_callback(PIN_HALL, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    multicore_launch_core1(core1_entry);

    uint32_t prev_millis = to_ms_since_boot(get_absolute_time());
    uint32_t curr_millis = to_ms_since_boot(get_absolute_time());

    while(1){  // core 0 loop ***  core 0 loop ***  core 0 loop ***  core 0 loop ***  core 0 loop *** 

        sleep_ms(1);
        // display
        curr_millis = to_ms_since_boot(get_absolute_time());
        if( curr_millis - prev_millis > 100){
            prev_millis = curr_millis;
            update_display(); 

            // printf("midi cent %d\n", inf_midi_cent_idx);

            int inf_midi_cent_idx_cpy = inf_midi_cent_idx;
            if(inf_midi_cent_idx_cpy > 0){
                midi_idx = round(inf_midi_cent_idx_cpy/100);
                midi_cent = inf_midi_cent_idx_cpy - midi_idx*100;
                note_idx = midi_idx % 12;
                note_oct = (midi_idx - note_idx)/12 - 1;
            }

        }
        
    }

    return 0;
}

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, float freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (clock_get_hz(clk_sys) / (2 * freq)) - 3;
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

void setup_i2c1(void){

    i2c_init(i2c1, I2C0_BUADRATE);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    // gpio_pull_up(PIN_I2C_SDA);
    // gpio_pull_up(PIN_I2C_SCL);
}


void setup_oled(void) {

    disp.external_vcc=false;
#ifdef feather
    ssd1306_init(&disp, 128, 32, 0x3C, i2c1);
#else
    ssd1306_init(&disp, 128, 32, 0x3C, i2c0);
#endif
    ssd1306_clear(&disp);
    ssd1306_draw_string(&disp, 8, 0, 1, (char*)"SSD1306");
    ssd1306_draw_string(&disp, 8, 16, 2, (char*)"DISPLAY");
    ssd1306_show(&disp);
}

void update_display(void){ //inf_midi_cent_idx
    
    ssd1306_clear(&disp);

    char str[128];

    ssd1306_draw_string(&disp, 0, 0, 3, note[note_idx]);

    memset(str, 0, sizeof(char));
    sprintf(str, "%d:%d", note_oct, midi_cent);
    ssd1306_draw_string(&disp, 24, 0, 3, str);

    // memset(str, 0, sizeof(char));
    // sprintf(str, "%d, %d", midi_idx, midi_cent);
    // ssd1306_draw_string(&disp, 0, 16, 2, str);

    ssd1306_show(&disp);

    pin_toggle(PIN_LED); 

}
