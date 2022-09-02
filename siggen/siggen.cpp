/*



*/


#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <stdio.h>
#include <math.h>
#include <vector>
#include "ssd1306.h"
#include "cindex.h"
#include "sin.h"

using std::vector;

#define PIN_SIGNAL 15
#define PIN_LED 25

#define SINE_RES 1000 // how many points in sine wave period
#define PI 3.14159265
#define SIG_UPDATE_MICROS 50 // signal update interval in microseconds

#define pin_toggle(x) gpio_put(x, !gpio_get(x))

double amplitude = 0;
double amplitude_prev = 0;
int pin_signal_state = 0;

uint64_t update_counter = 0;
uint64_t update_counter_prev = 0;

double ConPitch = 440.0;  // concert pitch (piano middle A : midi 69 : A4)
// sampling frequency (hz)  for generating integer period lengths 
double SF = 1'000'000; // same as the sample frequency of the edges sent    

bool signal_timer_callback(struct repeating_timer *t);

void core1_entry() {  

    gpio_init(PIN_SIGNAL);
    gpio_set_dir(PIN_SIGNAL, GPIO_OUT);

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    int string_update_seconds = 1;

    struct repeating_timer timer;
    add_repeating_timer_us(-SIG_UPDATE_MICROS, signal_timer_callback, NULL, &timer);  

    int num_overtones = 7; // first overtone [0] is fundamental - makes code easier

    double over_tones[num_overtones] = {0};

    uint64_t prev_micros = 0;
    uint64_t curr_micros = 0;
    
    double fundamental_pitch = 111;
    double fundamental_period_micros = 1'000'000 / fundamental_pitch;

    while(1){

        // // decay overtones at different rates decreasing in order
        // for(auto i=0; i<num_overtones ; i++){
        //     // over_tones[i] = (rand()/RAND_MAX)/(i+1);
        // }     

        // uint64_t micros = to_us_since_boot(get_absolute_time());
        
        // for(auto i=0; i<num_overtones ; i++){
        //     amplitude += over_tones[i]*sine[(int)round((i+1)*test_pitch*micros)%1000];
        // }   

        // generate seed overtones 
        // curr_micros = to_us_since_boot(get_absolute_time());
        // if(curr_micros - prev_micros > string_update_seconds){
        //     prev_micros = curr_micros;

        //     for(auto i=0; i<num_overtones ; i++){
        //         over_tones[i] = (rand()/RAND_MAX)/(i+1);
        //     }         

        // }

        double num_updates_pp =  fundamental_period_micros/SIG_UPDATE_MICROS; // number of updates per fundamental period
        double sine_index_skip = SINE_RES / num_updates_pp;

        if(update_counter != update_counter_prev){
            update_counter_prev = update_counter;

            int s_index = ((int)round(update_counter*sine_index_skip)) % SINE_RES;
            amplitude = sine[s_index];

            if ((amplitude <= 0) && (amplitude_prev > 0)){
                pin_signal_state = 1; 
            } else if ((amplitude >= 0) && (amplitude_prev < 0)) {
                pin_signal_state = 0; 
            }
            
            amplitude_prev = amplitude;

        }

        curr_micros = to_us_since_boot(get_absolute_time());
        if(curr_micros - prev_micros > 500000){
            prev_micros = curr_micros;

        printf(" num_updates_pp: %f sine_index_skip, %f\n",num_updates_pp,  sine_index_skip);
        pin_toggle(PIN_LED);

        }

    }

}

int main() {
    stdio_init_all();
    multicore_launch_core1(core1_entry);

    while(1){


        sleep_ms(2000);
 

    }
    return 0;
}


bool signal_timer_callback(struct repeating_timer *t) {

    gpio_put(PIN_SIGNAL, pin_signal_state);
    update_counter++;

    return true;
}