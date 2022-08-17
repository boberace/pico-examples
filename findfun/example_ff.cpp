
#include "pico/stdlib.h"
#include <math.h>
#include <time.h>
#include <vector>

using std::vector;
// #include <algorithm>

// findfun ff = findfun();

float FunFreq = 440.0; // fundamental frequency
// sampling frequency (hz)  for generating integer period lengths 
float SF = 2.5 * 1000000;    
float SP = 0.05; // sampling period (seconds) -        (0.25s for 0 to 127) (0.1s   for piano) (0.03    for guitar)
int LMI = 38 ; // guitar 38 # piano 21
int HMI = 66 + 1; // guitar 66 # piaon 96
int NCS = 20 ; // number of correlation shifts

int NS = int(SF*SP);
int sample_freq_cents_off = -49;

clock_t t;

vector<vector<int>> m_sample_edges;

int main() {
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
}
