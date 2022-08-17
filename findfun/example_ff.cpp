
#include "pico/stdlib.h"
#include "findfun.h"
#include <stdio.h>
#include <math.h>
#include <vector>

using std::vector;


findfun ff = findfun();

float FunFreq = 440.0; // fundamental frequency
// sampling frequency (hz)  for generating integer period lengths 
float SF = 2.5 * 1000000;    
float SP = 0.035; // sampling period (seconds)  (over 0.05 will cause memory panic in pico)
int LMI = 38 ; // guitar 38 # piano 21
int HMI = 66 + 1; // guitar 66 # piano 96
int NCS = 20 ; // number of correlation shifts

int NS = int(SF*SP);
int sample_freq_cents_off = 47;

vector<vector<int>> m_sample_edges;

int main() {
    stdio_init_all();
    ff.begin(FunFreq, SF, LMI, HMI, NCS);

    for(auto mi = LMI; mi < HMI; ++mi ){
        float sample_freq = FunFreq*(pow(2,((mi*100. + sample_freq_cents_off -6900.)/1200.)));
        int sample_period = ceil(SF/sample_freq);
        int sample_half_period = ceil(sample_period/2);
        vector<int> sample_edges;
        for(auto se = sample_half_period; se < SF*SP; se += sample_half_period ){
            sample_edges.push_back(se);
        }
        m_sample_edges.push_back(sample_edges);
    } 

    while(1){

        int idx = LMI;
        for (auto mse: m_sample_edges){


            absolute_time_t  start = get_absolute_time();
                int fmc = ff.find_midi_cent(mse);
            absolute_time_t  end = get_absolute_time();
            fmc = fmc + LMI*100;
            int t = absolute_time_diff_us(start,end);
            t = t/1000;
            int tmc = (LMI + idx)*100 + sample_freq_cents_off;
            printf("target midi cent: %d found midi cent: %d, time %d\n", tmc, fmc,t);

            idx++;
        }

        sleep_ms(2000);
        printf("\n");

    }
    return 0;
}
