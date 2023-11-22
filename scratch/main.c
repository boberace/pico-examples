/*

    
    todo : create functions to set up the DACs and ADCs
*/
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ad5593r_pico.h"
#include <malloc.h>

#define I2CA_INST i2c0
#define I2CA_BUADRATE 400*1000
#define PIN_I2CA_SDA 12
#define PIN_I2CA_SCL 13

#define PIN_FBS 14

#define ADR5593R_I2C_ADDR 0x10
#define ADR5593R_ADC_CHANS 0b00111111
#define ADR5593R_DAC_CHANS 0b11000000

#define LOOP_SIG_BIT 10
#define LOOP_LENGTH (1 << LOOP_SIG_BIT)
#define NUM_ADC 6
#define NUM_DAC 2
#define NUM_BITS_ADC 12
#define NUM_BITS_DAC 12
#define ADC_MID (1<<(NUM_BITS_ADC-1))
#define ST_OFFSET 50

uint16_t adc_values[LOOP_LENGTH][NUM_ADC] = {0};
uint64_t adc_sum[NUM_ADC] = {0}; // sum of all adc values per channel per circular buffer
uint16_t adc_min[NUM_ADC] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF}; // min of all adc values per channel per circular buffer
uint16_t adc_max[NUM_ADC] = {0}; // max of all adc values per channel per circular buffer
uint16_t adc_avg[NUM_ADC] = {0}; // average of all adc values per channel per circular buffer
int16_t adc_ofs[NUM_ADC] = {-565, -1117, -1102, 763 ,569 ,301}; // offset of all adc values per channel
uint16_t dac_values[LOOP_LENGTH][NUM_DAC] = {0}; // dac values per channel per circular buffer
uint16_t combined_values[NUM_ADC]; // combined adc values per channel per circular buffer (12 bit adc values in 16 bit int)
uint16_t loop_counter = LOOP_LENGTH-1;
uint16_t prev_loop_counter = LOOP_LENGTH-2;

uint64_t adc_tus[NUM_ADC] = {0}; // time of last adc value channel rising zero crossing
uint64_t adc_per[NUM_ADC] = {0}; // period of last channel cycle
uint8_t adc_zcf[NUM_ADC] = {0}; // flag for rising zero crossing

int8_t pflag=0;
uint64_t pt=0;

extern char _heap_start, _heap_end;

uint32_t getTotalHeap(void) {
   extern char __StackLimit, __bss_end__;
   
   return &__StackLimit  - &__bss_end__;
}

uint32_t getFreeHeap(void) {
   struct mallinfo m = mallinfo();

   return getTotalHeap() - m.uordblks;
}

void setup_i2cA(void){

    i2c_init(I2CA_INST, I2CA_BUADRATE);
    gpio_set_function(PIN_I2CA_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2CA_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2CA_SDA);
    gpio_pull_up(PIN_I2CA_SCL);
    
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    setup_i2cA();
    printf("\nAD5593R\n");

    if(ad5593r_init_pico(I2CA_INST, ADR5593R_I2C_ADDR) !=0){
        printf("\nAD5593R initialization failed\n");
        return 1;
        }  


    printf(" total %i, free %i\n", getTotalHeap(), getFreeHeap());

    printf("ST ADC\n\n");

    for(int i=0; i<12; i++){
        gpio_init(i);
        gpio_set_dir(i, GPIO_OUT);
        gpio_put(i,0);
    }

    gpio_init(PIN_FBS);
    gpio_set_dir(PIN_FBS, GPIO_OUT);
    gpio_put(PIN_FBS,1);

    while(true){

        uint64_t bt = to_us_since_boot(get_absolute_time());   

        if(ad5593r_get_adc_values(ADR5593R_ADC_CHANS, combined_values) == 0){  
            prev_loop_counter = loop_counter;
            loop_counter++;
            if(loop_counter == LOOP_LENGTH) { // get average and reset

                for(int i=0; i<NUM_ADC; i++){
                    adc_avg[i] = adc_sum[i] >> LOOP_SIG_BIT; 
                    adc_sum[i] = 0;
                    adc_max[i] = 0;
                    adc_min[i] = 0xFFFF;
                }
                loop_counter = 0;
            }
            int16_t value = 0;
            int16_t pvalue = 0;
            uint value_avg = 0;
            for(int i=0; i<NUM_ADC; i++){
                uint index = (combined_values[i]>>12)&0b111;
                pvalue = value;
                value = (combined_values[i]&0xFFF) - adc_ofs[index];
                value = (value < 0) ? 0 : value;
                value = (value > 0xFFF) ? 0xFFF : value;
                adc_values[loop_counter][index] = value;
                adc_sum[index] += value;
                adc_max[index] = (adc_max[index] > value) ? adc_max[index] : value;
                adc_min[index] = (adc_min[index] < value) ? adc_min[index] : value;
                value_avg+=value;
                uint pindex = index*2;
                uint nindex = index*2+1;
                // the following test only works on a clean signal with no dips below zero
                if(value > (ADC_MID + ST_OFFSET)){ // rising zero crossing
                    if(pvalue < (ADC_MID + ST_OFFSET))
                    {
                        uint64_t tus = to_us_since_boot(get_absolute_time());
                        adc_per[i] = tus - adc_tus[i];
                        adc_tus[i] = tus;
                    }
                    // gpio_put(pindex,0);
                    // gpio_put(nindex,1);
                }else if (value < (ADC_MID - ST_OFFSET)){
                    // gpio_put(pindex,1);
                    // gpio_put(nindex,0);
                }else{
                    // gpio_put(pindex,0);
                    // gpio_put(nindex,0);
                }

            }    
            // dac_values[loop_counter][0] = value_avg/6;
            // dac_values[loop_counter][1] = value_avg/6;

            // uint64_t tt = to_us_since_boot(get_absolute_time()); 
            // tt%=1000000;
            // float frq = 220;
            // float us_period = 1000000/frq;
            // dac_values[loop_counter][0] = (ADC_MID-1)*cosf(tt * 2 * (float) (M_PI / us_period)) + ADC_MID;
            // dac_values[loop_counter][1] = (ADC_MID-1)*sinf(tt * 2 * (float) (M_PI / us_period)) + ADC_MID;

            // int32_t ret = ad5593r_set_dac_values(ADR5593R_DAC_CHANS, dac_values[loop_counter]);
            // if (ret != 0){
            //     printf("DAC write failed %d\n", ret);
            // }   

            // uint32_t ps = gpio_get_all();
        
        }          

        uint64_t at = to_us_since_boot(get_absolute_time());
        uint dt = at - bt;
        

        if(at - pt > 50000){
        pt = at;
        printf("usp: ");
        for(int i=0; i<NUM_ADC; i++) printf("ADC%d: %4d, ", i, (adc_per[i]));  
        printf("                \n\nmax: ");
        for(int i=0; i<NUM_ADC; i++) printf("ADC%d: %4d, ", i, (adc_max[i]-ADC_MID));  
        printf("                \n\navg: ");
        for(int i=0; i<NUM_ADC; i++) printf("ADC%d: %4d, ", i, (adc_avg[i]-ADC_MID));
        printf("                \n\nmin: ");
        for(int i=0; i<NUM_ADC; i++) printf("ADC%d: %4d, ", i, (adc_min[i]-ADC_MID));
        printf("                \n\ncur: ");
        for(int i=0; i<NUM_ADC; i++) printf("ADC%d: %4d, ", i, (adc_values[loop_counter][i]-ADC_MID));      
        for(int i=0; i<NUM_DAC; i++) printf("DAC%d: %4d, ", i, (dac_values[loop_counter][i]-ADC_MID));    
        printf("\n\ndt: %i, counter: %i \n", dt, loop_counter);
        printf("\033[11A"); // move cursor up 4 lines
        }


    }
    return 0;
}

