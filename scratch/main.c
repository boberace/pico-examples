/*

    
    todo : create functions to set up the DACs and ADCs
*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ad5593r_pico.h"
#include <malloc.h>

#define I2CA_INST i2c0
#define I2CA_BUADRATE 400*1000
#define PIN_I2CA_SDA 12
#define PIN_I2CA_SCL 13

#define ADR5593R_I2C_ADDR 0x10
#define ADR5593R_ADC_CHANS 0b00111111

#define LOOP_LENGTH 5*1024
uint16_t adc_values[LOOP_LENGTH][6] = {0};
uint16_t dac_values[LOOP_LENGTH][2] = {0};
uint16_t loop_counter = 0;

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
        return 1;}  
    
    // if(ret==0) printf("\nAD5593R initialized\n");
    // else printf("\nAD5593R initialization failed %d\n", ret);

    // uint32_t dt;
    // uint64_t pt = to_us_since_boot(get_absolute_time()), ct;

    uint16_t combined_values[6];
    int16_t values[6] = {0};
    // uint16_t left = 0, right = 0;

    for(uint i = 0; i < LOOP_LENGTH; i++){
        for(uint j = 0; j < 6; j++){
            adc_values[i][j] = i*j;
        }
        for(uint j = 0; j < 2; j++){
            dac_values[i][j] = i*j;
        }
    }

    printf(" total %i, free %i\n", getTotalHeap(), getFreeHeap());

    printf("ST ADC\n");
    
    while(true){  

        // pt = to_us_since_boot(get_absolute_time());   

        if(ad5593r_get_adc_values(ADR5593R_ADC_CHANS, combined_values) == 0){  

            for(int i=0; i<6; i++){
                adc_values[loop_counter][(combined_values[i]>>12)&0b111] = (combined_values[i]&0xFFF) - 0x7FF;
            }    
            dac_values[loop_counter][0] = adc_values[loop_counter][0] + adc_values[loop_counter][1] + adc_values[loop_counter][2];
            dac_values[loop_counter][1] = adc_values[loop_counter][3] + adc_values[loop_counter][4] + adc_values[loop_counter][5];

            uint32_t ps = gpio_get_all();
            uint8_t s0 = !!((1ul << 21) & ps);
            // uint8_t s1 = !!((1ul << 20) & ps);
            // uint8_t s2 = !!((1ul << 19) & ps);
            // uint8_t s3 = !!((1ul << 18) & ps);
            // uint8_t s4 = !!((1ul << 17) & ps);
            // uint8_t s5 = !!((1ul << 16) & ps);

            // gpio_put(12, !s1);
            // gpio_put(13, s1);

            // gpio_put(10, !s2);
            // gpio_put(11, s2);

            // gpio_put(8, !s3);
            // gpio_put(9, s3);


            // printf("%i %i %i %i %i %i",s1,s2,s3,s4,s5,s6);  

            // for(int i=16; i<=21; i++){ 
            //     printf("%i ",gpio_get(i));           
                
            // }
            // printf("\033[A\33[2K\r\n");
            
            // printf("%i %i\n", s0?2047:-2047, values[0]);

            loop_counter++;
            if(loop_counter == LOOP_LENGTH) loop_counter = 0;
        }

    }
    return 0;
}

