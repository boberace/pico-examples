/*

    i2c0 pins 12,13 initialized and used by add5593r
    todo : initialize in here and pass to ad5593r_init_pico
*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ad5593r_pico.h"

int main() {
    stdio_init_all();
    sleep_ms(2000);
    printf("\nAD5593R\n");
    int32_t ret = 0;

    ret = ad5593r_init_pico();   
    
    if(ret==0) printf("\nAD5593R initialized\n");
    else printf("\nAD5593R initialization failed %d\n", ret);

    uint32_t counter = 0;
    uint32_t counter_top = 100;
    uint64_t pt = to_us_since_boot(get_absolute_time());

    while(true){     
        counter++;


        // sleep_ms(100);
        // uint16_t value;
        // ret = ad5593r_get_adc_value(0, &value);
        // if (ret == 0)
        // printf("ADC0: %d\n", value);
        // else
        // printf("ADC0 read failed %d\n", ret);

        uint16_t chans = 0b111111;
        uint16_t values[6];
        ret = ad5593r_get_adc_values(chans, values);
        uint16_t left = 0, right = 0;
        if (ret == 0){            
            for(int i=0; i<3; i++){
                left += (values[i]&0xFFF) - 0x7FF;
            }
            for(int i=3; i<6; i++){
                right += (values[i]&0xFFF) - 0x7FF;
            }
            left /= 3;
            right /= 3;
            // for(int i=0; i<6; i++){
            //     printf("ADC%d: %d, ", (values[i]>>12)&0b111, (values[i]&0xFFF) - 0x7FF);
            // }
            // printf("\n");
        }
        // else
        // printf("ADC0 read failed %d\n", ret);

        ret =  ad5593r_set_dac_value(6,left);
        if (ret != 0){
            printf("DAC6 write failed %d\n", ret);
        }
        ret = ad5593r_set_dac_value(7,right);
        if (ret != 0){
            printf("DAC7 write failed %d\n", ret);
        }

        
        if(counter % counter_top == 0){            
            uint64_t ct = to_us_since_boot(get_absolute_time());
            uint32_t dt = (ct - pt)/counter_top;
            printf("\033[A\33[2K\r%i\n", dt);
            pt = to_us_since_boot(get_absolute_time());
        }



    }
    return 0;
}

