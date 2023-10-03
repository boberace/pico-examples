/*

    i2c0 pins 12,13 initialized and used by add5593r

*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ad5593r_pico.h"

int main() {
    stdio_init_all();
    setup_i2c0();

    sleep_ms(2000);
    printf("\nAD5593R\n");
    uint32_t ret = 0;

    ret = ad5593r_init_pico();   
    
    if(ret==0) printf("\nAD5593R initialized\n");
    else printf("\nAD5593R initialization failed %d\n", ret);

    while(true){     

        sleep_ms(1000);
        // uint16_t value;
        // ret = ad5593r_adc_value(0, &value);
        // if (ret == 0)
        // printf("ADC0: %d\n", value);
        // else
        // printf("ADC0 read failed %d\n", ret);

        uint16_t chans = 0b111111;
        uint16_t values[6];
        ad5593r_adc_values(chans, values);
        if (ret == 0){
            for(int i=0; i<6; i++){
                printf("ADC%d: %d, ", (values[i]>>12)&0b111, (values[i]&0xFFF) - 0x7FF);
            }
            printf("\n");
        }
        else
        printf("ADC0 read failed %d\n", ret);

    }
    return 0;
}

