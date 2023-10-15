/*

    
    todo : create functions to set up the DACs and ADCs
*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ad5593r_pico.h"

#define I2CA_INST i2c0
#define I2CA_BUADRATE 400*1000
#define PIN_I2CA_SDA 12
#define PIN_I2CA_SCL 13

#define ADR5593R_I2C_ADDR 0x10
#define ADR5593R_ADC_CHANS 0b00111111

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
    int32_t ret = 0;

    ret = ad5593r_init_pico(I2CA_INST, ADR5593R_I2C_ADDR);   
    
    if(ret==0) printf("\nAD5593R initialized\n");
    else printf("\nAD5593R initialization failed %d\n", ret);

    uint32_t dt;
    uint64_t pt = to_us_since_boot(get_absolute_time()), ct;

    uint16_t combined_values[6];
    int16_t values[6] = {0};
    uint16_t left = 0, right = 0;

    while(true){  

        pt = to_us_since_boot(get_absolute_time());   

        if(ad5593r_get_adc_values(ADR5593R_ADC_CHANS, combined_values) == 0){  

            for(int i=0; i<6; i++){
                values[(combined_values[i]>>12)&0b111] = (combined_values[i]&0xFFF) - 0x7FF;
            }     


            
            left = (values[0] + values[1] + values[2])/3;
            right = (values[3] + values[4] + values[5])/3;

            ret =  ad5593r_set_dac_value(6,left);
            if (ret != 0){
                printf("DAC6 write failed %d\n", ret);
            }
            ret = ad5593r_set_dac_value(7,right);
            if (ret != 0){
                printf("DAC7 write failed %d\n", ret);
            }

            ct = to_us_since_boot(get_absolute_time());
            dt = (ct - pt);

            for(int i=0; i<6; i++){
                printf("ADC%d: %d, ", i, values[i]);
            }          
            printf("\033[A\33[2K\r dt: %d \n", dt);
        }

    }
    return 0;
}

