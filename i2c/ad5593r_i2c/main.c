/*



*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ad5593r.h"

#define I2C0_BUADRATE 400*1000
#define PIN_I2C0_SDA 16 //12
#define PIN_I2C0_SCL 17 //13
i2c_inst_t *I2C_AD5593R = i2c0;

#define AD5593R_I2C_ADDR 0x29
#define AD5593R_CHIP_ID_ADDR 0x00


void setup_i2c0(void);

int main() {
    stdio_init_all();
    setup_i2c0();
    while(true){     
 

    }
    return 0;
}

void setup_i2c0(void){
    i2c_init(i2c0, I2C0_BUADRATE);
    gpio_set_function(PIN_I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C0_SDA);
    gpio_pull_up(PIN_I2C0_SCL);
}
