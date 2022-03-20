#include "bno055_i2c.h"
// #include <stdio.h>
// #include <stdint.h>
// #include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
// #include "hardware/adc.h"
// #include <math.h>
// #include "hardware/pwm.h"
// #include "hardware/pio.h"
// #include "hardware/clocks.h"
// #include "hardware/irq.h"

#define I2C0_BUADRATE 400*1000
#define PIN_I2C0_SDA 16
#define PIN_I2C0_SCL 17

bno055_i2c bno055(i2c0);

void setup_i2c0(void);


int main() {
    stdio_init_all();
    setup_i2c0();

    // while(true){


    // }
    return 0;
}

void setup_i2c0(void){
    i2c_init(i2c0, I2C0_BUADRATE);
    gpio_set_function(PIN_I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C0_SDA);
    gpio_pull_up(PIN_I2C0_SCL);
}
