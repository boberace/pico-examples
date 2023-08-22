/*



*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "kxtj3_1057_i2c.h"

const static uint I2C0_BUADRATE = 400*1000;
const static uint8_t PIN_I2C0_SDA = 26;
const static uint8_t PIN_I2C0_SCL = 27;
i2c_inst_t *I2C_KXTJ3 = i2c1;

const static uint8_t KXTJ3_I2C_ADDR = 0x0E;

// define interrupt and reset pins
const static uint8_t PIN_BNO085_INT = 22;

kxtj3_i2c kxtj3(I2C_KXTJ3, KXTJ3_I2C_ADDR, PIN_BNO085_INT);

void setup_i2c0(void);

int main() {
    stdio_init_all();
    setup_i2c0();
    sleep_ms(2000);
    printf("kxtj3 demo\n");
    int begin = kxtj3.begin(DATA_RATE_6, ACCEL_4G, true);
    if(begin == PICO_OK){
        printf("kxtj3 begin success \n");
    } else {
        printf("kxtj3 begin error %i, 0x%02x\n", begin, begin);
    }

    printf("\n");
    absolute_time_t pt = get_absolute_time();
    while(true){     


    absolute_time_t ct = get_absolute_time();
    if(ct - pt >= 500000){  
        pt = ct;
        printf("\033[A\033[2K\rkxtj3 demo running %llu seconds since boot\n", ct/1000000);

        axes a;
        int ret = kxtj3.get_accel_data(&a);
        if(ret == PICO_OK){
            printf("\033[B\033[2K\r x: %f, y: %f, z: %f\n", a.x, a.y, a.z);
            for (int i = 7; i >= 0; i--) { printf("%d", (a.x_h >> i) & 1); }
            for (int i = 7; i >= 0; i--) { printf("%d", (a.x_l >> i) & 1); }
            printf(" ");
            for (int i = 7; i >= 0; i--) { printf("%d", (a.y_h >> i) & 1); }
            for (int i = 7; i >= 0; i--) { printf("%d", (a.y_l >> i) & 1); }
            printf(" ");
            for (int i = 7; i >= 0; i--) { printf("%d", (a.z_h >> i) & 1); }
            for (int i = 7; i >= 0; i--) { printf("%d", (a.z_l >> i) & 1); }
        } else {
            printf("\033[B\033[2K\rkxtj3 get_accel_data error %i, 0x%02x\n", ret, ret);
        }
        printf("\033[2A");  // Move the cursor up to keep time since boot as the first line
    }


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
