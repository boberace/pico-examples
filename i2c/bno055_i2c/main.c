#include "bno055_i2c.h"
#include <stdio.h>
// #include <stdint.h>
// #include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C0_BUADRATE 400*1000
#define PIN_I2C0_SDA 16
#define PIN_I2C0_SCL 17
i2c_inst_t *I2C_BNO085 = i2c0;

void setup_i2c0(void);

BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
u8 power_mode ;
struct bno055_quaternion_t *quaternion_wxyz; // s16 {w,x,y,z}

int main() {
    stdio_init_all();
    setup_i2c0();

    printf("\n init %04x\n",bno055_i2c_init(i2c0));
    com_rslt += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
    quaternion_wxyz->w = 0;
    quaternion_wxyz->x = 0;
    quaternion_wxyz->y = 0;
    quaternion_wxyz->z = 0;

    while(true){
        com_rslt += bno055_read_quaternion_wxyz(quaternion_wxyz);  
        printf("w %04x, ",quaternion_wxyz->w );
        printf("x %04x, ",quaternion_wxyz->x );
        printf("y %04x, ",quaternion_wxyz->y );
        printf("z %04x\n",quaternion_wxyz->z );

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

