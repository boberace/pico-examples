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

    printf("\n\n\n start\n");
    comres = bno055_i2c_init(i2c0);
    printf("\n bno055_i2c_init %04x\n",comres);
    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF); //
    // comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_AMG); // 
    printf("\nbno055_set_operation_mode %04x\n",comres);
    sleep_us(30);

    // u8 cs = 0x80;
    // bno055_set_clk_src(cs);
    // u8reg = cmag = cacc = cgyr = csys = 0;
    while(true){

            if(comres == 0){                
                printf("counts %d\n", ++counter);
                                
                if(csys == 3 and cacc == 3 and cgyr == 3 and cmag == 3){
                    quaternion_wxyz.w = -1;
                    quaternion_wxyz.x = -1;
                    quaternion_wxyz.y = -1;
                    quaternion_wxyz.z = -1;
                    // comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
                    comres += bno055_read_quaternion_wxyz(&quaternion_wxyz);  
                    printf("\nbno055_read_quaternion_wxyz %02x\n",comres);
                    printf("w %04x, ",quaternion_wxyz.w );
                    printf("x %04x, ",quaternion_wxyz.x );
                    printf("y %04x, ",quaternion_wxyz.y );
                    printf("z %04x\n",quaternion_wxyz.z );

                    // comres += bno055_read_accel_xyz(&accel_xyz);
                    // printf("bno055_read_accel_xyz %04x\n",comres);
                    // printf("x %04x, ",accel_xyz.x );
                    // printf("y %04x, ",accel_xyz.y );
                    // printf("z %04x\n",accel_xyz.z );

                    // comres += bno055_read_mag_xyz(&mag_xyz);
                    // printf("bno055_read_mag_xyz %04x\n",comres);
                    // printf("x %04x, ",mag_xyz.x );
                    // printf("y %04x, ",mag_xyz.y );
                    // printf("z %04x\n",mag_xyz.z );

                    // comres += bno055_read_gyro_xyz(&gyro_xyz);
                    // printf("bno055_read_gyro_xyz %04x\n",comres);
                    // printf("x %04x, ",gyro_xyz.x );
                    // printf("y %04x, ",gyro_xyz.y );
                    // printf("z %04x\n",gyro_xyz.z );
                } else {
                    u8reg = cmag = cacc = cgyr = csys = -1;    
                    u8 re = getcalib();                
                    printf("\ncsys %d, cacc %d, cgyr %d, cmag %d, e %d \n", csys, cacc,  cgyr, cmag, re);
                }
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

