// #include "bno055_i2c.h"
extern "C" { // can't get this from .c ?
#include "bno055.h"
}
#include <stdio.h>
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
i2c_inst_t *I2C_BNO085 = i2c0;

void setup_i2c0(void);

struct bno055_t bno055;

void BNO055_delay_msek(u32 msek);
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);


// bno055_i2c bno055(i2c0);

int main() {
    stdio_init_all();
    setup_i2c0();

    // intialize address and function pointers of init struct
    bno055.bus_write = BNO055_I2C_bus_write;
    bno055.bus_read = BNO055_I2C_bus_read;
    bno055.delay_msec = BNO055_delay_msek;
    bno055.dev_addr = BNO055_I2C_ADDR2;
    // pass init struct to init function to collect init data
    if(bno055_init(&bno055) != 0) printf("bno055_i2c init error");
 
    printf("id %0x\n", bno055.chip_id);

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



void BNO055_delay_msek(u32 msek)
{
    sleep_ms(msek);
}

/*  \Brief: The API is used as I2C bus write
 *  \Return : Status of the I2C write
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *   will data is going to be written
 *  \param reg_data : It is a value hold in the array,
 *      will be used for write the value into the register
 *  \param cnt : The no of byte of data to be write
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    u8 IERROR = 0;
    if(i2c_write_blocking(I2C_BNO085, dev_addr, &reg_addr, 1, true)){
        if(i2c_write_blocking(I2C_BNO085, dev_addr, reg_data, cnt, false) != cnt){
            printf("BNO055_I2C_bus_write failed to write to %d bytes to address %02x on I2C_BNO085\n", cnt, reg_addr);
            IERROR = -1;
        }

    }else{
        printf("BNO055_I2C_bus_write failed to write address %02x to I2C_BNO085\n", reg_addr);
        IERROR = -1;
    }

    return IERROR;
}

/*  \Brief: The API is used as I2C bus read
 *  \Return : Status of the I2C read
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *  will data is going to be read
 *  \param reg_data : This data read from the sensor,
 *   which is hold in an array
 *  \param cnt : The no of byte of data to be read
 */

s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    u8 IERROR = 0;
    if(i2c_write_blocking(I2C_BNO085, dev_addr, &reg_addr, 1, true)){
        if(i2c_read_blocking(I2C_BNO085, dev_addr, reg_data, cnt, false) != cnt){
            printf("BNO055_I2C_bus_read failed to read %d bytes from adrress %02x on I2C_BNO085\n", cnt, reg_addr);
            IERROR = -1;
        }
    } else {
        printf("BNO055_I2C_bus_read failed to write address %02x to I2C_BNO085\n", reg_addr);
        IERROR = -1;
    }
    return IERROR;
}