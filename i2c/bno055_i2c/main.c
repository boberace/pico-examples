/*

issues:  using external clock only good on first flash - zero output after reset

*/
#include <stdio.h>
// #include <stdint.h>
// #include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C0_BUADRATE 400*1000
#define PIN_I2C0_SDA 12
#define PIN_I2C0_SCL 13
i2c_inst_t *I2C_BNO085 = i2c0;

#define BNO055_I2C_ADDR 0x29
#define BNO055_CHIP_ID_ADDR 0x00

void setup_i2c0(void);
// void bno055status(void);
// u8 getcalib(void);
void ndof_init(void);

int main() {
    stdio_init_all();
    setup_i2c0();
    ndof_init();
   
    uint8_t acc[6]; // Store data from the 6 acc registers
    int16_t acc_x, acc_y, acc_z; // Combined 3 axis data
    uint8_t acc_reg = 0x08; // 4.3.9 ACC_DATA_X_LSB 0x08

    uint8_t mag[6]; // Store data from the 6 mag registers
    int16_t mag_x, mag_y, mag_z; // Combined 3 axis data
    uint8_t mag_reg = 0x0E; // 4.3.15 MAG_DATA_X_LSB 0x0E    

    uint8_t gyr[6]; // Store data from the 6 gyr registers
    int16_t gyr_x, gyr_y, gyr_z; // Combined 3 axis data
    uint8_t gyr_reg = 0x14; // 4.3.21 GYR_DATA_X_LSB 0x14 

    uint8_t eul[6]; // Store data from the 6 eul registers
    int16_t eul_x, eul_y, eul_z; // Combined 3 axis data
    uint8_t eul_reg = 0x1A; // 4.3.27 EUL_DATA_X_LSB 0x1A 

    uint8_t qat[8]; // Store data from the 6 qat registers
    int16_t qat_w, qat_x, qat_y, qat_z; // Combined 3 axis data
    uint8_t qat_reg = 0x20; // 4.3.33 QUA_DATA_W_LSB 0x20 

    while(true){
        
        i2c_write_blocking(I2C_BNO085, BNO055_I2C_ADDR, &acc_reg, 1, true);
        i2c_read_blocking(I2C_BNO085, BNO055_I2C_ADDR, acc, 6, false);
        acc_x = ((acc[1]<<8) | acc[0]);
        acc_y = ((acc[3]<<8) | acc[2]);
        acc_z = ((acc[5]<<8) | acc[4]);
        printf("accel X: %d    Y: %d    Z: %d\n", acc_x, acc_y, acc_z);

        i2c_write_blocking(I2C_BNO085, BNO055_I2C_ADDR, &mag_reg, 1, true);
        i2c_read_blocking(I2C_BNO085, BNO055_I2C_ADDR, mag, 6, false);
        mag_x = ((mag[1]<<8) | mag[0]);
        mag_y = ((mag[3]<<8) | mag[2]);
        mag_z = ((mag[5]<<8) | mag[4]);        
        printf("mag X: %d    Y: %d    Z: %d\n", mag_x, mag_y, mag_z);

        i2c_write_blocking(I2C_BNO085, BNO055_I2C_ADDR, &gyr_reg, 1, true);
        i2c_read_blocking(I2C_BNO085, BNO055_I2C_ADDR, gyr, 6, false);
        gyr_x = ((gyr[1]<<8) | gyr[0]);
        gyr_y = ((gyr[3]<<8) | gyr[2]);
        gyr_z = ((gyr[5]<<8) | gyr[4]);
        printf("gyr X: %d    Y: %d    Z: %d\n", gyr_x, gyr_y, gyr_z);

        i2c_write_blocking(I2C_BNO085, BNO055_I2C_ADDR, &eul_reg, 1, true);
        i2c_read_blocking(I2C_BNO085, BNO055_I2C_ADDR, eul, 6, false);
        eul_x = ((eul[1]<<8) | eul[0]);
        eul_y = ((eul[3]<<8) | eul[2]);
        eul_z = ((eul[5]<<8) | eul[4]);
        printf("eul X: %d    Y: %d    Z: %d\n", eul_x, eul_y, eul_z);

        i2c_write_blocking(I2C_BNO085, BNO055_I2C_ADDR, &qat_reg, 1, true);
        i2c_read_blocking(I2C_BNO085, BNO055_I2C_ADDR, qat, 8, false);
        qat_w = ((qat[1]<<8) | qat[0]);
        qat_x = ((qat[3]<<8) | qat[2]);
        qat_y = ((qat[5]<<8) | qat[4]);
        qat_z = ((qat[7]<<8) | qat[6]);
        printf("qat X: %d    Y: %d    Z: %d    W: %d\n", qat_x, qat_y, qat_z, qat_w);

        sleep_ms(300);            

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



void ndof_init(void){
    // Check to see if connection is correct
    sleep_ms(1000); // Add a short delay to help BNO005 boot up
    uint8_t reg = BNO055_CHIP_ID_ADDR;
    uint8_t chipID[1];
    i2c_write_blocking(I2C_BNO085, BNO055_I2C_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_BNO085, BNO055_I2C_ADDR, chipID, 1, false);

    if(chipID[0] != 0xA0){
        while(1){
            printf("Chip ID Not Correct - Check Connection!");
            sleep_ms(5000);
        }
    }

    uint8_t data[2];

    // Set operation to ndof
    data[0] = 0x3D; // 4.3.61 OPR_MODE 0x3D
    data[1] = 0b00001100; // xxxx1100 ndof
    i2c_write_blocking(I2C_BNO085, BNO055_I2C_ADDR, data, 2, true);
    sleep_ms(100);
}
