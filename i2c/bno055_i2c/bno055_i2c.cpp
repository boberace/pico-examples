/**
* Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*/

#include "bno055_i2c.h"
#include "pico/stdlib.h"
#include <stdio.h>

bno055_pico::bno055_pico(i2c_inst_t *i2c, uint gpio_sda, uint gpio_scl, uint i2c_khz){

    _i2c = i2c;
    _gpio_sda = gpio_sda;
    _gpio_scl = gpio_scl;
    _i2c_khz = i2c_khz;

    gpio_set_function(_gpio_sda, GPIO_FUNC_I2C);
    gpio_set_function(_gpio_scl, GPIO_FUNC_I2C);
    gpio_pull_up(_gpio_sda);
    gpio_pull_up(_gpio_scl);
    i2c_init(_i2c, _i2c_khz * 1000); 

}

void bno055_pico::BNO055_delay_msek(u32 msek)
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
s8 bno055_pico::BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    u8 IERROR = 0;
    if(i2c_write_blocking(_i2c, dev_addr, &reg_addr, 1, true)){
        if(i2c_write_blocking(_i2c, dev_addr, reg_data, cnt, false) != cnt){
            printf(" failed to write to %d bytes to address %02x on I2C_BNO085\n", cnt, reg_addr);
            IERROR = -1;
        }

    }else{
        printf(" failed to write address %02x to I2C_BNO085\n", reg_addr);
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

s8 bno055_pico::BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    u8 IERROR = 0;
    if(i2c_write_blocking(_i2c, dev_addr, &reg_addr, 1, true)){
        if(i2c_read_blocking(_i2c, dev_addr, reg_data, cnt, false) != cnt){
            printf(" failed to read %d bytes from adrress %02x on I2C_BNO085\n", cnt, reg_addr);
            IERROR = -1;
        }
    } else {
        printf(" failed to write address %02x to I2C_BNO085\n", reg_addr);
        IERROR = -1;
    }
    return IERROR;
}