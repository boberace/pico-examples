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
#include <stdio.h>

i2c_inst_t *i2cx;

s32 bno055_i2c_init(i2c_inst_t *i2c){
    i2cx = i2c;
    s32 comres = BNO055_ERROR;

    /*---------------------------------------------------------------------------*
     *********************** START INITIALIZATION ************************
     *--------------------------------------------------------------------------*/


    /*  Based on the user need configure I2C interface.
     *  It is example code to explain how to use the bno055 API*/
    I2C_routine();


    /*--------------------------------------------------------------------------*
     *  This API used to assign the value/reference of
     *  the following parameters
     *  I2C address
     *  Bus Write
     *  Bus read
     *  Chip id
     *  Page id
     *  Accel revision id
     *  Mag revision id
     *  Gyro revision id
     *  Boot loader revision id
     *  Software revision id
     *-------------------------------------------------------------------------*/
    comres = bno055_init(&bno055);

    /*  For initializing the BNO sensor it is required to the operation mode
     * of the sensor as NORMAL
     * Normal mode can set from the register
     * Page - page0
     * register - 0x3E
     * bit positions - 0 and 1*/

    /* set the power mode as NORMAL*/
    comres += bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);

    /*----------------------------------------------------------------*
     ************************* END INITIALIZATION *************************
     *-----------------------------------------------------------------*/
     return comres;
}

/*--------------------------------------------------------------------------*
 *  The following API is used to map the I2C bus read, write, delay and
 *  device address with global structure bno055_t
 *-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*
 *  By using bno055 the following structure parameter can be accessed
 *  Bus write function pointer: BNO055_WR_FUNC_PTR
 *  Bus read function pointer: BNO055_RD_FUNC_PTR
 *  Delay function pointer: delay_msec
 *  I2C address: dev_addr
 *--------------------------------------------------------------------------*/
s8 I2C_routine()
{

    bno055.bus_write = BNO055_I2C_bus_write;
    bno055.bus_read = BNO055_I2C_bus_read;
    bno055.delay_msec = BNO055_delay_msek;
    bno055.dev_addr = BNO055_I2C_ADDR2;

    return BNO055_INIT_VALUE;
}

/*-------------------------------------------------------------------*
 *
 *  This is a sample code for read and write the data by using I2C
 *  Use either I2C  based on your need
 *  The device address defined in the bno055.h file
 *
 *--------------------------------------------------------------------*/

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
    if(i2c_write_blocking(i2cx, dev_addr, &reg_addr, 1, true)){
        if(i2c_write_blocking(i2cx, dev_addr, reg_data, cnt, false) != cnt){
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
    if(i2c_write_blocking(i2cx, dev_addr, &reg_addr, 1, true)){
        if(i2c_read_blocking(i2cx, dev_addr, reg_data, cnt, false) != cnt){
            printf("BNO055_I2C_bus_read failed to read %d bytes from adrress %02x on I2C_BNO085\n", cnt, reg_addr);
            IERROR = -1;
        }
    } else {
        printf("BNO055_I2C_bus_read failed to write address %02x to I2C_BNO085\n", reg_addr);
        IERROR = -1;
    }
    return IERROR;
}

/*  Brief : The delay routine
 *  \param : delay in ms
 */
void BNO055_delay_msek(u32 msek)
{
    /*Here you can write your own delay routine*/
    sleep_ms(msek);
}

