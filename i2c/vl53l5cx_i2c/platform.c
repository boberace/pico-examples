/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
// 

#include "platform.h"
#include <stdio.h>

// uncomment to print debug information over usb serial
// #define DEBUG

#ifdef DEBUG
# define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
# define DEBUG_PRINT(...)
#endif

uint8_t RdByte(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_value)
{

	/* Need to be implemented by customer. This function returns 0 if OK */
	uint8_t status = 255;
	uint8_t data_write[2];
	uint8_t data_read[1];
	int ret;

	data_write[0] = (RegisterAdress >> 8) & 0xFF;
	data_write[1] = RegisterAdress & 0xFF;
	ret = i2c_write_blocking(p_platform->i2c_instance, 
								(p_platform->address >> 1) & 0x7F, 
								data_write, 
								2, 
								true);
	DEBUG_PRINT("RdByte i2c_write_blocking, reg %04X, size %d, ret = %d\n",RegisterAdress, 2,ret);
	if (ret != 2) return status;
	ret = i2c_read_blocking( p_platform->i2c_instance, 
								(p_platform->address >> 1) & 0x7F, 
								data_read, 
								1, 
								false);
	if (ret != 1) return status;
	DEBUG_PRINT("RdByte i2c_read_blocking size %d, ret = %d\n", 1,ret);
	*p_value = data_read[0];

	return 0;
}

uint8_t RdMulti(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{

	/* Need to be implemented by customer. This function returns 0 if OK */
	uint8_t status = 255;
	uint8_t data_write[2];
	int ret;

	data_write[0] = (RegisterAdress >> 8) & 0xFF;
	data_write[1] = RegisterAdress & 0xFF;
	ret = i2c_write_blocking(p_platform->i2c_instance, 
								(p_platform->address >> 1) & 0x7F, 
								data_write, 
								2, 
								true);
	DEBUG_PRINT("RdMulti i2c_write_blocking, reg %04X, size %d, ret = %d\n",RegisterAdress, 2,ret);						
	if (ret != 2) return status;
	ret = i2c_read_blocking( p_platform->i2c_instance, 
								(p_platform->address >> 1) & 0x7F, 
								p_values, 
								size, 
								false);
	DEBUG_PRINT("RdMulti i2c_read_blocking size %d, ret = %d\n", size,ret);							
	if (ret != size) return status;

	return 0;
}


uint8_t WrByte(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value)
{
	/* Need to be implemented by customer. This function returns 0 if OK */
	uint8_t status = 255;
	uint8_t data_write[3];
	int ret;

	data_write[0] = (RegisterAdress >> 8) & 0xFF;
	data_write[1] = RegisterAdress & 0xFF;
	data_write[2] = value & 0xFF;
	ret = i2c_write_blocking(p_platform->i2c_instance, 
								(p_platform->address >> 1) & 0x7F, 
								data_write, 
								3, 
								false);
	DEBUG_PRINT("WrByte i2c_write_blocking, reg %04X, size %d, ret = %d\n",RegisterAdress, 3,ret);						
	if (ret != 3) return status;

	return 0;
}

uint8_t WrMulti(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	
		/* Need to be implemented by customer. This function returns 0 if OK */
	uint8_t status = 255;
	int ret;

	// Create a buffer to hold the register address and the data
	uint8_t buffer[size + 2];  // +2 for the 16-bit register address

	// Copy the register address to the buffer (big endian format)
	buffer[0] = (RegisterAdress >> 8) & 0xFF;
	buffer[1] = RegisterAdress & 0xFF;

	// Copy the data bytes to the buffer
	memcpy(&buffer[2], p_values, size);

	// Perform the I2C write operation
	ret = i2c_write_blocking(p_platform->i2c_instance, 
								(p_platform->address >> 1) & 0x7F, 
								buffer, 
								size + 2,  // +2 for the register address
								true);
	DEBUG_PRINT("WrMulti i2c_write_blocking, reg %04X, size %d, ret = %d\n", RegisterAdress, size, ret);							

	if (ret != (size + 2)) return status;  // +2 for the register address

	return 0;
}


uint8_t Reset_Sensor(
		VL53L5CX_Platform *p_platform)
{
	uint8_t status = 0;
	
	/* (Optional) Need to be implemented by customer. This function returns 0 if OK */
	
	/* Set pin LPN to LOW */
	/* Set pin AVDD to LOW */
	/* Set pin VDDIO  to LOW */
	WaitMs(p_platform, 100);

	/* Set pin LPN of to HIGH */
	/* Set pin AVDD of to HIGH */
	/* Set pin VDDIO of  to HIGH */
	WaitMs(p_platform, 100);

	return status;
}

void SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 	 	 size)
{
	uint32_t i, tmp;
	
	/* Example of possible implementation using <string.h> */
	for(i = 0; i < size; i = i + 4) 
	{
		tmp = (
		  buffer[i]<<24)
		|(buffer[i+1]<<16)
		|(buffer[i+2]<<8)
		|(buffer[i+3]);
		
		memcpy(&(buffer[i]), &tmp, 4);
	}
}	

uint8_t WaitMs(
		VL53L5CX_Platform *p_platform,
		uint32_t TimeMs)
{
	uint8_t status = 0;

	/* Need to be implemented by customer. This function returns 0 if OK */
	sleep_ms(TimeMs);
	
	return status;
}
