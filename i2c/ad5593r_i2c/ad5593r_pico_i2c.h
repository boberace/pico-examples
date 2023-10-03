#ifndef AD5593R_PICO_I2C_H_
#define AD5593R_PICO_I2C_H_


#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define FAILURE -1

typedef struct {
	i2c_inst_t *i2c_instance;
	uint32_t	  max_speed_hz;
	uint8_t		  slave_address;
} i2c_desc;

/* Write data to a slave device. */
int32_t i2c_write(i2c_desc *desc,
		  uint8_t *data,
		  size_t bytes_number,
		  bool stop_bit);

/* Read data from a slave device. */
int32_t i2c_read(i2c_desc *desc,
		 uint8_t *data,
		 size_t bytes_number,
		 bool stop_bit);

/* Generate miliseconds delay. */
void mdelay(uint32_t msecs);


#endif /* AD5593R_PICO_I2C_H_ */
