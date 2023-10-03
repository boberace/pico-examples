#include <stdint.h>
#include <stdlib.h>
#include "ad5593r_pico_i2c.h"
#include <string.h>
#include <stdio.h>
#include "hardware/i2c.h"

/**
 * Write data to a slave device.
 *
 * @param [in] desc         - The I2C descriptor.
 * @param [in] data         - Buffer that stores the transmission data.
 * @param [in] bytes_number - Number of bytes to write.
 * @param [in] stop_bit     - Stop condition control.
 *                          Example: 0 - A stop condition will not be generated;
 *                                   1 - A stop condition will be generated.
 *
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t i2c_write(i2c_desc *desc, uint8_t *data, size_t bytes_number,
		  bool stop_bit)
{

	if (!desc || !data)
		return -FAILURE;

	return i2c_write_blocking(desc->i2c_instance, desc->slave_address, data,
				 bytes_number, !stop_bit);
				 
}

/**
 * Read data from a slave device.
 *
 * @param [in] desc         - The I2C descriptor.
 * @param [out] data        - Buffer that will store the received data.
 * @param [in] bytes_number - Number of bytes to read.
 * @param [in] stop_bit     - Stop condition control.
 *                          Example: 0 - A stop condition will not be generated;
 *                                   1 - A stop condition will be generated.
 *
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t i2c_read(i2c_desc *desc, uint8_t *data, size_t bytes_number,
		 bool stop_bit)
{
	if (!desc || !data)
		return -FAILURE;

	return i2c_read_blocking(desc->i2c_instance, desc->slave_address, data,
				bytes_number, !stop_bit);
}

/**
 * Generate miliseconds delay.
 *
 * @param [in] msecs - Delay in miliseconds.
 *
 * @return None.
 */
void mdelay(uint32_t msecs)
{
	sleep_ms(msecs);
}
