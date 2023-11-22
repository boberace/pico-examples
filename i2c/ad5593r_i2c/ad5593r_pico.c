#include "ad5593r_pico.h"


// #define DEBUG // uncomment to print debug information over usb serial
#ifdef DEBUG
# define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
# define DEBUG_PRINT(...)
#endif

#define AD5593R_MODE_CONF		(0 << 4)
#define AD5593R_MODE_DAC_WRITE		(1 << 4)
#define AD5593R_MODE_ADC_READBACK	(4 << 4)
#define AD5593R_MODE_DAC_READBACK	(5 << 4)
#define AD5593R_MODE_GPIO_READBACK	(6 << 4)
#define AD5593R_MODE_REG_READBACK	(7 << 4)

#define STOP_BIT	1
#define RESTART_BIT	0
#define AD5593R_ADC_VALUES_BUFF_SIZE	    18

const struct ad5592r_rw_ops ad5593r_rw_ops = {
	.write_dac = ad5593r_write_dac,
	.read_adc = ad5593r_read_adc,
	.multi_read_adc= ad5593r_multi_read_adc,
	.reg_write = ad5593r_reg_write,
	.reg_read = ad5593r_reg_read,
	.gpio_read = ad5593r_gpio_read,
};


i2c_desc  i2c_desc_user;

struct ad5592r_dev ad5592r_dev_user = {
	.ops = &ad5593r_rw_ops,
	.i2c = &i2c_desc_user,
	.num_channels = 8,
	.cached_dac = { 0, 0, 0, 0, 0, 0, 0 },
	.cached_gp_ctrl = 0,
	.channel_modes = {
		CH_MODE_ADC,
		CH_MODE_ADC,
		CH_MODE_ADC,
		CH_MODE_ADC,
		CH_MODE_ADC,
		CH_MODE_ADC,
		CH_MODE_DAC,
		CH_MODE_DAC
	},
	.channel_offstate = {
		CH_OFFSTATE_OUT_TRISTATE,
		CH_OFFSTATE_OUT_TRISTATE,
		CH_OFFSTATE_OUT_TRISTATE,
		CH_OFFSTATE_OUT_TRISTATE,
		CH_OFFSTATE_OUT_TRISTATE,
		CH_OFFSTATE_OUT_TRISTATE,
		CH_OFFSTATE_OUT_TRISTATE,
		CH_OFFSTATE_OUT_TRISTATE
	},
	.gpio_out = 0,
	.gpio_in = 0,
	.gpio_val = 0
};

struct ad5592r_init_param ad5592r_user_param = {
	.int_ref = false
};

/**
 * initialize the ad5593r on pico
 *
 * @param [in] i2c         -  The pico I2C instance.
 * @param [in] slave_address        - ad5593r I2C address.
 *
 * @return 0 in case of success, negative error code otherwise.
 */

int32_t ad5593r_init_pico(i2c_inst_t *i2c, uint8_t slave_address){

		i2c_desc_user.i2c_instance = i2c;
		i2c_desc_user.slave_address = slave_address;

        int32_t ret = ad5593r_init(&ad5592r_dev_user, &ad5592r_user_param); 
		if(ret != 0){
			DEBUG_PRINT("Error ad5593r_init_pico %i\n", ret);
		}

		// //Data written to an input register is not copied to a DAC register. The DAC output is not updated.
		// ret = ad5593r_reg_write(&ad5592r_dev_user, AD5592R_REG_LDAC, AD5592R_REG_LDAC_INPUT_REG_ONLY);
		// if(ret != 0){
		// 	DEBUG_PRINT("Error ad5593r_reg_write AD5592R_REG_LDAC AD5592R_REG_LDAC_INPUT_REG_ONLY \n", ret);
		// 	return -1;
		// }


        return 0;
}

/**
 * Read ADC channel.
 *
 * @param channel - The channel number.
 * @param value - ADC value
 * 
 * @return 0 in case of success, negative error code otherwise
 */

int32_t ad5593r_get_adc_value(uint8_t channel, uint16_t *value){

	int32_t ret = ad5593r_read_adc(&ad5592r_dev_user, channel, value);
	if(ret != 0){
		DEBUG_PRINT("Error ad5593r_get_adc_value\n", ret);
		return -1;
	}
	return 0;
}

/**
 * Read Multiple ADC Channels.
 *
 * @param channels - The ADC channels to be readback
 * @param values - ADC value array
 * 
 * @return 0 in case of success, negative error code otherwise
 */

int32_t ad5593r_get_adc_values(uint16_t channels, uint16_t *values){
	int32_t ret = ad5593r_multi_read_adc(&ad5592r_dev_user, channels, values);
	if(ret != 0){
		DEBUG_PRINT("Error ad5593r_get_adc_values\n", ret);
		return -1;
	}
	return 0;
}

/**
 * Write DAC channel.
 *
 * @param channel - The channel number.
 * @param value - DAC value
 * 
 * @return 0 in case of success, negative error code otherwise
 */

int32_t ad5593r_set_dac_value(uint8_t channel, uint16_t value){
	int32_t ret = ad5593r_write_dac(&ad5592r_dev_user, channel, value);
		if(ret != 0){
			DEBUG_PRINT("Error ad5593r_set_dac_value\n", ret);
			return -1;
		}
		return 0;
}

/**
 * Write DAC channels.
 *
 * @param channels - The DAC channels to be written
 * @param values - DAC values array
 * 
 * @return 0 in case of success, negative error code otherwise
 */

int32_t ad5593r_set_dac_values(uint16_t channels, uint16_t *values){

	int32_t ret;

	//Data written to an input register is not copied to a DAC register. The DAC output is not updated.
	ret = ad5593r_reg_write(&ad5592r_dev_user, AD5592R_REG_LDAC, AD5592R_REG_LDAC_INPUT_REG_ONLY);
	if(ret != 0){
		DEBUG_PRINT("Error ad5593r_reg_write AD5592R_REG_LDAC AD5592R_REG_LDAC_INPUT_REG_ONLY \n", ret);
		return -1;
	}

	for(uint i = 0; i < 16; i++){
		// values are written to an input register
		if(((channels >> i) & 0x1)){
			ret = ad5593r_write_dac(&ad5592r_dev_user, i, values[i]);
			if(ret != 0){
				DEBUG_PRINT("Error ad5593r_set_dac_values ad5593r_write_dac values\n", ret);
			}
		}
	}

	// Data in the input registers is copied to the corresponding DAC registers. When the data has
	// been transferred, the DAC outputs are updated simultaneously.
	ret = ad5593r_reg_write(&ad5592r_dev_user, AD5592R_REG_LDAC, AD5592R_REG_LDAC_INPUT_REG_OUT);
	if(ret != 0){
		DEBUG_PRINT("Error ad5593r_reg_write AD5592R_REG_LDAC AD5592R_REG_LDAC_INPUT_REG_OUT \n", ret);
		return -1;
	}

	return 0;
}


   


// int32_t ad5593r_multi_read_adc(struct ad5592r_dev *dev, uint16_t chans,
// 			       uint16_t *values)
// {
// 	uint8_t data[AD5593R_ADC_VALUES_BUFF_SIZE], i;
// 	int32_t ret;
// 	uint8_t samples;

// 	if (!dev)
// 		return -1;

// 	samples = hweight16(chans);

// 	data[0] = AD5593R_MODE_CONF | AD5592R_REG_ADC_SEQ;
// 	data[1] = chans >> 8;
// 	data[2] = chans & 0xFF;

// 	ret = i2c_write(dev->i2c, data, 3, STOP_BIT);
// 	if (ret < 0)
// 		return ret;

// 	data[0] = AD5593R_MODE_ADC_READBACK;
// 	ret = i2c_write(dev->i2c, data, 1, RESTART_BIT);
// 	if (ret < 0)
// 		return ret;

// 	ret = i2c_read(dev->i2c, data, (2 * samples), STOP_BIT);
// 	if (ret < 0)
// 		return ret;

// 	for (i = 0; i < samples; i++) {
// 		values[i] = ((uint16_t)(((data[2 * i] & 0xFF) << 8) + data[(2 * i) + 1]));
// 	}

// 	return 0;
// }