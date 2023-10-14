#include "ad5593r_pico.h"

// uncomment to print debug information over usb serial
// #define DEBUG

// define the dubug print function
#ifdef DEBUG
# define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
# define DEBUG_PRINT(...)
#endif

const struct ad5592r_rw_ops ad5593r_rw_ops = {
	.write_dac = ad5593r_write_dac,
	.read_adc = ad5593r_read_adc,
	.multi_read_adc= ad5593r_multi_read_adc,
	.reg_write = ad5593r_reg_write,
	.reg_read = ad5593r_reg_read,
	.gpio_read = ad5593r_gpio_read,
};


i2c_desc  i2c_desc_user = {
    .i2c_instance = i2c0,
    .slave_address = 0x10
};

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

void setup_i2c0(void){

    i2c_init(i2c0, I2C0_BUADRATE);
    gpio_set_function(PIN_I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C0_SDA);
    gpio_pull_up(PIN_I2C0_SCL);
    
}

int32_t ad5593r_init_pico(void){

        setup_i2c0();
        int32_t ret = ad5593r_init(&ad5592r_dev_user, &ad5592r_user_param); 
		if(ret != 0){
			DEBUG_PRINT("Error ad5593r_init_pico %i\n", ret);
		}
        return ret;
}

int32_t ad5593r_get_adc_value(uint8_t channel, uint16_t *value){

	int32_t ret = ad5593r_read_adc(&ad5592r_dev_user, channel, value);
	if(ret != 0){
		DEBUG_PRINT("Error ad5593r_get_adc_value\n", ret);
	}
	return ret;
}

int32_t ad5593r_get_adc_values(uint16_t channels, uint16_t *values){
	int32_t ret = ad5593r_multi_read_adc(&ad5592r_dev_user, channels, values);
	if(ret != 0){
		DEBUG_PRINT("Error ad5593r_get_adc_values\n", ret);
	}
	return ret;
}

int32_t ad5593r_set_dac_value(uint8_t channel, uint16_t value){
	int32_t ret = ad5593r_write_dac(&ad5592r_dev_user, channel, value);
		if(ret != 0){
			DEBUG_PRINT("Error ad5593r_set_dac_value\n", ret);
		}
		return ret;
}

