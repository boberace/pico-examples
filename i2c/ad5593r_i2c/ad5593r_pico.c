// modeled from https://github.com/analogdevicesinc/precision-converters-firmware/tree/main/projects/ad559xr_console
// https://github.com/analogdevicesinc/no-OS/tree/master

#include "ad5593r_pico.h"
#include "ad5592r-base.h"
#include "ad5593r.h"

#include "pico_i2c.h"

#include "no_os_error.h"
#include "no_os_i2c.h"
#include "no_os_delay.h"
#include "no_os_util.h"

#include "hardware/i2c.h"

#include <string.h>
#include <stdio.h>

// uncomment to print debug information over usb serial
#define DEBUG

#ifdef DEBUG
# define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
# define DEBUG_PRINT(...)
#endif

// extern struct ad5592r_init_param ad5592r_user_param;
// extern struct no_os_i2c_init_param i2c_user_params;
// extern struct no_os_i2c_desc i2c_user_descr;
// static struct no_os_i2c_desc *pico_i2c_desc;

#define EXTERNAL_VREF_VOLTAGE			3.3
float vref_voltage = EXTERNAL_VREF_VOLTAGE;

#define	AD5592R_CHANNEL(N)			    (N)
#define AD5592R_REG_ADC_SEQ_INCL(x)		NO_OS_BIT(x)
#define AD5592R_REG_PD_CHANNEL(x)		NO_OS_BIT(x)
#define AD5592R_GPIO(x)				    NO_OS_BIT(x)

#define TEMP_SAMPLE_SIZE			    5
#define CLEAR_CHANNEL_SELECTION			1000
#define MDELAY_TO_DISPLAY_INSTRUCTION   1000
#define TEMPERATURE_READBACK_CHANNEL	8

#define MAX_ADC_CODE				    4095.0
#define ADC_GAIN_LOW_CONVERSION_VALUE	2.654
#define ADC_GAIN_HIGH_CONVERSION_VALUE	1.327


static struct ad5592r_dev sAd5592r_dev;

#define NUM_CHANNELS 8

#define AD5593R_A0_STATE 0

#define AD5593R_I2C (0x10 | (AD5593R_A0_STATE & 0x01))   

#define I2C_SCL		    13	 // todo move i2c to main
#define I2C_SDA		    12	

struct pico_i2c_init_param i2c_init_extra_params = {
	.i2c_sda_pin = I2C_SDA,
	.i2c_scl_pin = I2C_SCL
};

struct no_os_i2c_init_param i2c_user_params = {
	.max_speed_hz = 100000,
	.slave_address = AD5593R_I2C,
	.platform_ops = &pico_i2c_ops, // pico_i2c
	.extra = &i2c_init_extra_params
};

struct ad5592r_init_param ad5592r_user_param = {
	.int_ref = false
};

const struct ad5592r_dev ad5592r_dev_user = {
	.ops = NULL,
	.spi = NULL,
	.i2c = NULL,
	.spi_msg = 0,
	.num_channels = NUM_CHANNELS,
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
		CH_OFFSTATE_PULLDOWN,
		CH_OFFSTATE_PULLDOWN,
		CH_OFFSTATE_PULLDOWN,
		CH_OFFSTATE_PULLDOWN,
		CH_OFFSTATE_PULLDOWN,
		CH_OFFSTATE_PULLDOWN,
		CH_OFFSTATE_PULLDOWN,
		CH_OFFSTATE_PULLDOWN
	},
	.gpio_out = 0,
	.gpio_in = 0,
	.gpio_val = 0,
	.ldac_mode = 0,
};

const struct ad5592r_dev ad5592r_dev_reset = {
	.ops = NULL,
	.spi = NULL,
	.i2c = NULL,
	.spi_msg = 0,
	.num_channels = NUM_CHANNELS,
	.cached_dac = { 0, 0, 0, 0, 0, 0, 0 },
	.cached_gp_ctrl = 0,
	.channel_modes = {
		CH_MODE_UNUSED,
		CH_MODE_UNUSED,
		CH_MODE_UNUSED,
		CH_MODE_UNUSED,
		CH_MODE_UNUSED,
		CH_MODE_UNUSED,
		CH_MODE_UNUSED,
		CH_MODE_UNUSED
	},
	.channel_offstate = {
		CH_OFFSTATE_PULLDOWN,
		CH_OFFSTATE_PULLDOWN,
		CH_OFFSTATE_PULLDOWN,
		CH_OFFSTATE_PULLDOWN,
		CH_OFFSTATE_PULLDOWN,
		CH_OFFSTATE_PULLDOWN,
		CH_OFFSTATE_PULLDOWN,
		CH_OFFSTATE_PULLDOWN
	},
	.gpio_out = 0,
	.gpio_in = 0,
	.gpio_val = 0,
	.ldac_mode = 0,
};



void no_os_udelay(uint32_t usecs){
    sleep_us(usecs);
}

void no_os_mdelay(uint32_t msecs){
    sleep_ms(msecs);
};

uint32_t pico_ad5593r_init(){
	int32_t status;
	memcpy(&sAd5592r_dev, &ad5592r_dev_user, sizeof(ad5592r_dev_user));

	status = no_os_i2c_init(&sAd5592r_dev.i2c, &i2c_user_params);
	if (status != 0) {
		DEBUG_PRINT("DEBUG pico_i2c_init failed %d\n", status);
		return (status);
	} else {
		DEBUG_PRINT("DEBUG pico_i2c_init success %d\n", status);
	}

    status = ad5593r_init(&sAd5592r_dev, &ad5592r_user_param);		
	if (status != 0) {
		DEBUG_PRINT("DEBUG ad5593r_init failed %d\n", status);
		return (status);
	} else {
		DEBUG_PRINT("DEBUG ad5593r_init success %d\n", status);
	}
	return 0;
};