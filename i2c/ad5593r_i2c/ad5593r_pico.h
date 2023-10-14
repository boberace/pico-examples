#ifndef AD5593R_PICO_H_
#define AD5593R_PICO_H_

#include "ad5593r.h"

#define I2C0_BUADRATE 400*1000
#define PIN_I2C0_SDA 12
#define PIN_I2C0_SCL 13


void setup_i2c0(void);

int32_t ad5593r_init_pico();
int32_t ad5593r_get_adc_value(uint8_t channel, uint16_t *value);
int32_t ad5593r_get_adc_values(uint16_t channels, uint16_t *values); 
int32_t ad5593r_set_dac_value(uint8_t channel, uint16_t value);
#endif /* AD5593R_PICO_H_ */