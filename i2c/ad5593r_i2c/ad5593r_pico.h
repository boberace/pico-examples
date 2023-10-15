#ifndef AD5593R_PICO_H_
#define AD5593R_PICO_H_

#include "ad5593r.h"

int32_t ad5593r_init_pico(i2c_inst_t *i2c, uint8_t slave_address);
int32_t ad5593r_get_adc_value(uint8_t channel, uint16_t *value);
int32_t ad5593r_get_adc_values(uint16_t channels, uint16_t *values); 
int32_t ad5593r_set_dac_value(uint8_t channel, uint16_t value);
#endif /* AD5593R_PICO_H_ */