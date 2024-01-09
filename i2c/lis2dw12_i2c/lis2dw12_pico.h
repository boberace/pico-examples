#ifndef LIS2DW12_PICO_H
#define LIS2DW12_PICO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
void tx_com( uint8_t *tx_buffer, uint16_t len );
void platform_delay(uint32_t ms);
void platform_init(i2c_inst_t* pico_i2c_inst, uint8_t i2c_address, uart_inst_t *pico_uart);

#ifdef __cplusplus
}
#endif

#endif /*LIS2DW12_PICO_H */