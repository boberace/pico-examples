#include "lis2dw12_pico.h"


i2c_inst_t* PICO_I2C_INST;
uart_inst_t *PICO_UART;
uint8_t I2C_ADDRESS;

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
    int ret;
    uint8_t data[len+1];
    data[0] = reg;
    memcpy(&data[1], bufp, len);
    ret = i2c_write_blocking(PICO_I2C_INST, I2C_ADDRESS, data, len+1, false);
    if (ret != len+1) {
        return -3;
    }

  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
    int ret;
    ret = i2c_write_blocking(PICO_I2C_INST, I2C_ADDRESS, &reg, 1, true);
    if (ret != 1) {
        return -1;
    }
    ret = i2c_read_blocking(PICO_I2C_INST, I2C_ADDRESS, bufp, len, false);
    if (ret != len) {
        return -2;
    }

  return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
void tx_com(uint8_t *tx_buffer, uint16_t len)
{
    uart_write_blocking(PICO_UART, tx_buffer, len);
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
void platform_delay(uint32_t ms)
{
    sleep_ms(ms);
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
void platform_init(i2c_inst_t* pico_i2c_inst, uint8_t i2c_address, uart_inst_t *pico_uart)
{
    PICO_I2C_INST = pico_i2c_inst;
    PICO_UART = pico_uart;
    I2C_ADDRESS = i2c_address;
    platform_delay(1000);
}