/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "lis2dw12_pico.h"
#include "lis2dw12_reg.h"

#define LED_PIN 12
#define POW_PIN 21

#define I2CA_INST i2c0
#define I2CA_BUADRATE 400*1000
#define PIN_I2CA_SDA 0
#define PIN_I2CA_SCL 1

#define LIS2DW12_I2C_ADD 0x18

#define UART_A_ID uart1
#define UART_A_BAUD_RATE 115200
#define UART_A_TX_PIN 8
#define UART_A_RX_PIN 9

#define BUFFER_SIZE 256
#define PRINT(...)                     \
    do {                               \
        char buffer[BUFFER_SIZE];      \
        snprintf(buffer, sizeof(buffer), __VA_ARGS__); \
        uart_puts(UART_A_ID, buffer);      \
    } while (0)


static int16_t data_raw_acceleration[3];
static float acceleration_mg[3];
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];
static lis2dw12_ctrl4_int1_pad_ctrl_t  ctrl4_int1_pad;
/* Self-test recommended samples */
#define SELF_TEST_SAMPLES  5
/* Self-test positive difference */
#define ST_MIN_POS      70.0f
#define ST_MAX_POS      1500.0f

uint setup_uart() {
    uint uart_ret = uart_init(UART_A_ID, UART_A_BAUD_RATE);
    gpio_set_function(UART_A_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_A_RX_PIN, GPIO_FUNC_UART);
    return uart_ret;
}

void setup_i2cA(void){

    i2c_init(I2CA_INST, I2CA_BUADRATE);
    gpio_set_function(PIN_I2CA_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2CA_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2CA_SDA);
    gpio_pull_up(PIN_I2CA_SCL);
    
}

void lis2dw12_read_data_polling(void);

int main() {
    stdio_init_all();
    sleep_ms(1000);
    setup_i2cA();
    setup_uart();
    platform_init(I2CA_INST, LIS2DW12_I2C_ADD, UART_A_ID);


    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    gpio_init(POW_PIN);
    gpio_set_dir(POW_PIN, GPIO_OUT);
    gpio_put(POW_PIN, 1);

    lis2dw12_read_data_polling();

    int counter = 0;

    while (true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(1);
        gpio_put(LED_PIN, 0);
        sleep_ms(200);
        gpio_put(LED_PIN, 1);
        sleep_ms(1);
        gpio_put(LED_PIN, 0);
        sleep_ms(798);

        PRINT("\033[A\33[2K\rlis2dw12_i2c, %i\n", counter);
        counter++;
    }

}

void lis2dw12_read_data_polling(void)
{
  /* Initialize mems driver interface */
  stmdev_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
//   dev_ctx.handle = &SENSOR_BUS;
//   /* Initialize platform specific hardware */
//   platform_init();
//   /* Wait sensor boot time */
//   platform_delay(BOOT_TIME);
  /* Check device ID */
  lis2dw12_device_id_get(&dev_ctx, &whoamI);
  PRINT("whoamI (%i) :, LIS2DW12_ID (%i)\r\n", whoamI, LIS2DW12_ID);

  if (whoamI != LIS2DW12_ID)
    while (1) {
      /* manage here device not found */
      }

  /* Restore default configuration */
  lis2dw12_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lis2dw12_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update */
  lis2dw12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set full scale */
  //lis2dw12_full_scale_set(&dev_ctx, LIS2DW12_8g);
  lis2dw12_full_scale_set(&dev_ctx, LIS2DW12_2g);
  /* Configure filtering chain
   * Accelerometer - filter path / bandwidth
   */
  lis2dw12_filter_path_set(&dev_ctx, LIS2DW12_LPF_ON_OUT);
  lis2dw12_filter_bandwidth_set(&dev_ctx, LIS2DW12_ODR_DIV_4);
  /* Configure power mode */
  lis2dw12_power_mode_set(&dev_ctx, LIS2DW12_HIGH_PERFORMANCE);
  //lis2dw12_power_mode_set(&dev_ctx, LIS2DW12_CONT_LOW_PWR_LOW_NOISE_12bit);
  /* Set Output Data Rate */
  lis2dw12_data_rate_set(&dev_ctx, LIS2DW12_XL_ODR_25Hz);

  /* Read samples in polling mode (no int) */
  while (1) {
    uint8_t reg;
    /* Read output only if new value is available */
    lis2dw12_flag_data_ready_get(&dev_ctx, &reg);

    if (reg) {
      /* Read acceleration data */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      lis2dw12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
      //acceleration_mg[0] = lis2dw12_from_fs8_lp1_to_mg(data_raw_acceleration[0]);
      //acceleration_mg[1] = lis2dw12_from_fs8_lp1_to_mg(data_raw_acceleration[1]);
      //acceleration_mg[2] = lis2dw12_from_fs8_lp1_to_mg(data_raw_acceleration[2]);
      acceleration_mg[0] = lis2dw12_from_fs2_to_mg(
                             data_raw_acceleration[0]);
      acceleration_mg[1] = lis2dw12_from_fs2_to_mg(
                             data_raw_acceleration[1]);
      acceleration_mg[2] = lis2dw12_from_fs2_to_mg(
                             data_raw_acceleration[2]);
      sprintf((char *)tx_buffer,
              "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }
  }
}