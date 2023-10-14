#ifndef BNO085_SPI_H
#define BNO085_SPI_H


#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "sh2_hal.h"
#include "sh2_util.h"
#include "shtp.h"


#include <stdio.h>
#include "pico/stdlib.h"
#include <string.h>

#include "hardware/spi.h"



extern sh2_ProductIds_t prodIds; ///< The product IDs returned by the sensor


// -------------------------------------------------------------------------

static inline void cs_select();
static inline void cs_deselect();
static inline void rst_select();
static inline void rst_deselect();
static int spihal_wait_for_int(void);
static int spi_open(sh2_Hal_t *self);
static void spi_close(sh2_Hal_t *self);
static int spi_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
static int spi_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
static uint32_t getTimeUs(sh2_Hal_t *self);
static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
static void sensorHandler(void *cookie, sh2_SensorEvent_t *event);
static void hal_hardwareReset(void);

#endif // BNO085_SPI_H