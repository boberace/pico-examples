#ifndef BNO085_I2C_H
#define BNO085_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

#ifdef __cplusplus
}
#endif

#include <stdio.h>
#include "pico/stdlib.h"
#include <string.h>

#include "hardware/i2c.h"

class bno085_i2c{

    public:
    bno085_i2c(uint pin_rst = 0xFF, uint pin_int = 0xFF);
    ~bno085_i2c();  

    bool connect_i2c(i2c_inst_t *i2c_instance, uint8_t i2c_address = 0x4A);
    bool enableReport(sh2_SensorId_t sensorId, float frequency);
    bool getSensorEvent(sh2_SensorValue_t *value);
    bool wasReset(void);

    sh2_ProductIds_t prodIds; ///< The product IDs returned by the sensor

    protected:

    sh2_Hal_t _sh2_hal; ///< The struct representing the SH2 Hardware Abstraction Layer

};

// -------------------------------------------------------------------------

static int i2c_open(sh2_Hal_t *self);
static void i2c_close(sh2_Hal_t *self);
static int i2c_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
static int i2c_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
static uint32_t getTimeUs(sh2_Hal_t *self);
static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
static void sensorHandler(void *cookie, sh2_SensorEvent_t *event);
static void hal_hardwareReset(void);

#endif