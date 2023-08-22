#ifndef KXTJ3_I2C_H
#define KXTJ3_I2C_H



#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <string.h>
#include "kxtj3_1057_i2c.h"

typedef const uint8_t cuint8_t;

cuint8_t KXTJ3_XOUT_L = 0x06;
cuint8_t KXTJ3_XOUT_H = 0x07;
cuint8_t KXTJ3_YOUT_L = 0x08;
cuint8_t KXTJ3_YOUT_H = 0x09;
cuint8_t KXTJ3_ZOUT_L = 0x0A;
cuint8_t KXTJ3_ZOUT_H = 0x0B;
cuint8_t KXTJ3_DCST_RESP = 0x0C;

cuint8_t KXTJ3_WHO_AM_I = 0x0F;

cuint8_t KXTJ3_INT_SOURCE1 = 0x16;
cuint8_t KXTJ3_INT_SOURCE2 = 0x17;
cuint8_t KXTJ3_STATUS_REG = 0x18;

cuint8_t KXTJ3_INT_REL = 0x1A;
cuint8_t KXTJ3_CTRL_REG1 = 0x1B; // PC1 bit in CTRL_REG1 must first be set to “0”

cuint8_t KXTJ3_CTRL_REG2 = 0x1D; // PC1 bit in CTRL_REG1 must first be set to “0”
cuint8_t KXTJ3_INT_CTRL_REG1 = 0x1E; // PC1 bit in CTRL_REG1 must first be set to “0”
cuint8_t KXTJ3_INT_CTRL_REG2 = 0x1F; // PC1 bit in CTRL_REG1 must first be set to “0”
 
cuint8_t KXTJ3_DATA_CTRL_REG = 0x21; // PC1 bit in CTRL_REG1 must first be set to “0”

cuint8_t KXTJ3_WAKEUP_COUNTER = 0x29; // PC1 bit in CTRL_REG1 must first be set to “0”
cuint8_t KXTJ3_NA_COUNTER = 0x2A; // PC1 bit in CTRL_REG1 must first be set to “0”

cuint8_t KXTJ3_SELF_TEST = 0x3A; // PC1 bit in CTRL_REG1 must first be set to “0”

cuint8_t KXTJ3_WAKEUP_THRESHOLD_H = 0x6A; // PC1 bit in CTRL_REG1 must first be set to “0”
cuint8_t KXTJ3_WAKEUP_THRESHOLD_L = 0x6B; // PC1 bit in CTRL_REG1 must first be set to “0”

cuint8_t KXTJ3_WHO_AM_I_VAL = 0x35;
cuint8_t KXTJ3_SELF_TEST_VAL = 0xCA;

cuint8_t KXTJ3_DCST_RESP_NORMAL_VAL = 0x55;
cuint8_t KXTJ3_DCST_RESP_TEST_VAL = 0xAA;

cuint8_t KXTJ3_CTRL_REG1_PC1 = (1 << 7); 
cuint8_t KXTJ3_CTRL_REG1_RES = (1 << 6); 
cuint8_t KXTJ3_CTRL_REG1_DRDYE = (1 << 5); 
cuint8_t KXTJ3_CTRL_REG1_GSEL1 = (1 << 4); 
cuint8_t KXTJ3_CTRL_REG1_GSEL0 = (1 << 3); 
cuint8_t KXTJ3_CTRL_REG1_EN16G = (1 << 2); 
cuint8_t KXTJ3_CTRL_REG1_WUFE = (1 << 1); 

cuint8_t KXTJ3_CTRL_REG2_SRST = (1 << 7); 
cuint8_t KXTJ3_CTRL_REG2_DCST = (1 << 4); 
cuint8_t KXTJ3_CTRL_REG2_OWUFA = (1 << 2); 
cuint8_t KXTJ3_CTRL_REG2_OWUFB = (1 << 1); 
cuint8_t KXTJ3_CTRL_REG2_OWUFC = (1 << 0); 

cuint8_t KXTJ3_INT_CTRL_REG1_IEN = (1 << 5); 
cuint8_t KXTJ3_INT_CTRL_REG1_IEA = (1 << 4); 
cuint8_t KXTJ3_INT_CTRL_REG1_IEL = (1 << 3); 
cuint8_t KXTJ3_INT_CTRL_REG1_STPOL = (1 << 1); 

cuint8_t KXTJ3_INT_CTRL_REG2_ULMODE = (1 << 7);
cuint8_t KXTJ3_INT_CTRL_REG2_XNWUE = (1 << 5);
cuint8_t KXTJ3_INT_CTRL_REG2_XPWUE = (1 << 4);
cuint8_t KXTJ3_INT_CTRL_REG2_YNWUE = (1 << 3);
cuint8_t KXTJ3_INT_CTRL_REG2_YPWUE = (1 << 2);
cuint8_t KXTJ3_INT_CTRL_REG2_ZNWUE = (1 << 1);
cuint8_t KXTJ3_INT_CTRL_REG2_ZPWUE = (1 << 0);

cuint8_t KXTJ3_DATA_CTRL_REG_OSAA = (1 << 3);
cuint8_t KXTJ3_DATA_CTRL_REG_OSAB = (1 << 2);
cuint8_t KXTJ3_DATA_CTRL_REG_OSAC = (1 << 1);
cuint8_t KXTJ3_DATA_CTRL_REG_OSAD = (1 << 0);


// input values
typedef enum {
    ACCEL_2G    = 0,
    ACCEL_4G    = KXTJ3_CTRL_REG1_GSEL0,
    ACCEL_8G    = KXTJ3_CTRL_REG1_GSEL1,
    ACCEL_8G_14B   = KXTJ3_CTRL_REG1_GSEL1 | KXTJ3_CTRL_REG1_GSEL0,
    ACCEL_16G1  = KXTJ3_CTRL_REG1_EN16G,
    ACCEL_16G2  = KXTJ3_CTRL_REG1_EN16G | KXTJ3_CTRL_REG1_GSEL0,
    ACCEL_16G3  = KXTJ3_CTRL_REG1_EN16G | KXTJ3_CTRL_REG1_GSEL1,
    ACCEL_16G_14B  = KXTJ3_CTRL_REG1_EN16G | KXTJ3_CTRL_REG1_GSEL1 | KXTJ3_CTRL_REG1_GSEL0,
} KXTJ3_accel_range_t;

typedef enum {
    DATA_RATE_1  = KXTJ3_DATA_CTRL_REG_OSAA ,    // 0.781Hz
    DATA_RATE_2  = KXTJ3_DATA_CTRL_REG_OSAA | KXTJ3_DATA_CTRL_REG_OSAD,    // 1.563Hz
    DATA_RATE_3  = KXTJ3_DATA_CTRL_REG_OSAA | KXTJ3_DATA_CTRL_REG_OSAC,    // 3.125Hz
    DATA_RATE_4  = KXTJ3_DATA_CTRL_REG_OSAA | KXTJ3_DATA_CTRL_REG_OSAC | KXTJ3_DATA_CTRL_REG_OSAD,    // 6.25Hz
    DATA_RATE_5  = 0,    // 12.5Hz
    DATA_RATE_6  = KXTJ3_DATA_CTRL_REG_OSAD,    // 25Hz
    DATA_RATE_7  = KXTJ3_DATA_CTRL_REG_OSAC,    // 50Hz
    DATA_RATE_8  = KXTJ3_DATA_CTRL_REG_OSAC | KXTJ3_DATA_CTRL_REG_OSAD,    // 100Hz
    DATA_RATE_9  = KXTJ3_DATA_CTRL_REG_OSAB,    // 200Hz
    DATA_RATE_10 = KXTJ3_DATA_CTRL_REG_OSAB | KXTJ3_DATA_CTRL_REG_OSAD,   // 400Hz
    DATA_RATE_11 = KXTJ3_DATA_CTRL_REG_OSAB | KXTJ3_DATA_CTRL_REG_OSAC,   // 800Hz
    DATA_RATE_12 = KXTJ3_DATA_CTRL_REG_OSAB | KXTJ3_DATA_CTRL_REG_OSAC | KXTJ3_DATA_CTRL_REG_OSAD,   // 1600Hz
} KXTJ3_data_rate_t;

typedef enum {
    WAKE_DATA_RATE_1  = 0 ,    // 0.781Hz
    WAKE_DATA_RATE_2  = KXTJ3_CTRL_REG2_OWUFC,    // 1.563Hz
    WAKE_DATA_RATE_3  = KXTJ3_CTRL_REG2_OWUFB,    // 3.125Hz
    WAKE_DATA_RATE_4  = KXTJ3_CTRL_REG2_OWUFC | KXTJ3_CTRL_REG2_OWUFB ,    // 6.25Hz
    WAKE_DATA_RATE_5  = KXTJ3_CTRL_REG2_OWUFA,    // 12.5Hz
    WAKE_DATA_RATE_6  = KXTJ3_CTRL_REG2_OWUFA | KXTJ3_CTRL_REG2_OWUFC,    // 25Hz
    WAKE_DATA_RATE_7  = KXTJ3_CTRL_REG2_OWUFA | KXTJ3_CTRL_REG2_OWUFB,    // 50Hz
    WAKE_DATA_RATE_8  = KXTJ3_CTRL_REG2_OWUFA | KXTJ3_CTRL_REG2_OWUFB | KXTJ3_CTRL_REG2_OWUFC,    // 100Hz
} KXTJ3_wake_data_rate_t;

struct axes{
  uint8_t x_l = 0;
  uint8_t x_h = 0;
  uint8_t y_l = 0;
  uint8_t y_h = 0;
  uint8_t z_l = 0;
  uint8_t z_h = 0;
  double x = 0;
  double y = 0;
  double z = 0;
};

class kxtj3_i2c{

    public:
    kxtj3_i2c(  i2c_inst_t *i2c_instance, 
                uint8_t i2c_address = 0x0E, 
                uint pin_int = 0xFF, 
                uint pin_rst = 0xFF);
    ~kxtj3_i2c();

    int begin( KXTJ3_data_rate_t data_rate = DATA_RATE_7,
                KXTJ3_accel_range_t accel_range = ACCEL_8G,
                bool high_res_mode = false);

    int get_accel_data(axes *axes);   

    // todo

    int set_wake_data_rate(KXTJ3_wake_data_rate_t wake_data_rate);

    // CTRL_REG1
    int set_operating_mode(bool operating_mode);
    int set_resolution(bool high_res_mode);
    int set_interrupt(bool interrupt);
    int set_accel_range(KXTJ3_accel_range_t accel_range);
    int set_wake_up(bool wake_up);

    // CTRL_REG2
    int set_software_reset(bool software_reset);
    int set_self_test(bool self_test);
    int set_wake_up_data_rate(KXTJ3_wake_data_rate_t wake_data_rate);

    // INT_CTRL_REG1
    int enable_interrupt_pin(bool enable_interrupt_pin);
    int set_interrupt_polarity(bool interrupt_polarity);
    int set_interrupt_latch(bool interrupt_latch);

    // INT_CTRL_REG2
    int set_unlatched_mode(bool unlatched_mode);
    int set_wakeup_modes(uint8_t wakeup_modes);

    // DATA_CTRL_REG
    int set_data_rate(KXTJ3_data_rate_t data_rate);

    // WAKEUP_COUNTER
    int set_wakeup_counter(uint8_t wakeup_counter);

    // NA_COUNTER
    int set_na_counter(uint8_t na_counter);

    // SELF_TEST
    int set_self_test_enable(bool self_test_enable);

    // WAKEUP_THRESHOLD
    int set_wakeup_threshold(uint16_t wakeup_threshold);




    private:

    i2c_inst_t *_I2C_INST_KXTJ3;
    uint8_t _KXTJ3_I2C_ADDR, _PIN_KXTJ3_RST, _PIN_KXTJ3_INT, _high_res_mode, _wake_up_mode;
    KXTJ3_data_rate_t _data_rate;
    KXTJ3_accel_range_t _accel_range;


    int _read_regs(uint8_t addr, uint8_t *regs, uint8_t qty );
    int _write_regs(uint8_t addr, uint8_t regs, uint8_t qty );
    int _write_regs_with_clear(uint8_t addr, uint8_t regs, uint8_t qty );
    int _write_masked_reg(uint8_t addr, uint8_t mask, bool set);

    void _debug_print_write_regs();
};


#endif //KXTJ3_I2C_H