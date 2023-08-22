/*

Documents referenced in comments:
DS: KXTJ3-1057 Specifications

todo:  software reset (only good if can power cycle the device)
*/

#include "kxtj3_1057_i2c.h"

// uncomment to print debug information over usb serial
#define DEBUG

// define the dubug print function
#ifdef DEBUG
# define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
# define DEBUG_PRINT(...)
#endif

kxtj3_i2c::kxtj3_i2c(i2c_inst_t *i2c_instance, 
                    uint8_t i2c_address, 
                    uint pin_int , 
                    uint pin_rst ){

    _I2C_INST_KXTJ3 = i2c_instance;
    _KXTJ3_I2C_ADDR = i2c_address;
    _PIN_KXTJ3_RST = pin_rst;
    _PIN_KXTJ3_INT = pin_int;

    if(_PIN_KXTJ3_RST != 0xFF){
        gpio_init(_PIN_KXTJ3_RST);
        gpio_set_dir(_PIN_KXTJ3_RST, GPIO_OUT);
    }
    
    if(_PIN_KXTJ3_INT != 0xFF){   
        gpio_init(_PIN_KXTJ3_INT);
        gpio_set_dir(_PIN_KXTJ3_INT, GPIO_IN);
    }
}

int kxtj3_i2c::begin(   KXTJ3_data_rate_t data_rate,
                        KXTJ3_accel_range_t accel_range,
                        bool high_res_mode){

    _data_rate = data_rate;
    _accel_range = accel_range;
    _high_res_mode = ((uint8_t)high_res_mode) << 6;

    if (data_rate > DATA_RATE_9) 
        _high_res_mode = true;
 
    uint8_t reg = 0;
    int ret = PICO_OK;
    DEBUG_PRINT("\033[2J");// clear screen
    DEBUG_PRINT("DEBUG_PRINT +++ start begin \n\n");

    // who am I check
    DEBUG_PRINT("\nDEBUG_PRINT *** who am I test\n");
    ret = _read_regs(KXTJ3_WHO_AM_I, &reg, 1);
    if(ret != PICO_OK) return ret;
    DEBUG_PRINT("DEBUG_PRINT KXTJ3_WHO_AM_I _read_regs success\n");
    if(reg != KXTJ3_WHO_AM_I_VAL) return reg;
    DEBUG_PRINT("DEBUG_PRINT KXTJ3_WHO_AM_I_VAL success\n");

    // reset
    DEBUG_PRINT("\nDEBUG_PRINT *** reset\n");
    ret = _write_masked_reg(KXTJ3_CTRL_REG2, KXTJ3_CTRL_REG2_SRST, true);
    if(ret != PICO_OK) return PICO_ERROR_GENERIC; DEBUG_PRINT("DEBUG_PRINT KXTJ3_CTRL_REG2 _write_masked_reg KXTJ3_CTRL_REG2_SRST success\n");
    sleep_ms(10);
    _debug_print_write_regs();

    // MEMS self test (Electrostatic-actuation of the accelerometer, results in a DC shift of the X, Y and Z axes outputs)
    DEBUG_PRINT("\nDEBUG_PRINT *** MEMS self test \n");
    ret = _write_regs_with_clear(KXTJ3_SELF_TEST, KXTJ3_SELF_TEST_VAL, 1);
    if(ret != PICO_OK) return PICO_ERROR_GENERIC; DEBUG_PRINT("DEBUG_PRINT KXTJ3_SELF_TEST _write_regs KXTJ3_SELF_TEST_VAL success\n");
    // todo: read and compare axes to to table 1 here

    // turn off self test
    ret = _write_regs_with_clear(KXTJ3_SELF_TEST, 0, 1);
    if(ret != PICO_OK) return PICO_ERROR_GENERIC;
    DEBUG_PRINT("DEBUG_PRINT KXTJ3_SELF_TEST _write_regs 0 success\n\n");
    ret = PICO_OK;
    // end MEMS self test


    // digital communication self-test KXTJ3_CTRL_REG2_DCST
    DEBUG_PRINT("\nDEBUG_PRINT *** digital communication self test \n");
    // check for KXTJ3_DCST_RESP_NORMAL_VAL
    ret = _read_regs(KXTJ3_DCST_RESP, &reg, 1);
    if(ret != PICO_OK) return ret;  DEBUG_PRINT("DEBUG_PRINT KXTJ3_DCST_RESP _read_regs success\n");
    if(reg != KXTJ3_DCST_RESP_NORMAL_VAL) return reg;   DEBUG_PRINT("DEBUG_PRINT KXTJ3_DCST_RESP_NORMAL_VAL success\n");
    // check for KXTJ3_DCST_RESP_TEST_VAL
    ret = _write_masked_reg(KXTJ3_CTRL_REG2, KXTJ3_CTRL_REG2_DCST, true);
    if(ret != PICO_OK) return ret;    DEBUG_PRINT("DEBUG_PRINT KXTJ3_CTRL_REG2 _write_regs KXTJ3_CTRL_REG2_DCST success\n");
    ret = _read_regs(KXTJ3_DCST_RESP, &reg, 1);
    if(ret != PICO_OK) return ret;    DEBUG_PRINT("DEBUG_PRINT KXTJ3_DCST_RESP _read_regs success\n");
    if(reg != KXTJ3_DCST_RESP_TEST_VAL) return reg;    DEBUG_PRINT("DEBUG_PRINT KXTJ3_DCST_RESP_TEST_VAL success\n");
    // check for KXTJ3_DCST_RESP_NORMAL_VAL again to make sure it is back to normal after last read
    ret = _read_regs(KXTJ3_DCST_RESP, &reg, 1);
    if(ret != PICO_OK) return ret;    DEBUG_PRINT("DEBUG_PRINT KXTJ3_DCST_RESP _read_regs success\n");
    if(reg != KXTJ3_DCST_RESP_NORMAL_VAL) return reg;    DEBUG_PRINT("DEBUG_PRINT KXTJ3_DCST_RESP_NORMAL_VAL success\n");
    ret = PICO_OK;
    // end digital communication self-test KXTJ3_CTRL_REG2_DCST

    DEBUG_PRINT("\nDEBUG_PRINT *** set KXTJ3_DATA_CTRL_REG  _data_rate \n");   
    ret = _write_regs_with_clear(KXTJ3_DATA_CTRL_REG, _data_rate, 1);
    if(ret != PICO_OK) return ret; DEBUG_PRINT("DEBUG_PRINT KXTJ3_CTRL_REG1 _write_masked_reg set KXTJ3_CTRL_REG1_PC1 success\n");

    DEBUG_PRINT("\nDEBUG_PRINT *** set KXTJ3_CTRL_REG1 _accel_range and _high_res_mode \n");   
    ret = _write_masked_reg(KXTJ3_CTRL_REG1, _accel_range | _high_res_mode, true);
    if(ret != PICO_OK) return ret; DEBUG_PRINT("DEBUG_PRINT KXTJ3_CTRL_REG1 _write_masked_reg set KXTJ3_CTRL_REG1_PC1 success\n");

    DEBUG_PRINT("\nDEBUG_PRINT ***KXTJ3_CTRL_REG1 set KXTJ3_CTRL_REG1_PC1 operating mode\n");   
    ret = _write_masked_reg(KXTJ3_CTRL_REG1, KXTJ3_CTRL_REG1_PC1, true);
    if(ret != PICO_OK) return ret; DEBUG_PRINT("DEBUG_PRINT KXTJ3_CTRL_REG1 _write_masked_reg set KXTJ3_CTRL_REG1_PC1 success\n");

    _debug_print_write_regs();

    DEBUG_PRINT("\nDEBUG_PRINT --- end begin \n\n");
    return ret;
}

int kxtj3_i2c::_read_regs(uint8_t addr, uint8_t *regs, uint8_t qty ){

    int ret = i2c_write_blocking(_I2C_INST_KXTJ3, _KXTJ3_I2C_ADDR, &addr, 1, true);

    if(ret == 1){
        ret = i2c_read_blocking(_I2C_INST_KXTJ3, _KXTJ3_I2C_ADDR, regs, qty, false);
        return (ret == qty) ? PICO_OK : ret;
    } else  return ret;
}

int kxtj3_i2c::_write_regs(uint8_t addr, uint8_t regs, uint8_t qty ){

    uint8_t bytes[qty+1];
    bytes[0] = addr;
    for(int i = 1; i < qty+1; i++){
        bytes[i] = regs;
    }
    int ret = i2c_write_blocking(_I2C_INST_KXTJ3, _KXTJ3_I2C_ADDR, bytes, qty+1, false);
    return (ret == (qty + 1)) ? PICO_OK : ret;

}

int kxtj3_i2c::_write_regs_with_clear(uint8_t addr, uint8_t regs, uint8_t qty ){

    // DS Table 9: Register Map : all writes to kxtj3 must be done with KXTJ3_CTRL_REG1_PC1 cleared    
    uint8_t reg = 0;
    int ret = _read_regs(KXTJ3_CTRL_REG1, &reg, 1);
    if(ret != PICO_OK) return ret; DEBUG_PRINT("DEBUG_PRINT KXTJ3_CTRL_REG1 _read_regs success\n");
    ret = _write_regs(KXTJ3_CTRL_REG1,  reg & ~KXTJ3_CTRL_REG1_PC1 ,1);
    if(ret != PICO_OK) return ret; DEBUG_PRINT("DEBUG_PRINT KXTJ3_CTRL_REG1 _write_regs clear KXTJ3_CTRL_REG1_PC1 success\n");

    return _write_regs(addr,  regs , qty);

}

int kxtj3_i2c::_write_masked_reg(uint8_t addr, uint8_t mask, bool set){
    // if set is true then set the mask bit(s), else clear the mask bit(s)
    uint8_t reg = 0;
    int ret = _read_regs(addr, &reg, 1);
    if(ret != PICO_OK) return ret;    DEBUG_PRINT("DEBUG_PRINT write mask initial _read_regs success\n");    

    reg = set? reg | mask : reg & ~mask;

    return _write_regs(addr, reg, 1);
}

int kxtj3_i2c::get_accel_data(axes *axes){

    uint8_t regs[6];

    int ret = _read_regs(KXTJ3_XOUT_L, regs, 6);

    axes->x_l = regs[0];
    axes->x_h = regs[1];
    axes->y_l = regs[2];
    axes->y_h = regs[3];
    axes->z_l = regs[4];
    axes->z_h = regs[5];
    
    int s = 1;

    switch (_accel_range)
    {
    case ACCEL_2G:
        s = 8;
        break;
    case ACCEL_4G:
        s = 4;
        break;
    case ACCEL_8G:
    case ACCEL_8G_14B:
        s = 2;
        break;
    default:
        break;
    }

    axes->x = ((double)((int16_t)((axes->x_h << 8) | axes->x_l)))/(2048.0*s);
    axes->y = ((double)((int16_t)((axes->y_h << 8) | axes->y_l)))/(2048.0*s);
    axes->z = ((double)((int16_t)((axes->z_h << 8) | axes->z_l)))/(2048.0*s);

    return ret;
}


void kxtj3_i2c::_debug_print_write_regs(){

    DEBUG_PRINT("\nDEBUG_PRINT *** Writable registers\n");

    uint8_t reg = 0;
    int ret = _read_regs(KXTJ3_CTRL_REG1, &reg, 1);
    DEBUG_PRINT("DEBUG_PRINT KXTJ3_CTRL_REG1, \t\t 0x%02x, 0b", reg);
        for (int i = 7; i >= 0; i--) { DEBUG_PRINT("%d", (reg >> i) & 1); }
        for (int i = 7; i >= 0; i--) { if((reg >> i) & 1) DEBUG_PRINT(" %d,", i); }
        DEBUG_PRINT("\n");

    ret = _read_regs(KXTJ3_CTRL_REG2, &reg, 1);
    DEBUG_PRINT("DEBUG_PRINT KXTJ3_CTRL_REG2, \t\t 0x%02x, 0b", reg);
        for (int i = 7; i >= 0; i--) { DEBUG_PRINT("%d", (reg >> i) & 1); }
        for (int i = 7; i >= 0; i--) { if((reg >> i) & 1) DEBUG_PRINT(" %d,", i); }
        DEBUG_PRINT("\n");

    ret = _read_regs(KXTJ3_INT_CTRL_REG1, &reg, 1);
    DEBUG_PRINT("DEBUG_PRINT KXTJ3_INT_CTRL_REG1, \t 0x%02x, 0b", reg);
        for (int i = 7; i >= 0; i--) { DEBUG_PRINT("%d", (reg >> i) & 1); }
        for (int i = 7; i >= 0; i--) { if((reg >> i) & 1) DEBUG_PRINT(" %d,", i); }
        DEBUG_PRINT("\n");

    ret = _read_regs(KXTJ3_INT_CTRL_REG2, &reg, 1);
    DEBUG_PRINT("DEBUG_PRINT KXTJ3_INT_CTRL_REG2, \t 0x%02x, 0b", reg);
        for (int i = 7; i >= 0; i--) { DEBUG_PRINT("%d", (reg >> i) & 1); }
        for (int i = 7; i >= 0; i--) { if((reg >> i) & 1) DEBUG_PRINT(" %d,", i); }
        DEBUG_PRINT("\n");

    ret = _read_regs(KXTJ3_DATA_CTRL_REG, &reg, 1);
    DEBUG_PRINT("DEBUG_PRINT KXTJ3_DATA_CTRL_REG, \t 0x%02x, 0b", reg);
        for (int i = 7; i >= 0; i--) { DEBUG_PRINT("%d", (reg >> i) & 1); }
        for (int i = 7; i >= 0; i--) { if((reg >> i) & 1) DEBUG_PRINT(" %d,", i); }
        DEBUG_PRINT("\n");

    ret = _read_regs(KXTJ3_WAKEUP_COUNTER, &reg, 1);
    DEBUG_PRINT("DEBUG_PRINT KXTJ3_WAKEUP_COUNTER, \t 0x%02x, 0b", reg);
        for (int i = 7; i >= 0; i--) { DEBUG_PRINT("%d", (reg >> i) & 1); }
        for (int i = 7; i >= 0; i--) { if((reg >> i) & 1) DEBUG_PRINT(" %d,", i); }
        DEBUG_PRINT("\n");

    ret = _read_regs(KXTJ3_NA_COUNTER, &reg, 1);
    DEBUG_PRINT("DEBUG_PRINT KXTJ3_NA_COUNTER, \t\t 0x%02x, 0b", reg);
        for (int i = 7; i >= 0; i--) { DEBUG_PRINT("%d", (reg >> i) & 1); }
        for (int i = 7; i >= 0; i--) { if((reg >> i) & 1) DEBUG_PRINT(" %d,", i); }
        DEBUG_PRINT("\n");

    ret = _read_regs(KXTJ3_SELF_TEST, &reg, 1);
    DEBUG_PRINT("DEBUG_PRINT KXTJ3_SELF_TEST, \t\t 0x%02x, 0b", reg);
        for (int i = 7; i >= 0; i--) { DEBUG_PRINT("%d", (reg >> i) & 1); }
        for (int i = 7; i >= 0; i--) { if((reg >> i) & 1) DEBUG_PRINT(" %d,", i); }
        DEBUG_PRINT("\n");

    ret = _read_regs(KXTJ3_WAKEUP_THRESHOLD_H, &reg, 1);
    DEBUG_PRINT("DEBUG_PRINT KXTJ3_WAKEUP_THRESHOLD_H, \t 0x%02x, 0b", reg);
        for (int i = 7; i >= 0; i--) { DEBUG_PRINT("%d", (reg >> i) & 1); }
        for (int i = 7; i >= 0; i--) { if((reg >> i) & 1) DEBUG_PRINT(" %d,", i); }
        DEBUG_PRINT("\n");

    ret = _read_regs(KXTJ3_WAKEUP_THRESHOLD_L, &reg, 1);
    DEBUG_PRINT("DEBUG_PRINT KXTJ3_WAKEUP_THRESHOLD_L, \t 0x%02x, 0b", reg);
        for (int i = 7; i >= 0; i--) { DEBUG_PRINT("%d", (reg >> i) & 1); }
        for (int i = 7; i >= 0; i--) { if((reg >> i) & 1) DEBUG_PRINT(" %d,", i); }
        DEBUG_PRINT("\n");

}
