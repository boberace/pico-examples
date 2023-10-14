/*

Documents referenced in comments:
DS: KXTJ3-1057 Specifications

todo:  
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

    uint8_t reg = 0;
    int ret = PICO_OK;
    DEBUG_PRINT("\033[2J");// clear screen
    DEBUG_PRINT("DP +++ start begin \n\n");

    // who am I check
    ret = _read_regs(KXTJ3_WHO_AM_I, &reg, 1);
    if(ret != PICO_OK) return ret;
    DEBUG_PRINT("DP KXTJ3_WHO_AM_I _read_regs success\n");
    if(reg != KXTJ3_WHO_AM_I_VAL) return reg;
    DEBUG_PRINT("DP KXTJ3_WHO_AM_I_VAL success\n");

    ret = initiate_software_reset(); if(ret != PICO_OK) return ret;
    _debug_print_writable_regs();
    ret = set_resolution(high_res_mode); if(ret != PICO_OK) return ret;    
    ret = set_data_rate(data_rate); if(ret != PICO_OK) return ret;
    ret = set_accel_range(accel_range); if(ret != PICO_OK) return ret;    
    ret = initiate_analog_self_test(); if(ret != PICO_OK) return ret;
    ret = initiate_digital_self_test(); if(ret != PICO_OK) return ret;    
    ret = enter_normal_mode(); if(ret != PICO_OK) return ret;
    _debug_print_writable_regs();

    DEBUG_PRINT("\nDP --- end begin \n\n");
    return ret;
}


int kxtj3_i2c::enter_normal_mode(){
    int ret = _write_masked_reg(KXTJ3_CTRL_REG1, KXTJ3_CTRL_REG1_PC1, true);
    if(ret == PICO_OK) DEBUG_PRINT("DP enter_normal_mode success\n"); else DEBUG_PRINT("DP enter_normal_mode failure\n");
    sleep_ms(10);
    
    return ret;
}

int kxtj3_i2c::enter_standby_mode(){
    int ret = _write_masked_reg(KXTJ3_CTRL_REG1, KXTJ3_CTRL_REG1_PC1, false);
    if(ret == PICO_OK) DEBUG_PRINT("DP enter_standby_mode success\n"); else DEBUG_PRINT("DP enter_standby_mode failure\n");
    sleep_ms(10);
    
    return ret;
}

int kxtj3_i2c::set_resolution(bool high_res_mode){  
    int ret = _write_masked_reg(KXTJ3_CTRL_REG1, KXTJ3_CTRL_REG1_RES, high_res_mode);
    if(ret == PICO_OK) DEBUG_PRINT("DP set_resolution success\n"); else DEBUG_PRINT("DP set_resolution failure\n");
    return ret;
}

int kxtj3_i2c::set_accel_range(KXTJ3_accel_range_t accel_range){
    uint8_t reg;
    int ret = _read_regs(KXTJ3_CTRL_REG1, &reg, 1);
    if(ret != PICO_OK) {DEBUG_PRINT("DP _read_regs KXTJ3_CTRL_REG1 failure\n"); return PICO_ERROR_IO;}
    reg &= ~_accel_range_mask; // clear the accel range bits
    reg |= accel_range;  // set new bits
    ret = _write_masked_reg(KXTJ3_CTRL_REG1, reg, true);
    if(ret == PICO_OK) DEBUG_PRINT("DP set_accel_range success\n"); else DEBUG_PRINT("DP set_accel_range failure\n");
    return ret;
}

int kxtj3_i2c::initiate_software_reset(){
    int ret = _write_masked_reg(KXTJ3_CTRL_REG2, KXTJ3_CTRL_REG2_SRST, true);
    if(ret == PICO_OK) DEBUG_PRINT("DP initiate_software_reset success\n"); else DEBUG_PRINT("DP initiate_software_reset failure\n");
    sleep_ms(10);
    return ret;
}

int kxtj3_i2c::initiate_analog_self_test(){
    // MEMS self test (Electrostatic-actuation of the accelerometer, results in a DC shift of the X, Y and Z axes outputs)
    DEBUG_PRINT("\nDP *** MEMS self test \n");
    int ret = _write_regs_with_clear(KXTJ3_SELF_TEST, KXTJ3_SELF_TEST_VAL, 1);
    if(ret != PICO_OK) return PICO_ERROR_GENERIC; DEBUG_PRINT("DP KXTJ3_SELF_TEST _write_regs_with_clear KXTJ3_SELF_TEST_VAL success\n");
    // todo: read and compare axes to to table 1 here
    //**********************************************************************
    // turn off self test
    ret = _write_regs_with_clear(KXTJ3_SELF_TEST, 0, 1);
    if(ret != PICO_OK) return PICO_ERROR_GENERIC;
    DEBUG_PRINT("DP KXTJ3_SELF_TEST _write_regs_with_clear 0 success\n\n");
    // end MEMS self test
   return PICO_OK;
}

int kxtj3_i2c::initiate_digital_self_test(){
    // digital communication self-test KXTJ3_CTRL_REG2_DCST
    DEBUG_PRINT("\nDP *** digital communication self test \n");
    // check for KXTJ3_DCST_RESP_NORMAL_VAL
    uint8_t reg;
    int ret = _read_regs(KXTJ3_DCST_RESP, &reg, 1);
    if(ret != PICO_OK) return ret;  DEBUG_PRINT("DP KXTJ3_DCST_RESP _read_regs success\n");
    if(reg != KXTJ3_DCST_RESP_NORMAL_VAL) return reg;   DEBUG_PRINT("DP KXTJ3_DCST_RESP_NORMAL_VAL success\n");
    // check for KXTJ3_DCST_RESP_TEST_VAL
    ret = _write_masked_reg(KXTJ3_CTRL_REG2, KXTJ3_CTRL_REG2_DCST, true);
    if(ret != PICO_OK) return ret;    DEBUG_PRINT("DP KXTJ3_CTRL_REG2 _write_masked_reg KXTJ3_CTRL_REG2_DCST success\n");
    ret = _read_regs(KXTJ3_DCST_RESP, &reg, 1);
    if(ret != PICO_OK) return ret;    DEBUG_PRINT("DP KXTJ3_DCST_RESP _read_regs success\n");
    if(reg != KXTJ3_DCST_RESP_TEST_VAL) return reg;    DEBUG_PRINT("DP KXTJ3_DCST_RESP_TEST_VAL success\n");
    // check for KXTJ3_DCST_RESP_NORMAL_VAL again to make sure it is back to normal after last read
    ret = _read_regs(KXTJ3_DCST_RESP, &reg, 1);
    if(ret != PICO_OK) return ret;    DEBUG_PRINT("DP KXTJ3_DCST_RESP _read_regs success\n");
    if(reg != KXTJ3_DCST_RESP_NORMAL_VAL) return reg;    DEBUG_PRINT("DP KXTJ3_DCST_RESP_NORMAL_VAL success\n");
    return PICO_OK;
}

int kxtj3_i2c::set_data_rate(KXTJ3_data_rate_t data_rate){
    int ret = _write_regs_with_clear(KXTJ3_DATA_CTRL_REG, data_rate, 1);
    if(ret == PICO_OK) DEBUG_PRINT("DP set_data_rate success\n"); else DEBUG_PRINT("DP set_data_rate failure\n");
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
    if(ret != PICO_OK) return ret; 
    ret = _write_regs(KXTJ3_CTRL_REG1,  reg & ~KXTJ3_CTRL_REG1_PC1 ,1);
    if(ret != PICO_OK) return ret;
    ret = _write_regs(addr,  regs , qty);
    return (ret == qty) ? PICO_OK : ret;
}

int kxtj3_i2c::_write_masked_reg(uint8_t addr, uint8_t mask, bool set){
    // if set is true then set the mask bit(s), else clear the mask bit(s)
    uint8_t reg = 0;
    int ret = _read_regs(addr, &reg, 1);
    if(ret != PICO_OK) return ret;    DEBUG_PRINT("DP write mask initial _read_regs success\n");    

    reg = set? reg | mask : reg & ~mask;

    return _write_regs_with_clear(addr, reg, 1);
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

    uint8_t reg;
    ret = _read_regs(KXTJ3_CTRL_REG1, &reg, 1);
    reg &= _accel_range_mask;
    switch (reg)
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

void kxtj3_i2c::_debug_print_reg_binary(__uint8_t reg){
        for (int i = 7; i >= 0; i--) { DEBUG_PRINT("%d", (reg >> i) & 1); }
        for (int i = 7; i >= 0; i--) { if((reg >> i) & 1) DEBUG_PRINT(" %d,", i); }
        DEBUG_PRINT("\n");
}

void kxtj3_i2c::_debug_print_writable_regs(){

    DEBUG_PRINT("\nDP *** Writable registers\n");

    uint8_t reg = 0;

    int ret = _read_regs(KXTJ3_CTRL_REG1, &reg, 1);
    DEBUG_PRINT("DP KXTJ3_CTRL_REG1, \t\t 0x%02x, 0b", reg);
    _debug_print_reg_binary(reg);

    ret = _read_regs(KXTJ3_CTRL_REG2, &reg, 1);
    DEBUG_PRINT("DP KXTJ3_CTRL_REG2, \t\t 0x%02x, 0b", reg);
    _debug_print_reg_binary(reg);

    ret = _read_regs(KXTJ3_INT_CTRL_REG1, &reg, 1);
    DEBUG_PRINT("DP KXTJ3_INT_CTRL_REG1, \t 0x%02x, 0b", reg);
    _debug_print_reg_binary(reg);
    
    ret = _read_regs(KXTJ3_INT_CTRL_REG2, &reg, 1);
    DEBUG_PRINT("DP KXTJ3_INT_CTRL_REG2, \t 0x%02x, 0b", reg);
    _debug_print_reg_binary(reg);

    ret = _read_regs(KXTJ3_DATA_CTRL_REG, &reg, 1);
    DEBUG_PRINT("DP KXTJ3_DATA_CTRL_REG, \t 0x%02x, 0b", reg);
    _debug_print_reg_binary(reg);

    ret = _read_regs(KXTJ3_WAKEUP_COUNTER, &reg, 1);
    DEBUG_PRINT("DP KXTJ3_WAKEUP_COUNTER, \t 0x%02x, 0b", reg);
    _debug_print_reg_binary(reg);

    ret = _read_regs(KXTJ3_NA_COUNTER, &reg, 1);
    DEBUG_PRINT("DP KXTJ3_NA_COUNTER, \t\t 0x%02x, 0b", reg);
    _debug_print_reg_binary(reg);

    ret = _read_regs(KXTJ3_SELF_TEST, &reg, 1);
    DEBUG_PRINT("DP KXTJ3_SELF_TEST, \t\t 0x%02x, 0b", reg);
    _debug_print_reg_binary(reg);

    ret = _read_regs(KXTJ3_WAKEUP_THRESHOLD_H, &reg, 1);
    DEBUG_PRINT("DP KXTJ3_WAKEUP_THRESHOLD_H, \t 0x%02x, 0b", reg);
    _debug_print_reg_binary(reg);

    ret = _read_regs(KXTJ3_WAKEUP_THRESHOLD_L, &reg, 1);
    DEBUG_PRINT("DP KXTJ3_WAKEUP_THRESHOLD_L, \t 0x%02x, 0b", reg);
    _debug_print_reg_binary(reg);

    DEBUG_PRINT("ret %i\n", ret);

}
