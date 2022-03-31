/*



*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"


#define DEBUG

#ifdef DEBUG
# define DEBUG_PRINT printf 
#else
# define DEBUG_PRINT
#endif

#define I2C0_BUADRATE 400*1000
#define PIN_I2C0_SDA 12
#define PIN_I2C0_SCL 13
i2c_inst_t *I2C_BNO085 = i2c0;

#define BNO085_I2C_ADDR 0x4A

struct vec3{
    uint8_t data[6];
    int16_t x, y, z;
    uint8_t addr;
};

struct vec4{
    uint8_t data[8];
    int16_t w, x, y, z;
    uint8_t addr;
};

void setup_i2c0(void);
void ndof_init(void);
void get_data_v3(struct vec3 *t);
void get_data_v4(struct vec4 *t);
uint8_t ret_reg(uint8_t addr);
void get_regs(uint8_t addr, uint8_t *regs, uint8_t qty );

//Global Variables
uint8_t shtpHeader[4]; 
uint8_t shtpData[2^15];
uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; 
uint8_t commandSequenceNumber = 0;				
uint32_t metaData[9]; //Figure 30: Metadata Record Format – Revision 3			

int main() {
    stdio_init_all();
    setup_i2c0();
    ndof_init();

    DEBUG_PRINT("test debug print %d, %02x", 255,255);


    // struct vec3 acc;
    // acc.addr = 0x08; // 4.3.9 ACC_DATA_X_LSB 0x08  

    // struct vec3 mag;
    // mag.addr = 0x0E; // 4.3.15 MAG_DATA_X_LSB 0x0E    

    // struct vec3 gyr;
    // gyr.addr = 0x14; // 4.3.21 GYR_DATA_X_LSB 0x14 

    // struct vec3 eul;
    // eul.addr = 0x1A; // 4.3.27 EUL_DATA_X_LSB 0x1A 

    // struct vec4 qat;
    // qat.addr = 0x20; // 4.3.33 QUA_DATA_W_LSB 0x20 

    // struct vec3 lia;
    // lia.addr = 0x28; // 4.3.41 LIA_DATA_X_LSB 0x28

    // struct vec3 grv;
    // grv.addr = 0x2E; // 4.3.47 GRV_DATA_X_LSB 0x2E 

    while(false){     

        // uint8_t calib_stat =  ret_reg(0x35); //4.3.54 CALIB_STAT 0x35
        // if(calib_stat != 0b11111111){
        //     printf("SYS %01x\n", (calib_stat&0b11000000) >> 6);
        //     printf("GYR %01x\n", (calib_stat&0b00110000) >> 4);
        //     printf("ACC %01x\n", (calib_stat&0b00001100) >> 2);
        //     printf("MAG %01x\n", (calib_stat&0b00000011) );
        // }

        // get_data_v3(&acc);
        // printf("accel X: %d    Y: %d    Z: %d\n", acc.x, acc.y, acc.z);

        // get_data_v3(&mag);    
        // printf("mag X: %d    Y: %d    Z: %d\n", mag.x, mag.y, mag.z);

        // get_data_v3(&gyr);  
        // printf("gyr X: %d    Y: %d    Z: %d\n", gyr.x, gyr.y, gyr.z);

        // get_data_v3(&eul);  
        // printf("eul X: %d    Y: %d    Z: %d\n", eul.x, eul.y, eul.z);

        // get_data_v4(&qat);  
        // printf("qat X: %d    Y: %d    Z: %d    W: %d\n", qat.x, qat.y, qat.z, qat.w);

        // get_data_v3(&lia);  
        // printf("lia X: %d    Y: %d    Z: %d\n", lia.x, lia.y, lia.z);

        // get_data_v3(&grv);  
        // printf("grv X: %d    Y: %d    Z: %d\n", grv.x, grv.y, grv.z);

        sleep_ms(300);            

    }
    return 0;
}

void setup_i2c0(void){
    i2c_init(i2c0, I2C0_BUADRATE);
    gpio_set_function(PIN_I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C0_SDA);
    gpio_pull_up(PIN_I2C0_SCL);
}

void ndof_init(void){

    uint8_t data[2];

    // Check to see if connection is correct
    sleep_ms(600); // Add a short delay to help BNO085 boot up

    shtpData[0] = 0xF9; //Request the product ID and reset info
	shtpData[1] = 0;
    get_regs(0xF9, shtpData, 2);

    while(shtpData[0] != 0xF8){
        printf("Waiting for Chip Connection\n");
        sleep_ms(100); 
        get_regs(0xF9, shtpData, 2);
    }
    DEBUG_PRINT("connected");


    // // configure for external oscillator    
    // data[0] = 0x3D; // 4.3.61 OPR_MODE 0x3D
    // data[1] = 0; // set operation mode to configure
    // i2c_write_blocking(I2C_BNO085, BNO085_I2C_ADDR, data, 2, true);        
    // while(ret_reg(0x38)&0b00000001) {// 4.3.57 SYS_CLK_STATUS 0x38 - wait to clear before setting
    //     printf(" waiting for go to configure clock\n");
    //     sleep_ms(100); 
    // }
    // data[0] = 0x3F; // 4.3.63 SYS_TRIGGER 0x3F
    // data[1] = 1 << 7; // CLK_SEL 7 [0: Use internal oscillator [1: Use external oscillator. Set this bit only if external crystal is connected
    // i2c_write_blocking(I2C_BNO085, BNO085_I2C_ADDR, data, 2, true);

    // set operation mode to nine degrees of freedom (ndof)
    data[0] = 0x3D; // 4.3.61 OPR_MODE 0x3D
    data[1] = 0b00001100; // xxxx1100 ndof
    i2c_write_blocking(I2C_BNO085, BNO085_I2C_ADDR, data, 2, true);
    sleep_ms(100);
}

void get_data_v3(struct vec3 *t){

    get_regs(t->addr, t->data, 6);
    t->x = ((t->data[1]<<8) | t->data[0]);
    t->y = ((t->data[3]<<8) | t->data[2]);
    t->z = ((t->data[5]<<8) | t->data[4]);
}

void get_data_v4(struct vec4 *t){

    get_regs(t->addr, t->data, 8);
    t->w = ((t->data[1]<<8) | t->data[0]);
    t->x = ((t->data[3]<<8) | t->data[2]);
    t->y = ((t->data[5]<<8) | t->data[4]);
    t->z = ((t->data[7]<<8) | t->data[6]);
}

uint8_t ret_reg(uint8_t addr){

    uint8_t reg;
    i2c_write_blocking(I2C_BNO085, BNO085_I2C_ADDR, &addr, 1, true);
    i2c_read_blocking(I2C_BNO085, BNO085_I2C_ADDR, &reg, 1, false);

    return reg;
}

void get_regs(uint8_t addr, uint8_t *regs, uint8_t qty ){

    i2c_write_blocking(I2C_BNO085, BNO085_I2C_ADDR, &addr, 1, true);
    i2c_read_blocking(I2C_BNO085, BNO085_I2C_ADDR, regs, qty, false);

}