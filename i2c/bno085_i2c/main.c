/*

Docs:
DS: BNO080/85/86 Data Sheet
RM: SH-2 Reference Manual v1.2
TP: Sensor Hub Transport Protocol R 1.7

*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <string.h>

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

// #define DEBUG

#ifdef DEBUG
# define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
# define DEBUG_PRINT(...)
#endif

#define PIN_BNO085_INT 15
#define PIN_BNO085_RST 16

#define I2C0_BUADRATE 100*1000
#define PIN_I2C0_SDA 4
#define PIN_I2C0_SCL 5
i2c_inst_t *I2C_BNO085 = i2c0;
#define I2C_BUFFER_LIMIT 32

#define BNO085_I2C_ADDR 0x4A

sh2_Hal_t sh2_hal;

#define CARGO_SIZE  SH2_HAL_MAX_PAYLOAD_IN // I2C_BUFFER_LIMIT
uint8_t shtp_header[4];
uint8_t shtp_cargo[CARGO_SIZE]; 	
bool reset_occurred = false;
sh2_ProductIds_t prodIds; ///< The product IDs returned by the sensor

int16_t ret = 0;

//DS: 1.3.1 SHTP - The BNO08X supports 6 channels
const uint8_t CHANNEL_COMMAND = 0;
const uint8_t CHANNEL_EXECUTABLE = 1;
const uint8_t CHANNEL_CONTROL = 2;
const uint8_t CHANNEL_REPORTS = 3;
const uint8_t CHANNEL_WAKE_REPORTS = 4;
const uint8_t CHANNEL_GYRO = 5;


void setup_i2c0(void);
int i2c_open(sh2_Hal_t *self);
void i2c_close(sh2_Hal_t *self);
int i2c_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
int i2c_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
uint32_t getTimeUs(sh2_Hal_t *self);
static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
bool bno085_init_i2c(void);

int main() {    
    stdio_init_all();
    DEBUG_PRINT("\nStarted\n");
    setup_i2c0();
    bool r = bno085_init_i2c();
    if(r == false){
        printf(" bno0855 will not initialize\n");
        return false;
    } else {
        printf(" bno0855 has initialized\n");
    }

  for (int n = 0; n < prodIds.numEntries; n++) {
    printf("Part %d\n", prodIds.entry[n].swPartNumber);
    printf(": Version %d.%d.%d\n", prodIds.entry[n].swVersionMajor, prodIds.entry[n].swVersionMinor, prodIds.entry[n].swVersionPatch);
    printf(" Build %d\n", prodIds.entry[n].swBuildNumber);
  }
    

     while(false){     

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

static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
  // If we see a reset, set a flag so that sensors will be reconfigured.
  if (pEvent->eventId == SH2_RESET) {
    // printfln("Reset!");
    reset_occurred = true;
  }
}

bool bno085_init_i2c(void){

    // interupt pin
    gpio_init(PIN_BNO085_INT);
    gpio_set_function(PIN_BNO085_INT, GPIO_OUT);
    gpio_init(PIN_BNO085_RST);
    gpio_set_function(PIN_BNO085_RST, GPIO_OUT);

    uint8_t dummy;
    ret = i2c_read_blocking(I2C_BNO085, BNO085_I2C_ADDR, &dummy, 1, false);
    if(ret < 1){
         printf(" BNO085 will not connect\n");
         return false;
    }
        DEBUG_PRINT("i2c_read_blocking dummy test if connected %d\n", ret);

    sh2_hal.open = i2c_open;
    sh2_hal.close = i2c_close;
    sh2_hal.read = i2c_read;
    sh2_hal.write = i2c_write;
    sh2_hal.getTimeUs = getTimeUs; 

    // hardware reset
    gpio_put(PIN_BNO085_RST, 1);
    sleep_ms(10);
    gpio_put(PIN_BNO085_RST, 0);
    sleep_ms(10);
    gpio_put(PIN_BNO085_RST, 1);
    sleep_ms(10);  

    int status;
    status = sh2_open(&sh2_hal, hal_callback, NULL);
    if (status != SH2_OK) { 
        return false;
    }        
    DEBUG_PRINT("---------------------------------------------------- sh2_open \n");
    memset(&prodIds, 0, sizeof(prodIds));
    status = sh2_getProdIds(&prodIds);
    if (status != SH2_OK) {
        return false;
    }
    DEBUG_PRINT("------------------------------------------------ sh2_getProdIds \n");

    return true;
}

int i2c_open(sh2_Hal_t *self){  
    DEBUG_PRINT("************************************ i2c_open \n");
    // send a software reset = { Length LSB, Length LSB, Channel, SeqNum, data }
    uint8_t softreset_pkt[5] = { 5, 0, CHANNEL_EXECUTABLE, 0, 1}; // DS: Figure 1-27: 1 – reset
    ret = i2c_write_blocking(I2C_BNO085,BNO085_I2C_ADDR, softreset_pkt, 5, false );
        DEBUG_PRINT("i2c_write_blocking softreset_pkt %d\n", ret); 
    if(ret < 1) return -1;

    ret = i2c_read_blocking(I2C_BNO085,BNO085_I2C_ADDR, shtp_header, 4, false );
        DEBUG_PRINT("i2c_read_blocking shtp_header %d\n", ret);
    if(ret < 1) return -1;
    uint16_t length = (shtp_header[1] << 8) |  shtp_header[0];
    length &= ~(1 << 15);
        DEBUG_PRINT("shtp_header 0 : 0x%02x, 0b%08b\n", shtp_header[0], shtp_header[0]);
        DEBUG_PRINT("shtp_header 1 : 0x%02x, 0b%08b\n", shtp_header[1], shtp_header[1]);
        DEBUG_PRINT("shtp_header 2 : 0x%02x, 0b%08b\n", shtp_header[2], shtp_header[2]);
        DEBUG_PRINT("shtp_header 3 : 0x%02x, 0b%08b\n", shtp_header[3], shtp_header[3]);
        DEBUG_PRINT("length %d\n", length);
    sleep_ms(100);
    i2c_read(&sh2_hal, shtp_cargo, length, NULL);
    sleep_ms(100);

    return 0;
}

void i2c_close(sh2_Hal_t *self){
    DEBUG_PRINT("************************************ i2c_close \n");
}

int i2c_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us){
    DEBUG_PRINT("************************************ i2c_read \n");
    ret = i2c_read_blocking(I2C_BNO085,BNO085_I2C_ADDR, shtp_header, 4, false );
        DEBUG_PRINT("i2c_read_blocking shtp_header %d\n", ret);
    if(ret != 4) return 0;

    uint16_t length = (shtp_header[1] << 8) |  shtp_header[0];
    length &= ~(1 << 15);
        DEBUG_PRINT("shtp_header 0 : 0x%02x, 0b%08b\n", shtp_header[0], shtp_header[0]);
        DEBUG_PRINT("shtp_header 1 : 0x%02x, 0b%08b\n", shtp_header[1], shtp_header[1]);
        DEBUG_PRINT("shtp_header 2 : 0x%02x, 0b%08b\n", shtp_header[2], shtp_header[2]);
        DEBUG_PRINT("shtp_header 3 : 0x%02x, 0b%08b\n", shtp_header[3], shtp_header[3]);
        DEBUG_PRINT("length %d\n", length);
  
    if(length){ // no zero reads    

        if(length <= CARGO_SIZE){ // fill the cargo
            
            if(length > len) length = len;  // do not get more than asked if more is available

            ret = i2c_read_blocking(I2C_BNO085,BNO085_I2C_ADDR, pBuffer, length, false);                
                DEBUG_PRINT("i2c_read_blocking pBuffer %d\n", ret);
            if(ret != length) return 0;

        } else { // fill up the cargo, flush the rest

            uint8_t flush_cargo[CARGO_SIZE];

            ret = i2c_read_blocking(I2C_BNO085,BNO085_I2C_ADDR, pBuffer, CARGO_SIZE, false);                
                DEBUG_PRINT("i2c_read_blocking pBuffer %d\n", ret);   
            if(ret != CARGO_SIZE) return 0;
            
            length -= CARGO_SIZE;

            while(length > 0){
                    if(length > CARGO_SIZE){ 
                        ret = i2c_read_blocking(I2C_BNO085,BNO085_I2C_ADDR, flush_cargo, CARGO_SIZE, false);
                            DEBUG_PRINT("i2c_read_blocking flush_cargo %d\n", ret); 
                        if(ret != CARGO_SIZE) return 0;
                        length -= CARGO_SIZE;
                    } else {
                        ret = i2c_read_blocking(I2C_BNO085,BNO085_I2C_ADDR, flush_cargo, length, false);
                            DEBUG_PRINT("i2c_read_blocking flush_cargo %d\n", ret);
                        if(ret != length) return 0;
                        length = 0;
                    }
            }

        }
    }
     *t_us = to_us_since_boot(get_absolute_time());

    return length;
}

int i2c_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len){
    DEBUG_PRINT("************************************ i2c_write \n");
    uint16_t length = (len > SH2_HAL_MAX_TRANSFER_OUT)?SH2_HAL_MAX_TRANSFER_OUT:len;

    ret = i2c_write_blocking(I2C_BNO085,BNO085_I2C_ADDR, pBuffer, length, false );
        DEBUG_PRINT("i2c_write_blocking softreset_pkt %d\n", ret);
    if(ret != length) return 0;

    return ret;
}

uint32_t getTimeUs(sh2_Hal_t *self){
    DEBUG_PRINT("************************************ getTimeUs \n");
    return to_us_since_boot(get_absolute_time());
}
