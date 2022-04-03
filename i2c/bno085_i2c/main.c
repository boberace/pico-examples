/*

Documents referenced in comments:
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

// uncomment to print debug information over usb serial
// #define DEBUG

#ifdef DEBUG
# define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
# define DEBUG_PRINT(...)
#endif

// define interrupt and reset pins
#define PIN_BNO085_INT 15
#define PIN_BNO085_RST 16

// define i2c pararmeters and pins
#define I2C0_BUADRATE 100*1000
#define PIN_I2C0_SDA 4
#define PIN_I2C0_SCL 5
i2c_inst_t *I2C_BNO085 = i2c0;
#define BNO085_I2C_ADDR 0x4A

// sh2 variables
sh2_Hal_t sh2_hal;
#define CARGO_SIZE  128 // arbitrarily set, can be up to (2^15 - 4), 
bool reset_occurred = false; // flag set in hal_callback when reset occurs
sh2_ProductIds_t prodIds; ///< The product IDs returned by the sensor
sh2_SensorValue_t sensorValue;
const sh2_SensorId_t sensorID = SH2_ROTATION_VECTOR;
const float reportFrequency = 400;

// global return variable 
int16_t ret = 0;

// function headers
void setup_i2c0(void);
int i2c_open(sh2_Hal_t *self);
void i2c_close(sh2_Hal_t *self);
int i2c_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
int i2c_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
uint32_t getTimeUs(sh2_Hal_t *self);
static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
bool bno085_init_i2c(void);
bool enableReport(sh2_SensorId_t sensorId, float frequency);
bool getSensorEvent();
static void sensorHandler(void *cookie, sh2_SensorEvent_t *event);

void setReports(void) {
  printf("Setting desired reports\n");
  if (! enableReport(sensorID, reportFrequency)) {
    printf("Could not enable game vector\n");
  }
}

int main() {    //  main  main  main  main  main  main  main  main  main  main  main  main  main  main  main  main
    stdio_init_all(); DEBUG_PRINT("\nStarted\n");
    setup_i2c0();
    if(bno085_init_i2c()) printf(" bno0855 has initialized\n");
    else printf(" bno0855 NOT initialize\n");

    for (int n = 0; n < prodIds.numEntries; n++) {
    printf("Part %d\n", prodIds.entry[n].swPartNumber);
    printf(": Version %d.%d.%d\n",  prodIds.entry[n].swVersionMajor, 
                                    prodIds.entry[n].swVersionMinor, 
                                    prodIds.entry[n].swVersionPatch);
    printf(" Build %d\n", prodIds.entry[n].swBuildNumber);
    }  

    setReports(); DEBUG_PRINT("setReports\n");


    while(true){     //   loop   loop   loop   loop   loop   loop   loop   loop   loop   loop   loop   loop   loop

        sleep_ms(1);

        if (getSensorEvent()) {

            switch (sensorValue.sensorId) {
            
            case sensorID:
            printf("Rotation Vector:  i:%f, j:%f, k:%f,  r:%f\n",  sensorValue.un.gameRotationVector.i,
                                                                    sensorValue.un.gameRotationVector.j,
                                                                    sensorValue.un.gameRotationVector.k,
                                                                    sensorValue.un.gameRotationVector.real);
            break;
            }
        }

        // if (reset_occurred) enableReport(sensorID, reportFrequency);

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

    // interupt and RST pins
    gpio_init(PIN_BNO085_INT);
    gpio_set_function(PIN_BNO085_INT, GPIO_OUT);
    gpio_init(PIN_BNO085_RST);
    gpio_set_function(PIN_BNO085_RST, GPIO_OUT);

    // read a single byte to determine if connected - 
    // this could cause a full report to generate that would have to be flushed
    // but no matter as this is followed with a hardware reset
    uint8_t dummy;
    ret = i2c_read_blocking(I2C_BNO085, BNO085_I2C_ADDR, &dummy, 1, false);
    if(ret < 1){
         printf(" BNO085 will not connect\n");
         return false;
    } DEBUG_PRINT("i2c_read_blocking dummy test if connected %d\n", ret);

    // hardware reset
    gpio_put(PIN_BNO085_RST, 1);
    sleep_ms(10);
    gpio_put(PIN_BNO085_RST, 0);
    sleep_ms(10);
    gpio_put(PIN_BNO085_RST, 1);
    sleep_ms(10);  

    // setup sh2 hal pointers
    sh2_hal.open = i2c_open;
    sh2_hal.close = i2c_close;
    sh2_hal.read = i2c_read;
    sh2_hal.write = i2c_write;
    sh2_hal.getTimeUs = getTimeUs;  

    //
    int status;
    status = sh2_open(&sh2_hal, hal_callback, NULL);
    if (status != SH2_OK) { 
        return false;
    } DEBUG_PRINT("---------------------------------------------------- sh2_open \n");

    memset(&prodIds, 0, sizeof(prodIds));
    status = sh2_getProdIds(&prodIds);
    if (status != SH2_OK) {
        return false;
    } DEBUG_PRINT("------------------------------------------------ sh2_getProdIds \n");

    // Register sensor listener
    sh2_setSensorCallback(sensorHandler, NULL);

    return true;
}

int i2c_open(sh2_Hal_t *self){
    DEBUG_PRINT("************************************ i2c_open \n");
    uint8_t shtp_header[4]; // DS: 1.3.1 SHTP
    uint8_t shtp_cargo[CARGO_SIZE]; 	
    // send a software reset = { Length LSB, Length LSB, Channel, SeqNum, data }
    uint8_t softreset_pkt[5] = { 5, 0, 1, 0, 1}; // DS: Figure 1-27: 1 – reset
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
    uint8_t shtp_header[4]; // DS: 1.3.1 SHTP	
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

bool enableReport(sh2_SensorId_t sensorId, float frequency) {
  uint32_t interval_us = (uint32_t)(1000000 / frequency);
  static sh2_SensorConfig_t config;

  // These sensor options are disabled or not used in most cases
  config.changeSensitivityEnabled = false;
  config.wakeupEnabled = false;
  config.changeSensitivityRelative = false;
  config.alwaysOnEnabled = false;
  config.changeSensitivity = 0;
  config.batchInterval_us = 0;
  config.sensorSpecific = 0;

  config.reportInterval_us = interval_us;
  int status = sh2_setSensorConfig(sensorId, &config);

  if (status != SH2_OK) {
    return false;
  }

  return true;
}

bool getSensorEvent() {

  sensorValue.timestamp = 0;

  sh2_service();

  if (sensorValue.timestamp == 0 && sensorValue.sensorId != SH2_GYRO_INTEGRATED_RV) {
    // no new events
    return false;
  }

  return true;
}

// Handle sensor events.
static void sensorHandler(void *cookie, sh2_SensorEvent_t *event) {

  ret = sh2_decodeSensorEvent(&sensorValue, event);
  if (ret != SH2_OK) {
    printf("BNO08x - Error decoding sensor event");
    sensorValue.timestamp = 0;
    return;
  }
}