/*

Documents referenced in comments:
DS: BNO080/85/86 Data Sheet
RM: SH-2 Reference Manual v1.2
TP: Sensor Hub Transport Protocol R 1.7

*/

#include "bno085_i2c.hpp"

// uncomment to print debug information over usb serial
// #define DEBUG
#define PRINT_PROBE_UART

#ifdef DEBUG
#define UART_ID uart1
#ifdef PRINT_PROBE_UART
#define BUFFER_SIZE 256
#define DEBUG_PRINT(...)                     \
    do {                               \
        char buffer[BUFFER_SIZE];      \
        snprintf(buffer, sizeof(buffer), __VA_ARGS__); \
        uart_puts(UART_ID, buffer);      \
    } while (0)
#else
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#endif
#else
#define DEBUG_PRINT(...)
#endif

i2c_inst_t *_I2C_BNO085;

int16_t _ret = 0;
// int16_t _CARGO_SIZE = 128; // arbitrarily set, can be up to (2^15 - 4), 

uint _PIN_BNO085_RST, _PIN_BNO085_INT;
uint8_t _BNO085_I2C_ADDR;

static sh2_SensorValue_t *_sensor_value = NULL;
static bool _reset_occurred = false;

bno085_i2c::bno085_i2c(uint pin_rst, uint pin_int){

    _PIN_BNO085_RST = pin_rst;
    _PIN_BNO085_INT = pin_int;

    if(_PIN_BNO085_RST != 0xFF){
        gpio_init(_PIN_BNO085_RST);
        gpio_set_dir(_PIN_BNO085_RST, GPIO_OUT);
        gpio_put(_PIN_BNO085_RST, 1);
    }
    
    if(_PIN_BNO085_INT != 0xFF){   
        gpio_init(_PIN_BNO085_INT);
        gpio_set_dir(_PIN_BNO085_INT, GPIO_IN);
        gpio_pull_up(_PIN_BNO085_INT);
    }

}

bool bno085_i2c::connect_i2c(i2c_inst_t *i2c_instance, uint8_t i2c_address){

    _I2C_BNO085 = i2c_instance;
    _BNO085_I2C_ADDR = i2c_address;

    // read a single byte to determine if connected - 
    // this could cause a full report to generate that would have to be flushed
    // but no matter as this is followed with a hardware reset
    uint8_t dummy;
    uint16_t _ret = i2c_read_blocking(_I2C_BNO085, _BNO085_I2C_ADDR, &dummy, 1, false);
    if(_ret < 1){
         DEBUG_PRINT(" BNO085 will not connect\r\n");
         return false;
    } DEBUG_PRINT("i2c_read_blocking dummy test is connected %d\r\n", _ret);

    if(hal_reset() == 0){
        DEBUG_PRINT("---return 0 early BNO085 reset failed\r\n");
        return false;
    }  

    // setup sh2 hal pointers
    _sh2_hal.open = i2c_open;
    _sh2_hal.close = i2c_close;
    _sh2_hal.read = i2c_read;
    _sh2_hal.write = i2c_write;
    _sh2_hal.getTimeUs = getTimeUs;  

    //
    int status;
    status = sh2_open(&_sh2_hal, hal_callback, NULL);
    if (status != SH2_OK) { 
        return false;
    } DEBUG_PRINT("---------------------------------------------------- sh2_open \r\n");

    memset(&prodIds, 0, sizeof(prodIds));
    status = sh2_getProdIds(&prodIds);
    if (status != SH2_OK) {
        return false;
    } DEBUG_PRINT("------------------------------------------------ sh2_getProdIds \r\n");

    // Register sensor listener
    sh2_setSensorCallback(sensorHandler, NULL);

    return true;
}

bool bno085_i2c::enableReport(sh2_SensorId_t sensorId, float frequency) {

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

bool bno085_i2c::getSensorEvent(sh2_SensorValue_t *value) {

  _sensor_value = value;

  _sensor_value->timestamp = 0;

  sh2_service();

  if (_sensor_value->timestamp == 0 && _sensor_value->sensorId != SH2_GYRO_INTEGRATED_RV) {
    // no new events
    return false;
  }

  return true;
}

bool bno085_i2c::wasReset(void) {
  bool x = _reset_occurred;
  _reset_occurred = false;

  return x;
}

// -------------------------------------------------------------------------

static int hal_wait_for_int(void) {
  DEBUG_PRINT("```hal_wait_for_int called\r\n");
  for (int i = 0; i < 500; i++) {
    if (!gpio_get(_PIN_BNO085_INT)){
        DEBUG_PRINT("~~~hal_wait_for_int INT triggered\r\n");
        return true;
    }
    DEBUG_PRINT(".");
    sleep_ms(1);
  }
  DEBUG_PRINT("--- hal_wait_for_int Timed out!\r\n");
  hal_reset();

  return false;
}

static int i2c_open(sh2_Hal_t *self){
    DEBUG_PRINT("************************************ i2c_open \r\n"); 

   if(hal_reset() == 0){
        DEBUG_PRINT("---return 0 early BNO085 reset failed\r\n");
        return 1;
    }  
    return 0;
}

static void i2c_close(sh2_Hal_t *self){
    DEBUG_PRINT("************************************ i2c_close \r\n");
}

static int i2c_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us){
    DEBUG_PRINT("************************************ i2c_read %i \r\n", len);

    *t_us = to_us_since_boot(get_absolute_time());

    uint8_t shtp_header[4]; // DS: 1.3.1 SHTP	

    if(_PIN_BNO085_INT != 0xFF){ 
        if (!hal_wait_for_int()) {
            DEBUG_PRINT("---return 0 early  i2c_read ::  hal_wait_for_int timed out!\r\n");
            return 0;
        }
    }
    _ret = i2c_read_blocking(_I2C_BNO085,_BNO085_I2C_ADDR, shtp_header, 4, false );        
    if(_ret != 4){
        DEBUG_PRINT("---return 0 early i2c_read i2c_read_blocking shtp_header 4 != %i\r\n", _ret);
        return 0;
    }
    uint16_t length = (shtp_header[1] << 8) |  shtp_header[0];
    length &= 0x7FFF;
        DEBUG_PRINT("shtp_header 0 : 0x%02x, 0b%08b\r\n", shtp_header[0], shtp_header[0]);
        DEBUG_PRINT("shtp_header 1 : 0x%02x, 0b%08b\r\n", shtp_header[1], shtp_header[1]);
        DEBUG_PRINT("shtp_header 2 : 0x%02x, 0b%08b\r\n", shtp_header[2], shtp_header[2]);
        DEBUG_PRINT("shtp_header 3 : 0x%02x, 0b%08b\r\n", shtp_header[3], shtp_header[3]);
        DEBUG_PRINT("length %d\r\n", length);

    if(length > len){
        DEBUG_PRINT("---return 0 early shtp_header length %i > len %i\r\n", length, len);
        return 0;
    }

    if(_PIN_BNO085_INT != 0xFF){ 
        if (!hal_wait_for_int()) {
            DEBUG_PRINT("---return 0 early  i2c_read ::  hal_wait_for_int timed out!\r\n");
            return 0;
        }
    }

    _ret = i2c_read_blocking(_I2C_BNO085,_BNO085_I2C_ADDR, pBuffer, length, false);             
    if(_ret != length) {
        DEBUG_PRINT("---return 0 early i2c_read i2c_read_blocking pBuffer %i != %i\r\n", _ret, length);
        return 0;
    }

    return length;
}

static int i2c_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len){
    DEBUG_PRINT("************************************ i2c_write \r\n");
    uint16_t length = (len > SH2_HAL_MAX_TRANSFER_OUT)?SH2_HAL_MAX_TRANSFER_OUT:len;

    _ret = i2c_write_blocking(_I2C_BNO085,_BNO085_I2C_ADDR, pBuffer, length, false );
        DEBUG_PRINT("i2c_write_blocking softreset_pkt %d\r\n", _ret);
    if(_ret != length) return 0;

    return _ret;
}

uint32_t getTimeUs(sh2_Hal_t *self){
    DEBUG_PRINT("************************************ getTimeUs \r\n");
    return to_us_since_boot(get_absolute_time());
}


// Handle sensor events.
static void sensorHandler(void *cookie, sh2_SensorEvent_t *event) {

  _ret = sh2_decodeSensorEvent(_sensor_value, event);
  if (_ret != SH2_OK) {
    DEBUG_PRINT("BNO08x - Error decoding sensor event\r\n");
    _sensor_value->timestamp = 0;
    return;
  }
}

static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
  // If we see a reset, set a flag so that sensors will be reconfigured.
  if (pEvent->eventId == SH2_RESET) {
    // printfln("Reset!");
    _reset_occurred = true;
  }
}

static void hal_hardwareReset(void) {
    DEBUG_PRINT("--- hal_softwareReset called \r\n");
    if (_PIN_BNO085_RST != 0xFF) {
        gpio_put(_PIN_BNO085_RST, 1);
        sleep_ms(10);
        gpio_put(_PIN_BNO085_RST, 0);
        sleep_ms(10);
        gpio_put(_PIN_BNO085_RST, 1);
        sleep_ms(10);  
    }
}

static int hal_softwareReset(void){

    uint8_t softreset_pkt[5] = { 5, 0, 1, 0, 1}; // DS: Figure 1-27: 1 – reset
    for(int i = 0; i < 5; i++){
        _ret = i2c_write_blocking(_I2C_BNO085,_BNO085_I2C_ADDR, softreset_pkt, 5, false );         
        if(_ret == 5){
            DEBUG_PRINT("--- hal_softwareReset occured \r\n");
            return 1;
        } 
    }
    DEBUG_PRINT("---return 0 early hal_softwareReset failed %i\r\n", _ret);
    return 0;    
}

static int hal_reset(void) {
  if (_PIN_BNO085_RST != 0xFF) {
    hal_hardwareReset();
    return 1;
  } else {
    return hal_softwareReset();
  }
}
