/*

Documents referenced in comments:
DS: BNO080/85/86 Data Sheet
RM: SH-2 Reference Manual v1.2
TP: Sensor Hub Transport Protocol R 1.7

*/

#include "bno085_spi.h"

// uncomment to print debug information over usb serial
#define DEBUG

#ifdef DEBUG
# define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
# define DEBUG_PRINT(...)
#endif

spi_inst_t *_SPI_BNO085;

int16_t _ret = 0;
int16_t _CARGO_SIZE = 256; // arbitrarily set, can be up to (2^15 - 4), 

uint _PIN_BNO085_CS, _PIN_BNO085_RST, _PIN_BNO085_INT;


static sh2_SensorValue_t *_sensor_value = NULL;
static bool _reset_occurred = false;

bno085_spi::bno085_spi(uint pin_cs, uint pin_rst, uint pin_int){
    _PIN_BNO085_CS = pin_cs;
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

    if(_PIN_BNO085_CS != 0xFF){   
        gpio_init(_PIN_BNO085_CS);
        gpio_set_dir(_PIN_BNO085_CS, GPIO_OUT);
        gpio_put(_PIN_BNO085_CS, 1);
    }
}

bool bno085_spi::connect_spi(spi_inst_t *spi_instance){
    DEBUG_PRINT("---------------------------------------------------- connect_spi \n");
    hal_hardwareReset();
    DEBUG_PRINT("---------------------------------------------------- hal_hardwareReset \n");
    //
    int status;
    status = sh2_open(&_sh2_hal, hal_callback, NULL);
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

    _SPI_BNO085 = spi_instance;

    // setup sh2 hal pointers
    _sh2_hal.open = spi_open;
    _sh2_hal.close = spi_close;
    _sh2_hal.read = spi_read;
    _sh2_hal.write = spi_write;
    _sh2_hal.getTimeUs = getTimeUs;  

    return true;
}

bool bno085_spi::enableReport(sh2_SensorId_t sensorId, float frequency) {

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

bool bno085_spi::getSensorEvent(sh2_SensorValue_t *value) {

  _sensor_value = value;

  _sensor_value->timestamp = 0;

  sh2_service();

  if (_sensor_value->timestamp == 0 && _sensor_value->sensorId != SH2_GYRO_INTEGRATED_RV) {
    // no new events
    return false;
  }

  return true;
}

bool bno085_spi::wasReset(void) {
  bool x = _reset_occurred;
  _reset_occurred = false;

  return x;
}
// -------------------------------------------------------------------------

static inline void cs_select() {
    
    asm volatile("nop \n nop \n nop");
    gpio_put(_PIN_BNO085_CS, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(_PIN_BNO085_CS, 1);
    asm volatile("nop \n nop \n nop");
}

static int spihal_wait_for_int(void) {
  for (int i = 0; i < 500; i++) {
    if (!gpio_get(_PIN_BNO085_INT))
      return true;
    DEBUG_PRINT(".");
    sleep_us(1);
  }
  DEBUG_PRINT("_spihal_wait_for_int Timed out!");
  hal_hardwareReset();

  return false;
}

static int spi_open(sh2_Hal_t *self){
    DEBUG_PRINT("************************************ spi_open \n");
    spihal_wait_for_int();
    return 0;
}

static void spi_close(sh2_Hal_t *self){
    DEBUG_PRINT("************************************ spi_close \n");
}

static int spi_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us){
    DEBUG_PRINT("************************************ spi_read \n");

    uint8_t shtp_header[4]; // DS: 1.3.1 SHTP	
    // _ret = i2c_read_blocking(_I2C_BNO085,_BNO085_I2C_ADDR, shtp_header, 4, false );

    if (!spihal_wait_for_int()) {
        return 0;
    }

    cs_select();
    _ret = spi_read_blocking(_SPI_BNO085, 0, shtp_header, 4);
    cs_deselect();
        DEBUG_PRINT("i2c_read_blocking shtp_header %d\n", _ret);
    if(_ret != 4) return 0;

    uint16_t length = (shtp_header[1] << 8) |  shtp_header[0];
    length &= ~(1 << 15);
        DEBUG_PRINT("shtp_header 0 : 0x%02x, 0b%08b\n", shtp_header[0], shtp_header[0]);
        DEBUG_PRINT("shtp_header 1 : 0x%02x, 0b%08b\n", shtp_header[1], shtp_header[1]);
        DEBUG_PRINT("shtp_header 2 : 0x%02x, 0b%08b\n", shtp_header[2], shtp_header[2]);
        DEBUG_PRINT("shtp_header 3 : 0x%02x, 0b%08b\n", shtp_header[3], shtp_header[3]);
        DEBUG_PRINT("length %d\n", length);

    if(length >  len){
        DEBUG_PRINT("length >  len\n");
        return 0;
    }

    if (!spihal_wait_for_int()) {
        return 0;
    }

    cs_select();
    _ret = spi_read_blocking(_SPI_BNO085, 0, pBuffer, length);   
    cs_deselect();   

    if(_ret == 0){
        DEBUG_PRINT("spi_read_blocking pBuffer %d\n", _ret);
        return 0;
    }
  
     *t_us = to_us_since_boot(get_absolute_time());

    return length;
}

static int spi_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len){
    DEBUG_PRINT("************************************ spi_write \n");
    uint16_t length = (len > SH2_HAL_MAX_TRANSFER_OUT)?SH2_HAL_MAX_TRANSFER_OUT:len;

    if (!spihal_wait_for_int()) {
        return 0;
    }

    // _ret = i2c_write_blocking(_I2C_BNO085,_BNO085_I2C_ADDR, pBuffer, length, false );
    cs_select();
    _ret = spi_write_blocking(_SPI_BNO085, pBuffer, length);
    cs_deselect();
        DEBUG_PRINT("i2c_write_blocking softreset_pkt %d\n", _ret);
    if(_ret != length) return 0;

    return _ret;
}

uint32_t getTimeUs(sh2_Hal_t *self){
    DEBUG_PRINT("************************************ getTimeUs \n");
    return to_us_since_boot(get_absolute_time());
}


// Handle sensor events.
static void sensorHandler(void *cookie, sh2_SensorEvent_t *event) {

  _ret = sh2_decodeSensorEvent(_sensor_value, event);
  if (_ret != SH2_OK) {
    printf("BNO08x - Error decoding sensor event");
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
  if (_PIN_BNO085_RST != 0xFF) {
    gpio_put(_PIN_BNO085_RST, 1);
    sleep_ms(10);
    gpio_put(_PIN_BNO085_RST, 0);
    sleep_ms(10);
    gpio_put(_PIN_BNO085_RST, 1);
    sleep_ms(10);  
  }
}

