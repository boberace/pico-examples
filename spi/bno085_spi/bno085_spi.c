/*

Documents referenced in comments:
DS: BNO080/85/86 Data Sheet
RM: SH-2 Reference Manual 
TP: Sensor Hub Transport Protocol 

*/

#include "bno085_spi.h"
#include <inttypes.h>

// #define DEBUG // comment out to disable debug prints                         
#define PRINT_PROBE_UART // comment out to print to serial console

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

spi_inst_t* _SPI_BNO085;

// int16_t _CARGO_SIZE = 256; // arbitrarily set, can be up to (2^15 - 4), 

uint _PIN_BNO085_CS, _PIN_BNO085_RST, _PIN_BNO085_INT;

static sh2_SensorValue_t *_sensor_value = NULL;
static bool _reset_occurred = false;

sh2_Hal_t _sh2_hal; ///< The struct representing the SH2 Hardware Abstraction Layer

bool bno085_connect_spi(spi_inst_t *spi_instance, uint pin_cs, uint pin_rst, uint pin_int){

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

    _SPI_BNO085 = spi_instance;

    // setup sh2 hal pointers
    _sh2_hal.open = spi_open;
    _sh2_hal.close = spi_close;
    _sh2_hal.read = spi_read;
    _sh2_hal.write = spi_write;
    _sh2_hal.getTimeUs = getTimeUs;  


    DEBUG_PRINT("\r\n connect_spi ");
    hal_hardwareReset();
    DEBUG_PRINT("\r\n hal_hardwareReset ");

    int status;
    DEBUG_PRINT("\r\n start sh2_open");
    status = sh2_open(&_sh2_hal, hal_callback, NULL);
    if (status != SH2_OK) { 
        DEBUG_PRINT("\r\n sh2_open failed");
        return false;
    } DEBUG_PRINT("\r\n complete sh2_open ");

    memset(&prodIds, 0, sizeof(prodIds));
    sleep_ms(100);
    status = sh2_getProdIds(&prodIds);
    if (status != SH2_OK) {
        DEBUG_PRINT("\r\n sh2_getProdIds failed");
        return false;
    } 
    DEBUG_PRINT("\r\n sh2_getProdIds completed ");

    // Register sensor listener
    sh2_setSensorCallback(sensorHandler, NULL);

    return true;
}



// 

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

static inline void rst_select() {
    
    asm volatile("nop \n nop \n nop");
    gpio_put(_PIN_BNO085_RST, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void rst_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(_PIN_BNO085_RST, 1);
    asm volatile("nop \n nop \n nop");
}

static int spihal_wait_for_int(void) {
  DEBUG_PRINT("\r\n```_spihal_wait_for_int called");
  for (int i = 0; i < 500; i++) {
    if (!gpio_get(_PIN_BNO085_INT)){
        DEBUG_PRINT("\r\n~~~_spihal_wait_for_int INT triggered");
        return true;
    }
    DEBUG_PRINT(".");
    sleep_ms(1);
  }
  DEBUG_PRINT("\r\n --- _spihal_wait_for_int Timed out!");
  hal_hardwareReset();

  return false;
}

static int spi_open(sh2_Hal_t *self){
    DEBUG_PRINT("\r\n``` spi_open started");

    spihal_wait_for_int();

    DEBUG_PRINT("\r\n~~~ spi_open completed");
    return 0;
}

static void spi_close(sh2_Hal_t *self){
    DEBUG_PRINT("\r\n``` spi_close started");

    DEBUG_PRINT("\r\n~~~ spi_close completed");
}

static int spi_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us){
    DEBUG_PRINT("\r\n``` spi_read requested started len %d ", len);

    uint8_t shtp_header[4]; // DS: 1.3.1 SHTP	

    if (!spihal_wait_for_int()) {
        DEBUG_PRINT("\r\n---return 0 early  spi_read ::  spihal_wait_for_int timed out!");
        return 0;
    }

    *t_us = (uint32_t)to_us_since_boot(get_absolute_time());

    cs_select();
    int spi_ret = spi_read_blocking(_SPI_BNO085, 0, shtp_header, 4);
    cs_deselect();
        DEBUG_PRINT("\r\nspi_read_blocking shtp_header spi_ret %d", spi_ret);
    if(spi_ret != 4) {
        DEBUG_PRINT("\r\n---return 0 early  spi_read ::  spi_read_blocking shtp_header spi_ret != 4:  %d", spi_ret);
      return 0;
    }

    uint16_t length = (shtp_header[1] << 8) |  shtp_header[0];
    length &= 0x7FFF;
        DEBUG_PRINT("\r\nshtp_header 0 : 0x%02x, 0b%08b", shtp_header[0], shtp_header[0]);
        DEBUG_PRINT("\r\nshtp_header 1 : 0x%02x, 0b%08b", shtp_header[1], shtp_header[1]);
        DEBUG_PRINT("\r\nshtp_header 2 : 0x%02x, 0b%08b", shtp_header[2], shtp_header[2]);
        DEBUG_PRINT("\r\nshtp_header 3 : 0x%02x, 0b%08b", shtp_header[3], shtp_header[3]);
        DEBUG_PRINT("\r\nlength %d", length);

    if(length >  len){
        DEBUG_PRINT("\r\n---return 0 early  spi_read ::  shtp_header length: %d > requested len: %d ", length, len);
        return 0;
    } 

    if (!spihal_wait_for_int()) {
        DEBUG_PRINT("\r\n---return 0 early  spi_read ::  spihal_wait_for_int timed out == 0");
        return 0;
    }

    cs_select();
    spi_ret = spi_read_blocking(_SPI_BNO085, 0, pBuffer, length);   
    cs_deselect();   
    DEBUG_PRINT("\r\nspi_read_blocking pBuffer spi_ret %d", spi_ret);


    if(spi_ret != (int)length){
        DEBUG_PRINT("\r\n---return 0 early  spi_read ::  spi_read_blocking spi_ret length: %d != length: %d ", spi_ret, length);
        return 0;
    }

    DEBUG_PRINT("\r\n~~~ spi_read requested completed len %d ", len);
  
    return spi_ret;
}

static int spi_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len){
    DEBUG_PRINT("\r\n``` spi_write started len %d ", len);
    // uint16_t length = (len > SH2_HAL_MAX_TRANSFER_OUT)?SH2_HAL_MAX_TRANSFER_OUT:len;

    for(uint i = 0; i < len; i++){
        DEBUG_PRINT("\r\npBuffer[%d] 0x%02x, 0b%08b", i, pBuffer[i], pBuffer[i]);
    }

    if (!spihal_wait_for_int()) {
        DEBUG_PRINT("\r\n---return 0 early  spi_write :: spihal_wait_for_int timed out!");
        return 0;
    }

    cs_select();
    int spi_ret = spi_write_blocking(_SPI_BNO085, pBuffer, len);
    cs_deselect();
        DEBUG_PRINT("\r\n*** spi_write_blocking spi_ret: %d", spi_ret);
    if(spi_ret != (int)len) {
        DEBUG_PRINT("\r\n---return 0 early  spi_write :: spi_write_blocking spi_ret length: %d  != requested len: %d ", len, spi_ret);
      return 0;
    }
    DEBUG_PRINT("\r\n~~~ spi_write completed len %d ", len);
    return spi_ret;
}

uint32_t getTimeUs(sh2_Hal_t *self){
    uint64_t t = to_us_since_boot(get_absolute_time());
    DEBUG_PRINT("\r\ngetTimeUs uint64_t: %" PRIu64, t);
    DEBUG_PRINT("\r\ngetTimeUs uint32_t: %d", (uint32_t)t);
    return t;
}


// Handle sensor events.
static void sensorHandler(void *cookie, sh2_SensorEvent_t *event) {

  int sh2_ret = sh2_decodeSensorEvent(_sensor_value, event);
  if (sh2_ret != SH2_OK) {
    DEBUG_PRINT("\r\n--- BNO08x - Error decoding sensor event ");
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

