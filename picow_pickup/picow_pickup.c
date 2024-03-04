#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"

#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#include "mdns_picow.h"

#include "pwp_tcp_server.h"

#define DEBUG       // uncomment to print debug messages
#define DEBUG_UART  // uncomment to print debug messages to uart else debug to usb serial

// i2c for acclelerometer and eeprom
#define I2C_DATA_PIN  0
#define I2C_CLOCK_PIN  1
#define I2C_INST i2c0 
#define I2C_BAUD_RATE 100 * 1000
// op-amp Schmitt trigger input
const uint8_t ST_PINS[] =  {2, 3, 4, 5, 6, 7};
// process loop pins for external mcu
#ifndef DEBUG_UART
#define PROC_1_PIN  8
#define PROC_2_PIN  9
#define PROC_3_PIN  10
#define PROC_4_PIN  11
#else
#define UART_A_ID uart1
#define UART_A_BAUD_RATE 115200
#define UART_A_TX_PIN 8
#define UART_A_RX_PIN 9
#endif
// status led
#define SLED_PIN  12
// DAC pins
#define DAC_BITCLK_PIN  13
#define DAC_DATA_PIN  14
#define DAC_WORDCLK_PIN  15
// ADC pins
#define ADC_DATAIN_PIN  16
#define ADC_CS_PIN  17
#define ADC_CLK_PIN  18
#define ADC_DATAOUT_PIN  19
// accelerometer interrupt pin
#define ACC_INT_PIN  20
// power control pin for peripherals on seperate power supply
#define PERIPH_POWER_PIN  21
// shift register pins
#define SR_SHIFT_PIN  22
#define SR_LATCH_PIN  26
#define SR_DATA_PIN  27
// battery voltage monitor
#define BAT_HALFVOLTS_PIN  28


uint curr_sled_millis = 0;
uint previous_sled_millis = 0;
uint curr_fb_millis = 0;
uint previous_fb_millis = 0;
uint ret = 0;

void DEBUG_PRINT(const char* format, ...);
uint setup_uart();
void setup_sled();
void setup_acc();
void setup_i2c();
void setup_striggers();
void setup_dac();
void setup_adc();
void setup_sr();
void setup_bat_mon();
void setup_periph_power();
void update_fbc(uint16_t * state);

void core1_entry() {

    float freq = 329.63;
    uint uperiod = round(1000000.0 / freq);
    uint uhperiod = round(1000000.0 / freq / 2);

    uint16_t state1 = 0b100000000000;
    uint16_t state2 = 0b010000000000;
    uint ff = 0;
    
    DEBUG_PRINT("uperiod %d\r\n", uperiod);
    gpio_put(PERIPH_POWER_PIN, 0);

    while (1) {
        curr_sled_millis = to_ms_since_boot(get_absolute_time());
        if ((curr_sled_millis - previous_sled_millis) > 500){
            previous_sled_millis = curr_sled_millis;
            gpio_put(SLED_PIN, !gpio_get(SLED_PIN));
        }

        // curr_fb_millis = to_us_since_boot(get_absolute_time());
        // if ((curr_fb_millis - previous_fb_millis) > uhperiod){
        //     previous_fb_millis = curr_fb_millis;
        //     ff = !ff;
        //     update_fbc(ff?&state1:&state2);
        // }

    }
}

// core0 loop - run non time critical functions here
void tcp_server_post_loop(){ 
    // this is the loop that runs after the tcp server is started
    // it is called from the tcp server thread

}

// main only used for setup - loop in tcp_server_post_loop
int main() {
    stdio_init_all();
    // sleep_ms(2000);
    #ifdef DEBUG_UART
    ret = setup_uart();
    #endif
    setup_sled();
    setup_acc();
    setup_i2c();
    setup_striggers();
    setup_dac();
    setup_adc();
    setup_sr();
    setup_bat_mon();
    setup_periph_power();

    DEBUG_PRINT("DEBUG uart setup %d\r\n", ret);

    if (cyw43_arch_init()) {
        DEBUG_PRINT("failed to initialise\r\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    DEBUG_PRINT("Connecting to Wi-Fi...\r\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        DEBUG_PRINT("failed to connect.\r\n");
        return 1;
    } else {
        DEBUG_PRINT("Connected.\r\n");
    }

    multicore_launch_core1(core1_entry);

    mdns_picow_init();

    TCP_SERVER_T *state;
    state->restart_tcp_server = false;
    state->poll_counter = 0;    

    run_tcp_server_post(state);        

    cyw43_arch_deinit();
    return 0;
}

uint setup_uart() {
    uint uart_ret = uart_init(UART_A_ID, UART_A_BAUD_RATE);
    gpio_set_function(UART_A_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_A_RX_PIN, GPIO_FUNC_UART);
    return uart_ret;
}


#ifdef DEBUG
    #ifdef DEBUG_UART
    #define BUFFER_SIZE 256
    void DEBUG_PRINT(const char* format, ...) {
        char buffer[BUFFER_SIZE];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        uart_puts(UART_A_ID, buffer);
        va_end(args);
    }    
    #else
    void DEBUG_PRINT(const char* format, ...) {
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
    }
    #endif
#else
void DEBUG_PRINT(const char* format, ...) {
}
#endif

void setup_sled(){
    gpio_init(SLED_PIN);
    gpio_set_dir(SLED_PIN, GPIO_OUT);
    gpio_put(SLED_PIN, 1);
}

void setup_acc(){
    gpio_init(ACC_INT_PIN);
    gpio_set_dir(ACC_INT_PIN, GPIO_IN);
}

void setup_i2c(){
    i2c_init(I2C_INST, I2C_BAUD_RATE);
    gpio_set_function(I2C_DATA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_CLOCK_PIN, GPIO_FUNC_I2C);
}   

void setup_striggers(){
    for (int i = 0; i < sizeof(ST_PINS); i++){
        gpio_init(ST_PINS[i]);
        gpio_set_dir(ST_PINS[i], GPIO_IN);
    }
}

void setup_dac(){
    uint8_t dac_pins[] = {DAC_BITCLK_PIN, DAC_DATA_PIN, DAC_WORDCLK_PIN};
    for (int i = 0; i < sizeof(dac_pins); i++){
        gpio_init(dac_pins[i]);
        gpio_set_dir(dac_pins[i], GPIO_OUT);
        gpio_put(dac_pins[i], 0);
    }
}

void setup_adc(){
    uint8_t adc_out_pins[] = {ADC_DATAIN_PIN, ADC_CLK_PIN, ADC_DATAOUT_PIN};
    for (int i = 0; i < sizeof(adc_out_pins); i++){
        gpio_init(adc_out_pins[i]);
        gpio_set_dir(adc_out_pins[i], GPIO_OUT);
        gpio_put(adc_out_pins[i], 0);
    }
        gpio_init(ADC_DATAOUT_PIN);
        gpio_set_dir(ADC_DATAOUT_PIN, GPIO_IN);
}

void setup_sr(){
    // two shift registers daisy chained for 12 outputs (4 not used)
    // used to control current direction in feedback coils
    uint8_t sr_pins[] = {SR_SHIFT_PIN, SR_LATCH_PIN, SR_DATA_PIN};
    for (int i = 0; i < sizeof(sr_pins); i++){
        gpio_init(sr_pins[i]);
        gpio_set_dir(sr_pins[i], GPIO_OUT);
        gpio_put(sr_pins[i], 0);
    }
}

void setup_bat_mon(){
    gpio_init(BAT_HALFVOLTS_PIN);
    gpio_set_dir(BAT_HALFVOLTS_PIN, GPIO_IN);
}

void setup_periph_power(){
    gpio_init(PERIPH_POWER_PIN);
    gpio_set_dir(PERIPH_POWER_PIN, GPIO_OUT);
    gpio_put(PERIPH_POWER_PIN, 0);
}

void update_fbc(uint16_t * state){
    // update the feedback coils on the shift registers with the new 12 bit state

    // shift in the new state
    uint16_t mask = 0b1;
    for (int i = 0; i < 12; i++){        
        gpio_put(SR_DATA_PIN, *state & mask);
        sleep_us(1);
        gpio_put(SR_SHIFT_PIN, 1);
        sleep_us(1);
        gpio_put(SR_SHIFT_PIN, 0);
        mask = mask << 1;
    }
    // latch the new state
    gpio_put(SR_LATCH_PIN, 1);
    sleep_us(1);
    gpio_put(SR_LATCH_PIN, 0);

}
