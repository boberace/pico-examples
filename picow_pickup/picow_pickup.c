#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#include "mdns_picow.h"

#include "pwp_tcp_server.h"

// i2c for acclelerometer and eeprom
#define I2C_DATA_PIN  0
#define I2C_CLOCK_PIN  1
#define I2C_INST i2c0 
#define I2C_BAUD_RATE 100 * 1000
// op-amp Schmitt trigger input
const uint8_t ST_PINS[] =  {2, 3, 4, 5, 6, 7};
// process loop pins for external mcu
#define PROC_1_PIN  8
#define PROC_2_PIN  9
#define PROC_3_PIN  10
#define PROC_4_PIN  11
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

uint curr_millis = 0;
uint previous_millis = 0;

void setup_i2c();
void setup_st_pins();
void setup_dac();
void setup_adc();

void tcp_server_post_loop(){ 
    // this is the loop that runs after the tcp server is started
    // it is called from the tcp server thread
    // run non time critical functions here
    curr_millis = to_ms_since_boot(get_absolute_time());
    if ((curr_millis - previous_millis) > 500){
        previous_millis = curr_millis;
        gpio_put(SLED_PIN, !gpio_get(SLED_PIN));
    }
}

int main() {
    stdio_init_all();

    // sleep_ms(2000);

    gpio_init(SLED_PIN);
    gpio_set_dir(SLED_PIN, GPIO_OUT);
    gpio_put(SLED_PIN, 1);

    gpio_init(ACC_INT_PIN);
    gpio_set_dir(ACC_INT_PIN, GPIO_IN);

    gpio_init(PERIPH_POWER_PIN);
    gpio_set_dir(PERIPH_POWER_PIN, GPIO_OUT);
    gpio_put(PERIPH_POWER_PIN, 0);

    gpio_init(BAT_HALFVOLTS_PIN);
    gpio_set_dir(BAT_HALFVOLTS_PIN, GPIO_IN);

    setup_i2c();
    setup_st_pins();
    setup_dac();
    setup_adc();


    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        return 1;
    } else {
        printf("Connected.\n");
    }

    mdns_picow_init();

    TCP_SERVER_T *state;
    state->restart_tcp_server = false;
    state->poll_counter = 0;

    run_tcp_server_post(state);        

    cyw43_arch_deinit();
    return 0;
}

void setup_i2c(){
    i2c_init(I2C_INST, I2C_BAUD_RATE);
    gpio_set_function(I2C_DATA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_CLOCK_PIN, GPIO_FUNC_I2C);
}   

void setup_st_pins(){
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


