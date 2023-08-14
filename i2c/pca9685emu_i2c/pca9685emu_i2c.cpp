
#include <hardware/i2c.h>
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include <pico/i2c_slave.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>
// #include "FreeRTOS.h"
// #include "semphr.h"
#include "../../cindex/cindex.h"

static const uint8_t LED_PIN = 25;
static const uint8_t OE_PIN = 22;
static const uint8_t PWM_PINS[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};

static const uint num_mem_bytes = 256;

static const uint8_t I2C_SLAVE_ADDRESS = 0x40;
static const uint8_t I2C_SLAVE_SDA_PIN = 18; // 4
static const uint8_t I2C_SLAVE_SCL_PIN = 19; // 5
uint I2C_BAUDRATE = 100000; // 100 kHz
i2c_inst_t* I2CC = i2c1;

// REGISTER ADDRESSES
static const uint8_t PCA9685_MODE1 = 0x00;      /**< Mode Register 1 */
static const uint8_t PCA9685_MODE2 = 0x01;      /**< Mode Register 2 */
static const uint8_t PCA9685_SUBADR1 = 0x02;    /**< I2C-bus subaddress 1 */
static const uint8_t PCA9685_SUBADR2 = 0x03;    /**< I2C-bus subaddress 2 */
static const uint8_t PCA9685_SUBADR3 = 0x04;    /**< I2C-bus subaddress 3 */
static const uint8_t PCA9685_ALLCALLADR = 0x05; /**< LED All Call I2C-bus address */
static const uint8_t PCA9685_LED0_ON_L = 0x06;  /**< LED0 on tick, low byte*/
static const uint8_t PCA9685_LED0_ON_H = 0x07;  /**< LED0 on tick, high byte*/
static const uint8_t PCA9685_LED0_OFF_L = 0x08; /**< LED0 off tick, low byte */
static const uint8_t PCA9685_LED0_OFF_H = 0x09; /**< LED0 off tick, high byte */
// etc all 16:  LED15_OFF_H 0x45
static const uint8_t PCA9685_ALLLED_ON_L = 0xFA;  /**< load all the LEDn_ON registers, low */
static const uint8_t PCA9685_ALLLED_ON_H = 0xFB;  /**< load all the LEDn_ON registers, high */
static const uint8_t PCA9685_ALLLED_OFF_L = 0xFC; /**< load all the LEDn_OFF registers, low */
static const uint8_t PCA9685_ALLLED_OFF_H = 0xFD; /**< load all the LEDn_OFF registers,high */
static const uint8_t PCA9685_PRESCALE = 0xFE;     /**< Prescaler for PWM output frequency */
static const uint8_t PCA9685_TESTMODE = 0xFF;     /**< defines the test mode to be entered */

// MODE1 bits
static const uint8_t MODE1_ALLCAL = 0x01;  /**< respond to LED All Call I2C-bus address */
static const uint8_t MODE1_SUB3 = 0x02;    /**< respond to I2C-bus subaddress 3 */
static const uint8_t MODE1_SUB2 = 0x04;    /**< respond to I2C-bus subaddress 2 */
static const uint8_t MODE1_SUB1 = 0x08;    /**< respond to I2C-bus subaddress 1 */
static const uint8_t MODE1_SLEEP = 0x10;   /**< Low power mode. Oscillator off */
static const uint8_t MODE1_AI = 0x20 ;     /**< Auto-Increment enabled */
static const uint8_t MODE1_EXTCLK = 0x40;  /**< Use EXTCLK pin clock */
static const uint8_t MODE1_RESTART = 0x80; /**< Restart enabled */
// MODE2 bits
static const uint8_t MODE2_OUTNE_0 = 0x01; /**< Active LOW output enable input */
static const uint8_t MODE2_OUTNE_1 = 0x02; /**< Active LOW output enable input - high impedience */
static const uint8_t MODE2_OUTDRV = 0x04; /**< totem pole structure vs open-drain */
static const uint8_t MODE2_OCH = 0x08;    /**< Outputs change on ACK vs STOP */
static const uint8_t MODE2_INVRT = 0x10;  /**< Output logic state inverted */

static const uint8_t PCA9685_I2C_ADDRESS = 0x40;      /**< Default PCA9685 I2C Slave Address */

static const uint8_t PCA9685_PRESCALE_MIN = 3;   /**< minimum prescale value */
static const uint8_t PCA9685_PRESCALE_MAX = 255; /**< maximum prescale value */

static struct
{
    uint8_t mem[num_mem_bytes] = {0};
    cindex mem_address = num_mem_bytes;
    bool mem_address_written;
} pca;

void initial_pca_mem(){
    pca.mem[PCA9685_MODE1] = 0b00010001;
    pca.mem[PCA9685_MODE2] = 0b00000010;
    pca.mem[PCA9685_SUBADR1] = 0b11100010;
    pca.mem[PCA9685_SUBADR2] = 0b11100100;
    pca.mem[PCA9685_SUBADR3] = 0b11101000;
    pca.mem[PCA9685_ALLCALLADR] = 0b11100000;
    pca.mem[PCA9685_PRESCALE] = 30; // 7.3.5 PWM frequency PRE_SCALE
}

static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
        case I2C_SLAVE_RECEIVE: // master has written some data
            if (!pca.mem_address_written) {
                pca.mem_address = static_cast<unsigned int>(i2c_read_byte_raw(i2c));
                pca.mem_address_written = true;

            } else {
                pca.mem[pca.mem_address] = i2c_read_byte_raw(i2c);

                switch (pca.mem_address) {
                    case PCA9685_MODE1:

                        break;
                    case PCA9685_MODE2:

                        break;                    
                    case PCA9685_SUBADR1:

                        break;
                    case PCA9685_SUBADR2:

                        break; 
                    case PCA9685_SUBADR3:

                        break;            
                    case PCA9685_ALLCALLADR:

                        break;     
                    case PCA9685_PRESCALE:

                        break;
                    case PCA9685_TESTMODE:

                        break;
                    default: // led bytes
                        uint lad = pca.mem_address; // led address
                        uint ladofs = lad - 6; // led address offsetted
                        if(ladofs % 4 == 3){ // at last of four bytes ( high byte of on)
                            uint lon = pca.mem[lad-3] + (pca.mem[lad-2] << 8);
                            uint loff = pca.mem[lad-1] + (pca.mem[lad-0] << 8);
                            uint ldel = loff - lon;
                            uint led = ladofs >> 2;                            
                            uint PWM_PIN = PWM_PINS[led];
                            uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);  
                            uint level = (lon > 4095)?  0 : ldel; // if lon is greater than 4095 then turn off
                            pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWM_PIN), level);
                        }

                        break;                   

                }

                pca.mem_address++; // auto-increment always enabled

            }
            break;
        case I2C_SLAVE_REQUEST: // master is requesting data
            // load from memory
            i2c_write_byte_raw(i2c, pca.mem[pca.mem_address]);
            pca.mem_address++;
            break;
        case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
            pca.mem_address_written = false;
            break;
        default:
            break;
    }
}

static void setup_slave() {
    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);

    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    i2c_init(I2CC, I2C_BAUDRATE);
    i2c_slave_init(I2CC, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
}

static void setup_pwm(){
    uint FREQUENCY_OSCILLATOR = 125000000;// clock_get_hz(clock_index(core_clock));
    float pca_div_fac = FREQUENCY_OSCILLATOR/25000000;
    uint length = sizeof(PWM_PINS) / sizeof(PWM_PINS[0]);
    uint top = 4096; //12 bit same as pca9685
    uint div_int = pca_div_fac*pca.mem[PCA9685_PRESCALE]; // the pca9685 prescale value is scaled so the expected refresh is close
    uint div_frac = 0;
    for(uint i = 0; i < length; i++){
        uint PWM_PIN = PWM_PINS[i];
        gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
        pwm_set_clkdiv_int_frac(slice_num, div_int, div_frac);
        pwm_set_wrap(slice_num, top);
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWM_PIN), 0);
        pwm_set_enabled(slice_num, true);
    }    
}

static void update_pwm(uint8_t pin, uint level){
        uint slice_num = pwm_gpio_to_slice_num(pin);
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(pin), level);
        pwm_set_enabled(slice_num, true);
   
}


int main() {
    stdio_init_all();
    setup_slave();
    initial_pca_mem();
    setup_pwm();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    uint32_t pus = time_us_32();
    while(1){
        uint32_t cus = time_us_32();
        if(cus - pus >= 500000){
            pus=cus;
            gpio_put(LED_PIN,!gpio_get(LED_PIN));
        }
    }

}
