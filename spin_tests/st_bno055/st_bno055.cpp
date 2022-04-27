#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"

#include "ssd1306.h"
#include "hall.pio.h"

#define PIN_STP_DIR 2 
#define PIN_STP_PWM 3 
#define PIN_STP_ENA 4
#define PIN_LEDB 5
#define PIN_LEDG 6
#define PIN_LEDR 7
#define PIN_HALL 8
#define PIN_LED 9
#define PIN_I2C_SDA 16
#define PIN_I2C_SCL 17
#define PIN_POT 26

// Xdof ++++++++++++++++++++++++++++
i2c_inst_t *I2C_BNO085 = i2c0;

#define BNO055_I2C_ADDR 0x29
#define BNO055_CHIP_ID_ADDR 0x00

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

// Xdof --------------------------

#define I2C0_BUADRATE 400*1000

#define POT_LOW_VALUE 0x016
#define POT_HIGH_VALUE 0xfff

#define DELTA_DISP_MS 200
#define DELTA_SPEED_MS 10  // wait between rotary table speed refresh
#define DELTA_SENS_MS 1000 // wait between sensor refresh

#define RT_MAXRPS  5.0 // maximum rotatary table rotations per second
#define RT_MICSTEPS  32 // stepper microsteps
#define RT_STEPSROT  200 // steps per rotation of stepper motor
#define RT_GEARRAT  2 // gear ration between rotary table and stepper motor

#define RT_CSR 0 // pwm CSR_PH_CORRECT
#define RT_DIV_INT 2 // pwm DIV_INT
#define RT_DIV_FRAC 0 // pwm DIV_FRAC

#define SM_HALL 0
PIO PIO_HALL = pio0;

const uint FREQ_SYS = clock_get_hz(clk_sys); // system clock frequency

ssd1306_t disp; // create oled display instance
uint stepper_pwm_slice_num = pwm_gpio_to_slice_num(PIN_STP_PWM); 

#define pin_toggle(x) gpio_put(x, !gpio_get(x))

void setup_i2c0(void);
void setup_oled(void);
void setup_leds(void);
bool display_timer_callback(struct repeating_timer *t);
void setup_pot(void);
void update_display(void);
long map(long x, long in_min, long in_max, long out_min, long out_max);
void read_pot(void);
void setup_stepper(void);
void set_rps(float rps);
void hall_blink(PIO pio, uint sm, uint offset, uint pin_hall, uint pin_led);
// Xdof ++++++++++++++++++++++++++++
void ndof_init(void);
void get_data_v3(struct vec3 *t);
void get_data_v4(struct vec4 *t);
uint8_t ret_reg(uint8_t addr);
void get_regs(uint8_t addr, uint8_t *regs, uint8_t qty );
// Xdof --------------------------

int display_previous_ms = 0;
volatile bool display_timer_flagged = false;
long pot_pct = 0;
uint16_t pot_result = 0;
const float conversion_factor = 3.3f / (1 << 12);
float rps = 0;

uint counter = 0;

bool run = false;

void hall_isr0()
{    
    gpio_put(PIN_LEDR, 0);
    // printf("counts %d\n", ++counter);
    pio_interrupt_clear(PIO_HALL, 0);
}

void hall_isr1()
{   
    gpio_put(PIN_LEDR, 1);
    // counter=0;
    // printf("counts %d\n",counter );
    pio_interrupt_clear(PIO_HALL, 1);
}

float v_i, v_j, v_k, v_r, v_temp = 0;

int main() {
    stdio_init_all();
    setup_i2c0();
    setup_oled();
    setup_leds();
    setup_pot(); 
    setup_stepper();  
// Xdof ++++++++++++++++++++++++++++
    ndof_init();
    struct vec3 acc;
    acc.addr = 0x08; // 4.3.9 ACC_DATA_X_LSB 0x08  
    struct vec3 mag;
    mag.addr = 0x0E; // 4.3.15 MAG_DATA_X_LSB 0x0E    
    struct vec3 gyr;
    gyr.addr = 0x14; // 4.3.21 GYR_DATA_X_LSB 0x14 
    struct vec3 eul;
    eul.addr = 0x1A; // 4.3.27 EUL_DATA_X_LSB 0x1A 
    struct vec4 qat;
    qat.addr = 0x20; // 4.3.33 QUA_DATA_W_LSB 0x20 
    struct vec3 lia;
    lia.addr = 0x28; // 4.3.41 LIA_DATA_X_LSB 0x28
    struct vec3 grv;
    grv.addr = 0x2E; // 4.3.47 GRV_DATA_X_LSB 0x2E 
// Xdof --------------------------

    struct repeating_timer timer;
    add_repeating_timer_ms(-DELTA_DISP_MS, display_timer_callback, NULL, &timer);    

    irq_set_exclusive_handler(PIO0_IRQ_0, hall_isr0);
    irq_set_enabled(PIO0_IRQ_0, true);

    irq_set_exclusive_handler(PIO0_IRQ_1, hall_isr1);
    irq_set_enabled(PIO0_IRQ_1, true);

    uint hall_offset = pio_add_program(PIO_HALL, &hall_program);    
    hall_blink(PIO_HALL, SM_HALL, hall_offset, PIN_HALL, PIN_LEDG);


    while (true) {

        if(display_timer_flagged) {
            display_timer_flagged = false;
            read_pot();           
            rps = RT_MAXRPS*(pot_pct/100.0);
            set_rps(rps);
            update_display();        
            
        }
// Xdof ++++++++++++++++++++++++++++
        uint8_t calib_stat =  ret_reg(0x35); //4.3.54 CALIB_STAT 0x35
        if(calib_stat != 0b11111111){
            printf("SYS %01x\n", (calib_stat&0b11000000) >> 6);
            printf("GYR %01x\n", (calib_stat&0b00110000) >> 4);
            printf("ACC %01x\n", (calib_stat&0b00001100) >> 2);
            printf("MAG %01x\n", (calib_stat&0b00000011) );
        }

        // get_data_v3(&acc);
        // printf("accel X: %d    Y: %d    Z: %d\n", acc.x, acc.y, acc.z);

        // get_data_v3(&mag);    
        // printf("mag X: %d    Y: %d    Z: %d\n", mag.x, mag.y, mag.z);

        // get_data_v3(&gyr);  
        // printf("gyr X: %d    Y: %d    Z: %d\n", gyr.x, gyr.y, gyr.z);

        // get_data_v3(&eul);  
        // printf("eul X: %d    Y: %d    Z: %d\n", eul.x, eul.y, eul.z);

        get_data_v4(&qat);  
        // printf("qat X: %d    Y: %d    Z: %d    W: %d\n",qat.x , qat.y, qat.z, qat.w);

        // get_data_v3(&lia);  
        // printf("lia X: %d    Y: %d    Z: %d\n", lia.x, lia.y, lia.z);

        // get_data_v3(&grv);  
        // printf("grv X: %d    Y: %d    Z: %d\n", grv.x, grv.y, grv.z);

                if(qat.x > 0.1 && v_temp < 0){
                    gpio_put(PIN_LEDB, 0);
                } else {
                    gpio_put(PIN_LEDB, 1);
                }
                v_temp = qat.x;

// Xdof --------------------------

    sleep_ms(10); 

    }

    return 0;
}


void setup_i2c0(void){

    i2c_init(i2c0, I2C0_BUADRATE);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);
}


void setup_oled(void) {

    disp.external_vcc=false;
    ssd1306_init(&disp, 128, 64, 0x3C, i2c0);
    ssd1306_clear(&disp);
    ssd1306_draw_string(&disp, 8, 0, 1, (char*)"SSD1306");
    ssd1306_draw_string(&disp, 8, 16, 2, (char*)"DISPLAY");
    ssd1306_draw_string(&disp, 8, 32, 3, (char*)"DRIVER");
    ssd1306_show(&disp);
}

void setup_leds(void){
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 1);
    gpio_init(PIN_LEDR);
    gpio_set_dir(PIN_LEDR, GPIO_OUT);
    gpio_put(PIN_LEDR, 1);
    gpio_init(PIN_LEDB);
    gpio_set_dir(PIN_LEDB, GPIO_OUT);
    gpio_put(PIN_LEDB, 1);
}

bool display_timer_callback(struct repeating_timer *t) {

    display_timer_flagged = true;
    // printf("Repeat at %lld\n", time_us_64());
    return true;
}

void setup_pot(void){
    adc_init();
    adc_gpio_init(PIN_POT);
    adc_select_input(0);
}

void update_display(void){

    
    ssd1306_clear(&disp);

    char str[128];
    sprintf(str, "Raw: 0x%03x", pot_result);
    ssd1306_draw_string(&disp, 0, 0, 1, str);
    memset(str, 0, sizeof(char));
    sprintf(str, "Voltage: %f V", pot_result * conversion_factor);
    ssd1306_draw_string(&disp, 0, 16, 1, str);
    memset(str, 0, sizeof(char));
    sprintf(str, "Percent: %d", pot_pct);
    ssd1306_draw_string(&disp, 0, 32, 1, str);
    memset(str, 0, sizeof(char));
    sprintf(str, "rps: %.2f", rps);
    ssd1306_draw_string(&disp, 0, 48, 1, str);
    ssd1306_show(&disp);

    pin_toggle(PIN_LED); 

}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void read_pot(void){

    uint16_t pp = adc_read();
    pot_result = (abs(pot_result - pp) > 3)? pp: pot_result;
    pot_pct = map(pot_result,POT_LOW_VALUE, POT_HIGH_VALUE,0 ,100);

}

void setup_stepper(void){

    gpio_set_function(PIN_STP_PWM, GPIO_FUNC_PWM);
    gpio_init(PIN_STP_DIR);
    gpio_set_dir(PIN_STP_DIR, GPIO_OUT);
    gpio_init(PIN_STP_ENA);
    gpio_set_dir(PIN_STP_ENA, GPIO_OUT);
   
    pwm_set_clkdiv_mode(stepper_pwm_slice_num, PWM_DIV_FREE_RUNNING);
    pwm_set_clkdiv_int_frac(stepper_pwm_slice_num, RT_DIV_INT, RT_DIV_FRAC );
    // pwm_set_output_polarity(stepper_pwm_slice_num, false, false);
    pwm_set_phase_correct(stepper_pwm_slice_num, RT_CSR);   
}

void set_rps(float rps){

    uint16_t top = 0xFFFF;

    if(rps > RT_MAXRPS) rps = RT_MAXRPS;

    if (rps > 0){
        float pps = rps*RT_GEARRAT*RT_STEPSROT*RT_MICSTEPS;
        top = (int)(FREQ_SYS/(pps*(RT_CSR+1)*(RT_DIV_INT + RT_DIV_FRAC/16)));
        pwm_set_wrap(stepper_pwm_slice_num, top);
        pwm_set_gpio_level(PIN_STP_PWM, top >> 1);
        pwm_set_enabled(stepper_pwm_slice_num, true);
        gpio_put(PIN_STP_ENA, 1);
    }else {
        pwm_set_wrap(stepper_pwm_slice_num, top);
        pwm_set_gpio_level(PIN_STP_PWM, top >> 1);
        pwm_set_enabled(stepper_pwm_slice_num, false);
        gpio_put(PIN_STP_ENA, 0);
    }   

}

void hall_blink(PIO pio, uint sm, uint offset, uint pin_hall, uint pin_led) {
    hall_program_init(pio, sm, offset, pin_hall, pin_led);
    pio_sm_set_enabled(pio, sm, true);
}

// Xdof ++++++++++++++++++++++++++++
void ndof_init(void){

    uint8_t data[2];

    // Check to see if connection is correct
    sleep_ms(600); // Add a short delay to help BNO005 boot up

    while(ret_reg(BNO055_CHIP_ID_ADDR) != 0xA0){
        printf("Waiting for Chip Connection\n");
        sleep_ms(100); 
    }

    // // configure for external oscillator    
    // data[0] = 0x3D; // 4.3.61 OPR_MODE 0x3D
    // data[1] = 0; // set operation mode to configure
    // i2c_write_blocking(I2C_BNO085, BNO055_I2C_ADDR, data, 2, true);        
    // while(ret_reg(0x38)&0b00000001) {// 4.3.57 SYS_CLK_STATUS 0x38 - wait to clear before setting
    //     printf(" waiting for go to configure clock\n");
    //     sleep_ms(100); 
    // }
    // data[0] = 0x3F; // 4.3.63 SYS_TRIGGER 0x3F
    // data[1] = 1 << 7; // CLK_SEL 7 [0: Use internal oscillator [1: Use external oscillator. Set this bit only if external crystal is connected
    // i2c_write_blocking(I2C_BNO085, BNO055_I2C_ADDR, data, 2, true);

    // set operation mode to nine degrees of freedom (ndof)
    data[0] = 0x3D; // 4.3.61 OPR_MODE 0x3D
    data[1] = 0b00001100; // xxxx1100 ndof
    i2c_write_blocking(I2C_BNO085, BNO055_I2C_ADDR, data, 2, true);
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
    i2c_write_blocking(I2C_BNO085, BNO055_I2C_ADDR, &addr, 1, true);
    i2c_read_blocking(I2C_BNO085, BNO055_I2C_ADDR, &reg, 1, false);

    return reg;
}

void get_regs(uint8_t addr, uint8_t *regs, uint8_t qty ){

    i2c_write_blocking(I2C_BNO085, BNO055_I2C_ADDR, &addr, 1, true);
    i2c_read_blocking(I2C_BNO085, BNO055_I2C_ADDR, regs, qty, false);

}
// Xdof --------------------------