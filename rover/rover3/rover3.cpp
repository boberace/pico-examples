/*
for two motors with direction and pwm speed inputs
with pulsed feedback from motor driver
45 ppr encoders
1200 pps max speed no load

todo: 
* update enable after reconnect 
* add connection live status to html
* create vector control of motors
* add pid control of motors and put on scope to make sure 50% duty
* add imu
* add sd card
* add four bumper sensors

ideas:
* use imu to limit speed when bumpy

*/

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "pico/cyw43_arch.h"
#include "messages.h"
#include "html_server.h"
#include "mdns_picow.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "pid.h"
#include "pin_encoder.pio.h"
#include "hardware/pwm.h"

#define UART_A_ID uart1
#define UART_A_BAUD_RATE 115200

#define UART_A_TX_PIN 0
#define UART_A_RX_PIN 1

#define LED_C0_PIN 22 // used for core 0 to show running status
#define LED_C1_PIN 26 // used for core 1 to show running status

#define GPS_RX_PIN 4
#define GPS_TX_PIN 5

#define LED_SS1_PIN 6 // used for motor 1 to show hall feedback from motor driver
#define LED_SS2_PIN 7 // used for motor 2 to show hall feedback from motor driver

#define MOTOR2_DIR_PIN 10
#define MOTOR2_BRK_PIN 11
#define MOTOR2_PULSE_IN_PIN 8
#define MOTOR2_PWM_OUT_PIN 9
#define MOTOR1_DIR_PIN 14
#define MOTOR1_BRK_PIN 15
#define MOTOR1_PULSE_IN_PIN 12
#define MOTOR1_PWM_OUT_PIN 13

#define SD_SER_OUT_PIN 16
#define SD_CS_PIN 17
#define SD_CLK_PIN 18
#define SD_SER_IN_PIN 19

#define IMU_SDA_PIN 20
#define IMU_SCL_PIN 21

#define BUMP_FL_PIN 2
#define BUMP_FR_PIN 3
#define BUMP_RL_PIN 27
#define BUMP_RR_PIN 28

const int ML_REF_MS = 100; // motor loop refresh rate in milliseconds 

const int MOT_PWM_TOP = 99; // 99 - if change then must change pwm setup or PWM will break

PIO pio_encoder = pio0;
uint sm_encoder1 = 0;
uint sm_encoder2 = 1;
volatile int new_value_encoder1 =0; // encoder value
volatile int new_value_encoder2 =0; // encoder value
volatile bool motor_loop_timer_flag = false; // flag for motor loop 
int delta_encoder1 = 0, old_value_encoder1=0; // encoder values
int delta_encoder2 = 0, old_value_encoder2=0; // encoder values
int val_motor1 , val_motor2; // motor pwm value
float max_pps = 1200.0; // expected max pps for motor encoders (no load full speed)
float ppr = 45.0; // pulses per revolution for motor encoders
float max_rps = max_pps / ppr; // expected max rps for motor encoders (no load full speed)


// pid
float kp = 0.5, ki = 0.15, kd = 0.01;
pid pid_mot1(kp,ki,kd);
pid pid_mot2(kp,ki,kd);
float dt = ML_REF_MS / 1000.0; // refresh rate in seconds
float cr_motor1 = 0; // calculated motor rate
float cr_encoder1; // calculated encoder rate 
float cr_motor2 = 0; // calculated motor rate
float cr_encoder2; // calculated encoder rate 

// function delcarations
void setup_pins();
uint setup_uart();
void print_ip_address();
int setup_wifi();
void pin_encoder_program_init(PIO pio, uint sm, uint mon_pin, uint fb_pin);
static inline int32_t pin_encoder_get_count(PIO pio, uint sm);
static bool pulse_feedback_callback(struct repeating_timer *t);
static void setup_pwm_motors();

void core1_main() { // CORE1 

    printf("Core 1 started\n");
    int counter_core1 = 0;
    uint64_t pt = to_ms_since_boot(get_absolute_time());
    uint led_state_core1 = 1;

    setup_pwm_motors();

    pio_add_program(pio_encoder, &pin_encoder_program);
    pin_encoder_program_init(pio_encoder, sm_encoder1, MOTOR1_PULSE_IN_PIN, LED_SS1_PIN);
    pin_encoder_program_init(pio_encoder, sm_encoder2, MOTOR2_PULSE_IN_PIN, LED_SS2_PIN);

    // setup timer to get peridic updates for encoder values
    struct repeating_timer pe_timer;
    add_repeating_timer_ms(-ML_REF_MS, pulse_feedback_callback, NULL, &pe_timer);

    // initialize pids to zero
    pid_mot1.set_setpoint(0);
    pid_mot2.set_setpoint(0);

    pwm_set_gpio_level(MOTOR1_PWM_OUT_PIN, 10);
    pwm_set_gpio_level(MOTOR2_PWM_OUT_PIN, 10);

    while (true) {

        uint64_t ct = to_ms_since_boot(get_absolute_time());
        if(ct - pt >= 500){  
            pt = ct;
            led_state_core1 = led_state_core1?0:1;
            gpio_put(LED_C1_PIN, led_state_core1);
            
            if(led_state_core1)
            {
                counter_core1++;           

            }            

        }

        if (motor_loop_timer_flag){ // set in callback
            motor_loop_timer_flag = false;

            delta_encoder1 = new_value_encoder1 - old_value_encoder1; // new value continuously updated in timer callback
            old_value_encoder1 = new_value_encoder1;
            cr_encoder1 = (double)delta_encoder1 * (double)(1000 / ML_REF_MS)/max_pps; // encoder calculated rate

            delta_encoder2 = new_value_encoder2 - old_value_encoder2; // new value continuously updated in timer callback
            old_value_encoder2 = new_value_encoder2;
            cr_encoder2 = (double)delta_encoder2 * (double)(1000 / ML_REF_MS)/max_pps; // encoder calculated rate          

            cr_motor1 = pid_mot1.output_update(cr_encoder1, dt);  // motor calculated rate  
            cr_motor2 = pid_mot2.output_update(cr_encoder2, dt);  // motor calculated rate 
            //mot control
            val_motor1 = cr_motor1*MOT_PWM_TOP;
            val_motor2 = cr_motor2*MOT_PWM_TOP;
            // pwm_set_gpio_level(MOTOR1_PWM_OUT_PIN, val_motor1);
            // pwm_set_gpio_level(MOTOR2_PWM_OUT_PIN, val_motor2);

            printf("\033[A\33[2K\r e1: %f, m1: %f, v1 %i, e2: %f, m2 %f, v2 %i \n", cr_encoder1, cr_motor1, val_motor1, cr_encoder2, cr_motor2, val_motor2 );

         }


    }

}

int main() { // CORE0 
    setup_pins();
    stdio_init_all();    
    setup_uart();

    if(!setup_wifi()) return -1;
    html_server_init();
    mdns_picow_init();

    multicore_launch_core1(core1_main);
    int counter_core0 = 0;
    uint64_t pt = to_ms_since_boot(get_absolute_time());
    uint led_state_core0 = 1;
    printf("r\n");   

    while (true) {        

        uint64_t ct = to_ms_since_boot(get_absolute_time());
        if(ct - pt >= 500){  
            pt = ct;
            led_state_core0 = led_state_core0?0:1;
            gpio_put(LED_C0_PIN, led_state_core0);

            // printf("\033[A\33[2K\rdirection: %i, speed: %i, enable: %i\n", control_data.direction, control_data.speed, control_data.enable);
            
            if(led_state_core0)
            {

                reset_slider_if_timed_out();
                counter_core0++;            
                // printf("\033[A\33[2K\rrover2, %i\n", counter_core0);
            }
        }
        
    }
    return 0;
}

void setup_pins(){

    gpio_init(LED_C0_PIN);
    gpio_set_dir(LED_C0_PIN, GPIO_OUT);
    gpio_put(LED_C0_PIN, 1);

    gpio_init(LED_C1_PIN);
    gpio_set_dir(LED_C1_PIN, GPIO_OUT);
    gpio_put(LED_C1_PIN, 1);

    gpio_init(MOTOR1_PULSE_IN_PIN);
    gpio_set_dir(MOTOR1_PULSE_IN_PIN, GPIO_IN);   

    // gpio_init(MOTOR1_PWM_OUT_PIN);
    // gpio_set_dir(MOTOR1_PWM_OUT_PIN, GPIO_OUT);
    // gpio_put(MOTOR1_PWM_OUT_PIN, 0);

    gpio_init(MOTOR1_DIR_PIN);
    gpio_set_dir(MOTOR1_DIR_PIN, GPIO_OUT);
    gpio_put(MOTOR1_DIR_PIN, 1);

    gpio_init(MOTOR1_BRK_PIN);
    gpio_set_dir(MOTOR1_BRK_PIN, GPIO_OUT);
    gpio_put(MOTOR1_BRK_PIN, 0);

    gpio_init(MOTOR2_PULSE_IN_PIN);
    gpio_set_dir(MOTOR2_PULSE_IN_PIN, GPIO_IN);

    // gpio_init(MOTOR2_PWM_OUT_PIN);
    // gpio_set_dir(MOTOR2_PWM_OUT_PIN, GPIO_OUT);
    // gpio_put(MOTOR2_PWM_OUT_PIN, 0);

    gpio_init(MOTOR2_DIR_PIN);
    gpio_set_dir(MOTOR2_DIR_PIN, GPIO_OUT);
    gpio_put(MOTOR2_DIR_PIN, 0);

    gpio_init(MOTOR2_BRK_PIN);
    gpio_set_dir(MOTOR2_BRK_PIN, GPIO_OUT);  
    gpio_put(MOTOR2_BRK_PIN, 0);
    
}

uint setup_uart() {
    uint uart_ret = uart_init(UART_A_ID, UART_A_BAUD_RATE);
    gpio_set_function(UART_A_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_A_RX_PIN, GPIO_FUNC_UART);
    return uart_ret;
}

void print_ip_address() {
    struct netif *netif = &cyw43_state.netif[0]; // Assuming single network interface
    if (netif_is_up(netif)) {
        printf("IP Address: %s\n", ipaddr_ntoa(&netif->ip_addr));
    } else {
        printf("Network interface is down\n");
    }
}

int setup_wifi(){
    if (cyw43_arch_init()) {
        printf("WiFi init failed");
        return 0;
    }

    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("failed to connect to WiFi\n");
        return 0;
    } else {
        printf("Connected to WiFi\n");
    }

    print_ip_address();
    return 1;
}

void pin_encoder_program_init(PIO pio, uint sm, uint mon_pin, uint fb_pin) {
    // the code must be loaded at address 0, because it uses computed jumps
    pio_sm_config c = pin_encoder_program_get_default_config(0);

    pio_sm_set_consecutive_pindirs(pio, sm, fb_pin, 1, true);
    pio_gpio_init(pio, fb_pin);
    sm_config_set_sideset_pins(&c, fb_pin);

    pio_gpio_init(pio, mon_pin);
    sm_config_set_in_pins(&c, mon_pin);
    // shift to left, autopull disabled
    sm_config_set_in_shift(&c, false, false, 32);
    // don't join FIFO's
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE); 

    pio_sm_init(pio, sm, 0, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static inline int32_t pin_encoder_get_count(PIO pio, uint sm)
{
    uint ret;
    int n;

    // if the FIFO has N entries, we fetch them to drain the FIFO,
    // plus one entry which will be guaranteed to not be stale
    n = pio_sm_get_rx_fifo_level(pio, sm) + 1;
    while (n > 0) {
        ret = pio_sm_get(pio, sm);
        n--;
    }
    return ret;
}

static bool pulse_feedback_callback(struct repeating_timer *t){
    // get pulse count
    new_value_encoder1 = pin_encoder_get_count(pio_encoder, sm_encoder1);
    new_value_encoder2 = pin_encoder_get_count(pio_encoder, sm_encoder2);
    motor_loop_timer_flag = true;
    return true;
}

static void setup_pwm_motors(){

    gpio_set_function(MOTOR1_PWM_OUT_PIN, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR2_PWM_OUT_PIN, GPIO_FUNC_PWM);
    uint8_t pwm_slice1 = pwm_gpio_to_slice_num(MOTOR1_PWM_OUT_PIN);
    uint8_t pwm_slice2 = pwm_gpio_to_slice_num(MOTOR2_PWM_OUT_PIN);
    // uint32_t fsys = clock_get_hz(clk_sys);
    // 4.5.2.6. Configuring PWM Period
    uint16_t pwm_TOP = MOT_PWM_TOP; // 99
    uint8_t pwm_DIV_int = 62;//31;
    uint8_t pwm_DIV_frac = 8;//4;
    bool pwm_CSR = 1;
    // uint32_t pwm_freq = fsys / ((pwm_TOP + 1)*(pwm_CSR + 1)*(pwm_DIV_int + pwm_DIV_frac/16 ));    
    // 20k = 125m / ((99+1)*(1+1)*(31+4/16))
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, pwm_TOP); 
    pwm_config_set_clkdiv_int_frac(&config, pwm_DIV_int, pwm_DIV_frac);
    pwm_config_set_phase_correct(&config,pwm_CSR);
    pwm_config_set_clkdiv_mode(&config, PWM_DIV_FREE_RUNNING);
    pwm_config_set_output_polarity(&config, 0, 0);
    pwm_init(pwm_slice1, &config, true);
    pwm_init(pwm_slice2, &config, true);
}
