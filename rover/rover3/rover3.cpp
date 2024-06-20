/*
for two motors with direction and speed inputs
with pulsed feedback from motor driver

todo: 
* update enable after reconnect 
* add connection live status to html
* create vector control of motors
* add pid control of motors and put on scope to make sure 50% duty

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

#define UART_A_ID uart1
#define UART_A_BAUD_RATE 115200

#define UART_A_TX_PIN 0
#define UART_A_RX_PIN 1

#define LED_SS1_PIN 2 // used for motor 1 to show hall feedback from motor driver
#define LED_SS2_PIN 3 // used for motor 2 to show hall feedback from motor driver

#define NOT_USED_PIN4 4
#define NOT_USED_PIN5 5
#define NOT_USED_PIN6 6
#define NOT_USED_PIN7 7

#define MOTOR2_DIR_PIN 8
#define MOTOR2_EN_PIN 9
#define MOTOR2_PULSE_IN_PIN 10
#define MOTOR2_PULSE_OUT_PIN 11
#define MOTOR1_DIR_PIN 12
#define MOTOR1_EN_PIN 13
#define MOTOR1_PULSE_IN_PIN 14
#define MOTOR1_PULSE_OUT_PIN 15

#define NOT_USED_PIN16 16
#define NOT_USED_PIN17 17

#define LED_C0_PIN 18 // used for core 0 to show running status
#define LED_C1_PIN 19 // used for core 1 to show running status

#define NOT_USED_PIN20 20
#define NOT_USED_PIN21 21
#define NOT_USED_PIN22 22
#define NOT_USED_PIN26 26
#define NOT_USED_PIN27 27
#define NOT_USED_PIN28 28


// adjust PWM_TOP to make sure (system clock) 125,000,000 / (MOT_PWM_FREQ * PWM_TOP ) < 256
const float MOT_PWM_FREQ =  20000; // Hz
const uint16_t PWM_TOP = 500000/MOT_PWM_FREQ; // 2^14 - 1
const int ML_REF_MS = 50; // motor loop refresh rate in milliseconds 

PIO pio_encoder = pio0;
uint sm_encoder1 = 0;
uint sm_encoder2 = 1;
volatile int new_value_encoder1 =0; // encoder value
volatile int new_value_encoder2 =0; // encoder value
volatile bool motor_loop_timer_flag = false; // flag for motor loop 
int delta_encoder1 = 0, old_value_encoder1=0; // encoder values
int delta_encoder2 = 0, old_value_encoder2=0; // encoder values
float max_pps = 780.0; // expected max pps for motor encoders (no load full speed)


// pid
float kp = 0.5, ki = 0.15, kd = 0.01;
pid pid_mot1(kp,ki,kd);
pid pid_mot2(kp,ki,kd);
float dt = ML_REF_MS / 1000.0; // refresh rate in seconds
float cr_motor1 = 0; // calculated motor rate
float cr_encoder1; // calculated encoder rate 
float cr_motor2 = 0; // calculated motor rate
float cr_encoder2; // calculated encoder rate 
bool pid_on = true;


void setup_pins();
uint setup_uart();
void print_ip_address();
int setup_wifi();
void pin_encoder_program_init(PIO pio, uint sm, uint mon_pin, uint fb_pin);
static inline int32_t pin_encoder_get_count(PIO pio, uint sm);
static bool pulse_feedback_callback(struct repeating_timer *t);

void core1_main() { // CORE1 handle motor control and encoder feedback
    printf("Core 1 started\n");
    int counter_core1 = 0;
    uint64_t pt = to_ms_since_boot(get_absolute_time());
    uint led_state_core1 = 1;

    pio_add_program(pio_encoder, &pin_encoder_program);
    pin_encoder_program_init(pio_encoder, sm_encoder1, MOTOR1_PULSE_IN_PIN, LED_SS1_PIN);
    pin_encoder_program_init(pio_encoder, sm_encoder2, MOTOR2_PULSE_IN_PIN, LED_SS2_PIN);

    // setup timer to get peridic updates for encoder values
    struct repeating_timer pe_timer;
    add_repeating_timer_ms(-ML_REF_MS, pulse_feedback_callback, NULL, &pe_timer);

    // initialize pids to zero
    pid_mot1.set_setpoint(0);
    pid_mot2.set_setpoint(0);

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

            printf("\033[A\33[2K\r left: %f, right: %f \n", cr_encoder1, cr_encoder2);

        }

        if (motor_loop_timer_flag){ // set in callback
            motor_loop_timer_flag = false;

            delta_encoder1 = (new_value_encoder1 - old_value_encoder1); // new_value_encoder1 continuously updated in callback
            old_value_encoder1 = new_value_encoder1;
            cr_encoder1 = (float)((float)delta_encoder1 / (1000.0 / (float)ML_REF_MS));///max_pps; // encoder calculated rate

            delta_encoder2 = (new_value_encoder2 - old_value_encoder2); // new_value_encoder1 continuously updated in callback
            old_value_encoder2 = new_value_encoder2;
            cr_encoder2 = (float)((float)delta_encoder2 / (1000.0 / (float)ML_REF_MS));///max_pps; // encoder calculated rate

           
            if(pid_on){ //pid-loop              

                cr_motor1 = pid_mot1.output_update(cr_encoder1, dt);  // motor calculated rate  
                cr_motor2 = pid_mot2.output_update(cr_encoder2, dt);  // motor calculated rate 
                //mot control

            }

         }

    }
}

int main() { // CORE0 handle web server and user input
    stdio_init_all();
    setup_pins();
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

    gpio_init(MOTOR1_PULSE_OUT_PIN);
    gpio_set_dir(MOTOR1_PULSE_OUT_PIN, GPIO_OUT);
    gpio_put(MOTOR1_PULSE_OUT_PIN, 0);

    gpio_init(MOTOR1_DIR_PIN);
    gpio_set_dir(MOTOR1_DIR_PIN, GPIO_OUT);
    gpio_put(MOTOR1_DIR_PIN, 0);

    gpio_init(MOTOR1_EN_PIN);
    gpio_set_dir(MOTOR1_EN_PIN, GPIO_OUT);
    gpio_put(MOTOR1_EN_PIN, 0);

    gpio_init(MOTOR2_PULSE_IN_PIN);
    gpio_set_dir(MOTOR2_PULSE_IN_PIN, GPIO_IN);

    gpio_init(MOTOR2_PULSE_OUT_PIN);
    gpio_set_dir(MOTOR2_PULSE_OUT_PIN, GPIO_OUT);
    gpio_put(MOTOR2_PULSE_OUT_PIN, 0);

    gpio_init(MOTOR2_DIR_PIN);
    gpio_set_dir(MOTOR2_DIR_PIN, GPIO_OUT);
    gpio_put(MOTOR2_DIR_PIN, 0);

    gpio_init(MOTOR2_EN_PIN);
    gpio_set_dir(MOTOR2_EN_PIN, GPIO_OUT);  
    gpio_put(MOTOR2_EN_PIN, 0);
    
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

