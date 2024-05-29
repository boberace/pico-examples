/*
for two motors with direction and speed inputs
with pulsed feedback from motor driver

todo: 
* update enable after reconnect 
* add connection live status to html
* create vector control of motors

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

#define UART_A_ID uart1
#define UART_A_BAUD_RATE 115200
#define UART_A_TX_PIN 16
#define UART_A_RX_PIN 17

#define LED_PIN_C0 18 // used for core 0 to show running status
#define LED_PIN_C1 19 // used for core 1 to show running status


// adjust PWM_TOP to make sure (system clock) 125,000,000 / (MOT_PWM_FREQ * PWM_TOP ) < 256
const float MOT_PWM_FREQ =  20000; // Hz
const uint16_t PWM_TOP = 500000/MOT_PWM_FREQ; // 2^14 - 1

PIO pio_encoder = pio0;
uint sm_encoder1 = 0;
uint sm_encoder2 = 1;
volatile int new_value_encoder1 =0; // encoder value
volatile int new_value_encoder2 =0; // encoder value
volatile bool motor_loop_timer_flag = false; // flag for motor loop 
int delta_encoder1, old_value_encoder1=0; // encoder values
int delta_encoder2, old_value_encoder2=0; // encoder values
float max_pps = 780.0; // expected max pps for motor encoders (no load full speed)


#define ML_REF_MS 50 // motor loop refresh rate in milliseconds 



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
static bool pulse_feedback_callback(struct repeating_timer *t);

void core1_main() { // CORE1 handle motor control and encoder feedback
    printf("Core 1 started\n");
    int counter_core1 = 0;
    uint64_t pt = to_ms_since_boot(get_absolute_time());
    uint led_state_core1 = 1;


    // setup timer to get peridic updates for encoder values
    struct repeating_timer qe_timer;
    add_repeating_timer_ms(-ML_REF_MS, pulse_feedback_callback, NULL, &qe_timer);

    // initialize pids to zero
    pid_mot1.set_setpoint(+0.5);
    pid_mot2.set_setpoint(-0.5);

    while (true) {

        uint64_t ct = to_ms_since_boot(get_absolute_time());
        if(ct - pt >= 500){  
            pt = ct;
            led_state_core1 = led_state_core1?0:1;
            gpio_put(LED_PIN_C1, led_state_core1);
            
            if(led_state_core1)
            {
                counter_core1++;           

            }
        }

         if (motor_loop_timer_flag){
            motor_loop_timer_flag = false;

            delta_encoder1 = (new_value_encoder1 - old_value_encoder1); // new_value_encoder1 continuously updated in quadrature_timer_callback
            old_value_encoder1 = new_value_encoder1;
            cr_encoder1 = (delta_encoder1 * (1000.0 / ML_REF_MS))/max_pps; // encoder calculated rate

            delta_encoder2 = (new_value_encoder2 - old_value_encoder2); // new_value_encoder1 continuously updated in quadrature_timer_callback
            old_value_encoder2 = new_value_encoder2;
            cr_encoder2 = (delta_encoder2 * (1000.0 / ML_REF_MS))/max_pps; // encoder calculated rate

           
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
            gpio_put(LED_PIN_C0, led_state_core0);

            printf("\033[A\33[2K\rdirection: %i, speed: %i, enable: %i\n", control_data.direction, control_data.speed, control_data.enable);
            
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

    gpio_init(LED_PIN_C0);
    gpio_set_dir(LED_PIN_C0, GPIO_OUT);
    gpio_put(LED_PIN_C0, 1);

    gpio_init(LED_PIN_C1);
    gpio_set_dir(LED_PIN_C1, GPIO_OUT);
    gpio_put(LED_PIN_C1, 1);
    
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

static bool pulse_feedback_callback(struct repeating_timer *t){
    // get pulse count
    motor_loop_timer_flag = true;
    return true;
}

