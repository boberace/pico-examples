/*


todo: 
* update enable after reconnect or disable on disconnect

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
#include "quadrature_encoder.pio.h"
#include "pid.h"
#include "bmc.h"

#define UART_A_ID uart1
#define UART_A_BAUD_RATE 115200
#define UART_A_TX_PIN 16
#define UART_A_RX_PIN 17

#define LED_PIN_C0 14
#define LED_PIN_C1 15

#define PIN_M1_AB 18 // 19 motor pin pairs
#define PIN_HALL_AB 2 // 3 encoder pin pairs

// adjust PWM_TOP to make sure (system clock) 125,000,000 / (MOT_PWM_FREQ * PWM_TOP ) < 256
const float MOT_PWM_FREQ =  20000; // Hz
const uint16_t PWM_TOP = 500000/MOT_PWM_FREQ; // 2^14 - 1


#define ML_REF_MS 50 // motor loop refresh rate in milliseconds 
// Encoder PIO and State Machines
PIO pio_QE = pio0;
uint sm_QE_MA = 0;
volatile int new_value_e1 =0; // encoder value
volatile bool ml_timer_flagged = false; // flag for motor loop 
int delta_e1, old_value_e1=0; // encoder values
float max_pps = 780.0; // expected max pps for motor encoders

// pid
float kp = 0.5, ki = 0.15, kd = 0.01;
pid pid1(kp,ki,kd);
float sp_e1 = 0; // set point in fraction of max
float dt = ML_REF_MS / 1000.0; // refresh rate in seconds
float cr_m1 = 0; // calculated motor rate
float cr_e1; // calculated encoder rate 
bool pid_on = true;

bmc m1(PIN_M1_AB); //  motor instances

void setup_pins();
uint setup_uart();
void print_ip_address();
int setup_wifi();
void setup_motor();
static inline void quadrature_encoder_program_init(PIO pio, uint sm, uint pin, int max_step_rate);
static inline int32_t quadrature_encoder_get_count(PIO pio, uint sm);
static bool quadrature_timer_callback(struct repeating_timer *t);
float set_point(uint16_t pw);

void core1_main() {
    printf("Core 1 started\n");
    int counter_core1 = 0;
    uint64_t pt = to_ms_since_boot(get_absolute_time());
    uint led_state_core1 = 1;

    pio_add_program(pio_QE, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio_QE, sm_QE_MA, PIN_HALL_AB, 0);

    // setup timer to get peridic updates for encoder values
    struct repeating_timer qe_timer;
    add_repeating_timer_ms(-ML_REF_MS, quadrature_timer_callback, NULL, &qe_timer);

    // initialize pids to zero
    pid1.set_setpoint(0.0);

    int num_samples = 10;
    float samples[num_samples] = {0};
    int sample_index = 0;

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

         if (ml_timer_flagged){
            ml_timer_flagged = false;

            delta_e1 = (new_value_e1 - old_value_e1); // new_value_e1 continuously updated in quadrature_timer_callback
            old_value_e1 = new_value_e1;
            cr_e1 = (delta_e1 * (1000.0 / ML_REF_MS))/max_pps; // encoder calculated rate

           
            if(pid_on){ //pid-loop              

                cr_m1 = pid1.output_update(cr_e1, dt);  // motor calculated rate  
                m1.run(cr_m1);  

            }

            samples[sample_index] = cr_e1; // store motor rate for averaging

            sample_index++;
            sample_index%=num_samples;             
                
            // float max_pps_test = delta_e1 * (1000.0 / ML_REF_MS);
            float avg = 0;
            for(int i = 0; i < num_samples; i++){
                avg += samples[i];
            }
                avg /= num_samples;
            // printf("\033[A\33[2K\r motor %f, encoder %f, average encoder %f \r\n", cr_m1, cr_e1, avg);

         }


    }
}

int main() {
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

    gpio_init(PIN_M1_AB);
    gpio_set_dir(PIN_M1_AB, GPIO_OUT);
    gpio_put(PIN_M1_AB, 0);

    gpio_init(PIN_M1_AB + 1);
    gpio_set_dir(PIN_M1_AB + 1, GPIO_OUT);
    gpio_put(PIN_M1_AB + 1, 0);

    gpio_init(PIN_HALL_AB);
    gpio_set_dir(PIN_HALL_AB, GPIO_IN);

    gpio_init(PIN_HALL_AB + 1);
    gpio_set_dir(PIN_HALL_AB + 1, GPIO_IN);
    
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


// max_step_rate is used to lower the clock of the state machine to save power
// if the application doesn't require a very high sampling rate. Passing zero
// will set the clock to the maximum

static inline void quadrature_encoder_program_init(PIO pio, uint sm, uint pin, int max_step_rate)
{
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 2, false);
    gpio_pull_up(pin);
    gpio_pull_up(pin + 1);

    pio_sm_config c = quadrature_encoder_program_get_default_config(0);

    sm_config_set_in_pins(&c, pin); // for WAIT, IN
    sm_config_set_jmp_pin(&c, pin); // for JMP
    // shift to left, autopull disabled
    sm_config_set_in_shift(&c, false, false, 32);
    // don't join FIFO's
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);

    // passing "0" as the sample frequency,
    if (max_step_rate == 0) {
        sm_config_set_clkdiv(&c, 1.0);
    } else {
        // one state machine loop takes at most 10 cycles
        float div = (float)clock_get_hz(clk_sys) / (10 * max_step_rate);
        sm_config_set_clkdiv(&c, div);
    }

    pio_sm_init(pio, sm, 0, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static inline int32_t quadrature_encoder_get_count(PIO pio, uint sm)
{
    uint ret;
    int n;

    // if the FIFO has N entries, we fetch them to drain the FIFO,
    // plus one entry which will be guaranteed to not be stale
    n = pio_sm_get_rx_fifo_level(pio, sm) + 1;
    while (n > 0) {
        ret = pio_sm_get_blocking(pio, sm);
        n--;
    }
    return ret;
}

static bool quadrature_timer_callback(struct repeating_timer *t) {

    new_value_e1 = quadrature_encoder_get_count(pio_QE, sm_QE_MA);
    ml_timer_flagged = true;

    return true;
}

