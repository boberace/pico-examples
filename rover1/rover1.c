
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"

#define PRINT(...) printf(__VA_ARGS__)

#define ENC_A1_PIN 6
#define ENC_A2_PIN 7
#define ENC_B1_PIN 8
#define ENC_B2_PIN 9

#define MOT_A_ENA 10
#define MOT_A_IN1 11
#define MOT_A_IN2 12
#define MOT_B_IN3 13
#define MOT_B_IN4 14
#define MOT_B_ENA 15
// adjust PWM_TOP to make sure (system clock) 125,000,000 / (MOT_PWM_FREQ * PWM_TOP ) < 256
const float MOT_PWM_FREQ =  20000; // Hz
const uint16_t PWM_TOP = 500000/MOT_PWM_FREQ; // 2^14 - 1

void init_pins();
void setup_pwm();
void set_motors_direction(uint8_t mot_a, uint8_t mot_b);
void set_motors_speed(float mot_a, float mot_b);

int main() {
    stdio_init_all();
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed");
        return -1;
    }

    init_pins();
    setup_pwm();

    set_motors_direction(1, 1);
    set_motors_speed(0, 0.7);

    int counter = 0;
    uint64_t pt = to_us_since_boot(get_absolute_time());
    uint led_state = 1;
    while (true) {

        


        uint64_t ct = to_us_since_boot(get_absolute_time());
        if(ct - pt >= 500000){  
            pt = ct;
            led_state = led_state?0:1;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
            
            if(led_state)
            {
            counter++;
            PRINT("\033[A\33[2K\rrover1, %i\n", counter);
            }
        }
        
    }
}

void init_pins(){

    uint8_t enc_pins[]={ENC_A1_PIN, ENC_A2_PIN, ENC_B1_PIN, ENC_B2_PIN};
    uint8_t mot_pins[]={ MOT_A_IN1, MOT_A_IN2, MOT_B_IN3, MOT_B_IN4};

    for(int i=0; i<4; i++){
        gpio_init(enc_pins[i]);
        gpio_set_dir(enc_pins[i], GPIO_IN);
    }
    for(int i=0; i<4; i++){
        gpio_init(mot_pins[i]);
        gpio_set_dir(mot_pins[i], GPIO_OUT);
    }
    gpio_set_function(MOT_A_ENA, GPIO_FUNC_PWM);
    gpio_set_function(MOT_B_ENA, GPIO_FUNC_PWM);
}

void set_motors_direction(uint8_t mot_a, uint8_t mot_b){
    gpio_put(MOT_A_IN1, mot_a);
    gpio_put(MOT_A_IN2, !mot_a);
    gpio_put(MOT_B_IN3, mot_b);
    gpio_put(MOT_B_IN4, !mot_b);
}

void setup_pwm(){
    uint8_t PWM_PINS[] = {MOT_A_ENA, MOT_B_ENA};

    float freq_sys = clock_get_hz(clk_sys);
    uint pwm_clock_freq = MOT_PWM_FREQ * PWM_TOP;
    float pwm_clkdiv = freq_sys / pwm_clock_freq; // Floating point clock divider, 1.f <= value < 256.f

    uint num_pwm_pins = sizeof(PWM_PINS) / sizeof(PWM_PINS[0]);    
    for(uint i = 0; i < num_pwm_pins; i++){
        uint PWM_PIN = PWM_PINS[i];
        gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
        pwm_set_clkdiv(slice_num, pwm_clkdiv);
        pwm_set_wrap(slice_num, PWM_TOP);
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWM_PIN), 0);
        pwm_set_enabled(slice_num, true);
    }    
}

void set_motors_speed(float mot_a, float mot_b){
    uint slice_num_a = pwm_gpio_to_slice_num(MOT_A_ENA);
    uint slice_num_b = pwm_gpio_to_slice_num(MOT_B_ENA);
    pwm_set_chan_level(slice_num_a, pwm_gpio_to_channel(MOT_A_ENA), mot_a * PWM_TOP);
    pwm_set_chan_level(slice_num_b, pwm_gpio_to_channel(MOT_B_ENA), mot_b * PWM_TOP);
}

// serial interface for rover1 pwm frequency and duty cycle control for testing
void serial_interface(){
    char c;
    while (true) {
        c = getchar_timeout_us(0);
        if(c != PICO_ERROR_TIMEOUT){
            if(c == 'f'){
                PRINT("Enter PWM frequency: ");
                float freq;
                scanf("%f", &freq);
                PRINT("Setting PWM frequency to %f Hz\n", freq);
                MOT_PWM_FREQ = freq;
                setup_pwm();
            }
            else if(c == 'd'){
                PRINT("Enter PWM duty cycle: ");
                float duty;
                scanf("%f", &duty);
                PRINT("Setting PWM duty cycle to %f\n", duty);
                set_motors_speed(duty, duty);
            }
        }
    }
}