#include <stdio.h>
#include "pico/stdlib.h"
#include <regex>
#include <string>

#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"

#include "quadrature_encoder.pio.h"
#include "bmc.h"
#include "pid.h"
#include "i2c_fifo.h"
#include "i2c_slave.h"

using namespace std;

// --- This project configures the MAKER PI RP2040 into a closed loop motor controller
// A pio block and two pio state machines are used for the two quadrature enoder inputs.
// A motor loop timer is setup with a callback that reads the encoder states that are provided
// to the motor pid loops for processing.
// --- Refer to pico-examples/pio/quadrature_encoder for quadrature encoder interface details.
// --- Refer to the MAKER PI RP2040 datasheet for gpio and motor truth table
// --- One of two i2c instances is setup as a slave which responds to PCA9685 
// pwm commands. The slave address is set to the default 0x40 address of the PCA9685.
// The PCA99685 pwm commands are processed to determine the pulse width.  The phase
// control of the PCA9685 is not emulated.
// --- Refer to the NXP PCA9685 product data sheet for register commands.
// --- This example is setup to recieve pwm commands from Ardupilot that is
// setup for bipolar motor control using the PCA9685 for pwm output. The frequency is 1000 hz 
// with full negative speed at 600 us, zero speed 800 us and full positive speed 1000 us.
// This leave
// 
// todo -   filter the pid
//          removed gpio button callback if not used
//          create persistant memory for pid constants
//          investigate pid constant as a function of speed - linear
//          setup Ardupilot pwm - and pca9685 output enable pin emul

#define PIN_E2_AB  2 // &(3)  first pin of encoder 2 - second pin is automatically next pin 
#define PIN_UART_TX 4
#define PIN_UART_RX 5
#define PIN_M1_AB  8 // &(9) first pin of motor 1 - second pin is automatically next pin
#define PIN_M2_AB  10 // &(11) first pin of motor 2 - second pin is automatically next pin
#define PIN_E1_AB  16  // &(17)  first pin of encoder 1 - second pin is automatically next pin 1
#define PIN_I2C_SDA_S 26
#define PIN_I2C_SCL_S 27

static void gpio_callback(uint gpio, uint32_t events); // function decleration - code below main()
static void get_usb_serial(); // function decleration - code below main()
static bool quadrature_timer_callback(struct repeating_timer *t); // function decleration - code below main()
static void setup_slave(void); // function decleration - code below main()
float set_point(uint16_t p); // function decleration - code below main()

bmc m1(PIN_M1_AB), m2(PIN_M2_AB); //  motor instances

PIO PIO_ENC = pio0; // pio block
const uint SM_E1 = 0, SM_E2 = 1; // encoder state machines

volatile int new_value_e1 =0, new_value_e2 = 0; // encoder values
volatile bool ml_timer_flagged = false; // flag for motor loop 
int delta_e1, old_value_e1=0, delta_e2, old_value_e2 = 0; // encoder values

#define UART_ID uart1 // used to manually control motors and tune pid loop
#define BAUD_RATE 115200

#define ML_REF_MS 50 // motor loop refresh rate in milliseconds 

// pid
float kp = 0.5, ki = 0.15, kd = 0, db = 0.02, mo = 0.25;
pid pid1(kp,ki,kd,db,mo), pid2(kp,ki,kd,db,mo);
float sp_e1 = 0,  sp_e2 = 0; // set point in fraction of max
float dt = ML_REF_MS / 1000.0; // refresh rate in seconds
float cr_m1 = 0, cr_m2 = 0; // calculated motor rate
float cr_e1, cr_e2; // calculated encoder rate 

float max_pps = 600.0; // expected max pps for encoders
bool pid_on = true;

#define NUM_PWMCH 16
#define FULL_NEG_PW (int)(0xFFF*600)/1000
#define FULL_POS_PW (int)(0xFFF*1000)/1000
#define NUETRAL_PW (int)(0xFFF*800)/1000
uint16_t pwch[NUM_PWMCH] = {0};

i2c_inst_t* I2C_INSTANCE_S = i2c1;
#define PICO_I2C_ADDR 0x40
static const uint I2C_BAUDRATE = 100*1000; 

static struct
{
    uint8_t mem[256] = {0};
    uint8_t mem_address = 0;
    bool mem_address_written = false;
    bool recieved = false;
} context;


int main() {

    stdio_init_all(); 
    setup_slave();
    sleep_ms(1000);

    // setup pio and state machines for encoders
    uint offset = pio_add_program(PIO_ENC, &quadrature_encoder_program); // add encoder program get location
    quadrature_encoder_program_init(PIO_ENC, SM_E1, offset, PIN_E1_AB, 0); // initialize encoder state machine
    quadrature_encoder_program_init(PIO_ENC, SM_E2, offset, PIN_E2_AB, 0); // initialize encoder state machine 

    // setup buttons
    gpio_set_irq_enabled_with_callback(20, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback); // set interrupt and point to callback function
    gpio_set_irq_enabled_with_callback(21, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback); // set interrupt and point to callback function

    //setup uart to output motor / encoder values for plotting
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);

    // setup timer to get peridic updates for encoder values
    struct repeating_timer timer;
    add_repeating_timer_ms(-ML_REF_MS, quadrature_timer_callback, NULL, &timer);

    // initialize pids to zero
    pid1.set_setpoint(0);
    pid2.set_setpoint(0);

    pwch[0] = NUETRAL_PW; // initialize motor 1 channel
    pwch[2] = NUETRAL_PW; // initialize motor 2 channel

    while (1) {

         get_usb_serial();

         if(context.recieved){
            // printf("recieved data for PCA9685\n");
            context.recieved = false;

            //go through the led on/off-high/low registers and calculate pulse witdth to put in pwch
            for(int i=0; i<NUM_PWMCH*4;i+=4){
                uint16_t pw =   (((context.mem[6+i+3]<<8) + context.mem[6+i+2])- // led off high/low minus
                                ((context.mem[6+i+1]<<8) + context.mem[6+i+0])); // led on high/low - typically zero but jic
                // make sure motors are in range
                if (i == 0 or i == 2) {                                   
                    if (pw >= FULL_NEG_PW and pw <= FULL_POS_PW) pwch[(int)i/4] = pw;
                    
                    // else printf("out of range pulse width for channel %d, value: %d\n", i, pw);

                    // printf("- recieved channel %d, value: %d\n", i, pw);
                }
            }
            pid1.set_setpoint(set_point(pwch[0]));
            pid2.set_setpoint(set_point(pwch[2]));
            
         }

         if (ml_timer_flagged){
            ml_timer_flagged = false;

            delta_e1 = (new_value_e1 - old_value_e1); // measured pulses per refresh rate
            old_value_e1 = new_value_e1;
            cr_e1 = (delta_e1 * (1000.0 / ML_REF_MS))/max_pps; // encoder calculated rate

            delta_e2 = (new_value_e2 - old_value_e2); // measured pulses per refresh rate
            old_value_e2 = new_value_e2;
            cr_e2 = (delta_e2 * (1000.0 / ML_REF_MS))/max_pps; // encoder calculated rate

            if(pid_on){ //pid-loop

                cr_m1 = pid1.output_update(cr_e1, dt);  // motor calculated rate
                cr_m2 = pid2.output_update(cr_e2, dt);  // motor calculated rate        

                // clamp rate and update motor
                if(cr_m1 > 1) m1.run(1);
                else if(cr_m1 < -1) m1.run(-1);
                else m1.run(cr_m1);                            
                // clamp rate and update motor
                if(cr_m2 > 2) m2.run(2);
                else if(cr_m2 < -2) m2.run(-2);
                else m2.run(cr_m2);                

            }

            // output setpoints, input and outputs for plotting
            char str[128];
             sprintf(str, "%1.2f\t%1.2f\t%1.2f\t%1.2f\t%1.2f\t%1.2f\n", 
                pid1.get_setpoint()*100, cr_e1*100, cr_m1*100, 
                pid2.get_setpoint()*100, cr_e2*100, cr_m2*100);
            uart_puts(UART_ID, str);
         }

    }
}

static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
    case I2C_SLAVE_RECEIVE: // master has written some data
        if (!context.mem_address_written) {
            // writes always start with the memory address
            context.mem_address = i2c_read_byte(i2c);
            context.mem_address_written = true;
        } else {
            // save into memory
            context.mem[context.mem_address] = i2c_read_byte(i2c);
            context.mem_address++;
        }
        break;
    case I2C_SLAVE_REQUEST: // master is requesting data
        // load from memory
        i2c_write_byte(i2c, context.mem[context.mem_address]);
        context.mem_address++;
        break;
    case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
        context.mem_address_written = false;
        context.recieved = true;
        break;
    default:
        break;
    }
}

static void setup_slave(void) {
    gpio_init(PIN_I2C_SDA_S);
    gpio_set_function(PIN_I2C_SDA_S, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA_S);

    gpio_init(PIN_I2C_SCL_S);
    gpio_set_function(PIN_I2C_SCL_S, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SCL_S);

    i2c_init(I2C_INSTANCE_S, I2C_BAUDRATE);
    i2c_slave_init(I2C_INSTANCE_S, PICO_I2C_ADDR, &i2c_slave_handler);
}


static void gpio_callback(uint gpio, uint32_t events) {

    if(gpio == 20){
        if (events == 4 ){ //EDGE_FALL = button push

        } else if ( events == 8 ) {// EDGE_RISE = button release

        } else {
            printf("unkown event GPIO %d %d\n", gpio, events);
        }
    } else if (gpio == 21){
        if (events == 4 ){ //EDGE_FALL = button push

        } else if ( events == 8 ) {// EDGE_RISE = button release

        } else {
            printf("unkown event GPIO %d %d\n", gpio, events);
        }
    } else {
        printf("unkown GPIO and event %d %d\n", gpio, events);
    }
        
}

static void get_usb_serial(){

        uint8_t NUM_CHAR = 16;
        char cstr[NUM_CHAR];
        uint16_t time_out_us = 10000;
        uint8_t counter = 0;
        sleep_ms(1);
        char chr = getchar_timeout_us(time_out_us);
        while (chr != 0xFF and chr != 0xA and counter < NUM_CHAR) { 
                cstr[counter] = chr;
                counter++;
            chr = getchar_timeout_us(time_out_us);
        }
        if(counter) {  
            float p = 0;
            bool negative = false;
            cmatch motors_m, pidcoef_m, pid_switch_m, set_point_m;      

            regex motors_exp("m[12]:-?[0-9f]");
            regex_match (cstr,motors_m, motors_exp);

            regex pidcoef_exp("[pid]([0-9]+.[0-9]+)");
            regex_match (cstr,pidcoef_m, pidcoef_exp);

            regex pid_switch_exp("pid-off|pid-on");
            regex_match (cstr,pid_switch_m, pid_switch_exp);

            regex set_point_exp("sp[12]:-?([0-9]+.[0-9]+)");
            regex_match (cstr,set_point_m, set_point_exp);

            if (!empty(motors_m)) {
                printf("\nentered motor command: %s \n", cstr);
                if(cstr[3] == '-') negative = true;
                if(cstr[3+negative] == 'f') p = 1.0;
                else p = ((int)cstr[3+negative] - '0')/10.0;
                p = negative?-p:p;
                if(cstr[1] == '1') m1.run(p);
                else m2.run(p); 
            } else if (!empty(pidcoef_m)){
                printf("\nentered pid coeficient command: %s \n", cstr);
                 char* s = cstr + 1;
                float mag = atof(s);
                if (cstr[0] == 'p'){
                    // kp = mag;
                    pid1.set_kp(mag);
                    pid2.set_kp(mag);
                } else if (cstr[0] == 'i'){
                    // ki = mag;
                    pid1.set_ki(mag);
                    pid2.set_ki(mag); 
                } else { // d
                    // kd = mag;
                    pid1.set_kd(mag);
                    pid2.set_kd(mag);
                }
                printf("\n me1  p:%1.2f i:%1.2f d:%1.2f\n",pid1.get_kp(),pid1.get_ki(),pid1.get_kd());
                printf("me2  p:%1.2f i:%1.2f d:%1.2f\n",pid1.get_kp(),pid1.get_ki(),pid1.get_kd());
            } else if (!empty(set_point_m)){
                printf("\nentered set point command: %s \n", cstr);
                char* s = cstr + 4;
                float mag = atof(s);
                if(cstr[2]=='1') pid1.set_setpoint(mag);
                else pid2.set_setpoint(mag);
                printf("\nsp%c: %1.2f",cstr[2],mag);
            } else if (!empty(pid_switch_m)){  
                printf("commanded %s \n", cstr);              
                pid_on = 7 - pid_switch_m.length();
                if(pid_on) printf("pid on\n");
                else {
                    printf("pid off\n");
                    m1.run(0);
                    m2.run(0);
                }
                printf("\nme1  p:%1.2f i:%1.2f d:%1.2f\n",pid1.get_kp(),pid1.get_ki(),pid1.get_kd());
                printf("me2  p:%1.2f i:%1.2f d:%1.2f\n",pid1.get_kp(),pid1.get_ki(),pid1.get_kd());
            } else {
                printf("\ndid not get that, you entered :  %s \n",cstr); 
                for(auto c : cstr) if(c) printf("%1x ", c);                
                printf("\n%1x\n",chr);

            }


            for(int i = 0; i < NUM_CHAR; i++) cstr[i] = 0; // clear out char array 
        }


}

static bool quadrature_timer_callback(struct repeating_timer *t) {

    new_value_e1 = quadrature_encoder_get_count(PIO_ENC, SM_E1);
    new_value_e2 = quadrature_encoder_get_count(PIO_ENC, SM_E2);
    ml_timer_flagged = true;

    return true;
}

float set_point(uint16_t pw){
    //takes a pulse width and maps to the set point range of -1 to 1 
    
    return  2*(pw-FULL_NEG_PW)/(FULL_POS_PW-FULL_NEG_PW) - 1;
    }