/**
 * Copyright (c) 2021 pmarques-dev @ github
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include <regex>
#include <string>

#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"

#include "quadrature_encoder.pio.h"
#include "bmc.h"
#include "pid.h"

using namespace std;
//
// ---- This project configures the MAKER PI RP2040 into a closed loop motor controller
// A pio block and two pio state machines are used for the two quadrature enoder inputs
// for quadrature encoder interface details refer to pio/quadrature_encoder
// ---- Refer to the MAKER PI RP2040 datasheet for gpio and motor truth table
// 
// todo -   filter the pid
//          removed gpio callback if not used
//          create persistant memory for pid constants
//          investigate pid constant as a function of speed - linear

void gpio_callback(uint gpio, uint32_t events); // function decleration - definition below main()
void get_usb_serial(); // function decleration - definition below main()
bool quadrature_timer_callback(struct repeating_timer *t); // function decleration - definition below main()
// first motor pin as first gpio of a pwm slice.  Second is automatically second pin of slice.
const uint PIN_M1_AB = 8, PIN_M2_AB = 10; 

bmc m1(PIN_M1_AB), m2(PIN_M2_AB); //  motor instances

PIO pio = pio0; // pio block
const uint sm_e1 = 0, sm_e2 = 1; // encoder statem machines

volatile int new_value_e1 =0, new_value_e2 = 0; // encoder values
volatile bool ml_timer_flagged = false; // flag for motor loop 
int delta_e1, old_value_e1=0, delta_e2, old_value_e2 = 0; // encoder values

#define UART_ID uart0 // used to manually tune pid loop
#define BAUD_RATE 115200

#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define ML_REF_MS 50 // motor loop refresh rate in milliseconds 

float kp = 0.5, ki = 0.15, kd = 0;
pid pid1(kp,ki,kd), pid2(kp,ki,kd);
float sp_e1 = 0.7,  sp_e2 = 0.4; // set point in fraction of max
float dt = ML_REF_MS / 1000.0; // refresh rate in seconds
float ov_e1 = 0, mv_e1, ov_e2 = 0, mv_e2;

float max_pps = 600.0; // pulses per second at no load max speed
bool pid_on = false;

int main() {

    stdio_init_all(); 
    sleep_ms(1000);

    const uint PIN_AB_E1 = 16; //  first of two pins for encoder.  Second is automatically first + 1
    const uint PIN_AB_E2 = 2;  //  first of two pins for encoder.  Second is automatically first + 1     

    uint offset = pio_add_program(pio, &quadrature_encoder_program); // add encoder program get location
  
    quadrature_encoder_program_init(pio, sm_e1, offset, PIN_AB_E1, 0); // initialize encoder state machine
    quadrature_encoder_program_init(pio, sm_e2, offset, PIN_AB_E2, 0); // initialize encoder state machine 

    gpio_set_irq_enabled_with_callback(20, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback); // set interrupt and point to callback function
    gpio_set_irq_enabled_with_callback(21, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback); // set interrupt and point to callback function

    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    struct repeating_timer timer;
    add_repeating_timer_ms(-ML_REF_MS, quadrature_timer_callback, NULL, &timer);
    // sleep_ms(3000);
    pid1.set_setpoint(sp_e1);
    pid2.set_setpoint(sp_e2);

    while (1) {

         get_usb_serial();

         if (ml_timer_flagged){
            ml_timer_flagged = false;

            delta_e1 = (new_value_e1 - old_value_e1); // measured pulses per refresh rate
            old_value_e1 = new_value_e1;
            mv_e1 = (delta_e1 * (1000.0 / ML_REF_MS))/max_pps; // measured value in fraction of max

            delta_e2 = (new_value_e2 - old_value_e2); // measured pulses per refresh rate
            old_value_e2 = new_value_e2;
            mv_e2 = (delta_e2 * (1000.0 / ML_REF_MS))/max_pps; // measured value in fraction of max

            if(pid_on){ //pid-loop

                ov_e1 = pid1.output_update(mv_e1, dt);  
                ov_e2 = pid2.output_update(mv_e2, dt);          

                if(ov_e1 > 1) m1.run(1);
                else if(ov_e1 < -1) m1.run(-1);
                else m1.run(ov_e1);                            

                if(ov_e2 > 2) m2.run(2);
                else if(ov_e2 < -2) m2.run(-2);
                else m2.run(ov_e2);                

            }

            // output setpoints, input and outputs for plotting
            char str[128];
             sprintf(str, "%1.2f\t%1.2f\t%1.2f\t%1.2f\t%1.2f\t%1.2f\n", 
                pid1.get_setpoint()*100, mv_e1*100, ov_e1*100, 
                pid2.get_setpoint()*100, mv_e2*100, ov_e2*100);
            uart_puts(UART_ID, str);
         }

    }
}

void gpio_callback(uint gpio, uint32_t events) {

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

void get_usb_serial(){

        uint8_t NUM_CHAR = 16;
        char cstr[NUM_CHAR];
        uint8_t counter = 0;
        sleep_ms(1);
        char c = getchar_timeout_us(100);
        while (c != 0xFF and c != 0xA and counter < NUM_CHAR) { 
                cstr[counter] = c;
                counter++;
            c = getchar_timeout_us(100);
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

            }


            for(int i = 0; i < NUM_CHAR; i++) cstr[i] = 0; // clear out char array 
        }


}

bool quadrature_timer_callback(struct repeating_timer *t) {

    new_value_e1 = quadrature_encoder_get_count(pio, sm_e1);
    new_value_e2 = quadrature_encoder_get_count(pio, sm_e2);
    ml_timer_flagged = true;

    return true;
}

