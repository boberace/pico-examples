/*

Program to test speed of pio 

todo:  create alternate to connect two statementions to respond to each other

*/

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "blink.pio.h"
#include "hall.pio.h"
// three pins in a row are used starting with PIN_LED 
// second pin blinks with LED which is connected to 
// thrid pin that is used to detect the LED blink
const int PIN_LED = 25;
const int PIN_HALL = 27;

const int SM_HALL = 0;
PIO PIO_HALL = pio0;

const int SM_BLINK = 0;
PIO PIO_BLINK = pio1;

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq);
void hall_detect(PIO pio, uint sm, uint offset, uint pin_hall);
void hall_isr0();
void hall_isr1();

int main() {
    stdio_init_all(); 

    uint offset_blink = pio_add_program(PIO_BLINK, &blink_program);  
    blink_pin_forever(PIO_BLINK, SM_BLINK, offset_blink, PIN_LED, 1); 

    uint offset_hall = pio_add_program(PIO_HALL, &hall_program);    
    hall_detect(PIO_HALL, SM_HALL, offset_hall, PIN_HALL);

    while(1){
        printf("tick\n");
        sleep_ms(1000);
        printf("tock\n");
        sleep_ms(1000);
    }
}

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (clock_get_hz(clk_sys) / (2 * freq)) - 3;
}

void hall_detect(PIO pio, uint sm, uint offset, uint pin_hall) {
    hall_program_init(pio, sm, offset, pin_hall);
    pio_sm_set_enabled(pio, sm, true);
}

void hall_isr0()
{   
    printf("low");
    pio_interrupt_clear(PIO_HALL, 0);
}

void hall_isr1()
{   
    printf("high");
    pio_interrupt_clear(PIO_HALL, 1);
}
