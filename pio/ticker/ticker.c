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
#include "counter.pio.h"
// three pins in a row are used starting with PIN_LED 
// second pin blinks with LED which is connected to 
// thrid pin that is used to detect the LED blink
const int PIN_LED = 25;
const int PIN_TRIG = 26;
const int PIN_HALL = 27;

int lowc = 0;
int highc = 0;

const int SM_HALL = 0;
PIO PIO_HALL = pio0;

const int SM_COUNTER = 1;
PIO PIO_COUNTER = pio0;

const int SM_BLINK = 0;
PIO PIO_BLINK = pio1;

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq);
void hall_detect(PIO pio, uint sm, uint offset, uint pin);
void count_fast(PIO pio, uint sm, uint offset);

void hall_isr0()
{   
    lowc++;
    gpio_put(PIN_LED, 1);
    pio_interrupt_clear(PIO_HALL, 0 + SM_HALL);
}

void hall_isr1()
{   
    highc++;
    gpio_put(PIN_LED, 0);
    pio_interrupt_clear(PIO_HALL, 1 + SM_HALL);
}

int main() {
    stdio_init_all(); 

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    irq_set_exclusive_handler(PIO0_IRQ_0, hall_isr0);
    irq_set_enabled(PIO0_IRQ_0, true);

    irq_set_exclusive_handler(PIO0_IRQ_1, hall_isr1);
    irq_set_enabled(PIO0_IRQ_1, true);

    uint offset_blink = pio_add_program(PIO_BLINK, &blink_program);  
    blink_pin_forever(PIO_BLINK, SM_BLINK, offset_blink, PIN_TRIG, 1); 

    uint offset_hall = pio_add_program(PIO_HALL, &hall_program);    
    hall_detect(PIO_HALL, SM_HALL, offset_hall, PIN_HALL);

    uint offset_counter = pio_add_program(PIO_COUNTER, &counter_program);    
    count_fast(PIO_COUNTER, SM_COUNTER, offset_counter);

    while(1){
        printf("tick\n");
        printf("lowc: %d, highc %d\n", lowc, highc);
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

void hall_detect(PIO pio, uint sm, uint offset, uint pin) {
    hall_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);
}

void count_fast(PIO pio, uint sm, uint offset) {
    counter_program_init(pio, sm, offset);
    pio_sm_set_enabled(pio, sm, true);
    pio->txf[sm] = 0xFFFFFFFF;
}

