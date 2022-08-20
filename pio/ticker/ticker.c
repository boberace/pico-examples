/*

Program to test speed of pio 

todo:  create alternate to connect two statementions to respond to each other

*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "blink.pio.h"
// #include "hall.pio.h"
// #include "counter.pio.h"

#define pin_toggle(x) gpio_put(x, !gpio_get(x))
// three pins in a row are used starting with PIN_LED 
// second pin blinks with LED which is connected to 
// thrid pin that is used to detect the LED blink
const int PIN_LED = 25;
const int PIN_TRIG = 26;
const int PIN_HALL = 27;

int this_time = 0;
int last_time = 0;

// int lowc = 0;
// int highc = 0;

// const int SM_HALL = 0;
// PIO PIO_HALL = pio0;

// const int SM_COUNTER = 1;
// PIO PIO_COUNTER = pio0;

const int SM_BLINK = 0;
PIO PIO_BLINK = pio1;

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq);
// void hall_detect(PIO pio, uint sm, uint offset, uint pin);
// void count_fast(PIO pio, uint sm, uint offset);

static char event_str[128];

void gpio_event_string(char *buf, uint32_t events);

void gpio_callback(uint gpio, uint32_t events) {
    // Put the GPIO event(s) that just happened into event_str
    // so we can print it
    absolute_time_t t = get_absolute_time ();
    this_time = to_us_since_boot(t);
    pin_toggle(PIN_LED);
    gpio_event_string(event_str, events);
    printf("GPIO %d %s mics:%d\n", gpio, event_str, this_time - last_time );
    last_time = this_time;
}

// void hall_isr0()
// {   
//     lowc++;
//     gpio_put(PIN_LED, 1);
//     pio_interrupt_clear(PIO_HALL, 0 + SM_HALL);
// }

// void hall_isr1()
// {   
//     highc++;
//     gpio_put(PIN_LED, 0);
//     pio_interrupt_clear(PIO_HALL, 1 + SM_HALL);
// }

int main() {
    stdio_init_all(); 

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    gpio_init(PIN_HALL);
    gpio_set_dir(PIN_HALL, GPIO_IN);

    // irq_set_exclusive_handler(PIO0_IRQ_0, hall_isr0);
    // irq_set_enabled(PIO0_IRQ_0, true);

    // irq_set_exclusive_handler(PIO0_IRQ_1, hall_isr1);
    // irq_set_enabled(PIO0_IRQ_1, true);

    uint offset_blink = pio_add_program(PIO_BLINK, &blink_program);  
    blink_pin_forever(PIO_BLINK, SM_BLINK, offset_blink, PIN_TRIG, 1); 

    // uint offset_hall = pio_add_program(PIO_HALL, &hall_program);    
    // hall_detect(PIO_HALL, SM_HALL, offset_hall, PIN_HALL);

    // uint offset_counter = pio_add_program(PIO_COUNTER, &counter_program);    
    // count_fast(PIO_COUNTER, SM_COUNTER, offset_counter);

    printf("Hello GPIO IRQ\n");
    gpio_set_irq_enabled_with_callback(PIN_HALL, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    while(1){
        // printf("tick\n");
        // printf("lowc: %d, highc %d\n", lowc, highc);
        // sleep_ms(1000);

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

// void hall_detect(PIO pio, uint sm, uint offset, uint pin) {
//     hall_program_init(pio, sm, offset, pin);
//     pio_sm_set_enabled(pio, sm, true);
// }

// void count_fast(PIO pio, uint sm, uint offset) {
//     counter_program_init(pio, sm, offset);
//     pio_sm_set_enabled(pio, sm, true);
//     pio->txf[sm] = 0xFFFFFFFF;
// }

static const char *gpio_irq_str[] = {
        "LEVEL_LOW",  // 0x1
        "LEVEL_HIGH", // 0x2
        "EDGE_FALL",  // 0x4
        "EDGE_RISE"   // 0x8
};

void gpio_event_string(char *buf, uint32_t events) {
    for (uint i = 0; i < 4; i++) {
        uint mask = (1 << i);
        if (events & mask) {
            // Copy this event string into the user string
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0') {
                *buf++ = *event_str++;
            }
            events &= ~mask;

            // If more events add ", "
            if (events) {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf++ = '\0';
}