#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/timer.h"
#include "pin_blink.pio.h"

#define PIN_CA1 13
#define PIN_CA2 12
#define PIN_MTR_ENA 16

#define PIN_HALL_CEN 15
#define PIN_HALL_BTW 14

#define PIN_PIEZO 28

#define PIN_STROBE 16
#define PIN_LED 25

bool clockwise = true;
bool hall_cen_initial = false;
bool hall_btw_initial = false;
bool hall_cen = false;
bool hall_btw = false;
bool state = false;

int main() {
    stdio_init_all();
    sleep_ms(1000);

    setup_motor();
    hall_cen = gpio_get(PIN_HALL_CEN);
    hall_btw = gpio_get(PIN_HALL_BTW);

    if (clockwise) {
        if (hall_btw_initial)
        {
            state = !hall_cen_initial;            
            while(true){
                hall_cen = gpio_get(PIN_HALL_CEN);
                if(hall_cen && !state)
                {
                    state = true;
                    gpio_put(PIN_CA1, 0);
                    gpio_put(PIN_CA2, 1);              
                }
                else if(!hall_cen && state)
                {
                    state = false;
                    gpio_put(PIN_CA1, 1);
                    gpio_put(PIN_CA2, 0);
                }

            }
        }
        else
        {
            state = hall_cen_initial;
            while(true){
                hall_cen = gpio_get(PIN_HALL_CEN);
                if(hall_cen && state)
                {
                    state = false;
                    gpio_put(PIN_CA1, 0);
                    gpio_put(PIN_CA2, 1);
                }
                else if(!hall_cen && !state)
                {
                    state = true;
                    gpio_put(PIN_CA1, 1);
                    gpio_put(PIN_CA2, 0);
                }

            }

        }

    } else {
        if (hall_btw_initial)
        {
            state = !hall_cen_initial;            
            while(true){
                hall_cen = gpio_get(PIN_HALL_CEN);
                if(hall_cen && !state)
                {
                    state = true;
                    gpio_put(PIN_CA1, 1);
                    gpio_put(PIN_CA2, 0);              
                }
                else if(!hall_cen && state)
                {
                    state = false;
                    gpio_put(PIN_CA1, 0);
                    gpio_put(PIN_CA2, 1);
                }

            }
        }
        else
        {
            state = hall_cen_initial;
            while(true){
                hall_cen = gpio_get(PIN_HALL_CEN);
                if(hall_cen && state)
                {
                    state = false;
                    gpio_put(PIN_CA1, 1);
                    gpio_put(PIN_CA2, 0);
                }
                else if(!hall_cen && !state)
                {
                    state = true;
                    gpio_put(PIN_CA1, 0);
                    gpio_put(PIN_CA2, 1);
                }

            }

        }

    }



    while (1)
    {

    }

}

void blink_pin_forever(uint pin, float freq) {

    PIO pio = pio0;

    uint sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &pin_blink_program);

    pin_blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (uint)(clock_get_hz(clk_sys) / (2.0 * freq)) - 3;
}

void pin_blink_program_init(PIO pio, uint sm, uint offset, uint pin) {
   pio_gpio_init(pio, pin);
   pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
   pio_sm_config c = pin_blink_program_get_default_config(offset);
   sm_config_set_set_pins(&c, pin, 1);
   pio_sm_init(pio, sm, offset, &c);
}

void setup_motor() {
    gpio_init(PIN_CA1);
    gpio_set_dir(PIN_CA1, GPIO_OUT);
    gpio_put(PIN_CA1, 0);

    gpio_init(PIN_CA2);
    gpio_set_dir(PIN_CA2, GPIO_OUT);
    gpio_put(PIN_CA2, 0);

    gpio_init(PIN_MTR_ENA);
    gpio_set_dir(PIN_MTR_ENA, GPIO_OUT);
    gpio_put(PIN_MTR_ENA, 1);

    gpio_init(PIN_HALL_CEN);
    gpio_set_dir(PIN_HALL_CEN, GPIO_IN);

    gpio_init(PIN_HALL_BTW);
    gpio_set_dir(PIN_HALL_BTW, GPIO_IN);
}