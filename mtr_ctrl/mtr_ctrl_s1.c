#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/timer.h"
#include "pin_blink.pio.h"

#define PIN_CA1 14
#define PIN_CA2 15

#define PIN_HALL_D1 13
#define PIN_HALL_D2 14

#define PIN_HALL_A1 26
#define PIN_HALL_A2 27

#define PIN_PIEZO 28

#define PIN_STROBE 16



int main() {
    stdio_init_all();

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
