/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "blink.pio.h"
#include "test.pio.h"



#define PIN_LED 25u
#define PIN_SENSE1 27u
#define PIN_BASE_LEDARRAY 0u
#define NUM_LEDS 16u

#define SM_BLINK 0     // blink program state machine
PIO PIO_BLINK = pio0;  // blink program pio

#define SM_TEST 0     // test program state machine
PIO PIO_TEST = pio1;  // test program pio

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq);
void test_pin_forever(PIO pio, uint sm, uint offset, uint in_pin, uint out_pin_base, uint num_pins);

int main() {
    stdio_init_all();
    

    uint blink_offset = pio_add_program(PIO_BLINK, &blink_program);

    uint test_offset = pio_add_program(PIO_TEST, &test_program);    

    blink_pin_forever(PIO_BLINK, SM_BLINK, blink_offset, PIN_LED, 16);
    test_pin_forever(PIO_TEST, SM_TEST, test_offset, PIN_SENSE1, PIN_BASE_LEDARRAY, NUM_LEDS);

    while (1)
    {
        sleep_ms(1000);
        printf("Loaded blink_program at %d\n", blink_offset);
        printf("Loaded test_offset at %d\n", test_offset);
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

void test_pin_forever(PIO pio, uint sm, uint offset, uint in_pin, uint out_pin_base, uint num_pins) {
    test_program_init(pio, sm, offset, in_pin, out_pin_base, num_pins);
    pio_sm_set_enabled(pio, sm, true);

}