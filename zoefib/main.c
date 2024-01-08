/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pwm.pio.h"
#include "swf.pio.h"


#include "tmc2208_pico.h"
#include "tmc/helpers/Macros.h"
#include "tmc/ic/TMC2208/TMC2208_Fields.h"

#define PIN_LED 18
#define PIN_DIR 20
#define PIN_STEP 19
#define PIN_EN 21


const uint32_t REV_PER_SEC = 14;
const uint32_t PULSES_PER_REV = 200*4*256 / 2; // 200 steps per rev, 4 quarter waves, 256 microsteps per sine quarter wave /  2 (step on rising and falling edge)
const uint32_t PULSES_PER_SEC = PULSES_PER_REV * REV_PER_SEC;
const float DIVERGENCE = 137.3f;  // angle betwen nodes in degrees (approx golden angle)
const uint32_t PULSES_PER_DIV = (PULSES_PER_REV * DIVERGENCE / 360.0f); // number of pulses per divergence angle

// used to blink the led for a specific width in each period
static inline void pwm_program_init(PIO pio, uint sm, uint offset, uint pin, uint baud) {
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    pio_sm_config c = pwm_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, pin);
    float div = (float)clock_get_hz(clk_sys) / ((float)baud);
    sm_config_set_clkdiv(&c, div);
    pio_sm_init(pio, sm, offset, &c);
}

// used to drive the stepper motor
static inline void swf_program_init(PIO pio, uint sm, uint offset, uint pin, uint baud) {
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    pio_sm_config c = swf_program_get_default_config(offset);
    sm_config_set_set_pins(&c, pin, 1);
    float div = (float)clock_get_hz(clk_sys) / ((float)baud);
    sm_config_set_clkdiv(&c, div);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

// Write `period` to the input shift register
void pio_pwm_set_period(PIO pio, uint sm, uint32_t period) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_put_blocking(pio, sm, period);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, sm, pio_encode_out(pio_isr, 32));
    pio_sm_set_enabled(pio, sm, true);    
}

// Write `level` to TX FIFO. State machine will copy this into X.
void pio_pwm_set_level(PIO pio, uint sm, uint32_t level) {
    pio_sm_put_blocking(pio, sm, level);
}

int main() {
    stdio_init_all();
    sleep_ms(2000);

    gpio_init(PIN_EN);
    gpio_set_dir(PIN_EN, GPIO_OUT);
    gpio_put(PIN_EN, 0);

    gpio_init(PIN_STEP);
    gpio_set_dir(PIN_STEP, GPIO_OUT);
    gpio_put(PIN_STEP, 0);

    tmc2208_pico_init();

    tmc2208_write_register(TMC2208_GCONF, 0b0111000010);

    PIO pio = pio0;

    int sm_pwm = pio_claim_unused_sm(pio, true);
    uint offset_pwm = pio_add_program(pio, &pwm_program);
    printf("Loaded pwm_program program at %d\n", offset_pwm);
    pwm_program_init(pio, sm_pwm, offset_pwm, PIN_LED, PULSES_PER_SEC);
    pio_pwm_set_period(pio, sm_pwm, PULSES_PER_DIV / 3); // three cycles per pio pwm loop
    pio_pwm_set_level(pio, sm_pwm, 0xF);

    int sm_swf = pio_claim_unused_sm(pio, true);
    uint offset_swf = pio_add_program(pio, &swf_program);
    printf("Loaded swf_program program at %d\n", offset_swf);


    swf_program_init(pio, sm_swf, offset_swf, PIN_STEP, 0xFFFF); // slowest possible baud rate
    // ramp up the baud rate to the desired value
    uint32_t sys_clk_hz = clock_get_hz(clk_sys);
    float div = sys_clk_hz / (float)(PULSES_PER_SEC);
    uint16_t int_div = div;  
    uint8_t frac_div = ((div - (float)int_div) * (float)256); 
    printf("sys_clk_hz: %d, div: %f, int_div: %i, frac_div: %i\n\n\n", sys_clk_hz, div, int_div, frac_div);
    uint32_t tint_div;
     for(tint_div=0xFFFF; tint_div > int_div ; tint_div /= 1.025){
        pio->sm[sm_swf].clkdiv = (uint32_t) ((tint_div << 16) | (frac_div << 8)); 
        sleep_ms(20);
    }
    printf("tint_div: %i\n", tint_div);
    pio->sm[sm_swf].clkdiv = (uint32_t) ( (int_div << 16) | (frac_div << 8)); // desired baud rate

    printf("\n\n");

    int counter = 0;

    while (1) {

        printf("\033[A\33[2K\rscratch, %i\n", counter);
        sleep_ms(1000);

        counter++;
    }

}
