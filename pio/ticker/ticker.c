#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ticker.pio.h"

void ticker_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq);

int main() {
    setup_default_uart();

    // todo get free sm
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &ticker_program);
    printf("Loaded program at %d\n", offset);

    ticker_pin_forever(pio, 0, offset, 0, 3);
    ticker_pin_forever(pio, 1, offset, 6, 4);
    ticker_pin_forever(pio, 2, offset, 11, 1);
}

void ticker_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    ticker_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("tickering pin %d at %d Hz\n", pin, freq);
    pio->txf[sm] = clock_get_hz(clk_sys) / (2 * freq);
}