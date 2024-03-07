#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/timer.h"
#include "pin_blink.pio.h"
#include "pin_monitor.pio.h"


#define PIN_MON_BUF_SIZE 1024

uint32_t pin1_mon_buf[PIN_MON_BUF_SIZE];
uint32_t * p_pin1_mon_buf = pin1_mon_buf;
uint DMA_PIN_MON_DAT_CHAN = 0;

void blink_pin_forever( uint pin, float freq);
void pin_blink_program_init(PIO pio, uint sm, uint offset, uint pin);
void pin_monitor_program_init(PIO pio, uint sm, uint offset, uint pin);
void monitor_pin_forever(uint pin);

int main() {
    stdio_init_all();

    blink_pin_forever( 6, 10);
    monitor_pin_forever( 6);

    dma_channel_start(DMA_PIN_MON_DAT_CHAN);

    uint counter = 1;
    while (1)
    {
        uint32_t delta = pin1_mon_buf[counter]-pin1_mon_buf[counter-1];
        printf("\033[A\33[2K\rblink pio, %i, %i, %i\n", counter, pin1_mon_buf[counter], delta);
        sleep_ms(1000);
        counter++;
    }

}

void blink_pin_forever(uint pin, float freq) {

    PIO pio = pio0;

    uint sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &pin_blink_program);
    printf("Loaded program at %d\n", offset);

    pin_blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %f Hz\n", pin, freq);

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

void monitor_pin_forever(uint pin) {
    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &pin_monitor_program);
    printf("Loaded program at %d\n", offset);

    pin_monitor_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);
    
    dma_channel_config config = dma_channel_get_default_config(DMA_PIN_MON_DAT_CHAN);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
    channel_config_set_read_increment(&config, false);
    channel_config_set_write_increment(&config, true);
    channel_config_set_dreq(&config, DREQ_PIO0_TX0 + sm);
    // channel_config_set_ring(&config, true, 2);  // Set up a ring buffer with a size of 2
    dma_channel_configure(DMA_PIN_MON_DAT_CHAN, &config, &p_pin1_mon_buf, &timer_hw->timerawl, PIN_MON_BUF_SIZE, false);

    printf("Monitoring pin %d\n", pin);
}

void pin_monitor_program_init(PIO pio, uint sm, uint offset, uint pin) {
    pio_sm_config c = pin_monitor_program_get_default_config(offset);
    sm_config_set_in_pins(&c, pin);
    sm_config_set_out_shift(&c, false, false, 0);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

