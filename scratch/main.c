#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/timer.h"
#include "pin_blink.pio.h"
#include "pin_monitor.pio.h"

#define CAPTURE_PIN 15
#define PIN_MON_BUF_SIZE (1 << 13) // 8192 samples
#define PIN_MON_BYTES_SIZE (PIN_MON_BUF_SIZE * 4) // 32 bit per sample (4 bytes)
#define CAPTURE_RING_BITS 15 // log2(PIN_MON_BYTES_SIZE)

uint32_t pin1_mon_buf[PIN_MON_BUF_SIZE] __attribute__((aligned(PIN_MON_BYTES_SIZE)));
uint32_t * p_pin1_mon_buf = pin1_mon_buf;

uint dma_data_cptr_chan = 0;
uint dma_data_ctrl_chan = 1;

void blink_pin_forever( uint pin, float freq);
void pin_blink_program_init(PIO pio, uint sm, uint offset, uint pin);
void pin_monitor_program_init(PIO pio, uint sm, uint offset, uint pin);
void monitor_pin_forever(uint pin);

int main() {
    stdio_init_all();

    blink_pin_forever( CAPTURE_PIN, 2);   
    monitor_pin_forever( CAPTURE_PIN + 1);

    dma_channel_start(dma_data_cptr_chan); 
    uint counter = 1;
    while (1)
    {
        uint pcounter = counter==0?PIN_MON_BUF_SIZE-1:counter-1;
        uint32_t delta = pin1_mon_buf[counter]-pin1_mon_buf[pcounter];
        printf("\033[A\33[2K\rblink pio, %d, %d, %d, %d\n", counter, pcounter, pin1_mon_buf[counter], delta);
        sleep_ms(1000);

        counter++;
        counter%=PIN_MON_BUF_SIZE;
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

void monitor_pin_forever(uint pin) {

    PIO pio = pio1;

    uint sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &pin_monitor_program);

    pin_monitor_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);
    
    dma_channel_config data_capture_cfg = dma_channel_get_default_config(dma_data_cptr_chan);
    channel_config_set_transfer_data_size(&data_capture_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&data_capture_cfg, false);
    channel_config_set_write_increment(&data_capture_cfg, true);
    channel_config_set_dreq(&data_capture_cfg, DREQ_PIO1_RX0 + sm);
    // channel_config_set_chain_to(&data_capture_cfg, dma_data_ctrl_chan);
    channel_config_set_ring(&data_capture_cfg, true, CAPTURE_RING_BITS);
    dma_channel_configure(
        dma_data_cptr_chan, 
        &data_capture_cfg, 
        pin1_mon_buf, // programed by dma_data_ctrl_chan        
        &pio->rxf[sm], 
        PIN_MON_BUF_SIZE, 
        false
    );

    // dma_channel_config data_control_cfg = dma_channel_get_default_config(dma_data_ctrl_chan);   
    // channel_config_set_transfer_data_size(&data_control_cfg, DMA_SIZE_32);                   
    // channel_config_set_read_increment(&data_control_cfg, false);                             
    // channel_config_set_write_increment(&data_control_cfg, false);

    // dma_channel_configure( 
    //     dma_data_ctrl_chan, //channel – DMA channel
    //     &data_control_cfg, // config – Pointer to DMA config structure
    //     &dma_hw->ch[dma_data_cptr_chan].al2_write_addr_trig, // write_addr – Initial write address
    //     &p_pin1_mon_buf, // read_addr – Initial read address
    //     1, // transfer_count – Number of transfers to perform
    //     false // trigger – True to start the transfer immediately
    // );    

    printf("Monitoring pin %d\n", pin);
}

void pin_monitor_program_init(PIO pio, uint sm, uint offset, uint pin) {
    pio_sm_config c = pin_monitor_program_get_default_config(offset);

    uint bpin = pin + 1;
    pio_sm_set_consecutive_pindirs(pio, sm, bpin, 1, true);
    pio_gpio_init(pio, bpin);
    sm_config_set_sideset_pins(&c, bpin);

    pio_gpio_init(pio, pin);
    sm_config_set_in_pins(&c, pin);
    // shift to left, autopull disabled
    sm_config_set_in_shift(&c, false, false, 32);
    // don't join FIFO's
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE); 

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

