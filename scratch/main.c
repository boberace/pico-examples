#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/timer.h"
#include "pin_blink.pio.h"
#include "pin_monitor.pio.h"

// todo DMA reset on max transfer

#define TEST_PIN 15
#define CAPTURE_PIN 16
#define DUBUG_PIN (CAPTURE_PIN + 1) // placeholder for relative use in pio program

#define SAMPLE_RESOLUTION 1000*1000 // samples per second
#define SAMPLE_TIME 0.025 // in seconds 
#define SAMPLE_DEPTH (SAMPLE_RESOLUTION * SAMPLE_TIME) 

#define PIN_MON_BUF_BITS 7 // min 2 
#define PIN_MON_TYPE_BITS 2 // (0 = 8bit/1byte, 1 = 16bit/2byte, 2 = 32bit/4byte) no 64bit/8byte for DMA

#define CAPTURE_RING_BITS (PIN_MON_BUF_BITS + PIN_MON_TYPE_BITS) // max 15
#define PIN_MON_BUF_SIZE (1 << PIN_MON_BUF_BITS) // number of buffer samples
#define PIN_MON_BYTES_SIZE (1 << CAPTURE_RING_BITS)  // how many bytes the buffer takes up

uint32_t pin1_mon_buf[PIN_MON_BUF_SIZE] __attribute__((aligned(PIN_MON_BYTES_SIZE))); // must align for circular DMA

uint32_t pin_mon_buf_cpy[PIN_MON_BUF_SIZE] __attribute__((aligned(PIN_MON_BYTES_SIZE))); // must align for circular DMA

uint dma_data_cptr_chan = 0;
uint dma_data_ctrl_chan = 1;

void blink_pin_forever( uint pin, float freq);
void pin_blink_program_init(PIO pio, uint sm, uint offset, uint pin);
void monitor_pin_forever(uint pin, uint32_t *buf);
void pin_monitor_program_init(PIO pio, uint sm, uint offset, uint pin);

int main() {
    stdio_init_all();

    blink_pin_forever(TEST_PIN, 100);   
    monitor_pin_forever(CAPTURE_PIN, pin1_mon_buf);

    dma_channel_start(dma_data_cptr_chan); 
    sleep_ms(1000);
    uint counter = 1;
    while (1)
    {
        // uint pcounter = counter==0?PIN_MON_BUF_SIZE-1:counter-1;
        // uint32_t delta = pin1_mon_buf[counter]-pin1_mon_buf[pcounter];
        // printf("\033[A\33[2K\rblink pio, index %d, val %d, diff %d\n", counter,  pin1_mon_buf[counter], delta);
        
        for (size_t i = 0; i < PIN_MON_BUF_SIZE; i++)
        {
            printf("index %d, val %d\n", i, pin1_mon_buf[i]);
        }
        

        sleep_ms(10000);
        printf("\033[H\033[J");
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

void monitor_pin_forever(uint pin, uint32_t *buf) {

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
    channel_config_set_ring(&data_capture_cfg, true, CAPTURE_RING_BITS);
    dma_channel_configure(
        dma_data_cptr_chan, 
        &data_capture_cfg, 
        buf,        
        &pio->rxf[sm], 
        0xFFFFFFFF, // maximum amount of data to transfer before resetting 
        false
    );

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
    // set up to create microsecond tics
    float div = (float)clock_get_hz(clk_sys) / 8 / (SAMPLE_RESOLUTION); // 8 pio cycles per sample / (1MHz)
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}
// SAMPLE_DEPTH

uint32_t find_fundamental(uint32_t *buf, int dma_chan){
    uint32_t fundamental;
    uint32_t transfer_count = dma_hw->ch[dma_chan].transfer_count;
    uint32_t index = (0xFFFFFFFF - transfer_count) % PIN_MON_BUF_SIZE + 1;  

    uint first_value = buf[index];
    for(int i = 0; i < PIN_MON_BUF_SIZE; i++){

        pin_mon_buf_cpy[i] = buf[index] - first_value;

        index++;
        index%=PIN_MON_BUF_SIZE;  
    }

    uint bindex = 0;
    for(bindex = PIN_MON_BUF_SIZE - 1; bindex >= 0; bindex--){
        
        if (pin_mon_buf_cpy[bindex] - SAMPLE_DEPTH <= 0){
            bindex++;
            break;
        }
    }   
    if(bindex == 0) // too much noise
        return 0;

    if((PIN_MON_BUF_SIZE - 1 - bindex) < 4) // not enough data
        return 0;

    // start searching for fundamental

    return fundamental;
}

