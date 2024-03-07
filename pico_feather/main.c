/**
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include <stdlib.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "ssd1306_i2c.h"

#define ADC_CAPTURE_CHANNEL 0 
const uint32_t ADC_DATA_DEPTH = 64*1024; //(32k)

#define I2C_SCL_PIN 3
#define I2C_SDA_PIN 2
i2c_inst_t *I2C_PORT = i2c1;


int main() {
    stdio_init_all();

    // Set up our I2C port
    i2c_init(I2C_PORT, SSD1306_I2C_CLK * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    SSD1306_init();

    // Initialize render area for entire frame (SSD1306_WIDTH pixels by SSD1306_NUM_PAGES pages)
    struct render_area frame_area = {
        start_col: 0,
        end_col : SSD1306_WIDTH - 1,
        start_page : 0,
        end_page : SSD1306_NUM_PAGES - 1
        };

    calc_render_area_buflen(&frame_area);

    // zero the entire display
    uint8_t disp_buf[SSD1306_BUF_LEN];
    memset(disp_buf, 0, SSD1306_BUF_LEN);
    render(disp_buf, &frame_area);

    // intro sequence: flash the screen 2 times
    for (int i = 0; i < 2; i++) {
        SSD1306_send_cmd(SSD1306_SET_ALL_ON);    // Set all pixels on
        sleep_ms(100);
        SSD1306_send_cmd(SSD1306_SET_ENTIRE_ON); // go back to following RAM for pixel state
        sleep_ms(100);
    }

    // Set up the ADC and FIFO
    adc_gpio_init(26 + ADC_CAPTURE_CHANNEL);
    adc_init();    
    adc_select_input(ADC_CAPTURE_CHANNEL);
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        true     // Shift each sample to 8 bits when pushing to FIFO
    );
    adc_set_clkdiv(0);     

    uint8_t data_buf[ADC_DATA_DEPTH];
    uint8_t * p_data_buf = data_buf;

    uint data_control_chan = dma_claim_unused_channel(true);
    uint data_capture_chan = dma_claim_unused_channel(true);    

    // ADC DMA
    dma_channel_config data_capture_cfg = dma_channel_get_default_config(data_capture_chan);
    channel_config_set_transfer_data_size(&data_capture_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&data_capture_cfg, false);
    channel_config_set_write_increment(&data_capture_cfg, true);
    channel_config_set_dreq(&data_capture_cfg, DREQ_ADC);    
    channel_config_set_chain_to(&data_capture_cfg, data_control_chan);

    dma_channel_configure(
        data_capture_chan, //channel – DMA channel
        &data_capture_cfg, // config – Pointer to DMA config structure
        NULL,  // (programmed by the data control channel)  write_addr – Initial write address
        &adc_hw->fifo, // read_addr – Initial read address
        ADC_DATA_DEPTH, // transfer_count – Number of transfers to perform
        false // trigger – True to start the transfer immediately
    );

    // reset data_buf address to start of buffer
    dma_channel_config data_control_cfg = dma_channel_get_default_config(data_control_chan);   
    channel_config_set_transfer_data_size(&data_control_cfg, DMA_SIZE_32);                   
    channel_config_set_read_increment(&data_control_cfg, false);                             
    channel_config_set_write_increment(&data_control_cfg, false);

    dma_channel_configure( 
        data_control_chan, //channel – DMA channel
        &data_control_cfg, // config – Pointer to DMA config structure
        &dma_hw->ch[data_capture_chan].al2_write_addr_trig, // write_addr – Initial write address
        &p_data_buf, // read_addr – Initial read address
        1, // (num entries in control block) transfer_count – Number of transfers to perform
        false // trigger – True to start the transfer immediately
    );       

    dma_channel_start(data_control_chan);
    adc_run(true);

    const uint LED_PIN = 6;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    int disp_counter = 0;
    uint curr_disp_millis = 0;
    uint previous_disp_millis = 0;

    int led_counter = 0;
    uint curr_led_micros = 0;
    uint previous_led_micros = 0;
    
    while(true){
        float test_freq = 220.0;
        float disp_freq = test_freq / 4.0;
        float sample_rate = 500000.0;
        float step = sample_rate / (disp_freq * 128.0);

        curr_disp_millis = to_ms_since_boot(get_absolute_time());
        if ((curr_disp_millis - previous_disp_millis) > 44){
            previous_disp_millis = curr_disp_millis;

            uint32_t transfer_count = dma_hw->ch[data_capture_chan].transfer_count;
            uint32_t data_index = ADC_DATA_DEPTH - transfer_count; 
            
            memset(disp_buf, 0, SSD1306_BUF_LEN);
            
            float findex = (float) data_index;
            for(int x = 127; x >= 0; x--){
                uint index = (int) (findex);
                uint y = data_buf[index] >> 3; // 8 bit ADC value to 5 bit value (32 vertical pixels)
                SetPixel(disp_buf, x, y, true);
                findex -= step;
                if(findex < 0){
                    findex = ADC_DATA_DEPTH - findex;
                }
            }
            char text[20] = {0};
            sprintf(text, "%d", data_index);
            WriteString(disp_buf, 5, 9, text); 

            render(disp_buf, &frame_area);
            disp_counter++;

        }
        
        curr_led_micros = to_us_since_boot(get_absolute_time());
        if((curr_led_micros - previous_led_micros) > 1000000.0/test_freq/2.0){
            previous_led_micros = curr_led_micros;
            gpio_put(LED_PIN, led_counter % 2);
            led_counter++;
        }

    }



    return 0;
}
