/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
// #include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#define PRINT_PROBE_UART // uncomment to print to uart

#ifdef PRINT_PROBE_UART

#define UART_A_ID uart1
#define UART_A_BAUD_RATE 115200
#define UART_A_TX_PIN 8
#define UART_A_RX_PIN 9

uint setup_uart() {
    uint uart_ret = uart_init(UART_A_ID, UART_A_BAUD_RATE);
    gpio_set_function(UART_A_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_A_RX_PIN, GPIO_FUNC_UART);
    return uart_ret;
}

#define BUFFER_SIZE 256
#define PRINT(...)                     \
    do {                               \
        char buffer[BUFFER_SIZE];      \
        snprintf(buffer, sizeof(buffer), __VA_ARGS__); \
        uart_puts(UART_A_ID, buffer);      \
    } while (0)
#else
#define PRINT(...) printf(__VA_ARGS__)
#endif

int main() {
    stdio_init_all();
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed");
        return -1;
    }

    #ifdef PRINT_PROBE_UART
    uint uart_ret = setup_uart();
    if(abs((int)(uart_ret - UART_A_BAUD_RATE)) > 0.02*UART_A_BAUD_RATE){
        PRINT("UART setup failed %d\r\n", uart_ret);
        return 1;
    };
    PRINT("UART setup baud rate %d\r\n", uart_ret);
    #endif

    const uint PP_PIN = 21;
    gpio_init(PP_PIN);
    gpio_set_dir(PP_PIN, GPIO_OUT);
    gpio_put(PP_PIN, 1);

    const uint LED_PIN = 12;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    uint ton = 1;
    uint counter = 0;
    while (true) {
        PRINT("\033[A\33[2K\rblink, %i\n", counter);
        counter++;
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        gpio_put(LED_PIN, 1);
        sleep_ms(ton);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        gpio_put(LED_PIN, 0);
        sleep_ms(200-ton);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        gpio_put(LED_PIN, 1);
        sleep_ms(ton);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        gpio_put(LED_PIN, 0);
        sleep_ms(800 - ton); 
    }
}
