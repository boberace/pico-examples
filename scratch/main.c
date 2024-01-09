#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pio_spi.h"


#define LED_PIN 12
#define POW_PIN 21

#define PRINT_UART

#ifdef PRINT_UART

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
#define PRINT(...)
#endif

int main() {
    stdio_init_all();
    sleep_ms(1000);
    #ifdef PRINT_UART
    setup_uart();
    #endif

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    gpio_init(POW_PIN);
    gpio_set_dir(POW_PIN, GPIO_OUT);
    gpio_put(POW_PIN, 1);

    int counter = 0;

    while (true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(1);
        gpio_put(LED_PIN, 0);
        sleep_ms(200);
        gpio_put(LED_PIN, 1);
        sleep_ms(1);
        gpio_put(LED_PIN, 0);
        sleep_ms(798);

        PRINT("\033[A\33[2K\radc pio spi, %i\n", counter);
        counter++;
    }
}
