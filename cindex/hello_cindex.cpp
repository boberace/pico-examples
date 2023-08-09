#include <stdio.h>
#include "pico/stdlib.h"
#include "cindex.h"
#include "hardware/uart.h"
#include <stdarg.h>

#define UART_ID uart1
#define BAUD_RATE 115200
#define UART_TX_PIN 8
#define UART_RX_PIN 9

void uart_printf(const char* format, ...) {
    char buf[256];                      // Buffer to store the formatted string.
    va_list args;                       // Variable to manage the variable arguments.
    va_start(args, format);             // Start retrieving variable arguments.
    vsnprintf(buf, sizeof(buf), format, args);  // Format the string and store it in 'buf'.
    va_end(args);                       // Clean up the variable arguments list.
    uart_puts(UART_ID, buf);            // Send the formatted string to the UART interface.
}


int main() {

    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    stdio_init_all();

    cindex cdx[8] = {10, 10, 10, 10, 10, 10, 10, 10};

    int counter = 0;
    uart_printf("--------------------------------------------------------\r\n");
    while(counter < 15){
        uart_printf("a++: %i, ++b: %i, ", static_cast<int>(cdx[0]++), static_cast<int>(++cdx[1]));
        uart_printf("c--: %i, --d: %i, ", static_cast<int>(cdx[2]--), static_cast<int>(--cdx[3]));
        cdx[4]+=3;
        cdx[5]-=3;
        uart_printf("e+=3: %i, f-=3: %i, ", static_cast<int>(cdx[4]), static_cast<int>(cdx[5]));
        cdx[6] = cdx[6] + 3;
        cdx[7] = cdx[7] - 3;
        uart_printf("g=g+3: %i, h=h-3: %i, ", static_cast<int>(cdx[6]), static_cast<int>(cdx[7]));
        uart_printf("\r\n");
        counter++;
    }  

    uart_printf("\r\n");
    for (int i = 0; i < 8; ++i) {
        cdx[i].set_top(20);
    }

    while(counter < 30){
        uart_printf("a++: %i, ++b: %i, ", static_cast<int>(cdx[0]++), static_cast<int>(++cdx[1]));
        uart_printf("c--: %i, --d: %i, ", static_cast<int>(cdx[2]--), static_cast<int>(--cdx[3]));
        cdx[4]+=3;
        cdx[5]-=3;
        uart_printf("e+=3: %i, f-=3: %i, ", static_cast<int>(cdx[4]), static_cast<int>(cdx[5]));
        cdx[6] = cdx[6] + 3;
        cdx[7] = cdx[7] - 3;
        uart_printf("g=g+3: %i, h=h-3: %i, ", static_cast<int>(cdx[6]), static_cast<int>(cdx[7]));
        uart_printf("\r\n");
        counter++;
    }  
}