#include "lwip/apps/httpd.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwipopts.h"
#include "ssi.h"
#include "cgi.h"
#include "mdns_picow.h"

// UART 
#define UART_A_ID uart1
#define UART_A_BAUD_RATE 115200
#define UART_A_TX_PIN 16
#define UART_A_RX_PIN 17

uint setup_uart() {
    uint uart_ret = uart_init(UART_A_ID, UART_A_BAUD_RATE);
    gpio_set_function(UART_A_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_A_RX_PIN, GPIO_FUNC_UART);
    return uart_ret;
}

int main() {
    stdio_init_all();
    setup_uart();
    printf("UART setup\r\n");
    cyw43_arch_init();

    cyw43_arch_enable_sta_mode();

    // Connect to the WiFI network - loop until connected
    while(cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) != 0){
        printf("Attempting to connect...\r\n");
    }
    // Print a success message once connected
    printf("Connected! \r\n");
    
    // Initialise web server
    httpd_init();
    printf("Http server initialised\r\n");

    // Configure SSI and CGI handler
    ssi_init(); 
    printf("SSI Handler initialised\r\n");
    cgi_init();
    printf("CGI Handler initialised\r\n");

    mdns_picow_init();
    
    // Infinite loop
    int counter = 0;
    while(1){
        printf("\033[A\33[2K\rweb server looping, %i\n", counter);
        sleep_ms(1000);
        counter++;
    }
}