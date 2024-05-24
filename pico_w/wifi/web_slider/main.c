#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include "lwip/dhcp.h"
#include "lwip/netif.h"
#include "lwip/ip_addr.h"
#include <string.h>

#define UART_A_ID uart1
#define UART_A_BAUD_RATE 115200
#define UART_A_TX_PIN 16
#define UART_A_RX_PIN 17

#define DEBUG_MAIN printf 

#ifdef DEBUG_MAIN
  #define DEBUG_printf(x, ...) DEBUG_MAIN(x , ##__VA_ARGS__)
#else
  #define DEBUG_printf(x, ...)
#endif

void print_ip_address() {
    struct netif *netif = &cyw43_state.netif[0]; // Assuming single network interface
    if (netif_is_up(netif)) {
        printf("IP Address: %s\n", ipaddr_ntoa(&netif->ip_addr));
    } else {
        printf("Network interface is down\n");
    }
}

static err_t http_server_serve(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    if (p != NULL) {
        tcp_recved(pcb, p->tot_len);
        const char *header_end = "\r\n\r\n";
        char *response;

        // Find end of headers
        char *header_end_pos = strstr((char *)p->payload, header_end);
        if (header_end_pos != NULL) {
            // Check if it's a POST request
            if (strncmp((char *)p->payload, "POST", 4) == 0) {
                // Extract slider value
                char *body = header_end_pos + strlen(header_end);
                int slider_value = atoi(body);
                printf("Slider Value: %d\n", slider_value);
            }
            response = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<!DOCTYPE html><html><body><h1>Hello, World!</h1><input type='range' min='0' max='100' value='0' oninput='sendValue(this.value)'><script>function sendValue(val) {fetch('/', {method: 'POST', body: val});}</script></body></html>";
        } else {
            response = "HTTP/1.1 400 Bad Request\r\n\r\n";
        }
        tcp_write(pcb, response, strlen(response), TCP_WRITE_FLAG_COPY);
        pbuf_free(p);
        tcp_close(pcb);
    }
    return ERR_OK;
}

static err_t http_server_accept(void *arg, struct tcp_pcb *pcb, err_t err) {
    tcp_recv(pcb, http_server_serve);
    return ERR_OK;
}

uint setup_uart() {
    uint uart_ret = uart_init(UART_A_ID, UART_A_BAUD_RATE);
    gpio_set_function(UART_A_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_A_RX_PIN, GPIO_FUNC_UART);
    return uart_ret;
}

int main() {
    stdio_init_all();
    setup_uart();
    

    if (cyw43_arch_init()) {
        printf("WiFi init failed");
        return -1;
    }

    cyw43_arch_enable_sta_mode();


    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("failed to connect to WiFi\n");
        return 1;
    } else {
        printf("Connected to WiFi\n");
    }

    print_ip_address();

    struct tcp_pcb *pcb = tcp_new();
    tcp_bind(pcb, IP_ADDR_ANY, 80);
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, http_server_accept);

    while (true) {
        cyw43_arch_poll();
        sleep_ms(1000);
    }
    
    return 0;
}
