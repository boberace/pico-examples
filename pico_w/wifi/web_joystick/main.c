#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include "lwip/dhcp.h"
#include "lwip/netif.h"
#include "lwip/ip_addr.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "mdns_picow.h"
#include "html_response.c"

#define UART_A_ID uart1
#define UART_A_BAUD_RATE 115200
#define UART_A_TX_PIN 16
#define UART_A_RX_PIN 17


#define HEARTBEAT_INTERVAL_MS 2000
#define TIMEOUT_MS 5000

volatile uint32_t last_heartbeat = 0;
bool connection_lost = false;
bool connection_active = false;

extern const char* html_response;

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
        char *response = NULL;

        // Find end of headers
        char *header_end_pos = strstr((char *)p->payload, header_end);
        if (header_end_pos != NULL) {
            // Check if it's a POST request
            if (strncmp((char *)p->payload, "POST", 4) == 0) {
                // Extract slider value or checkbox state
                char *body = header_end_pos + strlen(header_end);
                if (strncmp(body, "vs=", 3) == 0) {
                    int slider1_value = atoi(body + 3);
                    printf("Vertical Slider Value: %d\n", slider1_value);
                    last_heartbeat = to_ms_since_boot(get_absolute_time());
                    connection_active = true;
                } else if (strncmp(body, "hs=", 3) == 0) {
                    int slider2_value = atoi(body + 3);
                    printf("Horizontal Slider Value: %d\n", slider2_value);
                    last_heartbeat = to_ms_since_boot(get_absolute_time());
                    connection_active = true;
                } else if (strncmp(body, "en=", 3) == 0) {
                    int enable_state = strncmp(body + 3, "true", 4) == 0 ? 1 : 0;
                    printf("Enable State: %d\n", enable_state);
                    last_heartbeat = to_ms_since_boot(get_absolute_time());
                    connection_active = true;
                } else if (strncmp(body, "hb", 2) == 0) {
                    last_heartbeat = to_ms_since_boot(get_absolute_time());
                    connection_active = true;
                }
            }
            response = (char *)html_response;
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

void reset_slider_if_timed_out() {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (current_time - last_heartbeat > TIMEOUT_MS) {
        if (!connection_lost) {
            printf("Connection lost..\n");
            // Here you would perform any necessary actions to handle the lost connection, such as resetting hardware states
            connection_lost = true;
            connection_active = false;
        }
    } else {
        if (connection_lost && connection_active) {
            printf("Connection reestablished.\n");
            connection_lost = false;
        }
    }
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
    
    mdns_picow_init();

    struct tcp_pcb *pcb = tcp_new();
    tcp_bind(pcb, IP_ADDR_ANY, 80);
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, http_server_accept);

    last_heartbeat = to_ms_since_boot(get_absolute_time());

    while (true) {
        cyw43_arch_poll();
        reset_slider_if_timed_out();
        sleep_ms(1000);
    }
    
    return 0;
}
