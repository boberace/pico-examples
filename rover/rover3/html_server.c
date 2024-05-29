#include "html_server.h"

#include <string.h>
#include "pico/stdlib.h"
#include "html_response.c"

#include "messages.h"

volatile uint32_t last_heartbeat = 0;
bool connection_lost = false;
bool connection_active = false;

extern const char* html_response;

void html_server_init() {
    struct tcp_pcb *pcb = tcp_new();
    tcp_bind(pcb, IP_ADDR_ANY, 80);
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, http_server_accept);
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
                    control_data.speed = atoi(body + 3);
                    last_heartbeat = to_ms_since_boot(get_absolute_time());
                    connection_active = true;
                } else if (strncmp(body, "hs=", 3) == 0) {
                    control_data.direction = atoi(body + 3);
                    last_heartbeat = to_ms_since_boot(get_absolute_time());
                    connection_active = true;
                } else if (strncmp(body, "en=", 3) == 0) {
                    control_data.enable = strncmp(body + 3, "true", 4) == 0 ? 1 : 0;
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


void reset_slider_if_timed_out() {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (current_time - last_heartbeat > TIMEOUT_MS) {
        if (!connection_lost) {
            printf("\r\nConnection lost..\r\n");
            // Here you would perform any necessary actions to handle the lost connection, such as resetting hardware states
            connection_lost = true;
            connection_active = false;
        }
    } else {
        if (connection_lost && connection_active) {
            printf("\r\nConnection reestablished.\r\n");
            connection_lost = false;
        }
    }
}
