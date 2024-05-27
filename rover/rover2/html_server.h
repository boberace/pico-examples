// struct to hold the messages that are sent between the rover and the controller// shared_struct.h
#ifndef HTML_SERVER_H
#define HTML_SERVER_H


#include "lwip/tcp.h"


#define HEARTBEAT_INTERVAL_MS 2000
#define TIMEOUT_MS 5000


void html_server_init();

static err_t http_server_serve(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
static err_t http_server_accept(void *arg, struct tcp_pcb *pcb, err_t err);
void reset_slider_if_timed_out();

#endif // HTML_SERVER_H
