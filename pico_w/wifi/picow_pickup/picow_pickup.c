#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#define TCP_PORT 4242
#define BUF_SIZE 2048
#define POLL_TIME_S 5

#define DEBUG_MAIN printf 

#ifdef DEBUG_MAIN
  #define DEBUG_printf(x, ...) DEBUG_MAIN(x , ##__VA_ARGS__)
#else
  #define DEBUG_printf(x, ...)
#endif

uint8_t range_values[256] = {0};
uint8_t select_values[256] = {0};
bool toggle_values[256] = {false};

typedef struct TCP_SERVER_T_ {
    struct tcp_pcb *server_pcb;
    struct tcp_pcb *client_pcb;
    bool complete;
    char buffer_sent[BUF_SIZE];
    char buffer_recv[BUF_SIZE];
    int sent_len;
    int recv_len;
    uint32_t poll_counter;
    bool restart_tcp_server;
} TCP_SERVER_T;

static TCP_SERVER_T* tcp_server_init(void) {
    TCP_SERVER_T *state = calloc(1, sizeof(TCP_SERVER_T)); // mem_calloc fails to allocate?
    if (!state) {
        DEBUG_printf("failed to allocate state\n");
        return NULL;
    }
    return state;
}

static err_t tcp_server_close(void *arg) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    err_t err = ERR_OK;
    if (state->client_pcb != NULL) {
        tcp_arg(state->client_pcb, NULL);
        tcp_poll(state->client_pcb, NULL, 0);
        tcp_sent(state->client_pcb, NULL);
        tcp_recv(state->client_pcb, NULL);
        tcp_err(state->client_pcb, NULL);
        err = tcp_close(state->client_pcb);
        if (err != ERR_OK) {
            DEBUG_printf("close failed %d, calling abort\n", err);
            tcp_abort(state->client_pcb);
            err = ERR_ABRT;
        }
        state->client_pcb = NULL;
    }
    if (state->server_pcb) {
        tcp_arg(state->server_pcb, NULL);
        tcp_close(state->server_pcb);
        state->server_pcb = NULL;
    }
    return err;
}

static err_t tcp_server_result(void *arg, int status) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (status == 0) {
        DEBUG_printf("status ok\n");
    } else {
        DEBUG_printf("status not ok.  sending request to restart tcp server. %d\n", status);
        err_t etsc = tcp_server_close(arg);
        if(etsc == ERR_OK){
            state->restart_tcp_server = true;
        } else {
            DEBUG_printf("could not restart tcp server. %d\n", etsc);
        }

    }
    state->complete = true;
    return ERR_OK;
}

static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    DEBUG_printf("tcp_server_sent %u\n", len);
    state->sent_len += len;

    if (state->sent_len >= BUF_SIZE) {

        // We should get the data back from the client
        state->recv_len = 0;
        DEBUG_printf("Waiting for buffer from client\n");
    }

    return ERR_OK;
}

err_t tcp_server_send_data(void *arg, struct tcp_pcb *tpcb)
{
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;

    state->sent_len = 0;
    uint32_t len_buffer_sent = strlen(state->buffer_sent);
    DEBUG_printf("Writing %ld bytes to client\n", len_buffer_sent);
    // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
    // can use this method to cause an assertion in debug mode, if this method is called when
    // cyw43_arch_lwip_begin IS needed
    // cyw43_arch_lwip_check();
    err_t err = tcp_write(tpcb, state->buffer_sent, len_buffer_sent, TCP_WRITE_FLAG_COPY);

    if (err != ERR_OK) {
        DEBUG_printf("Failed to write data %d\n", err);
        return tcp_server_result(arg, -1);
    }

    return ERR_OK;
}

err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (!p) {
        return tcp_server_result(arg, -1);
    }
    // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
    // can use this method to cause an assertion in debug mode, if this method is called when
    // cyw43_arch_lwip_begin IS needed
    // cyw43_arch_lwip_check();
    if (p->tot_len > 0) {
        DEBUG_printf("\n\ntcp_server_recv %d/%d err %d\n", p->tot_len, state->recv_len, err);

        // todo : buffer len check

        // Receive the buffer
        memset(state->buffer_recv,0,sizeof(state->buffer_recv));
        state->recv_len = pbuf_copy_partial(p, state->buffer_recv, p->tot_len, 0);
        tcp_recved(tpcb, p->tot_len);
    }
    pbuf_free(p);

    // Parse the incoming data if a POST request
    if (strncmp(state->buffer_recv, "POST ", 5) == 0) {
        // Find the start of the POST data
        char *data_start = strstr(state->buffer_recv, "\r\n\r\n") + 4;
        // Print the POST data to the console
        DEBUG_printf("Received POST data: %s\n", data_start);

        char page, tp;
        int num, val;

        if (strncmp(data_start, "getvalue", 8) == 0) {

            sscanf(data_start + 8, "%c%d%c", &page, &num, &tp);  
        
            switch(tp) {
            
            case 'r'  :
                val = range_values[num];
                break;
                
            case 's'  :
                val = select_values[num];
                break; 
                
            case 't'  :
                val = toggle_values[num];
                break; 
        
            default :
                val = -1;
                DEBUG_printf("handleGetValues bad value type %c\n", tp);
                
            }

            DEBUG_printf("handleGetValues page = %c, num = %d, tp = %c, val = %d\n", page, num, tp, val);

        } else if (strncmp(data_start, "setvalue", 8) == 0){

            sscanf(data_start + 8, "%c%d%c%d", &page, &num, &tp, &val);  

            switch(tp) {
            
            case 'r'  :
                range_values[num] = val;
                break;
                
            case 's'  :
                select_values[num] = val;
                break;
                
            case 't'  :
                toggle_values[num] = val;
                break; 

            case 'b'  :
                // placeholder
                break; 

            default :
                DEBUG_printf("handleSetValues bad value type %c\n", tp);
                
            }
            
            DEBUG_printf("handleSetValues page = %c, num = %d, tp = %c, val = %d\n", page, num, tp, val);

        } else {
            DEBUG_printf(" recieved post other than setvalue or getvalue\n");
        }

        char  str_val[32];
        sprintf(str_val, "%d", val);
        DEBUG_printf("string val: %s\n",str_val);
        // Send the value to the client
 
        snprintf(state->buffer_sent, sizeof(state->buffer_sent), 
            "HTTP/1.1 200 OK\r\nAccess-Control-Allow-Origin: \
            *\r\nContent-Type: text/plain\r\nContent-Length: %zu\r\n\r\n%s", 
            strlen(str_val), str_val);
        DEBUG_printf("%s\n",state->buffer_sent);
        return tcp_server_send_data(arg, state->client_pcb);       


    } else {
        DEBUG_printf(" recieve TCP not HTTP POST");
    }
    return ERR_OK;
}

static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    DEBUG_printf("tcp_server_poll_fn: %d\n", ++state->poll_counter);
    // return tcp_server_result(arg, -1); // no response is an error?
    return ERR_OK;
}

static void tcp_server_err(void *arg, err_t err) {
    if (err != ERR_ABRT) {
        DEBUG_printf("tcp_client_err_fn %d\n", err);
        tcp_server_result(arg, err);
    }
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (err != ERR_OK || client_pcb == NULL) {
        DEBUG_printf("Failure in accept\n");
        tcp_server_result(arg, err);
        return ERR_VAL;
    }
    DEBUG_printf("Client connected\n");

    state->client_pcb = client_pcb;
    tcp_arg(client_pcb, state);
    tcp_sent(client_pcb, tcp_server_sent);
    tcp_recv(client_pcb, tcp_server_recv);
    tcp_poll(client_pcb, tcp_server_poll, POLL_TIME_S * 2);
    tcp_err(client_pcb, tcp_server_err);

    return ERR_OK;
}

static bool tcp_server_open(void *arg) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    DEBUG_printf("Starting server at %s on port %u\n", ip4addr_ntoa(netif_ip4_addr(netif_list)), TCP_PORT);

    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        DEBUG_printf("failed to create pcb\n");
        return false;
    }

    err_t err = tcp_bind(pcb, NULL, TCP_PORT);
    if (err) {
        DEBUG_printf("failed to bind to port %u\n", TCP_PORT);
        return false;
    }

    state->server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (!state->server_pcb) {
        DEBUG_printf("failed to listen\n");
        if (pcb) {
            tcp_close(pcb);
        }
        return false;
    }

    tcp_arg(state->server_pcb, state);
    tcp_accept(state->server_pcb, tcp_server_accept);

    return true;
}

void run_tcp_server_post(TCP_SERVER_T *state) {
    state = tcp_server_init();
    if (!state) {
        return;
    }
    if (!tcp_server_open(state)) {
        tcp_server_result(state, -1);
        return;
    }
    while(!state->complete) {
        // the following #ifdef is only here so this same example can be used in multiple modes;
        // you do not need it in your code
#if PICO_CYW43_ARCH_POLL
        // if you are using pico_cyw43_arch_poll, then you must poll periodically from your
        // main loop (not from a timer) to check for Wi-Fi driver or lwIP work that needs to be done.
        cyw43_arch_poll();
        // you can poll as often as you like, however if you have nothing else to do you can
        // choose to sleep until either a specified time, or cyw43_arch_poll() has work to do:
        cyw43_arch_wait_for_work_until(make_timeout_time_ms(1000));
#else
        // if you are not using pico_cyw43_arch_poll, then WiFI driver and lwIP work
        // is done via interrupt in the background. This sleep is just an example of some (blocking)
        // work you might be doing.
        sleep_ms(1000);
#endif
    }
    free(state);
}

int main() {
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        return 1;
    } else {
        printf("Connected.\n");
    }
    TCP_SERVER_T *state;
    run_tcp_server_post(state);
    state->restart_tcp_server = false;
    state->poll_counter = 0;

    while(1){
        if(state->restart_tcp_server){
            state->restart_tcp_server = false;
            printf("request recieved to restart server.\n");
            run_tcp_server_post(state);
        }
    }

    cyw43_arch_deinit();
    return 0;
}