#include "lwip/netif.h"
#include "lwip/tcp.h"

// Define the IP address and port number to listen on
#define SERVER_IP_ADDR "192.168.1.100"
#define SERVER_PORT 80

// Define the length of the buffer to receive data
#define RECV_BUF_LEN 512

// Define the HTTP response to send back
const char *http_response = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: 12\r\n\r\nHello World!";

// Define the callback function to handle incoming TCP connections
err_t tcp_server_accept_callback(void *arg, struct tcp_pcb *newpcb, err_t err) {
    // Allocate memory for the receive buffer
    char *recv_buf = malloc(RECV_BUF_LEN);

    // Set up a callback to handle incoming data
    tcp_recv(newpcb, tcp_server_recv_callback);

    // Send the HTTP response back to the client
    tcp_write(newpcb, http_response, strlen(http_response), 0);

    // Close the TCP connection
    tcp_close(newpcb);

    // Free the receive buffer memory
    free(recv_buf);

    return ERR_OK;
}

// Define the callback function to handle incoming data
err_t tcp_server_recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    // Check for errors
    if (err != ERR_OK) {
        return err;
    }

    // Check if the PBUF contains data
    if (p == NULL) {
        return ERR_OK;
    }

    // Allocate memory for the receive buffer
    char *recv_buf = malloc(RECV_BUF_LEN);

    // Copy the incoming data into the receive buffer
    int len = pbuf_copy_partial(p, recv_buf, RECV_BUF_LEN - 1, 0);

    // Add a null terminator to the end of the receive buffer
    recv_buf[len] = '\0';

    // Parse the incoming data as a POST request
    if (strncmp(recv_buf, "POST ", 5) == 0) {
        // Find the start of the POST data
        char *data_start = strstr(recv_buf, "\r\n\r\n") + 4;

        // Print the POST data to the console
        printf("Received POST data: %s\n", data_start);

        // Send the "done" response back to the client
        const char *http_response = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: 4\r\n\r\ndone";
        tcp_write(tpcb, http_response, strlen(http_response), 0);

        // Close the TCP connection
        tcp_close(tpcb);
    }

    // Free the receive buffer memory
    free(recv_buf);

    // Free the PBUF memory
    pbuf_free(p);

    return ERR_OK;
}

int main() {
    // Initialize lwIP
    lwip_init();

    // Create a new TCP PCB
    struct tcp_pcb *tcp_pcb = tcp_new();

    // Bind the TCP PCB to the server IP address and port number
    tcp_bind(tcp_pcb, IP_ADDR_ANY, SERVER_PORT);

    // Set up a callback to handle incoming TCP connections
    tcp_listen(tcp_pcb);

    // Set up a callback to handle incoming TCP connections
    tcp_accept(tcp_pcb, tcp_server_accept_callback);

    // Start the lwIP event loop
    while (1) {
        // Process any pending network events
        tcpip_input(NULL, NULL, 0);
    }

    return 0;
}