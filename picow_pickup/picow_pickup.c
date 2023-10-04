#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#include "mdns_picow.h"

#include "pwp_tcp_server.h"

const int SERVER_RUNNING_PIN = 0;
uint curr_millis = 0;
uint previous_millis = 0;

void tcp_server_post_loop(){
    curr_millis = to_ms_since_boot(get_absolute_time());
    if ((curr_millis - previous_millis) > 500){
        previous_millis = curr_millis;
        gpio_put(SERVER_RUNNING_PIN, !gpio_get(SERVER_RUNNING_PIN));
    }
}

int main() {
    stdio_init_all();

    gpio_init(SERVER_RUNNING_PIN);
    gpio_set_dir(SERVER_RUNNING_PIN, GPIO_OUT);

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

    mdns_picow_init();

    TCP_SERVER_T *state;
    state->restart_tcp_server = false;
    state->poll_counter = 0;

    run_tcp_server_post(state);        

    cyw43_arch_deinit();
    return 0;
}