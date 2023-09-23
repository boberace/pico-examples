#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "mdns_picow.h"

#define LED_TEMP 15

int main() {
    stdio_init_all();
    gpio_init(LED_TEMP);
    gpio_set_dir(LED_TEMP, GPIO_OUT);

    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    printf("\nConnecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("\nfailed to connect to Wi-Fi.\n");
        return 1;
    } else {
        printf("\nConnected to Wi-Fi.\n");
    }

    mdns_picow_init();


    while(true){
        sleep_ms(1000);
        gpio_put(LED_TEMP, !gpio_get(LED_TEMP));

    }

    return 0;
}