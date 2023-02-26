#include <stdio.h>  
#include "pico/stdlib.h"  
#include "pico/cyw43_arch.h"  
#include "lwip/apps/httpd.h"  

char ssid[] = WIFI_SSID;
char pass[] = WIFI_PASSWORD;
const char *ssitags[] = {"temp", "hum"};
uint32_t country = CYW43_COUNTRY_USA;
uint32_t auth = CYW43_AUTH_WPA2_MIXED_PSK;
int counter = 0;
u16_t mySSIHandler(int iIndex, char *pcInsert, int iInsertLen)  
{  
    switch(iIndex)
    {  counter++;
        case 0:              
            sprintf(pcInsert, "%d", counter);
            // snprintf(pcInsert, iInsertLen, countstring);  
            return strlen(pcInsert);
            break;  
        case 1:  
            sprintf(pcInsert, "%d", counter);
            // snprintf(pcInsert, iInsertLen, countstring);  
            return strlen(pcInsert);
            break; 
        default :
            break; 
    }  
} 

char name[30];

const char *myCGIHandler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    if (iIndex == 0)
    {
        for (int i=0; i<iNumParams; i++){
            if (strcmp(pcParam[i], "fname") == 0)
            {
                  strcpy(name, pcValue[i]);
            } 
            else if (strcmp(pcParam[i], "lname") == 0)
            {
                strcat(name, " ");
                strcat(name, pcValue[i]);
            }
        }
    }
    printf("%s\n", name);
    return "/index.shtml";
}

const tCGI FORM_CGI = {"/form.cgi", myCGIHandler};

int http_setup(uint32_t country, const char *ssid, const char *pass,  uint32_t auth, const char *hostname, ip_addr_t *ip,  ip_addr_t *mask, ip_addr_t *gw)  
{  
    if (cyw43_arch_init_with_country(country))  
    {  return 1;
    }  
    cyw43_arch_enable_sta_mode();
    if (hostname != NULL)  
    {  
        netif_set_hostname(netif_default, hostname);
    } 
    if (cyw43_arch_wifi_connect_async(ssid, pass, auth))  
    {  
        return 2;
    }  
    int flashrate = 1000;
    int status = CYW43_LINK_UP + 1;
    while (status >= 0 && status != CYW43_LINK_UP)  
    {  
        int new_status = cyw43_tcpip_link_status(&cyw43_state,  CYW43_ITF_STA);
        if (new_status != status)  
        {  
            status = new_status;
            flashrate = flashrate/ (status + 1);
            printf("connect status: %d %d\n", status, flashrate);
        }  
        
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(flashrate);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(flashrate);
    }  

    if (status < 0) 
    {  
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    }  
    else 
    {  
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        if (ip != NULL)  
        {  
            netif_set_ipaddr(netif_default, ip);
        }  
        if (mask != NULL)  
        {  
            netif_set_netmask(netif_default, mask);
        }  
        if (gw != NULL)  
        {  
            netif_set_gw(netif_default, gw);
        }  
        printf("IP: %s\n",  ip4addr_ntoa(netif_ip_addr4(netif_default)));
        printf("Mask: %s\n",  ip4addr_ntoa(netif_ip_netmask4(netif_default)));
        printf("Gateway: %s\n",  ip4addr_ntoa(netif_ip_gw4(netif_default)));
        printf("Host Name: %s\n",  netif_get_hostname(netif_default));
    }  
    return status;
} 

int main()  
{  
    stdio_init_all();
    http_setup(country, ssid, pass, auth, "MyPicoW", NULL, NULL, NULL);
    http_set_ssi_handler(mySSIHandler, ssitags, 2);  
    http_set_cgi_handlers(&FORM_CGI, 1);
    httpd_init();
    gpio_init(0);
    gpio_set_dir(0, GPIO_OUT);
    while (true)  
    {  
        gpio_put(0, !gpio_get(0));
        sleep_ms(500);
    }  
} 