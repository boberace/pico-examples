/*



*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ad5593r_pico.h"

// I2C0 is setup and used by ad5593r_pico


int main() {
    stdio_init_all();

    sleep_ms(2000);
    printf("\nAD5593R\n");
    uint32_t ret = pico_ad5593r_init();
    
    if(ret==0) printf("\nAD5593R initialized\n");
    else printf("\nAD5593R initialization failed %d\n", ret);

    while(true){     
 

    }
    return 0;
}

