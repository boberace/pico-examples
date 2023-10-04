/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "bno085_spi.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"

#define PIN_BNO085_MISO 12
#define PIN_BNO085_CS   13
#define PIN_BNO085_SCK  10
#define PIN_BNO085_MOSI 11
#define PIN_BNO085_INT  14
#define PIN_BNO085_RST  15

#define UART_ID uart1
#define BAUD_RATE 115200
#define UART_TX_PIN 8
#define UART_RX_PIN 9

// #define PRINT(...) printf(__VA_ARGS__)
// #define PRINT(...) uart_puts((UART_ID), __VA_ARGS__)


// bno085_spi bno085(PIN_BNO085_CS, PIN_BNO085_RST, PIN_BNO085_INT);

spi_inst_t *bno085_spi_instance = spi1;
sh2_SensorValue_t sensor_value;

// void setReports(void) {
//   printf("Setting desired reports\n");
//   if (! bno085.enableReport(SH2_ROTATION_VECTOR, 10)) { //sh2.h line 93, RM: 2.2.4 Rotation Vector
//     printf("Could not enable rotation vector\n");
//   }
// }

void setup_spi(){

    spi_init(bno085_spi_instance, 1000 * 1000);
    gpio_set_function(PIN_BNO085_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_BNO085_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_BNO085_MOSI, GPIO_FUNC_SPI); 

    //Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PIN_BNO085_CS);
    gpio_set_dir(PIN_BNO085_CS, GPIO_OUT);
    gpio_put(PIN_BNO085_CS, 1);

}

void setup_uart() {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}



int main() {
    stdio_init_all();
    sleep_ms(2000);
    setup_uart();
    printf("Hello, BNO085!\n");
    setup_spi();
    // printf("BNO085 setup_spi\n");
    // if(bno085.connect_spi(bno085_spi_instance)) printf(" bno0855 has initialized\n");
    // else printf(" bno0855 NOT initialize\n");

    // for (int n = 0; n < bno085.prodIds.numEntries; n++) { // sh2.h line 60
    // printf("Part %d\n", bno085.prodIds.entry[n].swPartNumber);
    // printf(": Version %d.%d.%d\n",  bno085.prodIds.entry[n].swVersionMajor, 
    //                                 bno085.prodIds.entry[n].swVersionMinor, 
    //                                 bno085.prodIds.entry[n].swVersionPatch);
    // printf(" Build %d\n", bno085.prodIds.entry[n].swBuildNumber);
    // }  

    // setReports();   
    uint counter = 0;
    while(true){    

        sleep_ms(1000);

        char str[80];
        sprintf(str, "\033[A\33[2K\rbno_085, %i\n", counter);
        uart_puts(UART_ID,str);

        counter++;
        // if (bno085.getSensorEvent(&sensor_value)) {

        //     switch (sensor_value.sensorId) {
            
        //     case SH2_ROTATION_VECTOR:
        //     printf("Rotation Vector:  i:%f, j:%f, k:%f,  r:%f\n",   sensor_value.un.gameRotationVector.i,
        //                                                             sensor_value.un.gameRotationVector.j,
        //                                                             sensor_value.un.gameRotationVector.k,
        //                                                             sensor_value.un.gameRotationVector.real);
        //     break;
        //     }
        // }

        // if (bno085.wasReset()) {
        //     printf("sensor was reset ");
        //     setReports();
        // }

    }
    
    return 0;
}