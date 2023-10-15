/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "bno085_spi.hpp"

#define PIN_BNO085_MISO 16
#define PIN_BNO085_CS   17
#define PIN_BNO085_SCK  18
#define PIN_BNO085_MOSI 19
#define PIN_BNO085_INT  26
#define PIN_BNO085_RST  14

#define SPI_A_BAUD_RATE  500 * 1000
#define SPI_A_INST spi0


// #define PRINT_PROBE_UART // uncomment to print to uart

#ifdef PRINT_PROBE_UART

#define UART_A_ID uart1
#define UART_A_BAUD_RATE 115200
#define UART_A_TX_PIN 8
#define UART_A_RX_PIN 9

uint setup_uart() {
    uint uart_ret = uart_init(UART_A_ID, UART_A_BAUD_RATE);
    gpio_set_function(UART_A_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_A_RX_PIN, GPIO_FUNC_UART);
    return uart_ret;
}

#define BUFFER_SIZE 256
#define PRINT(...)                     \
    do {                               \
        char buffer[BUFFER_SIZE];      \
        snprintf(buffer, sizeof(buffer), __VA_ARGS__); \
        uart_puts(UART_A_ID, buffer);      \
    } while (0)
#else
#define PRINT(...) printf(__VA_ARGS__)
#endif

bno085_spi bno085(PIN_BNO085_CS, PIN_BNO085_RST, PIN_BNO085_INT);

sh2_SensorValue_t sensor_value;

void setReports(void) {
  PRINT("Setting desired reports\r\n");
  if (! bno085.enableReport(SH2_ROTATION_VECTOR, 10)) { //sh2.h line 93, RM: 2.2.4 Rotation Vector
    PRINT("Could not enable rotation vector\r\n");
  }
}

uint setup_spia(){

    uint spi_ret = spi_init(SPI_A_INST, SPI_A_BAUD_RATE);
    gpio_set_function(PIN_BNO085_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_BNO085_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_BNO085_MOSI, GPIO_FUNC_SPI); 

    return spi_ret;

}

int main() {
    stdio_init_all();
    sleep_ms(2000);

    #ifdef PRINT_PROBE_UART
    uint uart_ret = setup_uart();
    if(abs((int)(uart_ret - UART_A_BAUD_RATE)) > 0.02*UART_A_BAUD_RATE){
        PRINT("UART setup failed %d\r\n", uart_ret);
        return 1;
    };
    PRINT("UART setup baud rate %d\r\n", uart_ret);
    #endif

    int spi_ret = setup_spia();
    if(abs((int)(SPI_A_BAUD_RATE-spi_ret)) > 0.02*SPI_A_BAUD_RATE ){
        PRINT("SPIA setup failed %d\r\n", spi_ret);
        return 1;
    };
    PRINT("setup_spi baud rate %d\r\n", spi_ret);

    if(bno085.connect_spi(SPI_A_INST)) PRINT(" bno0855 has initialized\r\n");
    else PRINT(" bno0855 NOT initialized\r\n");

    for (int n = 0; n < bno085.prodIds.numEntries; n++) { // sh2.h line 60
    PRINT("Part %d\n", bno085.prodIds.entry[n].swPartNumber);
    PRINT(": Version %d.%d.%d\n",  bno085.prodIds.entry[n].swVersionMajor, 
                                    bno085.prodIds.entry[n].swVersionMinor, 
                                    bno085.prodIds.entry[n].swVersionPatch);
    PRINT(" Build %d\n", bno085.prodIds.entry[n].swBuildNumber);
    }  

    setReports();   
    uint counter = 0;
    while(true){    

        sleep_ms(1000);

        // PRINT("\033[A\33[2K\rbno_085, %i\n", counter);

        counter++;
        if (bno085.getSensorEvent(&sensor_value)) {

            switch (sensor_value.sensorId) {
            
            case SH2_ROTATION_VECTOR:
            PRINT("Rotation Vector:  i:%f, j:%f, k:%f,  r:%f\r\n",   sensor_value.un.gameRotationVector.i,
                                                                    sensor_value.un.gameRotationVector.j,
                                                                    sensor_value.un.gameRotationVector.k,
                                                                    sensor_value.un.gameRotationVector.real);
            break;
            }
        } else {
            PRINT(".");
        }

        if (bno085.wasReset()) {
            PRINT("sensor was reset \r\n");
            setReports();
        }

    }
    
    return 0;
}