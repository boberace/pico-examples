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

bno085_spi bno085(PIN_BNO085_CS, PIN_BNO085_RST, PIN_BNO085_INT);

spi_inst_t *bno085_spi_instance = spi1;
sh2_SensorValue_t sensor_value;

void setReports(void) {
  printf("Setting desired reports\n");
  if (! bno085.enableReport(SH2_ROTATION_VECTOR, 10)) { //sh2.h line 93, RM: 2.2.4 Rotation Vector
    printf("Could not enable rotation vector\n");
  }
}

void setup_spi(){

    spi_init(bno085_spi_instance, 1000 * 1000);
    gpio_set_function(PIN_BNO085_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_BNO085_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_BNO085_MOSI, GPIO_FUNC_SPI);

    // Make the SPI pins available to picotool
    // bi_decl(bi_3pins_with_func(PIN_BNO085_MISO, PIN_BNO085_MOSI, PIN_BNO085_SCK, GPIO_FUNC_SPI));

    // Chip select is active-low, so we'll initialise it to a driven-high state
    // gpio_init(PIN_BNO085_CS);
    // gpio_set_dir(PIN_BNO085_CS, GPIO_OUT);
    // gpio_put(PIN_BNO085_CS, 1);
    // Make the CS pin available to picotool
    // bi_decl(bi_1pin_with_name(PIN_BNO085_CS, "SPI CS"));

}


int main() {
    stdio_init_all();
    sleep_ms(2000);
    printf("Hello, BNO085!\n");
    setup_spi();
    printf("BNO085 setup_spi\n");
    if(bno085.connect_spi(bno085_spi_instance)) printf(" bno0855 has initialized\n");
    else printf(" bno0855 NOT initialize\n");

    for (int n = 0; n < bno085.prodIds.numEntries; n++) { // sh2.h line 60
    printf("Part %d\n", bno085.prodIds.entry[n].swPartNumber);
    printf(": Version %d.%d.%d\n",  bno085.prodIds.entry[n].swVersionMajor, 
                                    bno085.prodIds.entry[n].swVersionMinor, 
                                    bno085.prodIds.entry[n].swVersionPatch);
    printf(" Build %d\n", bno085.prodIds.entry[n].swBuildNumber);
    }  

    setReports();   

    while(true){    

        sleep_ms(100);

        if (bno085.getSensorEvent(&sensor_value)) {

            switch (sensor_value.sensorId) {
            
            case SH2_ROTATION_VECTOR:
            printf("Rotation Vector:  i:%f, j:%f, k:%f,  r:%f\n",   sensor_value.un.gameRotationVector.i,
                                                                    sensor_value.un.gameRotationVector.j,
                                                                    sensor_value.un.gameRotationVector.k,
                                                                    sensor_value.un.gameRotationVector.real);
            break;
            }
        }

        if (bno085.wasReset()) {
            printf("sensor was reset ");
            setReports();
        }

    }
    
    return 0;
}