/*

Documents referenced in comments:
DS: BNO080/85/86 Data Sheet
RM: SH-2 Reference Manual v1.2
TP: Sensor Hub Transport Protocol R 1.7

*/
#include "bno085_i2c.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <string.h>

// define i2c pararmeters and pins
#define I2C0_BUADRATE 100*1000
#define PIN_I2C0_SDA 4 // 16 // 4
#define PIN_I2C0_SCL 5 // 17 // 5
i2c_inst_t *bno085_i2c_instance = i2c0;

// define interrupt and reset pins
#define PIN_BNO085_INT 15 // 28 // 15
#define PIN_BNO085_RST 16 // 27 // 16

bno085_i2c bno085(PIN_BNO085_RST, PIN_BNO085_INT);

sh2_SensorValue_t sensor_value;

void setReports(void) {
  printf("Setting desired reports\n");
  if (! bno085.enableReport(SH2_ROTATION_VECTOR, 10)) { //sh2.h line 93, RM: 2.2.4 Rotation Vector
    printf("Could not enable rotation vector\n");
  }
}

void setup_i2c0(void){
    i2c_init(i2c0, I2C0_BUADRATE);
    gpio_set_function(PIN_I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C0_SDA);
    gpio_pull_up(PIN_I2C0_SCL);
}

int main() {   

    stdio_init_all(); 

    setup_i2c0();

    if(bno085.connect_i2c(bno085_i2c_instance)) printf(" bno0855 has initialized\n");
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


