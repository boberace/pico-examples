/*

Documents referenced in comments:
DS: BNO080/85/86 Data Sheet
RM: SH-2 Reference Manual v1.2
TP: Sensor Hub Transport Protocol R 1.7

*/
#include "bno085_i2c.hpp"
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <string.h>

// define i2c pararmeters and pins
#define I2CA_BUADRATE 400*1000 //400k fastest
#define PIN_I2CA_SDA 18
#define PIN_I2CA_SCL 19
#define I2CA_INSTANCE i2c1

// define interrupt and reset pins
#define PIN_BNO085_INT 0xFF 
#define PIN_BNO085_RST 0xFF 

bno085_i2c bno085(PIN_BNO085_RST, PIN_BNO085_INT);

#define PRINT_PROBE_UART // uncomment to print to uart

#ifdef PRINT_PROBE_UART

#define UART_A_ID uart0
#define UART_A_BAUD_RATE 115200
#define UART_A_TX_PIN 0
#define UART_A_RX_PIN 1

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

sh2_SensorValue_t sensor_value;

void setReports(void) {
  PRINT("Setting desired reports\r\n");
  if (! bno085.enableReport(SH2_ROTATION_VECTOR, 10)) { //sh2.h line 93, RM: 2.2.4 Rotation Vector
    PRINT("Could not enable rotation vector\r\n");
  }
}

int setup_i2cA(void){
    int ret = i2c_init(I2CA_INSTANCE, I2CA_BUADRATE);
    gpio_set_function(PIN_I2CA_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2CA_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2CA_SDA);
    gpio_pull_up(PIN_I2CA_SCL);
    return ret;
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
    
    int i2c_ret = setup_i2cA();
    if(abs(int(I2CA_BUADRATE-i2c_ret)) > 0.02*I2CA_BUADRATE ){
        PRINT("I2CA setup failed %d\r\n", i2c_ret);
        return 1;
    }
    PRINT("setup_i2cA baud rate %d\r\n", i2c_ret);

    if(bno085.connect_i2c(I2CA_INSTANCE)) PRINT(" bno0855 has initialized\r\n");
    else PRINT(" bno0855 NOT initialized\r\n");

    for (int n = 0; n < bno085.prodIds.numEntries; n++) { // sh2.h line 60
    PRINT("Part %d\r\n", bno085.prodIds.entry[n].swPartNumber);
    PRINT(": Version %d.%d.%d\r\n",  bno085.prodIds.entry[n].swVersionMajor, 
                                    bno085.prodIds.entry[n].swVersionMinor, 
                                    bno085.prodIds.entry[n].swVersionPatch);
    PRINT(" Build %d\r\n", bno085.prodIds.entry[n].swBuildNumber);
    }  

    setReports();   

    while(true){    

        sleep_ms(100);

        if (bno085.getSensorEvent(&sensor_value)) {

            switch (sensor_value.sensorId) {
            
            case SH2_ROTATION_VECTOR:
            PRINT("Rotation Vector:  i:%f, j:%f, k:%f,  r:%f\r\n",   sensor_value.un.gameRotationVector.i,
                                                                    sensor_value.un.gameRotationVector.j,
                                                                    sensor_value.un.gameRotationVector.k,
                                                                    sensor_value.un.gameRotationVector.real);
            break;
            }
        }

        if (bno085.wasReset()) {
            PRINT("sensor was reset \r\n");
            setReports();
        }

    }
    
    return 0;
}


