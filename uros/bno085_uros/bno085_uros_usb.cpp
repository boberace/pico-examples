#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
// #include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include "hardware/spi.h"
#include "bno085_spi.hpp"

#define PIN_BNO085_MISO 12
#define PIN_BNO085_CS   13
#define PIN_BNO085_SCK  10
#define PIN_BNO085_MOSI 11
#define PIN_BNO085_INT  14
#define PIN_BNO085_RST  15

#define SPI_A_BAUD_RATE  1000 * 1000
#define SPI_A_INST spi1

#define UART_A_ID uart1
#define UART_A_BAUD_RATE 115200
#define UART_A_TX_PIN 8
#define UART_A_RX_PIN 9

#define REPORTS_PER_SECOND 10

#define BLINK_ERROR_FREQ 10
#define BLINK_NORMAL_FREQ 1

#define PRINT_PROBE_UART // comment out to not print to UART

#ifdef PRINT_PROBE_UART
#define BUFFER_SIZE 256
#define PRINT(...)                     \
    do {                               \
        char buffer[BUFFER_SIZE];      \
        snprintf(buffer, sizeof(buffer), __VA_ARGS__); \
        uart_puts(UART_A_ID, buffer);      \
    } while (0)
#else
#define PRINT(...)
#endif

#define BLINK_ERROR 1000000/BLINK_ERROR_FREQ
#define BLINK_NORMAL 1000000/BLINK_NORMAL_FREQ

bno085_spi bno085(PIN_BNO085_CS, PIN_BNO085_RST, PIN_BNO085_INT);
sh2_SensorValue_t sensor_value;

const uint LED_PIN = 25;
uint blink_interval = BLINK_NORMAL;
uint imu_event_counter = 0;
uint publish_event_counter = 0;
uint timer_calback_counter = 0;

rcl_publisher_t publisher;
// std_msgs__msg__Int32 msg;
sensor_msgs__msg__Imu imu_msg;

#ifdef PRINT_PROBE_UART
uint setup_uart(){
    uint uart_ret = uart_init(UART_A_ID, UART_A_BAUD_RATE);
    gpio_set_function(UART_A_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_A_RX_PIN, GPIO_FUNC_UART);
    return uart_ret;
}
#endif

uint setup_spia(){

    uint spi_ret = spi_init(SPI_A_INST, SPI_A_BAUD_RATE);
    gpio_set_function(PIN_BNO085_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_BNO085_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_BNO085_MOSI, GPIO_FUNC_SPI); 

    return spi_ret;
}

uint setReports(void) {
  PRINT("Setting desired reports\r\n");
  if (!bno085.enableReport(SH2_ROTATION_VECTOR, REPORTS_PER_SECOND)) { //sh2.h line 93, RM: 2.2.4 Rotation Vector
    PRINT("Could not enable rotation vector\r\n");
    blink_interval = BLINK_ERROR;
  } PRINT("Rotation vector enabled\r\n");
//   if (!bno085.enableReport(SH2_ACCELEROMETER, REPORTS_PER_SECOND)) {
//     PRINT("Could not enable accelerometer");
//     blink_interval = BLINK_ERROR;
//   }
//   if (!bno085.enableReport(SH2_GYROSCOPE_CALIBRATED, REPORTS_PER_SECOND)) {
//     PRINT("Could not enable gyroscope");
//     blink_interval = BLINK_ERROR;
//   }
    return 0;
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    timer_calback_counter++;
    rcl_ret_t ret = rcl_publish(&publisher, &imu_msg, NULL);
    // rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    if (ret == RCL_RET_OK){
        publish_event_counter++;
        if (bno085.getSensorEvent(&sensor_value)) {
            imu_event_counter++;
            switch (sensor_value.sensorId) {      
                     
            case SH2_ROTATION_VECTOR:
                imu_msg.orientation.w = sensor_value.un.gameRotationVector.real;
                imu_msg.orientation.x = sensor_value.un.gameRotationVector.i;
                imu_msg.orientation.y = sensor_value.un.gameRotationVector.j;
                imu_msg.orientation.z = sensor_value.un.gameRotationVector.k;
                // msg.data = sensor_value.un.gameRotationVector.real;
            break;
            // case SH2_ACCELEROMETER:
            //     imu_msg.linear_acceleration.x = sensor_value.un.accelerometer.x;
            //     imu_msg.linear_acceleration.y = sensor_value.un.accelerometer.y;
            //     imu_msg.linear_acceleration.z = sensor_value.un.accelerometer.z; 
            // break;
            // case SH2_GYROSCOPE_CALIBRATED:
            //     imu_msg.angular_velocity.x = sensor_value.un.gyroscope.x;
            //     imu_msg.angular_velocity.y = sensor_value.un.gyroscope.y;
            //     imu_msg.angular_velocity.z = sensor_value.un.gyroscope.z;
            // break;
            default:
            break;

            }
        }       
                
    }
    PRINT("\033[A\33[2K\rtcb: %i, pub: %i, imu %i\n", timer_calback_counter, publish_event_counter, imu_event_counter);
}

int main()
{
#ifdef PRINT_PROBE_UART
    uint uart_ret = setup_uart();
    if(abs(int(uart_ret - UART_A_BAUD_RATE)) > 0.02*UART_A_BAUD_RATE){
        PRINT("UART setup failed %d\r\n", uart_ret);
        blink_interval = BLINK_ERROR;
    }
    PRINT("\r\nUART setup baud rate %d\r\n", uart_ret);
#endif

    int spi_ret = setup_spia();
    if(abs(int(SPI_A_BAUD_RATE-spi_ret)) > 0.02*SPI_A_BAUD_RATE ){
        PRINT("SPIA setup failed %d\r\n", spi_ret);
        blink_interval = BLINK_ERROR;
    }
    PRINT("setup_spi baud rate %d\r\n", spi_ret);

    if(bno085.connect_spi(SPI_A_INST)) PRINT(" bno0855 has initialized\r\n");
    else {
         blink_interval = BLINK_ERROR;
        PRINT(" bno0855 NOT initialized\r\n");
    }

    for (int n = 0; n < bno085.prodIds.numEntries; n++) { // sh2.h line 60
    PRINT("Part %d\n", bno085.prodIds.entry[n].swPartNumber);
    PRINT(": Version %d.%d.%d\n",  bno085.prodIds.entry[n].swVersionMajor, 
                                    bno085.prodIds.entry[n].swVersionMinor, 
                                    bno085.prodIds.entry[n].swVersionPatch);
    PRINT(" Build %d\n", bno085.prodIds.entry[n].swBuildNumber);
    } 

    if(setReports() !=0){
        blink_interval = BLINK_ERROR;
    }

    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 10;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        PRINT("\nAgent not found!\n");
        return ret;
    } else {
        PRINT("\nAgent found!\n");
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_default(
        &publisher,
        &node,
        // ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "ppimu");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    gpio_put(LED_PIN, 1);
    uint64_t pt = to_us_since_boot(get_absolute_time());
    uint led_state = 1;
    // msg.data = 0;
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        uint64_t ct = to_us_since_boot(get_absolute_time());
        if(ct - pt >= blink_interval){  
            pt = ct;
            led_state = led_state?0:1;
            gpio_put(LED_PIN, led_state);
        }
    }
    return 0;
}
