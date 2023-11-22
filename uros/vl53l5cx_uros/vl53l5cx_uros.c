#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pico_uart_transports.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/point_cloud2.h>
#include <rmw_microros/rmw_microros.h>

#include "hardware/i2c.h"


#include "pico/cyw43_arch.h"

#define USING_PICO_W // uncomment if using pico-w so that we can blink the LED

#ifdef USING_PICO_W
#define led_toggle() cyw43_arch_gpio_put( CYW43_WL_GPIO_LED_PIN, !cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN));
#else
#define led_toggle() gpio_put(25, !gpio_get(25))
#endif

// define i2c pararmeters and pins
#define I2CA_BUADRATE 400*1000
#define PIN_I2CA_SDA 16 
#define PIN_I2CA_SCL 17 
#define I2CA_INSTANCE i2c0

// define interrupt and reset pins
#define PIN_BNO085_INT 26 
#define PIN_BNO085_RST 14 



char id_data[] = "pico_vl53l5cx";

#define UART_A_ID uart1
#define UART_A_BAUD_RATE 115200
#define UART_A_TX_PIN 8
#define UART_A_RX_PIN 9

#define TOF_FREQ 10

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

uint blink_interval = BLINK_NORMAL;
uint imu_event_counter = 0;
uint publish_event_counter = 0;
uint timer_calback_counter = 0;

rcl_publisher_t publisher;
sensor_msgs__msg__PointCloud2 pc2_msg;
static mutex_t tof_msg_mutex;

#ifdef PRINT_PROBE_UART
uint setup_uart(){
    uint uart_ret = uart_init(UART_A_ID, UART_A_BAUD_RATE);
    gpio_set_function(UART_A_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_A_RX_PIN, GPIO_FUNC_UART);
    return uart_ret;
}
#endif

int setup_i2cA(void){
    int ret = i2c_init(I2CA_INSTANCE, I2CA_BUADRATE);
    gpio_set_function(PIN_I2CA_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2CA_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2CA_SDA);
    gpio_pull_up(PIN_I2CA_SCL);
    return ret;
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    
    mutex_enter_blocking(&tof_msg_mutex);
    // imu_msg.header.frame_id.data = id_data;
    // rcl_ret_t ret = rcl_publish(&publisher, &imu_msg, NULL);     
    mutex_exit(&tof_msg_mutex);
    
    // if (ret == RCL_RET_OK){
    //     publish_event_counter++;                           
    // }
    timer_calback_counter++;
    PRINT("\033[A\33[2K\rtcb: %i, pub: %i, tof %i\n", timer_calback_counter, publish_event_counter, 1);
}



void core1_entry(){

    PRINT("\r\nENTER MAIN2--------------------------------------\r\n");


    while (true)
    {
        sleep_ms(10);
        
        if (true) {            
            mutex_enter_blocking(&tof_msg_mutex);
            {


            }
            mutex_exit(&tof_msg_mutex);            
        }
    }
    
}

int main()
{
    sleep_ms(2000);
#ifdef USING_PICO_W
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed");
        return -1;
    }
#else    
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
#endif

#ifdef PRINT_PROBE_UART
    uint uart_ret = setup_uart();
    if(abs((int)(uart_ret - UART_A_BAUD_RATE)) > 0.02*UART_A_BAUD_RATE){
        PRINT("UART setup failed %d\r\n", uart_ret);
        blink_interval = BLINK_ERROR;
    }
    PRINT("\r\nUART setup baud rate %d\r\n", uart_ret);
#endif

    mutex_init(&tof_msg_mutex);
    multicore_launch_core1(core1_entry); // ****************************

    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

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
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "ppimu");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(100),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 1, &allocator);   
    rclc_executor_add_timer(&executor, &timer);

    uint64_t pt = to_us_since_boot(get_absolute_time());

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        uint64_t ct = to_us_since_boot(get_absolute_time());
        if(ct - pt >= blink_interval){  
            pt = ct;
            led_toggle();
        }

    }
    return 0;
}
