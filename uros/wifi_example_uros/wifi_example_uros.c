#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_wifi_transport.h"

#define ROS_AGENT_IP "10.42.0.1" // "192.168.0.142"

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    msg.data++;
}

int main()
{

    if(set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, ROS_AGENT_IP, 8888))
    {
        // printf("\e[1;1H\e[2J");
        printf("\33[2K\r");
        printf("Error setting up wifi transport\n");
        return -1;
    } else {
        // printf("\e[1;1H\e[2J");
        printf("\33[2K\r");
        printf("Wifi transport set up\n");
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    }


    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 2000; 
    const uint8_t attempts = 10;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        printf("\33[2K\r");
        printf("Agent unreachable. Exiting...\n");
        return ret;
    } else  {
        printf("\33[2K\r");
        printf("Agent reachable\n");
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);

    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    
    absolute_time_t pt = get_absolute_time();
    uint led_state = 1;
    msg.data = 0;
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

        absolute_time_t ct = get_absolute_time();
        if(ct - pt >= 500000){  
            pt = ct;
            led_state = led_state?0:1;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
        }
    }
    return 0;
}
