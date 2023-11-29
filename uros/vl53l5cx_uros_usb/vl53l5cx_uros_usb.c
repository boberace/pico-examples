#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/mutex.h" // mutex_t
#include "pico_uart_transports.h"
#include <math.h>
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/point_cloud2.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/point_field.h>
#include <rosidl_runtime_c/string_functions.h>

#include "vl53l5cx_api.h"



#define USING_PICO_W // uncomment if using pico-w so that we can blink the LED

#ifdef USING_PICO_W
#define led_toggle() cyw43_arch_gpio_put( CYW43_WL_GPIO_LED_PIN, !cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN));
#else
#define led_toggle() gpio_put(25, !gpio_get(25))
#endif

// define i2c pararmeters and pins
#define I2CA_BUADRATE 1000*1000
#define PIN_I2CA_SDA 12
#define PIN_I2CA_SCL 13
#define I2CA_INSTANCE i2c0
static const uint8_t LP_GPIO[]={22,27,28,10,11,15,20,21};

char id_data[] = "pico_vl53l5cx";

// define uart parameters and pins connected to picoprobe
#define UART_A_ID uart1
#define UART_A_BAUD_RATE 115200
#define UART_A_TX_PIN 8
#define UART_A_RX_PIN 9

#define TOF_FREQ 10
#define TOF_MS 1000/TOF_FREQ

#define BLINK_ERROR_FREQ 10
#define BLINK_NORMAL_FREQ 1

static mutex_t print_mutex;
#define BUFFER_SIZE 256
#define PRINT(...)                     \
    do {                               \
        mutex_enter_blocking(&print_mutex); \
        char buffer[BUFFER_SIZE];      \
        snprintf(buffer, sizeof(buffer), __VA_ARGS__); \
        uart_puts(UART_A_ID, buffer);      \
        mutex_exit(&print_mutex); \
    } while (0)

#define BLINK_ERROR 1000000/BLINK_ERROR_FREQ
#define BLINK_NORMAL 1000000/BLINK_NORMAL_FREQ

rcl_publisher_t publisher;
sensor_msgs__msg__PointCloud2 pc2_msg;  
static mutex_t tof_msg_mutex;
#define WIDTH 8
#define HEIGHT 8
#define POINT_STEP (sizeof(float) * 3) // Size of each point (x, y, z)
#define ROW_STEP (POINT_STEP * WIDTH) // Size of each row
#define DATA_SIZE (HEIGHT * ROW_STEP) // Total size of the data array
uint8_t pointcloud_data[DATA_SIZE]; // Static array for point cloud data
#define NUM_FIELDS 3 // x, y, z
sensor_msgs__msg__PointField point_fields[NUM_FIELDS];

uint blink_interval = BLINK_NORMAL;
uint imu_event_counter = 0;
uint publish_event_counter = 0;
uint timer_calback_counter = 0;

uint setup_uart(){
    uint uart_ret = uart_init(UART_A_ID, UART_A_BAUD_RATE);
    gpio_set_function(UART_A_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_A_RX_PIN, GPIO_FUNC_UART);
    return uart_ret;
}

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
        rcl_ret_t ret = rcl_publish(&publisher, &pc2_msg, NULL);      
    mutex_exit(&tof_msg_mutex);
    
    if (ret == RCL_RET_OK){
        publish_event_counter++;                           
    }
    timer_calback_counter++;
    // PRINT("\033[A\33[2K\rtcb: %i, pub: %i, tof %i\n", timer_calback_counter, publish_event_counter, 1);
}

void init_point_cloud2_message(){
    // define data for point cloud2 message
    pc2_msg.header.frame_id.data = id_data;
    
    pc2_msg.height = WIDTH;
    pc2_msg.width = HEIGHT;    
    pc2_msg.point_step = POINT_STEP;
    pc2_msg.row_step = ROW_STEP;
    pc2_msg.data.size = DATA_SIZE;
    pc2_msg.data.data = pointcloud_data;
    pc2_msg.is_bigendian = false; // RP2040 is little endian (Cortex M0+)
    pc2_msg.is_dense = false;

    // Define fields for x, y, z
    pc2_msg.fields.size = NUM_FIELDS; 
    // pc2_msg.fields.data = malloc(sizeof(sensor_msgs__msg__PointField) * pc2_msg.fields.size);

    sensor_msgs__msg__PointField__init(&point_fields[0]);
    rosidl_runtime_c__String__assign(&point_fields[0].name, "x");
    point_fields[0].offset = 0;
    point_fields[0].datatype = sensor_msgs__msg__PointField__FLOAT32;
    point_fields[0].count = 1;

    sensor_msgs__msg__PointField__init(&point_fields[1]);
    rosidl_runtime_c__String__assign(&point_fields[1].name, "y");
    point_fields[1].offset = sizeof(float);
    point_fields[1].datatype = sensor_msgs__msg__PointField__FLOAT32;
    point_fields[1].count = 1;

    sensor_msgs__msg__PointField__init(&point_fields[2]);
    rosidl_runtime_c__String__assign(&point_fields[2].name, "z");
    point_fields[2].offset = sizeof(float) * 2;
    point_fields[2].datatype = sensor_msgs__msg__PointField__FLOAT32;
    point_fields[2].count = 1;

    pc2_msg.fields.data = point_fields;
}


void core1_entry(){ 
    PRINT("\r\n-------------------------------core1_entry-------------------------------\r\n");

    // float cc[]={-0.3368898534,-0.2429801799,-0.1467304745,-0.04906767433,
    //                     0.04906767433, 0.1467304745, 0.2429801799, 0.3368898534};
    // float ss[]={  0.3368898534,   0.2429801799,   0.1467304745,   0.04906767433,
    //                     -0.04906767433, -0.1467304745, -0.2429801799,   -0.3368898534};
    float cc[8], ss[8];
    float sfov = M_PI_4; 
    float pfov = sfov / 8.0f;
    float offest = sfov*7/16;                     
    for(int i=0 ; i<8; i++){ 
            cc[i] = cos(i*pfov - offest - M_PI_2);
            ss[i] = sin((7-i)*pfov - offest);
    }

	/*********************************/
	/*   VL53L5CX ranging variables  */
	/*********************************/

	uint8_t 				status, isAlive, isReady;
    uint32_t 				integration_time_ms;
	VL53L5CX_Configuration 	Dev;			/* Sensor configuration */
	VL53L5CX_ResultsData 	Results;		/* Results data from VL53L5CX */

    setup_i2cA();
    // setup LP mode comms enable for all 8 sensors
    for(uint i=0; i<8; i++){
        gpio_init(LP_GPIO[i]);
        gpio_set_dir(LP_GPIO[i], GPIO_OUT);
        gpio_put(LP_GPIO[i], 0);
    }    
    // enable LP mode comms for the first sensor - this is the only one we will use
	gpio_put(LP_GPIO[0], 1);

    Dev.platform.i2c_instance = I2CA_INSTANCE;
    Dev.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;

	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/

	/* (Optional) Check if there is a VL53L5CX sensor connected */
	status = vl53l5cx_is_alive(&Dev, &isAlive);
	if(!isAlive || status)
	{
		PRINT("VL53L5CX not detected at requested address\n");
		return; // status;
	}

	/* (Mandatory) Init VL53L5CX sensor */
	status = vl53l5cx_init(&Dev);
	if(status)
	{
		PRINT("VL53L5CX ULD Loading failed\n");
		return; // status;
	}

	PRINT("VL53L5CX ULD ready ! (Version : %s)\n",
			VL53L5CX_API_REVISION);



	/*********************************/
	/*        Set some params        */
	/*********************************/

	/* Set resolution in 8x8. WARNING : As others settings depend to this
	 * one, it must be the first to use.
	 */
	status = vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_8X8);
	if(status)
	{
		PRINT("vl53l5cx_set_resolution failed, status %u\n", status);
		return; // status;
	}

	/* Set ranging frequency to 10Hz.
	 * Using 4x4, min frequency is 1Hz and max is 60Hz
	 * Using 8x8, min frequency is 1Hz and max is 15Hz
	 */
	status = vl53l5cx_set_ranging_frequency_hz(&Dev, 10);
	if(status)
	{
		PRINT("vl53l5cx_set_ranging_frequency_hz failed, status %u\n", status);
		return; // status;
	}

	/* Set target order to closest */
	status = vl53l5cx_set_target_order(&Dev, VL53L5CX_TARGET_ORDER_CLOSEST);
	if(status)
	{
		PRINT("vl53l5cx_set_target_order failed, status %u\n", status);
		return; // status;
	}

	/* Get current integration time */
	status = vl53l5cx_get_integration_time_ms(&Dev, &integration_time_ms);
	if(status)
	{
		PRINT("vl53l5cx_get_integration_time_ms failed, status %u\n", status);
		return; // status;
	}
	PRINT("Current integration time is : %d ms\n", integration_time_ms);

	/*********************************/
	/*         Ranging loop          */
	/*********************************/

	status = vl53l5cx_start_ranging(&Dev);

    while (true)
    {
		status = vl53l5cx_check_data_ready(&Dev, &isReady);

		if(isReady)
		{

			vl53l5cx_get_ranging_data(&Dev, &Results);

            mutex_enter_blocking(&tof_msg_mutex);

                // pc2_msg size and fields are initialized in the Core0 main function
                // todo: check for timestamp in sensor message and use it if it exists

                rcutils_time_point_value_t now;
                if(RCUTILS_RET_OK == rcutils_system_time_now(&now)){      
                    pc2_msg.header.stamp.sec = RCUTILS_NS_TO_S(now);
                    pc2_msg.header.stamp.nanosec = now - RCUTILS_S_TO_NS(pc2_msg.header.stamp.sec);
                }
                int pindex = 0;
                for(uint w=0; w<8; w++){
                    for(uint h=0; h<8; h++){                        
                        pindex = w*8+h;
                        float d_meter = Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*pindex] / 1000.0f;
                        d_meter = d_meter > 0 ? d_meter : 0;
                        // sensor distance is normal to plane and not radial. 
                        float z = d_meter * cc[w];
                        float x = d_meter * ss[h];
                        float y = d_meter;

                        size_t bindex = pindex * POINT_STEP;
                        memcpy(&pointcloud_data[bindex], &x, sizeof(float));
                        memcpy(&pointcloud_data[bindex + sizeof(float)], &y, sizeof(float));
                        memcpy(&pointcloud_data[bindex + 2 * sizeof(float)], &z, sizeof(float));  
                    }
                }           
  
            mutex_exit(&tof_msg_mutex);     

		} 

		/* Wait a few ms to avoid too high polling (function in platform
		 * file, not in API) */
		WaitMs(&(Dev.platform), 3);        

    }

    status = vl53l5cx_stop_ranging(&Dev);
	PRINT("vl53l5cx stop status %i\n", status);
    
}

int main()
{
    sleep_ms(2000);
    mutex_init(&print_mutex);
    init_point_cloud2_message();


#ifdef USING_PICO_W
    if (cyw43_arch_init()) {
        PRINT("Wi-Fi init failed");
        return -1;
    }
#else    
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
#endif

    uint uart_ret = setup_uart();
    if(abs((int)(uart_ret - UART_A_BAUD_RATE)) > 0.02*UART_A_BAUD_RATE){
        PRINT("UART setup failed %d\r\n", uart_ret);
        blink_interval = BLINK_ERROR;
    }
    PRINT("\r\nUART setup baud rate %d\r\n", uart_ret);

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
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, PointCloud2),
        "tof");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(TOF_MS),
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
