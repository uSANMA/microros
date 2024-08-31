#include <time.h>
#include <math.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <rmw_microros/rmw_microros.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <uros_network_interfaces.h>

#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <sensor_msgs/msg/temperature.h>
#include <sensor_msgs/msg/battery_state.h>

#include <std_msgs/msg/multi_array_dimension.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include "driver/gpio.h"

#define CHECK(fn) {                                                                                                         \
		rcl_ret_t temp_rc = fn;                                                                                             \
		if ((temp_rc != RCL_RET_OK)) {                                                                                      \
			ESP_LOGE("SYSTEM-uROS", "Failed status on line: %d > returned: %d > Aborting", __LINE__, (int)temp_rc);         \
            uros_status = -1;                                                                                               \
            while(1){                                                                                                       \
                taskYIELD();                                                                                                \
            }                                                                                                               \
		}                                                                                                                   \
	}

#define PING_AGENT  1

#define UROS_LOOP_PERIOD_MS          15 //ms
#define UROS_LOOP_ID                 0

#define GPIO_PIN_LED_GREEN          39 //gpio led pin 32
#define GPIO_PIN_LED_RED            42 //gpio led pin 35

void uros_task(void *argument);
void cmdvel_sub_callback(const void *);
void sensors_pub_callback();
void lidar_pub_callback();
static void init_msgs_encoders();
static void init_msgs_imu();
static void init_msgs_temperature();
static void init_msgs_batterypack();
static void init_msgs_laserscan();
static void uros_loop_cb(TimerHandle_t);

extern void timestamp_update(void*);

//static rmw_publisher_allocation_t laserscan_allocation;

rmw_init_options_t* rmw_options;

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_init_options_t init_options;
static rcl_node_t node;
static rclc_executor_t executor;

static rcl_publisher_t pub_msgs_encoders;
static rcl_publisher_t pub_msgs_imu;
static rcl_publisher_t pub_msgs_temperature;
static rcl_publisher_t pub_msgs_batterypack;
static rcl_publisher_t pub_msgs_laserscan;

static rcl_subscription_t sub_msgs_cmdvel;

static rmw_qos_profile_t qos_profile_custom;

static rosidl_runtime_c__String__Sequence msgs_encoders_name_sequence;
sensor_msgs__msg__JointState msgs_encoders;
sensor_msgs__msg__Imu msgs_imu;
sensor_msgs__msg__Temperature msgs_temperature;
sensor_msgs__msg__BatteryState msgs_batterypack;
sensor_msgs__msg__LaserScan msgs_laserscan;
geometry_msgs__msg__TwistStamped msgs_cmdvel;

SemaphoreHandle_t uros_boot_lidar;
SemaphoreHandle_t uros_boot_sensors;
SemaphoreHandle_t uros_boot_motorcontrol;

static SemaphoreHandle_t timer_uros_semaphore;
static TimerHandle_t uros_timer;

static const char *TAG_MAIN = "Task-uROS";

static const int n_handles = 1; //number of handles that will be added in executor (executor_add_...)

extern volatile int8_t uros_status;
extern volatile int8_t uros_reset_semaphore;

extern volatile uint8_t schedule_flag;

volatile uint16_t ping_counter = 0;

extern volatile int8_t lidar_reset_semaphore;

static void uros_loop_cb(TimerHandle_t xTimer) {
    xSemaphoreGive(timer_uros_semaphore);
}

void cmdvel_sub_callback(const void *msgin) {
    geometry_msgs__msg__TwistStamped *msgs_cmdvel = (geometry_msgs__msg__TwistStamped *) msgin;
    //ESP_LOGI(TAG_MAIN,"Received: %ld", msgs_cmdvel_temp->header.stamp.sec);
}

void sensors_pub_callback() {
    if (uros_status != -1) {
        CHECK(rcl_publish(&pub_msgs_encoders, &msgs_encoders, NULL));
        CHECK(rcl_publish(&pub_msgs_imu, &msgs_imu, NULL));
        // timestamp_update(&msgs_temperature); // TO-DO Insert at the sample acquisition
        // CHECK(rcl_publish(&pub_msgs_temperature, &msgs_temperature, NULL));
        // timestamp_update(&msgs_batterypack); // TO-DO Insert at the sample acquisition
        // CHECK(rcl_publish(&pub_msgs_batterypack, &msgs_batterypack, NULL));
    }
}

void lidar_pub_callback() {
    if (uros_status != -1) {
        CHECK(rcl_publish(&pub_msgs_laserscan, &msgs_laserscan, NULL));
    }
}

static void init_msgs_encoders(){
    ESP_LOGI(TAG_MAIN,"Init_JointState configuring");
    msgs_encoders.header.frame_id.capacity = 7;
    msgs_encoders.header.frame_id.size = 6;
    msgs_encoders.header.frame_id.data = (char*) malloc(msgs_encoders.header.frame_id.capacity * sizeof(char));
    msgs_encoders.header.frame_id = micro_ros_string_utilities_init("motors");
    
    rosidl_runtime_c__String__Sequence__init(&msgs_encoders_name_sequence, 2);
    msgs_encoders.name = msgs_encoders_name_sequence;
    rosidl_runtime_c__String__assignn(&msgs_encoders.name.data[0], (const char *)"Right_motor", 12);
    rosidl_runtime_c__String__assignn(&msgs_encoders.name.data[1], (const char *)"Left_motor", 11);
    // rosidl_runtime_c__String__Sequence__fini(&msgs_encoders_name_sequence);

    msgs_encoders.position.capacity = 2;
    msgs_encoders.position.size = 2;
    msgs_encoders.position.data = (double*) malloc(msgs_encoders.position.capacity * sizeof(double));
    msgs_encoders.position.data[0] = 0; msgs_encoders.position.data[1] = 0;

    msgs_encoders.velocity.capacity = 2;
    msgs_encoders.velocity.size = 2;
    msgs_encoders.velocity.data = (double*) malloc(msgs_encoders.velocity.capacity * sizeof(double));
    msgs_encoders.velocity.data[0] = 0; msgs_encoders.velocity.data[1] = 0;

    msgs_encoders.effort.capacity = 2;
    msgs_encoders.effort.size = 2;
    msgs_encoders.effort.data = (double*) malloc(msgs_encoders.effort.capacity * sizeof(double));
    msgs_encoders.effort.data[0] = 0; msgs_encoders.effort.data[1] = 0;

    }

static void init_msgs_imu(){
    ESP_LOGI(TAG_MAIN,"Init_Imu configuring");
    msgs_imu.header.frame_id.capacity = 4;
    msgs_imu.header.frame_id.size = 3;
    msgs_imu.header.frame_id.data = (char*) malloc(msgs_imu.header.frame_id.capacity * sizeof(char));
    msgs_imu.header.frame_id = micro_ros_string_utilities_init("imu");

    msgs_imu.orientation.w = 0;
    msgs_imu.orientation.x = 0;
    msgs_imu.orientation.y = 0;
    msgs_imu.orientation.w = 0;

    memset(&msgs_imu.orientation_covariance, 0, sizeof(msgs_imu.orientation_covariance));
    memset(&msgs_imu.linear_acceleration_covariance, 0, sizeof(msgs_imu.linear_acceleration_covariance));

}

static void init_msgs_temperature(){
    ESP_LOGI(TAG_MAIN,"Init_Temperature configuring");
    msgs_temperature.header.frame_id.capacity = 14;
    msgs_temperature.header.frame_id.size = 13;
    msgs_temperature.header.frame_id.data = (char*) malloc(msgs_temperature.header.frame_id.capacity * sizeof(char));
    msgs_temperature.header.frame_id = micro_ros_string_utilities_init("esp32 buildin");

}

static void init_msgs_batterypack(){
    ESP_LOGI(TAG_MAIN,"Init_BatteryState configuring");
    msgs_batterypack.header.frame_id.capacity = 12;
    msgs_batterypack.header.frame_id.size = 11;
    msgs_batterypack.header.frame_id.data = (char*) malloc(msgs_batterypack.header.frame_id.capacity * sizeof(char));
    msgs_batterypack.header.frame_id = micro_ros_string_utilities_init("batterypack");

}

static void init_msgs_laserscan(){
    ESP_LOGI(TAG_MAIN,"Init_LaserScan configuring");
    //https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html

    //sensor_msgs__msg__LaserScan__init(&msgs_laserscan);

    msgs_laserscan = *sensor_msgs__msg__LaserScan__create();

    msgs_laserscan.header.frame_id.capacity = 12;
    msgs_laserscan.header.frame_id.size = 11;
    msgs_laserscan.header.frame_id.data = (char*) malloc(msgs_laserscan.header.frame_id.capacity * sizeof(char));
    msgs_laserscan.header.frame_id = micro_ros_string_utilities_init("laser_frame");

    msgs_laserscan.angle_min = 0.0;
    msgs_laserscan.angle_max = (float)(2*M_PI) - msgs_laserscan.angle_increment;
    msgs_laserscan.angle_increment = (float)(M_PI/180);
    msgs_laserscan.time_increment = (float)msgs_laserscan.scan_time/360; // period/360 scans
    msgs_laserscan.scan_time = (float)1/7; //period scans
    msgs_laserscan.range_min = 0.12;
    msgs_laserscan.range_max = 13.0;

    msgs_laserscan.ranges.capacity = 360;
    msgs_laserscan.ranges.size = 0;
    msgs_laserscan.ranges.data = (float*) calloc(msgs_laserscan.ranges.capacity, sizeof(float));
    for (uint16_t i = 0; i < (uint16_t)msgs_laserscan.ranges.capacity; i++){
        msgs_laserscan.ranges.size = i+1;
        msgs_laserscan.ranges.data[i] = (float)0;
    }

    msgs_laserscan.intensities.capacity = 360;
    msgs_laserscan.intensities.size = 0;
    msgs_laserscan.intensities.data = (float*) calloc(msgs_laserscan.intensities.capacity, sizeof(float));
    for (uint16_t i = 0; i < (uint16_t)msgs_laserscan.intensities.capacity; i++){
        msgs_laserscan.intensities.size = i+1;
        msgs_laserscan.intensities.data[i] = (float)0;
    }

    //assert(rosidl_runtime_c__float32__Sequence__init(&msgs_laserscan.ranges, 360));
    //assert(rosidl_runtime_c__float32__Sequence__init(&msgs_laserscan.intensities, 0));

}

void uros_task(void * arg) {
    ESP_LOGI(TAG_MAIN, "Creating uros_task");
    vTaskDelay(pdMS_TO_TICKS(200));

    timer_uros_semaphore = xSemaphoreCreateBinary();

    uros_status = 0;

    uros_boot_sensors = xSemaphoreCreateBinary();
    uros_boot_motorcontrol = xSemaphoreCreateBinary();
    uros_boot_lidar = xSemaphoreCreateBinary();

    allocator = rcl_get_default_allocator();

    pub_msgs_encoders = rcl_get_zero_initialized_publisher();
    pub_msgs_imu = rcl_get_zero_initialized_publisher();
    pub_msgs_temperature = rcl_get_zero_initialized_publisher();
    pub_msgs_batterypack = rcl_get_zero_initialized_publisher();
    pub_msgs_laserscan = rcl_get_zero_initialized_publisher();

    sub_msgs_cmdvel = rcl_get_zero_initialized_subscription();

    init_msgs_encoders();
    init_msgs_imu();
    init_msgs_temperature();
    init_msgs_batterypack();
    init_msgs_laserscan();

    qos_profile_custom.depth = (size_t)10;
    qos_profile_custom.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    qos_profile_custom.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    qos_profile_custom.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    qos_profile_custom.history = RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT;
    // qos_profile_custom.liveliness_lease_duration = ;
    // qos_profile_custom.deadline = ;
    // qos_profile_custom.lifespan = ;

    init_options = rcl_get_zero_initialized_init_options();
    CHECK(rcl_init_options_init(&init_options, allocator));

    rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    rmw_options->localhost_only = RMW_LOCALHOST_ONLY_ENABLED;
    rmw_options->security_options.enforce_security = RMW_SECURITY_ENFORCEMENT_PERMISSIVE;

    CHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));

    uros_status = 2;

#if (PING_AGENT == 1)
    uros_status = 3;
    ESP_LOGW(TAG_MAIN,"Ping agent...");
    CHECK(rmw_uros_ping_agent_options((int)1000, (uint8_t)255, rmw_options));
    ESP_LOGI(TAG_MAIN,"Agent found!");
#endif

    CHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    node = rcl_get_zero_initialized_node();
    CHECK(rclc_node_init_default(
        &node, 
        "uWABA_node", 
        "", 
        &support));
    ESP_LOGI(TAG_MAIN,"Node created");

    const rosidl_message_type_support_t * type_support_msgs_encoders = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState);
    CHECK(rclc_publisher_init_best_effort(
        &pub_msgs_encoders, 
        &node, 
        type_support_msgs_encoders,
        "micro_encoders"));
    ESP_LOGI(TAG_MAIN,"Encoder publisher created");

    const rosidl_message_type_support_t * type_support_msgs_imu = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu);
    CHECK(rclc_publisher_init_best_effort(
        &pub_msgs_imu, 
        &node, 
        type_support_msgs_imu,
        "micro_imu"));
    ESP_LOGI(TAG_MAIN,"IMU publisher created");

    const rosidl_message_type_support_t * type_support_msgs_temperature = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature);
    CHECK(rclc_publisher_init_best_effort(
        &pub_msgs_temperature, 
        &node, 
        type_support_msgs_temperature,
        "micro_temperature"));
    ESP_LOGI(TAG_MAIN,"Temperature publisher created");

    const rosidl_message_type_support_t * type_support_msgs_batterypack = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState);
    CHECK(rclc_publisher_init_best_effort(
        &pub_msgs_batterypack, 
        &node, 
        type_support_msgs_batterypack,
        "micro_batterypack"));
    ESP_LOGI(TAG_MAIN,"Batterypack publisher created");

    const rosidl_message_type_support_t *type_support_msgs_laserscan = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan);
    CHECK(rclc_publisher_init_best_effort(
        &pub_msgs_laserscan, 
        &node, 
        type_support_msgs_laserscan,
        "micro_laserscan"));
    ESP_LOGI(TAG_MAIN,"Laserscan publisher created");

    const rosidl_message_type_support_t * type_support_msgs_cmdvel = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped);
    CHECK(rclc_subscription_init(
		&sub_msgs_cmdvel,
		&node,
		type_support_msgs_cmdvel,
		"uwaba_controller_server_node/cmd_vel",
        &qos_profile_custom));
    ESP_LOGI(TAG_MAIN,"CmdVel subscriber created");

    CHECK(rclc_executor_init(&executor, &support.context, n_handles, &allocator)); 

    rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(20));

    CHECK(rclc_executor_add_subscription(&executor, &sub_msgs_cmdvel, &msgs_cmdvel, &cmdvel_sub_callback, ON_NEW_DATA));

    xSemaphoreGive(uros_boot_motorcontrol);
    xSemaphoreGive(uros_boot_sensors);
    xSemaphoreGive(uros_boot_lidar);
    
    ESP_LOGI(TAG_MAIN, "Start leds timer loop");
    uros_timer = xTimerCreate("Leds timer", UROS_LOOP_PERIOD_MS, pdTRUE, (void *)UROS_LOOP_ID, &uros_loop_cb);
    if(uros_timer == NULL) {
        ESP_LOGE(TAG_MAIN, "Leds timer create Error!");
    } else {
        ESP_LOGI(TAG_MAIN, "Leds timer created!");
        if(xTimerStart(uros_timer, portMAX_DELAY) != pdPASS) {
            ESP_LOGE(TAG_MAIN, "Leds timer start error!");
        } else {
            ESP_LOGI(TAG_MAIN, "Leds timer started!");
        }
    }

    uros_status = 1;

    while(1){
        xSemaphoreTake(timer_uros_semaphore, pdMS_TO_TICKS(25));
        //ESP_LOGI(TAG_MAIN,"Executor spin");
        if (uros_status != -1) {
            CHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20)));
            schedule_flag = 1;
        }
        //CHECK(rclc_executor_spin_some(&executor_pub_msgs, RCL_MS_TO_NS(20)));

        ping_counter++;
        if (ping_counter > 100) {
            ping_counter = 0;
            if (rmw_uros_ping_agent_options((int)200, (uint8_t)1, rmw_options) != RMW_RET_OK){
                uros_status = -1;
                lidar_reset_semaphore = 1;
                gpio_set_level(GPIO_PIN_LED_GREEN, 0);
                gpio_set_level(GPIO_PIN_LED_RED,   1);
            } else {
                uros_status = 1;
                gpio_set_level(GPIO_PIN_LED_GREEN, 1);
                gpio_set_level(GPIO_PIN_LED_RED,   0);
            }
        }
        while(uros_reset_semaphore) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            ESP_LOGI(TAG_MAIN,"Clear memory");
            CHECK(rclc_executor_fini(&executor));
            //CHECK(rclc_executor_fini(&executor_pub_msgs));

	        CHECK(rcl_publisher_fini(&pub_msgs_encoders, &node));
            CHECK(rcl_publisher_fini(&pub_msgs_imu, &node));
            CHECK(rcl_publisher_fini(&pub_msgs_temperature, &node));
            CHECK(rcl_publisher_fini(&pub_msgs_batterypack, &node));
            CHECK(rcl_publisher_fini(&pub_msgs_laserscan, &node));

            CHECK(rcl_subscription_fini(&sub_msgs_cmdvel, &node));

	        CHECK(rcl_node_fini(&node));
            CHECK(rclc_support_fini(&support));
            uros_status = 0;
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
        taskYIELD();
    }

    ESP_LOGI(TAG_MAIN,"Clear memory");
    CHECK(rclc_executor_fini(&executor));
    //CHECK(rclc_executor_fini(&executor_pub_msgs));

	CHECK(rcl_publisher_fini(&pub_msgs_encoders, &node));
    CHECK(rcl_publisher_fini(&pub_msgs_imu, &node));
    CHECK(rcl_publisher_fini(&pub_msgs_temperature, &node));
    CHECK(rcl_publisher_fini(&pub_msgs_batterypack, &node));
    CHECK(rcl_publisher_fini(&pub_msgs_laserscan, &node));

    CHECK(rcl_subscription_fini(&sub_msgs_cmdvel, &node));

	CHECK(rcl_node_fini(&node));
    CHECK(rclc_support_fini(&support));

    // rc = rclc_executor_fini(&executor);
    // rc += rcl_publisher_fini(&my_pub, &my_node);
    // rc += rcl_timer_fini(&my_timer);
    // rc += rcl_subscription_fini(&my_sub, &my_node);
    // rc += rcl_node_fini(&my_node);
    // rc += rclc_support_fini(&support);
    // std_msgs__msg__String__fini(&pub_msg);
    // std_msgs__msg__String__fini(&sub_msg);

    ESP_LOGE(TAG_MAIN, "Task Delete");
    vTaskDelete(NULL);
}