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

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

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

void uros_task(void *argument);
void cmdvel_sub_callback(const void *);
void motorcontrol_pub_callback();
void sensors_pub_callback();
void lidar_pub_callback();
static void init_msgs_encoders();
static void init_msgs_imu();
static void init_msgs_temperature();
static void init_msgs_batterypack();
static void init_msgs_laserscan();
static rcl_ret_t init_ping_struct();
static rcl_ret_t ping_agent();

extern void timestamp_update(void*arg);

static rmw_publisher_allocation_t laserscan_allocation;

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_init_options_t init_options;
static rcl_node_t node;

static rclc_executor_t executor_pub_msgs;
static rclc_executor_t executor_sub_msgs;

static rcl_publisher_t pub_msgs_encoders;
static rcl_publisher_t pub_msgs_imu;
static rcl_publisher_t pub_msgs_temperature;
static rcl_publisher_t pub_msgs_batterypack;
static rcl_publisher_t pub_msgs_laserscan;

static rcl_subscription_t sub_msgs_cmdvel;

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

static const char *TAG_MAIN = "Task-uROS";

static const int n_handles_pub = 5; //number of handles that will be added in executor (executor_add_...)
static const int n_handles_sub = 1; //number of handles that will be added in executor (executor_add_...)

extern volatile int8_t uros_status;
extern volatile int8_t uros_reset_semaphore;

void cmdvel_sub_callback(const void *msgin) {
    geometry_msgs__msg__TwistStamped *msgs_cmdvel = (geometry_msgs__msg__TwistStamped *) msgin;
    //ESP_LOGI(TAG_MAIN,"Received: %ld", msgs_cmdvel_temp->header.stamp.sec);
}

void motorcontrol_pub_callback() {
    CHECK(rcl_publish(&pub_msgs_encoders, &msgs_encoders, NULL));
}

void sensors_pub_callback() {
    CHECK(rcl_publish(&pub_msgs_imu, &msgs_imu, NULL));
    timestamp_update(&msgs_temperature);
    CHECK(rcl_publish(&pub_msgs_temperature, &msgs_temperature, NULL));
    timestamp_update(&msgs_batterypack);
    CHECK(rcl_publish(&pub_msgs_batterypack, &msgs_batterypack, NULL));
}

void lidar_pub_callback() {
    CHECK(rcl_publish(&pub_msgs_laserscan, &msgs_laserscan, &laserscan_allocation));
}

static void init_msgs_encoders(){
    ESP_LOGI(TAG_MAIN,"Init_JointState configured");
    rosidl_runtime_c__String__Sequence__init(&msgs_encoders_name_sequence, 2);
    msgs_encoders.name = msgs_encoders_name_sequence;
    rosidl_runtime_c__String__assignn(&msgs_encoders.name.data[0], (const char *)"Right_motor", 12);
    rosidl_runtime_c__String__assignn(&msgs_encoders.name.data[1], (const char *)"Left_motor", 11);
    // rosidl_runtime_c__String__Sequence__fini(&msgs_encoders_name_sequence);

    // msgs_encoders.position.capacity = sizeof(double) * 2;
    // msgs_encoders.position.size = 2;
    // msgs_encoders.position.data = (double*) malloc(msgs_encoders.position.capacity * sizeof(double));
    // msgs_encoders.position.data[0] = 0; msgs_encoders.position.data[1] = 0;

    msgs_encoders.velocity.capacity = 2;
    msgs_encoders.velocity.size = 2;
    msgs_encoders.velocity.data = (double*) malloc(msgs_encoders.velocity.capacity * sizeof(double));
    msgs_encoders.velocity.data[0] = 0; msgs_encoders.velocity.data[1] = 0;

    // msgs_encoders.effort.capacity = sizeof(double) * 2;
    // msgs_encoders.effort.size = 2;
    // msgs_encoders.effort.data = (double*) malloc(msgs_encoders.effort.capacity * sizeof(double));
    // msgs_encoders.effort.data[0] = 0; msgs_encoders.effort.data[1] = 0;

    msgs_encoders.header.frame_id.capacity = 7;
    msgs_encoders.header.frame_id.size = 6;
    msgs_encoders.header.frame_id.data = (char*) malloc(msgs_encoders.header.frame_id.capacity * sizeof(char));
    msgs_encoders.header.frame_id = micro_ros_string_utilities_init("motors");
    // msgs_encoders.header.frame_id.data = "motors";
    }

static void init_msgs_imu(){
    ESP_LOGI(TAG_MAIN,"Init_Imu configured");
    msgs_imu.header.frame_id.capacity = 4;
    msgs_imu.header.frame_id.size = 3;
    msgs_imu.header.frame_id.data = (char*) malloc(msgs_imu.header.frame_id.capacity * sizeof(char));
    msgs_imu.header.frame_id = micro_ros_string_utilities_init("imu");
    //msgs_imu.header.frame_id.data = "imu";

}

static void init_msgs_temperature(){
    ESP_LOGI(TAG_MAIN,"Init_Temperature configured");
    msgs_temperature.header.frame_id.capacity = 14;
    msgs_temperature.header.frame_id.size = 13;
    msgs_temperature.header.frame_id.data = (char*) malloc(msgs_temperature.header.frame_id.capacity * sizeof(char));
    msgs_temperature.header.frame_id = micro_ros_string_utilities_init("esp32 buildin");
    //msgs_temperature.header.frame_id.data = "esp32 buildin";

}

static void init_msgs_batterypack(){
    ESP_LOGI(TAG_MAIN,"Init_BatteryState configured");
    msgs_batterypack.header.frame_id.capacity = 12;
    msgs_batterypack.header.frame_id.size = 11;
    msgs_batterypack.header.frame_id.data = (char*) malloc(msgs_batterypack.header.frame_id.capacity * sizeof(char));
    msgs_batterypack.header.frame_id = micro_ros_string_utilities_init("batterypack");
    //msgs_batterypack.header.frame_id.data = "batterypack";

}

static void init_msgs_laserscan(){
    //https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
    //sensor_msgs__msg__LaserScan__init(&msgs_laserscan);
    //msgs_laserscan = sensor_msgs__msg__LaserScan__create();

    msgs_laserscan.header.frame_id.capacity = 12;
    msgs_laserscan.header.frame_id.size = 11;
    msgs_laserscan.header.frame_id.data = (char*) malloc(msgs_laserscan.header.frame_id.capacity * sizeof(char));

    //msgs_laserscan.header.frame_id.data = "laser_frame";
    //rosidl_runtime_c__String__init(&msgs_laserscan->header.frame_id);
    msgs_laserscan.header.frame_id = micro_ros_string_utilities_init("laser_frame");
    //msgs_laserscan->header.frame_id = micro_ros_string_utilities_set(msgs_laserscan->header.frame_id, "laser_frame");

    msgs_laserscan.angle_min = 0.0;
    msgs_laserscan.angle_max = (float)(2*M_PI) - msgs_laserscan.angle_increment;
    msgs_laserscan.angle_increment = (float)(M_PI/180);
    msgs_laserscan.time_increment = (float)msgs_laserscan.scan_time/360; // period/360 scans
    msgs_laserscan.scan_time = (float)1/7; //period scans
    msgs_laserscan.range_min = 0.12;
    msgs_laserscan.range_max = 16.0;

    msgs_laserscan.ranges.capacity = 360;
    msgs_laserscan.ranges.size = 360;
    msgs_laserscan.ranges.data = (float*) malloc(msgs_laserscan.ranges.capacity * sizeof(float));
    //msgs_laserscan.ranges.data = (float*) malloc(msgs_laserscan.ranges.capacity * sizeof(float));
    //(float*) calloc(msgs_laserscan.ranges.capacity, sizeof(float));

    //msgs_laserscan->intensities.capacity = 360;
    //msgs_laserscan->intensities.size = 360;
    //msgs_laserscan->intensities.data = (float*) malloc(msgs_laserscan->intensities.capacity * sizeof(float));
}

static rcl_ret_t init_ping_struct(){
    rclc_support_t ping_support;
    //CHECK(rcl_init_options_fini(&ping_init_options));
    rcl_ret_t rc0 = rclc_support_init_with_options(&ping_support, 0, NULL, &init_options, &allocator);
    if (rc0 == RMW_RET_OK){
        CHECK(rclc_support_fini(&ping_support));
    }
    return rc0;
}

static rcl_ret_t ping_agent(){
    ESP_LOGW(TAG_MAIN,"Searching agent...");
    rcl_ret_t rc = init_ping_struct();

    if (RMW_RET_OK == rc) { //timeout_ms, attempts
        ESP_LOGI(TAG_MAIN,"Agent found!");
        return RMW_RET_OK;
    } else {
        int uros_agent_attempts = 0;
        ESP_LOGW(TAG_MAIN,"Error on searching for agent");
        while (RMW_RET_OK != rc) {
            ESP_LOGW(TAG_MAIN,"Trying again: %d", uros_agent_attempts);
            rc = init_ping_struct();
            uros_agent_attempts++;
            if (uros_agent_attempts >= 3000){
                ESP_LOGE(TAG_MAIN,"Impossible to find agent! > Aborting");
                return RMW_RET_ERROR;
            }
            vTaskDelay(pdMS_TO_TICKS(2000));
            taskYIELD();
        }

        rc = init_ping_struct();
        if (RMW_RET_OK == rc) { //timeout_ms, attempts
            ESP_LOGI(TAG_MAIN,"Connection with agent established!");
            ESP_LOGI(TAG_MAIN,"Resuming...");
            return RMW_RET_OK;
        } else {
            ESP_LOGE(TAG_MAIN,"Unstable connection with agent! > Aborting");
            return RMW_RET_ERROR;
        }
    }
}

void uros_task(void * arg) {

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

    init_options = rcl_get_zero_initialized_init_options();
    CHECK(rcl_init_options_init(&init_options, allocator));

    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    rmw_options->localhost_only = RMW_LOCALHOST_ONLY_ENABLED;
    rmw_options->security_options.enforce_security = RMW_SECURITY_ENFORCEMENT_PERMISSIVE;

    CHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));

    uros_status = 2;

#if (PING_AGENT == 1)
    CHECK(ping_agent());
#endif

    CHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    uros_status = 3;

    init_msgs_encoders();
    init_msgs_imu();
    init_msgs_temperature();
    init_msgs_batterypack();
    init_msgs_laserscan();

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
    CHECK(rclc_subscription_init_best_effort(
		&sub_msgs_cmdvel,
		&node,
		type_support_msgs_cmdvel,
		"uwaba_controller_server_node/cmd_vel"));
    ESP_LOGI(TAG_MAIN,"CmdVel subscriber created");

    CHECK(rclc_executor_init(&executor_sub_msgs, &support.context, n_handles_sub, &allocator)); 
	CHECK(rclc_executor_init(&executor_pub_msgs, &support.context, n_handles_pub, &allocator));

    CHECK(rclc_executor_add_subscription(&executor_sub_msgs, &sub_msgs_cmdvel, &msgs_cmdvel, &cmdvel_sub_callback, ON_NEW_DATA));

    xSemaphoreGive(uros_boot_motorcontrol);
    xSemaphoreGive(uros_boot_sensors);
    xSemaphoreGive(uros_boot_lidar);

    uros_status = 1;

    while(1){
        CHECK(rclc_executor_spin_some(&executor_sub_msgs, RCL_MS_TO_NS(50)));
        CHECK(rclc_executor_spin_some(&executor_pub_msgs, RCL_MS_TO_NS(50)));
        //ESP_LOGI(TAG_MAIN,"Executor spin");
        taskYIELD();
        while(uros_reset_semaphore){
            vTaskDelay(pdMS_TO_TICKS(1000));
            ESP_LOGI(TAG_MAIN,"Clear memory");
            CHECK(rclc_executor_fini(&executor_sub_msgs));
            CHECK(rclc_executor_fini(&executor_pub_msgs));

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
            taskYIELD();
        }
    }

    ESP_LOGI(TAG_MAIN,"Clear memory");
    CHECK(rclc_executor_fini(&executor_sub_msgs));
    CHECK(rclc_executor_fini(&executor_pub_msgs));

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