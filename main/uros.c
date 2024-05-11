#include <time.h>

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

#define PING_AGENT  1

#define CHECK(fn)                                                                                           \
	{                                                                                                       \
		rcl_ret_t temp_rc = fn;                                                                             \
		if ((temp_rc != RCL_RET_OK))                                                                        \
		{                                                                                                   \
			ESP_LOGE("SYSTEM-uROS", "Failed status on line %d: %d. > Aborting\n", __LINE__, (int)temp_rc);  \
            while(1);                                                                                       \
		}                                                                                                   \
	}
#define SOFTCHECK(fn)                                                                                       \
	{                                                                                                       \
		rcl_ret_t temp_rc = fn;                                                                             \
		if ((temp_rc != RCL_RET_OK))                                                                        \
		{                                                                                                   \
			ESP_LOGE("SYSTEM-uROS","Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc);  \
		}                                                                                                   \
	}

static const char *TAG = "uROS";

void uros_task(void *argument);
void cmdvel_sub_callback(const void * msgin);
void mc_pub_callback();
void sensor_pub_callback();
void init_msgs_encoders();
void init_msgs_sensors();
void init_msgs_temperature();
void ping_agent();

SemaphoreHandle_t got_uros_boot;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t node;

rclc_executor_t executor_pub_msgs_encoders;
rcl_publisher_t pub_msgs_encoders;

rclc_executor_t executor_pub_msgs_imu;
rcl_publisher_t pub_msgs_imu;

rclc_executor_t executor_sub_msgs_cmdvel;
rcl_subscription_t sub_msgs_cmdvel;

rosidl_runtime_c__String__Sequence msgs_encoders_name_sequence;

sensor_msgs__msg__JointState msgs_encoders;
/*
    std_msgs__msg__Header header;
        builtin_interfaces__msg__Time stamp;
            int32_t sec;
            uint32_t nanosec;
        rosidl_runtime_c__String frame_id;
            char * data;
            size_t size;
            size_t capacity;

    rosidl_runtime_c__String__Sequence name;
        char * data;
        size_t size;
        size_t capacity;

    rosidl_runtime_c__double__Sequence position; TYPE_NAME = double
        TYPE_NAME * data;
        size_t size;
        size_t capacity;

    rosidl_runtime_c__double__Sequence velocity; TYPE_NAME = double
        TYPE_NAME * data;
        size_t size;
        size_t capacity;   

    rosidl_runtime_c__double__Sequence effort; TYPE_NAME = double
        TYPE_NAME * data;
        size_t size;
        size_t capacity;
*/
geometry_msgs__msg__TwistStamped msgs_cmdvel;
/*
    std_msgs__msg__Header header;
        builtin_interfaces__msg__Time stamp;
            int32_t sec;
            uint32_t nanosec;
        rosidl_runtime_c__String frame_id;
            char * data;
            size_t size;
            size_t capacity;

    geometry_msgs__msg__Twist twist;
        geometry_msgs__msg__Vector3 linear;
            double x;
            double y;
            double z;
        geometry_msgs__msg__Vector3 angular;
            double x;
            double y;
            double z;        
*/
sensor_msgs__msg__BatteryState msgs_batterypack;
/*
    std_msgs__msg__Header header;
        builtin_interfaces__msg__Time stamp;
            int32_t sec;
            uint32_t nanosec;
        rosidl_runtime_c__String frame_id;
            char * data;
            size_t size;
            size_t capacity;

    float voltage;
    float temperature;
    float current;
    float charge;
    float capacity;
    float design_capacity;
    float percentage;
    uint8_t power_supply_status;
    uint8_t power_supply_health;
    uint8_t power_supply_technology;
    bool present;

    rosidl_runtime_c__float__Sequence cell_voltage; TYPE_NAME = float
        TYPE_NAME * data;
        size_t size;
        size_t capacity;    

    rosidl_runtime_c__float__Sequence cell_temperature; TYPE_NAME = float
        TYPE_NAME * data;
        size_t size;
        size_t capacity;

    rosidl_runtime_c__String location;
        char * data;
        size_t size;
        size_t capacity;

    rosidl_runtime_c__String serial_number;
        char * data;
        size_t size;
        size_t capacity;
*/

sensor_msgs__msg__Temperature msgs_temperature;
/*
    std_msgs__msg__Header header;
        builtin_interfaces__msg__Time stamp;
            int32_t sec;
            uint32_t nanosec;
        rosidl_runtime_c__String frame_id;
            char * data;
            size_t size;
            size_t capacity;

    double temperature;
    double variance;
*/

sensor_msgs__msg__Imu msgs_imu;
/*
    std_msgs__msg__Header header;
        builtin_interfaces__msg__Time stamp;
            int32_t sec;
            uint32_t nanosec;
        rosidl_runtime_c__String frame_id;
            char * data;
            size_t size;
            size_t capacity;

    geometry_msgs__msg__Quaternion orientation;
        double x;
        double y;
        double z;
        double w;
    double orientation_covariance[9];

    geometry_msgs__msg__Vector3 angular_velocity;
        double x;
        double y;
        double z;
    double angular_velocity_covariance[9];

    geometry_msgs__msg__Vector3 linear_acceleration;
        double x;
        double y;
        double z;
    double linear_acceleration_covariance[9];
*/

sensor_msgs__msg__LaserScan msgs_lidar;
/*
    std_msgs__msg__Header header;
        builtin_interfaces__msg__Time stamp;
            int32_t sec;
            uint32_t nanosec;
        rosidl_runtime_c__String frame_id;
            char * data;
            size_t size;
            size_t capacity;

    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;

    rosidl_runtime_c__float__Sequence ranges; TYPE_NAME = float
        TYPE_NAME * data;
        size_t size;
        size_t capacity;

    rosidl_runtime_c__float__Sequence intensities; TYPE_NAME = float
        TYPE_NAME * data;
        size_t size;
        size_t capacity;
*/

static const int n_handles_pub = 1; //number of handles that will be added in executor (executor_add_...)
static const int n_handles_sub = 1; //number of handles that will be added in executor (executor_add_...)

void cmdvel_sub_callback(const void * msgin) {
    geometry_msgs__msg__TwistStamped * msg = (geometry_msgs__msg__TwistStamped *) msgin;
    ESP_LOGI(TAG,"Received: %ld", msg->header.stamp.sec);
}

void mc_pub_callback() {
    rcl_ret_t rt = rcl_publish(&pub_msgs_encoders, &msgs_encoders, NULL);
    if(RMW_RET_OK != rt) {
        ESP_LOGE(TAG,"Error on publishing encoders msgs");
        vTaskDelay(pdMS_TO_TICKS(10000));
        esp_restart();
    }
}

void sensor_pub_callback() {
    rcl_ret_t rt = rcl_publish(&pub_msgs_imu, &msgs_imu, NULL);
    if(RMW_RET_OK != rt) {
        ESP_LOGE(TAG,"Error on publishing imu msgs");
        vTaskDelay(pdMS_TO_TICKS(10000));
        esp_restart();
    }
}

void init_msgs_encoders(){
    ESP_LOGI(TAG,"Init_jointstate configured");
    rosidl_runtime_c__String__Sequence__init(&msgs_encoders_name_sequence, 2);
    msgs_encoders.name = msgs_encoders_name_sequence;
    rosidl_runtime_c__String__assignn(&msgs_encoders.name.data[0], (const char *)"Right_motor", 12);
    rosidl_runtime_c__String__assignn(&msgs_encoders.name.data[1], (const char *)"Left_motor", 11);
    //rosidl_runtime_c__String__Sequence__fini(&msgs_encoders_name_sequence);

    msgs_encoders.position.capacity = sizeof(double) * 2;
    msgs_encoders.position.size = 2;
    msgs_encoders.position.data = (double*) malloc(msgs_encoders.position.capacity * sizeof(double));
    msgs_encoders.position.data[0] = 0; msgs_encoders.position.data[1] = 0;

    msgs_encoders.velocity.capacity = sizeof(double) * 2;
    msgs_encoders.velocity.size = 2;
    msgs_encoders.velocity.data = (double*) malloc(msgs_encoders.velocity.capacity * sizeof(double));
    msgs_encoders.velocity.data[0] = 0; msgs_encoders.velocity.data[1] = 0;

    msgs_encoders.effort.capacity = sizeof(double) * 2;
    msgs_encoders.effort.size = 2;
    msgs_encoders.effort.data = (double*) malloc(msgs_encoders.effort.capacity * sizeof(double));
    msgs_encoders.effort.data[0] = 0; msgs_encoders.effort.data[1] = 0;

    msgs_encoders.header.frame_id.capacity = 7;
    msgs_encoders.header.frame_id.size = 6;
    msgs_encoders.header.frame_id.data = (char*) malloc(msgs_encoders.header.frame_id.capacity * sizeof(char));
    msgs_encoders.header.frame_id.data = "motors";
    }

void init_mgs_sensors(){
    msgs_imu.header.frame_id.capacity = 4;
    msgs_imu.header.frame_id.size = 3;
    msgs_imu.header.frame_id.data = (char*) malloc(msgs_imu.header.frame_id.capacity * sizeof(char));
    msgs_imu.header.frame_id.data = "imu";

}

void init_msgs_temperature(){
    msgs_temperature.header.frame_id.capacity = 14;
    msgs_temperature.header.frame_id.size = 13;
    msgs_temperature.header.frame_id.data = (char*) malloc(msgs_temperature.header.frame_id.capacity * sizeof(char));
    msgs_temperature.header.frame_id.data = "ESP32 Buildin";

}

rcl_ret_t init_ping_struct(){
    rclc_support_t ping_support;
    //CHECK(rcl_init_options_fini(&ping_init_options));
    rcl_ret_t rc0 = rclc_support_init_with_options(&ping_support, 0, NULL, &init_options, &allocator);
    if (rc0 == RMW_RET_OK){
        CHECK(rclc_support_fini(&ping_support));
    }
    return rc0;
}

void ping_agent(){
    ESP_LOGW(TAG,"Searching agent...");
    rcl_ret_t rc = init_ping_struct();

    if (RMW_RET_OK == rc) { //timeout_ms, attempts
        ESP_LOGI(TAG,"Agent found!");
    } else {
        int uros_agent_attempts = 0;
        ESP_LOGE(TAG,"Error searching for agent");
        while (RMW_RET_OK != rc) {
            ESP_LOGW(TAG,"Trying again: %d", uros_agent_attempts);
            rc = init_ping_struct();
            uros_agent_attempts++;
            if (uros_agent_attempts >= 300){esp_restart();}
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        rc = init_ping_struct();
        if (RMW_RET_OK == rc) { //timeout_ms, attempts
            ESP_LOGI(TAG,"Connection with agent reestablished!");
            ESP_LOGI(TAG,"Resuming...");
        } else {
            ESP_LOGE(TAG,"Impossible to find agent");
            ESP_LOGE(TAG,"Unstable connection! > Aborting");
            esp_restart();
        }
    }
}

void uros_task(void * arg) {

    got_uros_boot = xSemaphoreCreateBinary();

    init_msgs_encoders();
    init_msgs_sensors();

    allocator = rcl_get_default_allocator();

    init_options = rcl_get_zero_initialized_init_options();
    CHECK(rcl_init_options_init(&init_options, allocator));

    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    rmw_options->localhost_only = RMW_LOCALHOST_ONLY_DISABLED;
    rmw_options->security_options.enforce_security = RMW_SECURITY_ENFORCEMENT_PERMISSIVE;

    CHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));

#if (PING_AGENT == 1)
    ping_agent();
#endif

    CHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    node = rcl_get_zero_initialized_node();
    CHECK(rclc_node_init_default(
        &node, 
        "uWABA", 
        "", 
        &support));
    ESP_LOGI(TAG,"Node created");

    const rosidl_message_type_support_t * type_support_msgs_encoders = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState);
    CHECK(rclc_publisher_init_best_effort(
        &pub_msgs_encoders, 
        &node, 
        type_support_msgs_encoders,
        "encoder"));
    ESP_LOGI(TAG,"Encoder publisher created");

    const rosidl_message_type_support_t * type_support_msgs_imu = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu);
    CHECK(rclc_publisher_init_best_effort(
        &pub_msgs_imu, 
        &node, 
        type_support_msgs_imu,
        "micro_imu"));
    ESP_LOGI(TAG,"IMU publisher created");

    const rosidl_message_type_support_t * type_support_msgs_cmdvel = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped);
    CHECK(rclc_subscription_init_best_effort(
		&sub_msgs_cmdvel,
		&node,
		type_support_msgs_cmdvel,
		"cmd_vel"));
    ESP_LOGI(TAG,"CmdVel subscriber created");

	CHECK(rclc_executor_init(&executor_pub_msgs_encoders, &support.context, n_handles_pub, &allocator));
    CHECK(rclc_executor_init(&executor_pub_msgs_imu, &support.context, n_handles_pub, &allocator));
    CHECK(rclc_executor_init(&executor_sub_msgs_cmdvel, &support.context, n_handles_sub, &allocator)); 

    //CHECK(rclc_executor_set_timeout(&executor_publishers, RCL_MS_TO_NS(33)));
    CHECK(rclc_executor_add_subscription(&executor_sub_msgs_cmdvel, &sub_msgs_cmdvel, &msgs_cmdvel, &cmdvel_sub_callback, ON_NEW_DATA));

    xSemaphoreGive(got_uros_boot);
    
    ESP_LOGI(TAG,"Executor spin");
    rclc_executor_spin(&executor_pub_msgs_encoders);
    rclc_executor_spin(&executor_pub_msgs_imu);

    CHECK(rclc_executor_fini(&executor_pub_msgs_encoders));
    CHECK(rclc_executor_fini(&executor_pub_msgs_imu));
    CHECK(rclc_executor_fini(&executor_sub_msgs_cmdvel));
	CHECK(rcl_publisher_fini(&pub_msgs_encoders, &node));
    CHECK(rcl_publisher_fini(&pub_msgs_imu, &node));
    CHECK(rcl_subscription_fini(&sub_msgs_cmdvel, &node));
	CHECK(rcl_node_fini(&node));
    rclc_support_fini(&support);
    ESP_LOGI(TAG,"Clear memory");

    // rc = rclc_executor_fini(&executor);
    // rc += rcl_publisher_fini(&my_pub, &my_node);
    // rc += rcl_timer_fini(&my_timer);
    // rc += rcl_subscription_fini(&my_sub, &my_node);
    // rc += rcl_node_fini(&my_node);
    // rc += rclc_support_fini(&support);
    // std_msgs__msg__String__fini(&pub_msg);
    // std_msgs__msg__String__fini(&sub_msg);

    ESP_LOGW(TAG, "Task Delete");
    vTaskDelete(NULL);
}