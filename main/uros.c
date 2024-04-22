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

#define SUB_LOOP_PERIOD_MS  100

#define RCCHECK(fn)                                                                                         \
	{                                                                                                       \
		rcl_ret_t temp_rc = fn;                                                                             \
		if ((temp_rc != RCL_RET_OK))                                                                        \
		{                                                                                                   \
			ESP_LOGE("SYSTEM-uROS", "Failed status on line %d: %d. > Aborting\n", __LINE__, (int)temp_rc);  \
            while(1);                                                                                       \
		}                                                                                                   \
	}
#define RCSOFTCHECK(fn)                                                                                     \
	{                                                                                                       \
		rcl_ret_t temp_rc = fn;                                                                             \
		if ((temp_rc != RCL_RET_OK))                                                                        \
		{                                                                                                   \
			ESP_LOGE("SYSTEM-uROS","Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc);  \
		}                                                                                                   \
	}

SemaphoreHandle_t got_uros_boot;

rcl_allocator_t allocator;

rclc_support_t support;

rcl_init_options_t init_options;

rcl_node_t node;

rclc_executor_t executor_publishers;
rclc_executor_t executor_subscribers;

rcl_publisher_t publisher_jointstate;

rcl_subscription_t subscriber;

std_msgs__msg__Int32 recv_msg;

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
geometry_msgs__msg__TwistStamped msgs_twiststamped;
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

static const char *TAG = "uROS";

static const int n_handles_pub = 1; //number of handles that will be added in executor (executor_add_...)
static const int n_handles_sub = 1; //number of handles that will be added in executor (executor_add_...)

void subscription_callback(const void * msgin) {
    std_msgs__msg__Int32 * msg = (std_msgs__msg__Int32 *) msgin;
    msg->data += 1;
    ESP_LOGI(TAG,"Received: %ld", msg->data);
}

void mc_callback() {
    rcl_ret_t rt = rcl_publish(&publisher_jointstate, &msgs_encoders, NULL);
    if(RMW_RET_OK != rt) {
        ESP_LOGI(TAG,"Error on publishing msgs");
        vTaskDelay(pdMS_TO_TICKS(10000));
        esp_restart();
    }
}

void init_mgs_encoders(){
    ESP_LOGI(TAG,"Init_jointstate configured");
    rosidl_runtime_c__String__Sequence mgs_encoders_name_sequence;
    rosidl_runtime_c__String__Sequence__init(&mgs_encoders_name_sequence, 2);
    msgs_encoders.name = mgs_encoders_name_sequence;
    rosidl_runtime_c__String__Sequence__fini(&mgs_encoders_name_sequence);
    rosidl_runtime_c__String__assignn(&msgs_encoders.name.data[0], "Right_sprocket_base_joint", 25);
    rosidl_runtime_c__String__assignn(&msgs_encoders.name.data[1], "Left_sprocket_base_joint", 24);

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

rcl_ret_t init_ping_struct(){
    rclc_support_t ping_support;
    //RCCHECK(rcl_init_options_fini(&ping_init_options));
    rcl_ret_t rc0 = rclc_support_init_with_options(&ping_support, 0, NULL, &init_options, &allocator);
    if (rc0 == RMW_RET_OK){
        RCCHECK(rclc_support_fini(&ping_support));
    }
    return rc0;
}

void ping_agent(){
    ESP_LOGI(TAG,"Searching agent...");
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
            ESP_LOGW(TAG,"Resuming...");
        } else {
            ESP_LOGE(TAG,"Impossible to find agent");
            ESP_LOGE(TAG,"Unstable connection! > Aborting");
            esp_restart();
        }
    }
}

void uros_task(void * arg) {

    init_mgs_encoders();

    allocator = rcl_get_default_allocator();

    init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    rmw_options->localhost_only = RMW_LOCALHOST_ONLY_DISABLED;
    rmw_options->security_options.enforce_security = RMW_SECURITY_ENFORCEMENT_PERMISSIVE;

    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));

#if (PING_AGENT == 1)
    ping_agent();
#endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(
        &node, 
        "uWABA", 
        "", 
        &support));
    ESP_LOGI(TAG,"Node created");

    const rosidl_message_type_support_t * type_support_p = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState);
    RCCHECK(rclc_publisher_init_best_effort(
        &publisher_jointstate, 
        &node, 
        type_support_p,
        "encoder"));
    ESP_LOGI(TAG,"Publisher created");

    const rosidl_message_type_support_t * type_support_s = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
    RCCHECK(rclc_subscription_init_best_effort(
		&subscriber,
		&node,
		type_support_s,
		"cmd_vel"));
    ESP_LOGI(TAG,"Subscriber created");

	RCCHECK(rclc_executor_init(&executor_publishers, &support.context, n_handles_pub, &allocator));
    RCCHECK(rclc_executor_init(&executor_subscribers, &support.context, n_handles_sub, &allocator)); 

    //RCCHECK(rclc_executor_set_timeout(&executor_publishers, RCL_MS_TO_NS(33)));
    RCCHECK(rclc_executor_add_subscription(&executor_subscribers, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));

    xSemaphoreGive(got_uros_boot);
    
    ESP_LOGI(TAG,"Executor spin");
    rclc_executor_spin(&executor_publishers);

    RCCHECK(rclc_executor_fini(&executor_publishers));
    RCCHECK(rclc_executor_fini(&executor_subscribers));
	RCCHECK(rcl_publisher_fini(&publisher_jointstate, &node));
    RCCHECK(rcl_subscription_fini(&executor_subscribers, &node));
	RCCHECK(rcl_node_fini(&node));
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

    vTaskDelete(NULL);
}