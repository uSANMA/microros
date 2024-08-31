#include <string.h>
#include "math.h"

#include "esp_log.h"
#include "esp_types.h"

#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <geometry_msgs/msg/twist_stamped.h>

#include "driver/temperature_sensor.h"
#include <sensor_msgs/msg/temperature.h>

#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"

#define CHECK(fn) {                                                                                                         \
		esp_err_t temp_rc = fn;                                                                                             \
		if ((temp_rc != ESP_OK)) {                                                                                          \
			ESP_LOGE("SYSTEM-SENSORS", "Failed status on line: %d > returned: %d > Aborting", __LINE__, (int)temp_rc);      \
            sensors_status = -1;                                                                                            \
            while(1){                                                                                                       \
                taskYIELD();                                                                                                \
            }                                                                                                               \
		}                                                                                                                   \
	}

#define MCPWM_TIMER_RESOLUTION_HZ                               10000000 // 10MHz, 1 tick = 0.1us
#define MCPWM_FREQ_HZ                                           16000    // 16KHz PWM
#define MCPWM_DUTY_TICK_MAX                                     (MCPWM_TIMER_RESOLUTION_HZ / MCPWM_FREQ_HZ) - 20 // maximum value we can set for the duty cycle, in ticks

#define MCPWM_GPIO_A1                                           3 //forward in2 H brigde
#define MCPWM_GPIO_A2                                           10 //reverse in1 H bridge
#define ENCODER_GPIO_A1                                         17 //C2 motor A
#define ENCODER_GPIO_A2                                         18 //C1 motor A
#define ENCODER_PCNT_HIGH_LIMIT_A                               600
#define ENCODER_PCNT_LOW_LIMIT_A                                -600

#define MCPWM_GPIO_B1                                           11 //forward in1 H brigde
#define MCPWM_GPIO_B2                                           12 //reverse in2 H bridge
#define ENCODER_GPIO_B1                                         8 //C1 motor A
#define ENCODER_GPIO_B2                                         13 //C2 motor A
#define ENCODER_PCNT_HIGH_LIMIT_B                               600
#define ENCODER_PCNT_LOW_LIMIT_B                                -600

#define SENSORS_LOOP_PERIOD_MS                                  31 //us 0.03125s = 31.25ms = 31250us
#define SENSORS_LOOP_ID                                         0

#define GPIO_SDA                                                5 //gpio
#define GPIO_SCL                                                4 //gpio
#define I2C_FREQ                                                900000
#define I2C_NUM                                                 0
#define I2C_TIMEOUT_MS                                          10

#define GRAVITY_EARTH  (9.80665f)

void sensors_task(void *argument);
static void sensors_loop_cb(TimerHandle_t);
static esp_err_t imu_read();
static esp_err_t imu_softreset();
static esp_err_t imu_init();
static esp_err_t fuelgauge_init();
static esp_err_t temp_init();
static float lsb_to_dps(int16_t val, float dps);
static float lsb_to_mps(int16_t val, int8_t g_range);

extern void sensors_pub_callback();
extern void timestamp_update(void*arg);

static const char *TAG_MAIN = "Task-Sensors";
static const char *TAG_IMU = "IMU";
static const char *TAG_FG = "Fuel_Gauge";
static const char *TAG_ESP32TEMP = "ESP32_SENSORS";

static const uint8_t FG_ADDRESS =                        0x36;
static const uint8_t FG_CHIP_ID_REG =                    0x01; //chip_id[7-0]
static const uint16_t FG_CHIP_ID_RESET =                 0x4020;

static const uint8_t IMU_ADDRESS =                       0x68; //alternative 0X69 if SDO=VDD 0b 36
static const uint8_t IMU_CHIP_ID_REG =                   0x00; //chip_id[7-0]
static const uint16_t IMU_CHIP_ID_RESET =                0x0043;
//static const uint8_t IMU_STATUS_REG =                    0x02; //drdy_acc[7], drdy_gyr[6], drdy_temp[55]
static const uint8_t IMU_ACC_X_REG =                     0x03; //bit[15-0]
//static const uint8_t IMU_ACC_Y_REG =                     0x04; //bit[15-0]
//static const uint8_t IMU_ACC_Z_REG =                     0x05; //bit[15-0]
//static const uint8_t IMU_GYR_X_REG =                     0x06; //bit[15-0]
//static const uint8_t IMU_GYR_Y_REG =                     0x07; //bit[15-0]
//static const uint8_t IMU_GYR_Z_REG =                     0x08; //bit[15-0]
//static const uint8_t IMU_TEMP_REG =                      0x09; //bit[15-0]
static const uint8_t IMU_ACC_CONF_REG =                  0x20; //acc_mode[14-12], acc_avg_num[10-8], acc_bw[7], acc_range[6-4], acc_odr[3-0]
static const uint8_t IMU_GYR_CONF_REG =                  0x21; //gyr_mode[14-12], gyr_avg_num[10-8], gyr_bw[7], gyr_range[6-4], gyr_odr[3-0]
//static const uint8_t IMU_FEATURE_CTRL_REG =              0x40; //bit[0]
static const uint8_t IMU_CMD_REG =                       0x7E; //bit[15-0]

static const uint8_t IMU_CMD_SOFTRESET[3] = {IMU_CMD_REG, 0xAF, 0xDE}; //softreset on imu cmd reg

/*
acc_mode high-performance = 0b111
acc_avg_num 64 samples = 0b110
acc_bw -3dB cut-off = 0b1
acc_range 4g 8,19 LSB/mg = 0b001
acc_odr 50Hz = 0b0111
result = ACC_CONF = 0b0111011010010111 = 0x7697

4 
*/
static const uint8_t IMU_ACC_CONF[3] = {IMU_ACC_CONF_REG, 0x97, 0x76};
static const uint16_t IMU_ACC_CONF_RESET = 0x0028;

/*
gyr_mode high-performance = 0b111
gyr_avg_num 64 samples = 0b110
gyr_bw -3dB cut-off = 0b1
gyr_range 500º/s 65,536 LSB/º/s = 0b010
gyr_odr 50Hz = 0b0111
result = IMU_GYR_CONF = 0b0111011010100111 = 0x76A7
*/
static const uint8_t IMU_GYR_CONF[3]  = {IMU_GYR_CONF_REG, 0xA7, 0x76};
static const uint16_t IMU_GYR_CONF_RESET = 0x0048;

extern volatile int8_t sensors_status;
extern volatile int8_t sensors_reset_semaphore;

static SemaphoreHandle_t timer_sensors_semaphore;
static TimerHandle_t sensors_timer;
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t imu_handle;
static i2c_master_dev_handle_t fg_handle;

extern SemaphoreHandle_t uros_boot_sensors;
extern sensor_msgs__msg__Imu msgs_imu;
extern sensor_msgs__msg__Temperature msgs_temperature;
extern sensor_msgs__msg__JointState msgs_encoders;
extern geometry_msgs__msg__TwistStamped msgs_cmdvel;


static float PID_EXPECT_SPEED_A = 0;  // expected motor speed, in the pulses counted by the rotary encoder
static char orientation_motora = 'F';

static float PID_EXPECT_SPEED_B = 0;  // expected motor speed, in the pulses counted by the rotary encoder
static char orientation_motorb = 'F';

static const float wheels_separation = 0.13607;
static const float reduction_ratio = 18.8;
static const uint8_t encoder_ticks = 10; 

typedef struct {
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    int report_pulses;
} motor_control_context_t;

motor_control_context_t motor_ctrl_ctx_a = {
    .pcnt_encoder = NULL,
};

motor_control_context_t motor_ctrl_ctx_b = {
    .pcnt_encoder = NULL,
};

static void sensors_loop_cb(TimerHandle_t xTimer) {
    xSemaphoreGive(timer_sensors_semaphore);
}

static esp_err_t temp_init(){
    ESP_LOGI(TAG_ESP32TEMP,"Initializing internal temp sensor");
    temperature_sensor_handle_t temp_handle = NULL;
    temperature_sensor_config_t temp_sensor = {
        .range_min = 10,
        .range_max = 50,
    };
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor, &temp_handle));
    // Enable temperature sensor
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));
    // Get converted sensor data
    float tsens_out;
    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_handle, &tsens_out));
    ESP_LOGI(TAG_ESP32TEMP,"Temp: %.4f", tsens_out);
    timestamp_update(&msgs_temperature);
    msgs_temperature.temperature = tsens_out;
    // Disable the temperature sensor if it is not needed and save the power
    ESP_ERROR_CHECK(temperature_sensor_disable(temp_handle));
    return ESP_OK;
}

static esp_err_t imu_read(){
    static  volatile uint8_t raw_buffer[16];
    CHECK(i2c_master_transmit_receive(imu_handle,  &IMU_ACC_X_REG, 1, (uint8_t *)&raw_buffer, 16, I2C_TIMEOUT_MS));
    timestamp_update(&msgs_imu);
    //ESP_LOG_BUFFER_HEXDUMP(TAG_MAIN, &raw_buffer, 16, ESP_LOG_INFO);


    if (raw_buffer[0] != 0 && raw_buffer [1] != 0){
        return ESP_FAIL;
    }
    msgs_imu.linear_acceleration.x = lsb_to_mps(((int16_t)(raw_buffer[3] << 8 | raw_buffer[2])), (float)4); //Default axis Z
    msgs_imu.linear_acceleration.y = lsb_to_mps(((int16_t)(raw_buffer[5] << 8 | raw_buffer[4])), (float)4); //Default axis X
    msgs_imu.linear_acceleration.z = lsb_to_mps(((int16_t)(raw_buffer[7] << 8 | raw_buffer[6])), (float)4); //Default axis Y

    msgs_imu.angular_velocity.x = (lsb_to_dps(((int16_t)(raw_buffer[9] << 8 | raw_buffer[8])), (float)500) * (float)(M_PI/180)); //Default axis Z
    msgs_imu.angular_velocity.y = (lsb_to_dps(((int16_t)(raw_buffer[11] << 8 | raw_buffer[10])), (float)500) * (float)(M_PI/180)); //Default axis X
    msgs_imu.angular_velocity.z = (lsb_to_dps(((int16_t)(raw_buffer[13] << 8 | raw_buffer[12])), (float)500) * (float)(M_PI/180)); //Default axis Y

    msgs_temperature.temperature = (((int16_t)(raw_buffer[15] << 8 | raw_buffer[14]))/512) + 23;

    //ESP_LOGI(TAG_IMU,"AX: %d", imu_data.accel_x);
    //ESP_LOGI(TAG_IMU,"AY: %d", imu_data.accel_y);
    //ESP_LOGI(TAG_IMU,"AZ: %d", imu_data.accel_z);
    //ESP_LOGI(TAG_IMU,"GX: %d", imu_data.gyro_x);
    //ESP_LOGI(TAG_IMU,"GY: %d", imu_data.gyro_y);
    //ESP_LOGI(TAG_IMU,"GZ: %d", imu_data.gyro_z);
    //ESP_LOGI(TAG_IMU,"T: %d", imu_data.temperature);

    return ESP_OK;
}

static esp_err_t imu_softreset(){
    CHECK(i2c_master_transmit(imu_handle, (uint8_t *)&IMU_CMD_SOFTRESET, 3, I2C_TIMEOUT_MS));
    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
}

static esp_err_t imu_init(){
    ESP_LOGI(TAG_IMU,"Initializing IMU");
    static volatile uint8_t rt_buffer[4];
    CHECK(i2c_master_transmit_receive(imu_handle,  &IMU_CHIP_ID_REG, 1, (uint8_t *)&rt_buffer, 4, I2C_TIMEOUT_MS));
    ESP_LOG_BUFFER_HEXDUMP(TAG_FG, (uint8_t*)rt_buffer, 4, ESP_LOG_INFO);
    if (rt_buffer[2] == (uint8_t)IMU_CHIP_ID_RESET){
        ESP_LOGI(TAG_IMU,"CHIP_ID match on 0x43!");
        CHECK(imu_softreset());
    } else {
        ESP_LOGE(TAG_IMU,"ERROR ON CHIP_ID");
        return ESP_FAIL;
    }

    CHECK(i2c_master_transmit_receive(imu_handle,  &IMU_ACC_CONF_REG, 1, (uint8_t *)&rt_buffer, 4, I2C_TIMEOUT_MS));
    if ((rt_buffer[3] == (uint8_t)(IMU_ACC_CONF_RESET>>8)) && (rt_buffer[2] == IMU_ACC_CONF_RESET)){
        CHECK(i2c_master_transmit(imu_handle, (uint8_t *)&IMU_ACC_CONF, 3, I2C_TIMEOUT_MS));
        CHECK(i2c_master_transmit_receive(imu_handle,  &IMU_ACC_CONF_REG, 1, (uint8_t *)&rt_buffer, 4, I2C_TIMEOUT_MS));
        if (rt_buffer[3] == IMU_ACC_CONF[2] && rt_buffer[2] == IMU_ACC_CONF[1]){
            ESP_LOGI(TAG_IMU,"ACC config ready!");
        } else {
             ESP_LOGE(TAG_IMU,"ERROR ON ACC SET VALUE");
            return ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG_IMU,"ERROR ON ACC RESET VALUE");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    CHECK(i2c_master_transmit_receive(imu_handle,  &IMU_GYR_CONF_REG, 1, (uint8_t *)&rt_buffer, 4, I2C_TIMEOUT_MS));
    if (rt_buffer[3] == (uint8_t)(IMU_GYR_CONF_RESET>>8) && rt_buffer[2] == IMU_GYR_CONF_RESET){
        CHECK(i2c_master_transmit(imu_handle, (uint8_t *)&IMU_GYR_CONF, 3, I2C_TIMEOUT_MS));
        CHECK(i2c_master_transmit_receive(imu_handle,  &IMU_GYR_CONF_REG, 1, (uint8_t *)&rt_buffer, 4, I2C_TIMEOUT_MS));
        if (rt_buffer[3] == IMU_GYR_CONF[2] && rt_buffer[2] == IMU_GYR_CONF[1]){
            ESP_LOGI(TAG_IMU,"GYR config ready!");
        } else {
             ESP_LOGE(TAG_IMU,"ERROR ON GYR SET VALUE");
            return ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG_IMU,"ERROR ON GYR RESET VALUE");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}

static esp_err_t fuelgauge_init(){
    ESP_LOGI(TAG_FG,"Initializing Fuel Gauge");
    volatile uint8_t rt_buffer[4];
    CHECK(i2c_master_transmit_receive(fg_handle,  &FG_CHIP_ID_REG, 1, (uint8_t *)&rt_buffer, 4, I2C_TIMEOUT_MS));
    ESP_LOG_BUFFER_HEXDUMP(TAG_FG, (uint8_t*)rt_buffer, 4, ESP_LOG_INFO);
    if (rt_buffer[2] == (uint8_t)FG_CHIP_ID_RESET){
        ESP_LOGI(TAG_FG,"CHIP_ID match on 0x40!");
        CHECK(imu_softreset());
    } else {
        ESP_LOGE(TAG_FG,"ERROR ON CHIP_ID");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static float lsb_to_dps(int16_t val, float dps)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)16) / 2.0f));

    return (dps / (half_scale)) * (val);
}

static float lsb_to_mps(int16_t val, int8_t g_range)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)16) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

void sensors_task(void *arg){
    ESP_LOGI(TAG_MAIN, "Creating sensors_task");
    vTaskDelay(pdMS_TO_TICKS(200));

    sensors_status = 0;

    timer_sensors_semaphore = xSemaphoreCreateBinary();

    ESP_LOGI(TAG_MAIN, "Motor A Config");
    bdc_motor_config_t motor_config_a = {
        .pwm_freq_hz = MCPWM_FREQ_HZ,
        .pwma_gpio_num = MCPWM_GPIO_A1,
        .pwmb_gpio_num = MCPWM_GPIO_A2,
    };

    ESP_LOGI(TAG_MAIN, "Motor B Config");
    bdc_motor_config_t motor_config_b = {
        .pwm_freq_hz = MCPWM_FREQ_HZ,
        .pwma_gpio_num = MCPWM_GPIO_B1,
        .pwmb_gpio_num = MCPWM_GPIO_B2,
    };

    ESP_LOGI(TAG_MAIN, "MCPWM Config");
    bdc_motor_mcpwm_config_t mcpwm_config_a = {
        .group_id = 0,
        .resolution_hz = MCPWM_TIMER_RESOLUTION_HZ,
    };

    ESP_LOGI(TAG_MAIN, "MCPWM Config");
    bdc_motor_mcpwm_config_t mcpwm_config_b = {
        .group_id = 1,
        .resolution_hz = MCPWM_TIMER_RESOLUTION_HZ,
    };

    sensors_status = 2;

    ESP_LOGI(TAG_MAIN, "Create new motors devices");
    bdc_motor_handle_t motor_a = NULL;
    bdc_motor_handle_t motor_b = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config_a, &mcpwm_config_a, &motor_a));
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config_b, &mcpwm_config_b, &motor_b));
    motor_ctrl_ctx_a.motor = motor_a;
    motor_ctrl_ctx_b.motor = motor_b;

    ESP_LOGI(TAG_MAIN, "Initializing pcnt driver to decode A rotary signal");
    pcnt_unit_config_t unit_config_a = {
        .high_limit = ENCODER_PCNT_HIGH_LIMIT_A,
        .low_limit = ENCODER_PCNT_LOW_LIMIT_A,
        .flags.accum_count = true,
    };

    ESP_LOGI(TAG_MAIN, "Initializing pcnt driver to decode B rotary signal");
    pcnt_unit_config_t unit_config_b = {
        .high_limit = ENCODER_PCNT_HIGH_LIMIT_B,
        .low_limit = ENCODER_PCNT_LOW_LIMIT_B,
        .flags.accum_count = true,
    };

    pcnt_unit_handle_t pcnt_unit_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config_a, &pcnt_unit_a));
    pcnt_unit_handle_t pcnt_unit_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config_b, &pcnt_unit_b));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };

    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit_a, &filter_config));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit_b, &filter_config));

    ESP_LOGI(TAG_MAIN, "Encoder A config channel");
    pcnt_chan_config_t chan_e1a_config = { //config encoder 1 from A motor
        .edge_gpio_num = ENCODER_GPIO_A1,
        .level_gpio_num = ENCODER_GPIO_A2,
    };
    pcnt_channel_handle_t pcnt_chan_e1a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_a, &chan_e1a_config, &pcnt_chan_e1a));

    pcnt_chan_config_t chan_e2a_config = { //config encoder 2 from A motor
        .edge_gpio_num = ENCODER_GPIO_A2,
        .level_gpio_num = ENCODER_GPIO_A1,
    };
    pcnt_channel_handle_t pcnt_chan_e2a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_a, &chan_e2a_config, &pcnt_chan_e2a));

    ESP_LOGI(TAG_MAIN, "Encoder B config channel");
    pcnt_chan_config_t chan_e1b_config = { //config encoder 1 from B motor
        .edge_gpio_num = ENCODER_GPIO_B1,
        .level_gpio_num = ENCODER_GPIO_B2,
    };
    pcnt_channel_handle_t pcnt_chan_e1b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_b, &chan_e1b_config, &pcnt_chan_e1b));

    pcnt_chan_config_t chan_e2b_config = { //config encoder 2 from B motor
        .edge_gpio_num = ENCODER_GPIO_B2,
        .level_gpio_num = ENCODER_GPIO_B1,
    };
    pcnt_channel_handle_t pcnt_chan_e2b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_b, &chan_e2b_config, &pcnt_chan_e2b));

    ESP_LOGI(TAG_MAIN, "Encoder A config edge");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_e1a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_e1a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_e2a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_e2a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_a, ENCODER_PCNT_HIGH_LIMIT_A));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_a, ENCODER_PCNT_LOW_LIMIT_A));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_a));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_a));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_a));
    motor_ctrl_ctx_a.pcnt_encoder = pcnt_unit_a;

    ESP_LOGI(TAG_MAIN, "Encoder B config edge");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_e1b, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_e1b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_e2b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_e2b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_b, ENCODER_PCNT_HIGH_LIMIT_B));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_b, ENCODER_PCNT_LOW_LIMIT_B));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_b));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_b));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_b));
    motor_ctrl_ctx_b.pcnt_encoder = pcnt_unit_b;

    ESP_LOGI(TAG_MAIN, "PID config constants");
    pid_ctrl_parameter_t pid_runtime_param = {
        .kp = 0.6,
        .ki = 0.4,
        .kd = 0.2,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = MCPWM_DUTY_TICK_MAX,
        .min_output   = 0,
        .max_integral = 1000,
        .min_integral = -1000,
    };
    pid_ctrl_config_t pid_config = {
        .init_param = pid_runtime_param,
    };
    ESP_LOGI(TAG_MAIN, "Config PID control blocks");
    pid_ctrl_block_handle_t pid_ctrl_a = NULL;
    pid_ctrl_block_handle_t pid_ctrl_b = NULL;
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl_a));
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl_b));
    motor_ctrl_ctx_a.pid_ctrl = pid_ctrl_a;
    motor_ctrl_ctx_b.pid_ctrl = pid_ctrl_b;

    CHECK(temp_init());

    i2c_master_bus_config_t i2c_config = {
        .sda_io_num = GPIO_SDA,
        .scl_io_num = GPIO_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .glitch_ignore_cnt = 7,
    };
    CHECK(i2c_new_master_bus(&i2c_config, &bus_handle));

    ESP_LOGI(TAG_MAIN,"IMU handle config");
    i2c_device_config_t imu_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_ADDRESS,
        .scl_speed_hz = I2C_FREQ,
    };
    CHECK(i2c_master_bus_add_device(bus_handle, &imu_cfg, &imu_handle));

    ESP_LOGI(TAG_MAIN,"FG handle config");
    i2c_device_config_t fg_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = FG_ADDRESS,
        .scl_speed_hz = I2C_FREQ,
    };
    CHECK(i2c_master_bus_add_device(bus_handle, &fg_cfg, &fg_handle));

    CHECK(imu_init());

    CHECK(imu_read());

    //CHECK(fuelgauge_init());

    ESP_LOGW(TAG_MAIN, "Waiting semaphore from uROS boot");
    xSemaphoreTake(uros_boot_sensors, portMAX_DELAY);
    ESP_LOGI(TAG_MAIN, "Resuming semaphore...");

    ESP_LOGI(TAG_MAIN, "Enable motor A");
    ESP_ERROR_CHECK(bdc_motor_enable(motor_a));

    ESP_LOGI(TAG_MAIN, "Enable motor B");
    ESP_ERROR_CHECK(bdc_motor_enable(motor_b));

    bdc_motor_set_speed(motor_a, (uint32_t)0);
    bdc_motor_set_speed(motor_b, (uint32_t)0);

    ESP_ERROR_CHECK(bdc_motor_forward(motor_a));
    ESP_ERROR_CHECK(bdc_motor_forward(motor_b));

    ESP_LOGI(TAG_MAIN, "Start sensors timer loop");
    sensors_timer = xTimerCreate("Sensor Timer", SENSORS_LOOP_PERIOD_MS, pdTRUE, (void *)SENSORS_LOOP_ID, &sensors_loop_cb);
    if(sensors_timer == NULL) {
        ESP_LOGE(TAG_MAIN, "Sensor Timer create Error!");
    } else {
        ESP_LOGI(TAG_MAIN, "Sensor Timer created!");
        if(xTimerStart(sensors_timer, portMAX_DELAY) != pdPASS) {
            ESP_LOGE(TAG_MAIN, "Sensor Timer start error!");
        } else {
            ESP_LOGI(TAG_MAIN, "Sensor Timer started!");
        }
    }

    sensors_status = 1;

    static volatile int last_pulse_count_a = 0;
    static volatile int last_pulse_count_b = 0;
    motor_control_context_t *ctx_a = (motor_control_context_t *)&motor_ctrl_ctx_a;
    motor_control_context_t *ctx_b = (motor_control_context_t *)&motor_ctrl_ctx_b;

    static int cur_pulse_count_a = 0;
    static int cur_pulse_count_b = 0;
    static volatile double real_pulses_a = 0;
    static volatile double real_pulses_b = 0;

    static volatile float error_a = 0;
    static volatile float error_b = 0;
    static float new_speed_a = 0;
    static float new_speed_b = 0;

    while(1){
        xSemaphoreTake(timer_sensors_semaphore, pdMS_TO_TICKS(35));

        pcnt_unit_handle_t pcnt_unit_a = ctx_a->pcnt_encoder;
        pcnt_unit_handle_t pcnt_unit_b = ctx_b->pcnt_encoder;
        pid_ctrl_block_handle_t pid_ctrl_a = ctx_a->pid_ctrl;
        pid_ctrl_block_handle_t pid_ctrl_b = ctx_b->pid_ctrl;
        bdc_motor_handle_t motor_a = ctx_a->motor;
        bdc_motor_handle_t motor_b = ctx_b->motor;

        // encoder reading
        pcnt_unit_get_count(pcnt_unit_a, &cur_pulse_count_a);
        pcnt_unit_get_count(pcnt_unit_b, &cur_pulse_count_b);
        real_pulses_a = cur_pulse_count_a - last_pulse_count_a;
        real_pulses_b = cur_pulse_count_b - last_pulse_count_b;
        last_pulse_count_a = cur_pulse_count_a;
        last_pulse_count_b = cur_pulse_count_b;
        ctx_a->report_pulses = real_pulses_a;
        ctx_b->report_pulses = real_pulses_b;

        // calculate the speed error
        timestamp_update(&msgs_encoders);
        msgs_encoders.velocity.data[0] = real_pulses_a/(reduction_ratio*encoder_ticks);
        msgs_encoders.velocity.data[1] = real_pulses_b/(reduction_ratio*encoder_ticks);
        if (real_pulses_a < 0){real_pulses_a = -real_pulses_a;}
        if (real_pulses_b < 0){real_pulses_b = -real_pulses_b;}

        PID_EXPECT_SPEED_A = ((((float)msgs_cmdvel.twist.linear.x)*10 + 0.5*(wheels_separation * ((float)msgs_cmdvel.twist.angular.z)))) * reduction_ratio;
        PID_EXPECT_SPEED_B = ((((float)msgs_cmdvel.twist.linear.x)*10 - 0.5*(wheels_separation * ((float)msgs_cmdvel.twist.angular.z)))) * reduction_ratio;
        //ESP_LOGI(TAG_MAIN, "---------------------------LOG---------------------------");
        // ESP_LOGI(TAG_MAIN, "Right: %.4f", PID_EXPECT_SPEED_A);
        // ESP_LOGI(TAG_MAIN, "Left: %.4f", PID_EXPECT_SPEED_B);

        if ((PID_EXPECT_SPEED_A > 0) && (orientation_motora == 'R')){
            orientation_motora = 'F';
            bdc_motor_set_speed(motor_a, (uint32_t)0);
            vTaskDelay(pdMS_TO_TICKS(100));
            ESP_ERROR_CHECK(bdc_motor_forward(motor_a));
            pid_reset_ctrl_block(pid_ctrl_a);
            real_pulses_a = 0;
        } else if ((PID_EXPECT_SPEED_A < 0) && (orientation_motora == 'F')){
            orientation_motora = 'R';
            bdc_motor_set_speed(motor_a, (uint32_t)0);
            vTaskDelay(pdMS_TO_TICKS(100));
            ESP_ERROR_CHECK(bdc_motor_reverse(motor_a));
            pid_reset_ctrl_block(pid_ctrl_a);
            real_pulses_a = 0;
        }
        if (PID_EXPECT_SPEED_A < 0){PID_EXPECT_SPEED_A = -PID_EXPECT_SPEED_A;}


        if ((PID_EXPECT_SPEED_B > 0) && (orientation_motorb == 'R')){
            orientation_motorb = 'F';
            bdc_motor_set_speed(motor_b, (uint32_t)0);
            vTaskDelay(pdMS_TO_TICKS(100));
            ESP_ERROR_CHECK(bdc_motor_forward(motor_b));
            pid_reset_ctrl_block(pid_ctrl_b);
            real_pulses_b = 0;
        } else if ((PID_EXPECT_SPEED_B < 0) && (orientation_motorb == 'F')){
            orientation_motorb = 'R';
            bdc_motor_set_speed(motor_b, (uint32_t)0);
            vTaskDelay(pdMS_TO_TICKS(100));
            ESP_ERROR_CHECK(bdc_motor_reverse(motor_b));
            pid_reset_ctrl_block(pid_ctrl_b);
            real_pulses_b = 0;
        }
        if (PID_EXPECT_SPEED_B < 0){PID_EXPECT_SPEED_B = -PID_EXPECT_SPEED_B;}

        error_a = PID_EXPECT_SPEED_A - real_pulses_a;
        error_b = PID_EXPECT_SPEED_B - real_pulses_b;

        pid_compute(pid_ctrl_a, error_a, &new_speed_a);
        pid_compute(pid_ctrl_b, error_b, &new_speed_b);
        bdc_motor_set_speed(motor_a, (uint32_t)new_speed_a);
        bdc_motor_set_speed(motor_b, (uint32_t)new_speed_b);

        CHECK(imu_read());

        sensors_pub_callback();

        while(sensors_reset_semaphore){
            sensors_status = 0;
            vTaskDelay(portMAX_DELAY);
        }
    }
}