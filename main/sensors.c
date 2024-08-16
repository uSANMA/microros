#include "esp_log.h"
#include "esp_types.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include <sensor_msgs/msg/imu.h>

#include "driver/temperature_sensor.h"
#include <sensor_msgs/msg/temperature.h>

#include "driver/i2c_master.h"
#include "driver/gpio.h"

#include "math.h"

static const char *TAG = "Sensors";
static const char *TAG1 = "IMU";
static const char *TAG2 = "FG";
static const char *TAG3 = "ESP32_SENSORS";

#define CHECK(fn)                                                                                               \
	{                                                                                                           \
		esp_err_t temp_rc = fn;                                                                                 \
		if ((temp_rc != ESP_OK))                                                                                \
		{                                                                                                       \
			ESP_LOGE("SYSTEM-SENSORS", "Failed status on line %d: %d. > Aborting\n", __LINE__, (int)temp_rc);   \
            while(1);                                                                                           \
		}                                                                                                       \
	}

#define SENSORS_LOOP_PERIOD_MS          30
#define SENSORS_LOOP_ID                 0


#define GPIO_SDA                        5 //gpio
#define GPIO_SCL                        4 //gpio
#define I2C_FREQ                        900000
#define I2C_NUM                         0
#define I2C_TIMEOUT_MS                  10

#define SENSOR_LOOP_PERIOD_MS          31250 //us 0.03125s = 31.25ms = 31250us

#define GRAVITY_EARTH  (9.80665f)

void sensors_task(void *argument);
void sensors_loop_cb(TimerHandle_t);
esp_err_t imu_read();
esp_err_t imu_softreset();
esp_err_t imu_init();
esp_err_t fuelgauge_init();
esp_err_t temp_init();

static float lsb_to_dps(int16_t val, float dps);
static float lsb_to_mps(int16_t val, int8_t g_range);

TimerHandle_t sensors_timer;

extern void sensors_pub_callback();

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t imu_handle;
i2c_master_dev_handle_t fg_handle;

const uint8_t FG_ADDRESS =                        0x00;

const uint8_t IMU_ADDRESS =                       0x68; //alternative 0X69 if SDO=VDD 0b 36
const uint8_t IMU_CHIP_ID_REG =                   0x00; //chip_id[7-0]
const uint16_t IMU_CHIP_ID_RESET =                0x0043;
const uint8_t IMU_STATUS_REG =                    0x02; //drdy_acc[7], drdy_gyr[6], drdy_temp[55]
const uint8_t IMU_ACC_X_REG =                     0x03; //bit[15-0]
const uint8_t IMU_ACC_Y_REG =                     0x04; //bit[15-0]
const uint8_t IMU_ACC_Z_REG =                     0x05; //bit[15-0]
const uint8_t IMU_GYR_X_REG =                     0x06; //bit[15-0]
const uint8_t IMU_GYR_Y_REG =                     0x07; //bit[15-0]
const uint8_t IMU_GYR_Z_REG =                     0x08; //bit[15-0]
const uint8_t IMU_TEMP_REG =                      0x09; //bit[15-0]
const uint8_t IMU_ACC_CONF_REG =                  0x20; //acc_mode[14-12], acc_avg_num[10-8], acc_bw[7], acc_range[6-4], acc_odr[3-0]
const uint8_t IMU_GYR_CONF_REG =                  0x21; //gyr_mode[14-12], gyr_avg_num[10-8], gyr_bw[7], gyr_range[6-4], gyr_odr[3-0]
const uint8_t IMU_FEATURE_CTRL_REG =              0x40; //bit[0]
const uint8_t IMU_CMD_REG =                       0x7E; //bit[15-0]

const uint8_t DUMMY_BYTE = 0x00;

/*
softreset on imu cmd reg
*/
const uint8_t IMU_CMD_SOFTRESET[3] = {IMU_CMD_REG, 0xAF, 0xDE};

/*
acc_mode high-performance = 0b111
acc_avg_num 64 samples = 0b110
acc_bw -3dB cut-off = 0b1
acc_range 4g 8,19 LSB/mg = 0b001
acc_odr 50Hz = 0b0111
result = ACC_CONF = 0b0111011010010111 = 0x7697

4 
*/
const uint8_t IMU_ACC_CONF[3] = {IMU_ACC_CONF_REG, 0x97, 0x76};
const uint16_t IMU_ACC_CONF_RESET = 0x0028;

/*
gyr_mode high-performance = 0b111
gyr_avg_num 64 samples = 0b110
gyr_bw -3dB cut-off = 0b1
gyr_range 500º/s 65,536 LSB/º/s = 0b010
gyr_odr 50Hz = 0b0111
result = IMU_GYR_CONF = 0b0111011010100111 = 0x76A7
*/
const uint8_t IMU_GYR_CONF[3]  = {IMU_GYR_CONF_REG, 0xA7, 0x76};
const uint16_t IMU_GYR_CONF_RESET = 0x0048;

extern void timestamp_update(void*arg);

extern SemaphoreHandle_t uros_boot_sensors;
SemaphoreHandle_t timer_sensors;

extern sensor_msgs__msg__Imu msgs_imu;
extern sensor_msgs__msg__Temperature msgs_temperature;
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

esp_err_t temp_init(){
    ESP_LOGI(TAG3,"Initing internal temp sensor");
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
    ESP_LOGI(TAG3,"Temp: %.4f", tsens_out);
    timestamp_update(&msgs_temperature);
    msgs_temperature.temperature = tsens_out;
    // Disable the temperature sensor if it is not needed and save the power
    ESP_ERROR_CHECK(temperature_sensor_disable(temp_handle));
    return ESP_OK;
}

void sensors_loop_cb(TimerHandle_t xTimer) {
    xSemaphoreGive(timer_sensors);
}

esp_err_t imu_read(){
    vTaskDelay(pdMS_TO_TICKS(1));
    volatile uint8_t raw_buffer[16];
    CHECK(i2c_master_transmit_receive(imu_handle,  &IMU_ACC_X_REG, 1, (uint8_t *)&raw_buffer, 16, I2C_TIMEOUT_MS));

    if (raw_buffer[0] != 0 && raw_buffer [1] != 0){
        return ESP_FAIL;
    }

    msgs_imu.angular_velocity.x = lsb_to_dps(((int16_t)(raw_buffer[15] << 8 | raw_buffer[14])), (float)500);
    msgs_imu.angular_velocity.y = lsb_to_dps(((int16_t)(raw_buffer[13] << 8 | raw_buffer[12])), (float)500);
    msgs_imu.angular_velocity.z = lsb_to_dps(((int16_t)(raw_buffer[11] << 8 | raw_buffer[10])), (float)500);

    msgs_imu.linear_acceleration.x = lsb_to_mps(((int16_t)(raw_buffer[9] << 8 | raw_buffer[8])), (float)4);
    msgs_imu.linear_acceleration.y = lsb_to_mps(((int16_t)(raw_buffer[7] << 8 | raw_buffer[6])), (float)4);
    msgs_imu.linear_acceleration.z = lsb_to_mps(((int16_t)(raw_buffer[5] << 8 | raw_buffer[4])), (float)4);

    msgs_temperature.temperature = (((int16_t)(raw_buffer[3] << 8 | raw_buffer[2]))/512) + 23;

    //ESP_LOGI(TAG1,"AX: %d", imu_data.accel_x);
    //ESP_LOGI(TAG1,"AY: %d", imu_data.accel_y);
    //ESP_LOGI(TAG1,"AZ: %d", imu_data.accel_z);
    //ESP_LOGI(TAG1,"GX: %d", imu_data.gyro_x);
    //ESP_LOGI(TAG1,"GY: %d", imu_data.gyro_y);
    //ESP_LOGI(TAG1,"GZ: %d", imu_data.gyro_z);
    //ESP_LOGI(TAG1,"T: %d", imu_data.temperature);

    return ESP_OK;
}

esp_err_t imu_softreset(){
    CHECK(i2c_master_transmit(imu_handle, (uint8_t *)&IMU_CMD_SOFTRESET, 3, I2C_TIMEOUT_MS));
    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
}

esp_err_t imu_init(){
    ESP_LOGI(TAG1,"Initing IMU");
    volatile uint8_t rt_buffer[4];
    CHECK(i2c_master_transmit_receive(imu_handle,  &IMU_CHIP_ID_REG, 1, (uint8_t *)&rt_buffer, 4, I2C_TIMEOUT_MS));
    if (rt_buffer[2] == (uint8_t)IMU_CHIP_ID_RESET){
        ESP_LOGI(TAG1,"CHIP_ID match on 0x43!");
        CHECK(imu_softreset());
    } else {
        ESP_LOGE(TAG1,"ERROR ON CHIP_ID");
        return ESP_FAIL;
    }

    CHECK(i2c_master_transmit_receive(imu_handle,  &IMU_ACC_CONF_REG, 1, (uint8_t *)&rt_buffer, 4, I2C_TIMEOUT_MS));
    if (rt_buffer[3] == (uint8_t)(IMU_ACC_CONF_RESET>>8) && rt_buffer[2] == IMU_ACC_CONF_RESET){
        CHECK(i2c_master_transmit(imu_handle, (uint8_t *)&IMU_ACC_CONF, 3, I2C_TIMEOUT_MS));
        CHECK(i2c_master_transmit_receive(imu_handle,  &IMU_ACC_CONF_REG, 1, (uint8_t *)&rt_buffer, 4, I2C_TIMEOUT_MS));
        if (rt_buffer[3] == IMU_ACC_CONF[2] && rt_buffer[2] == IMU_ACC_CONF[1]){
            ESP_LOGI(TAG1,"ACC config ready!");
        } else {
             ESP_LOGE(TAG1,"ERROR ON ACC SET VALUE");
            return ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG1,"ERROR ON ACC RESET VALUE");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    CHECK(i2c_master_transmit_receive(imu_handle,  &IMU_GYR_CONF_REG, 1, (uint8_t *)&rt_buffer, 4, I2C_TIMEOUT_MS));
    if (rt_buffer[3] == (uint8_t)(IMU_GYR_CONF_RESET>>8) && rt_buffer[2] == IMU_GYR_CONF_RESET){
        CHECK(i2c_master_transmit(imu_handle, (uint8_t *)&IMU_GYR_CONF, 3, I2C_TIMEOUT_MS));
        CHECK(i2c_master_transmit_receive(imu_handle,  &IMU_GYR_CONF_REG, 1, (uint8_t *)&rt_buffer, 4, I2C_TIMEOUT_MS));
        if (rt_buffer[3] == IMU_GYR_CONF[2] && rt_buffer[2] == IMU_GYR_CONF[1]){
            ESP_LOGI(TAG1,"GYR config ready!");
        } else {
             ESP_LOGE(TAG1,"ERROR ON GYR SET VALUE");
            return ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG1,"ERROR ON GYR RESET VALUE");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}

esp_err_t fuelgauge_init(){
    ESP_LOGI(TAG2,"Initing Fuel Gauge");
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

    timer_sensors = xSemaphoreCreateBinary();

    CHECK(temp_init());

    i2c_master_bus_config_t i2c_config = {
        .sda_io_num = GPIO_SDA,
        .scl_io_num = GPIO_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };
    CHECK(i2c_new_master_bus(&i2c_config, &bus_handle));

    ESP_LOGI(TAG,"IMU handle config");
    i2c_device_config_t imu_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_ADDRESS,
        .scl_speed_hz = I2C_FREQ,
    };
    CHECK(i2c_master_bus_add_device(bus_handle, &imu_cfg, &imu_handle));

    ESP_LOGI(TAG,"FG handle config");
    i2c_device_config_t fg_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = FG_ADDRESS,
        .scl_speed_hz = I2C_FREQ,
    };
    CHECK(i2c_master_bus_add_device(bus_handle, &fg_cfg, &fg_handle));

    CHECK(imu_init());

    CHECK(imu_read());

    CHECK(fuelgauge_init());

    ESP_LOGW(TAG, "Waiting semaphore from uROS boot");
    xSemaphoreTake(uros_boot_sensors, portMAX_DELAY);
    ESP_LOGI(TAG, "Resuming semaphore...");

    ESP_LOGI(TAG, "Start sensors timer loop");
    sensors_timer = xTimerCreate("Sensor Timer", SENSORS_LOOP_PERIOD_MS, pdTRUE, (void *)SENSORS_LOOP_ID, &sensors_loop_cb);
    if(sensors_timer == NULL) {
            ESP_LOGE(TAG, "Sensor Timer create Error!");
        } else {
            ESP_LOGI(TAG, "Sensor Timer created!");
            if(xTimerStart(sensors_timer, portMAX_DELAY) != pdPASS) {
                ESP_LOGE(TAG, "Sensor Timer start error!");
            } else {
                ESP_LOGI(TAG, "Sensor Timer started!");
            }
        }

    while(1){
        xSemaphoreTake(timer_sensors, portMAX_DELAY);

        timestamp_update(&msgs_imu);
        CHECK(imu_read());
        timestamp_update(&msgs_temperature);
        sensors_pub_callback();
        taskYIELD();
    }
}