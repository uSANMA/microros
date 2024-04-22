#include "esp_log.h"
#include "esp_timer.h"
#include "esp_types.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"

static const char *TAG = "Sensors";

#define GPIO_SDA                        5 //gpio
#define GPIO_SCL                        4 //gpio
#define I2C_FREQ                        00000 //400KHz
#define I2C_NUM                         0
#define I2C_TIMEOUT_MS                  1000

#define MAX_ADDRESS                     0x00

#define IMU_ADDRESS                     0x36 //0x68 //alternative 0X69 if SDO=VDD 0b 36
const uint8_t IMU_CHIP_ID = 0x00; //chip_id[7-0] reset = 0x0043
const uint8_t IMU_STATUS = 0x02; //drdy_acc[7], drdy_gyr[6], drdy_temp[55]
#define IMU_ACC_X                       0x03 //bit[15-0]
#define IMU_ACC_Y                       0x04 //bit[15-0]
#define IMU_ACC_Z                       0x05 //bit[15-0]
#define IMU_GYR_X                       0x06 //bit[15-0]
#define IMU_GYR_Y                       0x07 //bit[15-0]
#define IMU_GYR_Z                       0x08 //bit[15-0]
#define IMU_TEMP                        0x09 //bit[15-0]
#define IMU_ACC_CONF                    0x20 //acc_mode[14-12], acc_avg_num[10-8], acc_bw[7], acc_range[6-4], acc_odr[3-0]
#define IMU_GYR_CONF                    0x21 //gyr_mode[14-12], gyr_avg_num[10-8], gyr_bw[7], gyr_range[6-4], gyr_odr[3-0]
#define IMU_FEATURE_CTRL                0x40 //bit[0]
#define IMU_CMD                         0x7E //bit[15-0]

/*
DATAs to cmd register
*/
#define IMU_CMD_SELF_CALIBRATION    

/*
feature_ctrl enable = 0b1
*/
#define IMU_FEATURE_CTRL_CONF_DATA_16b   0x0001

/*
acc_mode high-performance = 0b111
acc_avg_num 64 samples = 0b110
acc_bw -3dB cut-off = 0b1
acc_range 4g 8,19 LSB/mg = 0b001
acc_odr 50Hz = 0b0111
result = IMU_ACC_CONF = 0b0111011010010111 = 0x7697
*/
#define IMU_ACC_CONF_DATA_16b   0x7697

/*
gyr_mode high-performance = 0b111
gyr_avg_num 64 samples = 0b110
gyr_bw -3dB cut-off = 0b1
gyr_range 500º/s 65,536 LSB/º/s = 0b010
gyr_odr 50Hz = 0b0111
result = IMU_GYR_CONF = 0b0111011010100111 = 0x76A7
*/
#define IMU_GYR_CONF_DATA_16b   0x76A7

#define ESPCHECK(fn)                                                                                            \
	{                                                                                                           \
		esp_err_t temp_rc = fn;                                                                                 \
		if ((temp_rc != ESP_OK))                                                                                \
		{                                                                                                       \
			ESP_LOGE("SYSTEM-SENSORS", "Failed status on line %d: %d. > Aborting\n", __LINE__, (int)temp_rc);   \
            while(1);                                                                                           \
		}                                                                                                       \
	}

/*
void i2c_write_read(uint8_t device_addr, const uint8_t* write_buffer, size_t write_size,
                                       uint8_t* read_buffer, size_t read_size,
                                       TickType_t ticks_to_wait){
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();

    ESPCHECK(i2c_master_start(cmd_handle));
    //send reg
    ESPCHECK(i2c_master_write_byte(cmd_handle, (device_addr << 1) | I2C_MASTER_WRITE, true));
    ESPCHECK(i2c_master_write(cmd_handle, write_buffer, write_size, true));
    ESPCHECK(i2c_master_start(cmd_handle)); //verificar se precisa stop, e dps start

    //recive data
    ESPCHECK(i2c_master_write_byte(cmd_handle, (device_addr << 1) | I2C_MASTER_READ, true));
    ESPCHECK(i2c_master_read(cmd_handle, read_buffer, read_size, I2C_MASTER_LAST_NACK));

    i2c_master_stop(cmd_handle);

    ESPCHECK(i2c_master_cmd_begin(I2C_NUM, cmd_handle, pdMS_TO_TICKS(10)));
    i2c_cmd_link_delete(cmd_handle);
}*/

void sensors_task(void *arg){
    i2c_master_bus_config_t i2c_config = {
        .sda_io_num = GPIO_SDA,
        .scl_io_num = GPIO_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };

    i2c_master_bus_handle_t bus_handle;
    ESPCHECK(i2c_new_master_bus(&i2c_config, &bus_handle));

    i2c_device_config_t imu_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_ADDRESS << 1,
        .scl_speed_hz = 400000,
    };

    i2c_master_dev_handle_t imu_handle;
    ESPCHECK(i2c_master_bus_add_device(bus_handle, &imu_cfg, &imu_handle));

    i2c_device_config_t max_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MAX_ADDRESS,
        .scl_speed_hz = 400000,
    };

    i2c_master_dev_handle_t max_handle;
    ESPCHECK(i2c_master_bus_add_device(bus_handle, &max_cfg, &max_handle));

    //i2c_param_config(I2C_NUM, &i2c_config);
    //i2c_driver_install(I2C_NUM, i2c_config.mode, 0, 0, 0);

    uint8_t imu_raw[2];
    //i2c_master_write_read_device(I2C_NUM, IMU_ADDRESS, IMU_CHIP_ID, 1, imu_raw, 2, pdMS_TO_TICKS(10));
    //i2c_write_read(IMU_ADDRESS, IMU_CHIP_ID, 1, (uint8_t *)&imu_raw, 2, pdMS_TO_TICKS(10));
    ESPCHECK(i2c_master_transmit_receive(imu_handle,  &IMU_CHIP_ID, sizeof(IMU_CHIP_ID), (uint8_t *)&imu_raw, sizeof(imu_raw), -1));

    uint16_t data = (int16_t)((imu_raw[0] << 8) | imu_raw[1]);
    ESP_LOGI(TAG,"0x%x", imu_raw[0]);
    ESP_LOGI(TAG,"0x%x", imu_raw[1]);
    ESP_LOGI(TAG,"0x%x", data);
    while(1);
}