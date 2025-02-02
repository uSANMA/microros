#include "math.h"
#include <string.h>
#include <stdio.h>

#include <time.h>

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_types.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include <sensor_msgs/msg/laser_scan.h>

#include "driver/gpio.h"
#include "driver/uart.h"

#define CHECKDRIVER(fn) {                                                                                                           \
		driver_ret_t rt_driver = fn;                                                                                                \
		if ((rt_driver != DRIVER_RET_OK)) {                                                                                         \
			ESP_LOGE("SYSTEM-LidarDriver", "Failed status on line: %d > returned: %d > Aborting", __LINE__, (int)rt_driver);        \
            lidar_status = -1;                                                                                                      \
            while(1){                                                                                                               \
                taskYIELD();                                                                                                        \
            }                                                                                                                       \
		}                                                                                                                           \
	}

#define DRIVER_RET_ERROR -1
#define DRIVER_RET_OK 0
#define DRIVER_RET_TIMEOUT 2

#define UART_PORT_NUM                                           1
#define UART_BAUD_RATE                                          115200

#define UART_GPIO_TX                                            16 //gpio esp tx
#define UART_GPIO_RX                                            15 //gpio esp rx
#define GPIO_LIDAR_PWM                                          7 //gpio motor pin

typedef int8_t driver_ret_t;

void lidar_task(void *argument);
static void init_lidar();
static driver_ret_t get_info_lidar();
static driver_ret_t get_health_lidar();
static driver_ret_t start_scan_lidar();
static void stop_scan_lidar();

extern void timestamp_update(void*arg);
extern void lidar_pub_callback();

static const char *TAG_MAIN = "Task-Lidar";

static const int lidar_ms_timeout = 2;
static const int lidar_init_ms_timeout = 500;
static const int uart_buffer_size = 1024;
static const int uart_queue_size = 5;

static uint8_t LIDAR_PKG_STOP_SCAN[] = {0xA5, 0x25}; //no descriptor
//static uint8_t LIDAR_PKG_RESET_CORE[] = {0xA5, 0x40}; //no descriptor

static uint8_t LIDAR_PKG_START_SCAN[] = {0xA5, 0x20}; //Descriptor + Multiple response 5 bytes
static uint8_t LIDAR_PKG_START_SCAN_DESCRIPTOR[] = {0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81}; //Descriptor: A5 5A 05 00 00 40 81
//static uint8_t LIDAR_PKG_EXPRESS_SCAN[] = {0xA5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22}; //Descriptor + Multiple response 84 bytes + Payload 
//static uint8_t LIDAR_PKG_EXPRESS_SCAN_DESCRIPTOR[] = {0xA5, 0x5A, 0x54, 0x00, 0x00, 0x40, 0x82}; //Descriptor: A5 5A 54 00 00 40 82
//static uint8_t LIDAR_PKG_FORCE_SCAN[] = {0xA5, 0x21}; //Descriptor + Multiple response 5 bytes
//static uint8_t LIDAR_PKG_FORCE_SCAN_DESCRIPTOR[] = {0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81}; //Descriptor: A5 5A 05 00 00 40 81

static uint8_t LIDAR_PKG_GET_INFO[] = {0xA5, 0x50}; //Descriptor + Single response 20 bytes
static uint8_t LIDAR_PKG_GET_INFO_DESCRIPTOR[] = {0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04}; //Descriptor: A5 5A 14 00 00 00 04
static uint8_t LIDAR_PKG_GET_HEALTH[] = {0xA5, 0x52}; //Descriptor + Single response 3 bytes
static uint8_t LIDAR_PKG_GET_HEALTH_DESCRIPTOR[] = {0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06}; //Descriptor: A5 5A 03 00 00 00 06

//static uint8_t LIDAR_PKG_GET_SAMPLERATE[] = {0xA5, 0x59}; //Descriptor + Single response 4 bytes
//static uint8_t LIDAR_PKG_GET_SAMPLERATE_DESCRIPTOR[] = {0xA5, 0x5A, 0x04, 0x00, 0x00, 0x00, 0x15}; //Descriptor: A5 5A 04 00 00 00 15

extern volatile int8_t lidar_status;
extern volatile int8_t lidar_reset_semaphore;

static QueueHandle_t uart_queue;

extern sensor_msgs__msg__LaserScan msgs_laserscan;
extern SemaphoreHandle_t uros_boot_lidar;

static void init_lidar(){
    ESP_LOGI(TAG_MAIN,"Initializing lidar driver...");
    gpio_reset_pin(GPIO_LIDAR_PWM);
    gpio_set_direction(GPIO_LIDAR_PWM, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(GPIO_LIDAR_PWM, GPIO_PULLDOWN_ENABLE);
    gpio_set_level(GPIO_LIDAR_PWM, 0);

    CHECKDRIVER(get_info_lidar());
    CHECKDRIVER(get_health_lidar());
}

static driver_ret_t get_info_lidar(){
    uint8_t *data_uart = (uint8_t *) malloc(32);

    stop_scan_lidar();

    //read info
    uart_flush(UART_PORT_NUM);
    ESP_LOGI(TAG_MAIN,"Sending get info...");
    uart_write_bytes(UART_PORT_NUM, (const char *) LIDAR_PKG_GET_INFO, 2);

    ESP_LOGI(TAG_MAIN,"Receiving descriptor info...");
    int len = uart_read_bytes(UART_PORT_NUM, data_uart, 7, lidar_init_ms_timeout / portTICK_PERIOD_MS);
    if (len == 7){
        if(memcmp(data_uart, LIDAR_PKG_GET_INFO_DESCRIPTOR, len) == 0){
            ESP_LOGI(TAG_MAIN,"Descriptor info is good!");
            len = uart_read_bytes(UART_PORT_NUM, data_uart, 20, lidar_init_ms_timeout / portTICK_PERIOD_MS);
            if (len == 20){
                ESP_LOGI(TAG_MAIN,"Lidar info ->\n     Model: %x\n     Firmware minor: %x\n     Firmware major: %x\n     Hardware: %x", data_uart[0], data_uart[1], data_uart[2], data_uart[3]);
                //ESP_LOGE(TAG_MAIN,"Hardware: %x", data_uart[4]); serial [4]-[20]
            }
            return DRIVER_RET_OK;
        } else {
            ESP_LOGE(TAG_MAIN,"Receive 7 bytes but descriptor info is bad...");
            ESP_LOGI(TAG_MAIN,"Receive: ");
            ESP_LOG_BUFFER_HEXDUMP(TAG_MAIN, data_uart, len, ESP_LOG_INFO);
            ESP_LOGI(TAG_MAIN,"Expected: ");
            ESP_LOG_BUFFER_HEXDUMP(TAG_MAIN, LIDAR_PKG_GET_INFO_DESCRIPTOR, len, ESP_LOG_INFO);
            return DRIVER_RET_ERROR;
        }
    } else {
        ESP_LOGE(TAG_MAIN,"Error on receiving descriptor info! > no uart response...");
        return DRIVER_RET_ERROR;
    }
    free(data_uart);
}

static driver_ret_t get_health_lidar(){
    static uint8_t data_uart[16];

    stop_scan_lidar();

    //read health
    uart_flush(UART_PORT_NUM);
    ESP_LOGI(TAG_MAIN,"Sending get health...");
    uart_write_bytes(UART_PORT_NUM, (const char *) LIDAR_PKG_GET_HEALTH, 2);

    ESP_LOGI(TAG_MAIN,"Receiving health descriptor...");
    int len = uart_read_bytes(UART_PORT_NUM, data_uart, 7, lidar_init_ms_timeout / portTICK_PERIOD_MS);
    if (len == 7){
        if(memcmp(data_uart, LIDAR_PKG_GET_HEALTH_DESCRIPTOR, len) == 0){
            ESP_LOGI(TAG_MAIN,"Health descriptor is good!");
            len = uart_read_bytes(UART_PORT_NUM, data_uart, 3, lidar_init_ms_timeout / portTICK_PERIOD_MS);
            if (len == 3){
                ESP_LOGI(TAG_MAIN,"Lidar health ->\n    Status: %x\n    Error code: %x", data_uart[0], (data_uart[1] | (data_uart[2] << 8)));
            }
            return DRIVER_RET_OK;
        } else {
            ESP_LOGE(TAG_MAIN,"Receive 7 bytes but health descriptor is bad...");
            ESP_LOGI(TAG_MAIN,"Receive: ");
            ESP_LOG_BUFFER_HEXDUMP(TAG_MAIN, data_uart, len, ESP_LOG_INFO);
            ESP_LOGI(TAG_MAIN,"Expected: ");
            ESP_LOG_BUFFER_HEXDUMP(TAG_MAIN, LIDAR_PKG_GET_HEALTH_DESCRIPTOR, len, ESP_LOG_INFO);
            return DRIVER_RET_ERROR;
        }
    } else {
        ESP_LOGE(TAG_MAIN,"Error on receiving health descriptor! > no uart response...");
        return DRIVER_RET_ERROR;
    }
}

static driver_ret_t start_scan_lidar(){
    static uint8_t data_uart[16];

    gpio_set_level(GPIO_LIDAR_PWM, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));

    stop_scan_lidar();

    //start scan
    uart_flush(UART_PORT_NUM);
    ESP_LOGI(TAG_MAIN,"Sending start scan...");
    uart_write_bytes(UART_PORT_NUM, (const char *) LIDAR_PKG_START_SCAN, 2);

    ESP_LOGI(TAG_MAIN,"Receiving scan descriptor...");
    int len = uart_read_bytes(UART_PORT_NUM, data_uart, 7, lidar_init_ms_timeout / portTICK_PERIOD_MS); //(BUF_SIZE - 1)
    if (len == 7){
        if(memcmp(data_uart, LIDAR_PKG_START_SCAN_DESCRIPTOR, len) == 0){
            ESP_LOGI(TAG_MAIN,"Scan descriptor is good!");
            ESP_LOGI(TAG_MAIN,"Scan started!");
            return DRIVER_RET_OK;
        } else {
            ESP_LOGE(TAG_MAIN,"Receive more than 7 bytes but scan descriptor is bad...");
            ESP_LOGI(TAG_MAIN,"Receive: ");
            ESP_LOG_BUFFER_HEXDUMP(TAG_MAIN, data_uart, len, ESP_LOG_INFO);
            ESP_LOGI(TAG_MAIN,"Expected: ");
            ESP_LOG_BUFFER_HEXDUMP(TAG_MAIN, LIDAR_PKG_START_SCAN_DESCRIPTOR, len, ESP_LOG_INFO);
            return DRIVER_RET_ERROR;
        }
    } else {
        ESP_LOGE(TAG_MAIN,"Error on receiving scan descriptor! > Restarting esp32");
        return DRIVER_RET_ERROR;
    }
}

static void stop_scan_lidar(){
    //stopt scan
    ESP_LOGI(TAG_MAIN,"Sending stop scan...");
    uart_flush(UART_PORT_NUM);
    uart_write_bytes(UART_PORT_NUM, (const char *) LIDAR_PKG_STOP_SCAN, 2);
    vTaskDelay(pdMS_TO_TICKS(10));
    uart_flush(UART_PORT_NUM);
}

void lidar_task(void * arg){
    ESP_LOGI(TAG_MAIN, "Creating lidar_task");
    vTaskDelay(pdMS_TO_TICKS(200));

    lidar_status = 0;

    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, uart_buffer_size, uart_buffer_size, uart_queue_size, &uart_queue, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_GPIO_TX, UART_GPIO_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    //uart_set_rx_timeout(UART_PORT_NUM, 1);
    //uart_set_always_rx_timeout(UART_PORT_NUM, true);
    uart_flush(UART_PORT_NUM);

    lidar_status = 2;
    
    init_lidar();

    ESP_LOGW(TAG_MAIN, "Waiting for semaphore from uROS boot");
    xSemaphoreTake(uros_boot_lidar, portMAX_DELAY);
    ESP_LOGI(TAG_MAIN, "Resuming semaphore...");

    CHECKDRIVER(start_scan_lidar());

    lidar_status = 1;

    uint8_t *data_scan = (uint8_t *) malloc(16 * sizeof(uint8_t));

    uint16_t quality = 0;
    uint8_t new_scan_flag = 0;
    float angle = 0;
    float range = 0;

    uint16_t ring_buffer_len = 0;
    int uart_len = 0;
    uint16_t angle_index = 0;
    uint16_t measures = 0;

    while(1){
        uart_get_buffered_data_len(UART_PORT_NUM, (size_t*)&ring_buffer_len);
        if((ring_buffer_len >= 9)){
            uart_len = uart_read_bytes(UART_PORT_NUM, data_scan, 2, lidar_ms_timeout / portTICK_PERIOD_MS);
            while((((data_scan[1] & 0x01) & ((data_scan[0] ^ (data_scan[0] >> 1)) & 0x01)) != 1) && (uart_len == 2) && (ring_buffer_len >= 5)){
                uart_get_buffered_data_len(UART_PORT_NUM, (size_t*)&ring_buffer_len);
                if (ring_buffer_len >= 5){
                    uart_len = uart_read_bytes(UART_PORT_NUM, data_scan, 2, lidar_ms_timeout / portTICK_PERIOD_MS);
                } else {
                    vTaskDelay(pdMS_TO_TICKS(1));
                    taskYIELD();
                }
            }
            if ((uart_len == 2) && (ring_buffer_len >= 5)){
                uart_len = uart_read_bytes(UART_PORT_NUM, &data_scan[2], 3, lidar_ms_timeout / portTICK_PERIOD_MS);
                quality = data_scan[0] >> 2;
                new_scan_flag = data_scan[0] & 0x1;
                angle = ((data_scan[1] >> 1) | (data_scan[2] << 7)) / 64.0; //deg
                angle_index = round(angle); //int deg
                range = ((data_scan[3] | (data_scan[4] << 8)) / 4000.0); //meters

                if ((angle >= 0) && (angle <= 360)){
                    if (((new_scan_flag == 1) && (range >= 0) && (range <= msgs_laserscan.range_max)) || measures >= 360) {
                        //ESP_LOGI(TAG_MAIN,"Package>Measures: %3d | Last > Flag: %d | Quality: %02d | Angle_i: %03d | Range: %2.8f", measures, new_scan_flag, quality, angle_index, range);
                        msgs_laserscan.ranges.data[angle_index] = range;
                        lidar_pub_callback();
                        new_scan_flag = 0;
                        measures = 0;
                        timestamp_update(&msgs_laserscan);
                        for (uint16_t i = 0; i < (uint16_t) msgs_laserscan.ranges.capacity; i++){
                            msgs_laserscan.ranges.data[i] = 0;
                        }
                        if (ring_buffer_len > 400) {
                            ESP_LOGW(TAG_MAIN,"Ring uart OVERFLOW: %d", ring_buffer_len);
                        }
                        taskYIELD();
                    } else if ((new_scan_flag == 0) && (range > 0) && (range <= msgs_laserscan.range_max) && (range >= msgs_laserscan.range_min)) {
                        //ESP_LOGI(TAG_MAIN,"Measure > Quality: %02d | Angle: %3.8f | Angle_i: %03d | Range: %2.8f", quality, angle, angle_index, range);
                        measures++;
                        msgs_laserscan.ranges.data[angle_index] = range;
                    }
                }
            
            }
        }
        //vTaskDelay(pdMS_TO_TICKS(1));
        while(lidar_reset_semaphore) {
            stop_scan_lidar();
            gpio_set_level(GPIO_LIDAR_PWM, 0);
            lidar_status = 0;
            vTaskDelay(portMAX_DELAY);
        }
        taskYIELD();
    }
    ESP_LOGE(TAG_MAIN, "Task Delete");
    vTaskDelete(NULL);
}