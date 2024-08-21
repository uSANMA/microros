#include "math.h"

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

static const char *TAG = "Lidar";

const int lidar_ms_timeout = 2;
const int lidar_init_ms_timeout = 500;
const int uart_buffer_size = 1024;
const int uart_queue_size = 256;
QueueHandle_t uart_queue;

#define UART_PORT_NUM                       1
#define UART_BAUD_RATE                      115200

#define UART_GPIO_TX                        16 //gpio esp tx
#define UART_GPIO_RX                        15 //gpio esp rx
#define GPIO_LIDAR_PWM                      7 //gpio motor pin

void lidar_task(void *argument);
void init_lidar();
void start_scan_lidar();

extern void timestamp_update(void*arg);
extern void lidar_pub_callback();

extern sensor_msgs__msg__LaserScan msgs_laserscan;

extern SemaphoreHandle_t uros_boot_lidar;

uint8_t LIDAR_PKG_STOP_SCAN[] = {0xA5, 0x25}; //sem resposta
uint8_t LIDAR_PKG_RESET_CORE[] = {0xA5, 0x40}; //sem resposta

uint8_t LIDAR_PKG_START_SCAN[] = {0xA5, 0x20}; //resposta descriptor: A5 5A 05 00 00 40 81 - 5 bytes
uint8_t LIDAR_PKG_EXPRESS_SCAN[] = {0xA5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22}; //resposta descriptor: A5 5A 54 00 00 40 82 - 84 bytes
uint8_t LIDAR_PKG_FORCE_SCAN[] = {0xA5, 0x21}; //resposta descriptor: A5 5A 05 00 00 40 81 - 5 bytes

uint8_t LIDAR_PKG_GET_INFO[] = {0xA5, 0x50}; //resposta descriptor: A5 5A 14 00 00 00 04 - 20 bytes
uint8_t LIDAR_PKG_GET_HEALTH[] = {0xA5, 0x52}; //resposta descriptor: A5 5A 03 00 00 00 06 - 3 bytes
uint8_t LIDAR_PKG_GET_SAMPLERATE[] = {0xA5, 0x59}; //resposta descriptor: A5 5A 04 00 00 00 15 - 4 bytes

void init_lidar(){
    uint8_t *data_uart = (uint8_t *) malloc(uart_buffer_size);

    //stop scan
    uart_flush(UART_PORT_NUM);
    uart_write_bytes(UART_PORT_NUM, (const char *) LIDAR_PKG_STOP_SCAN, 2);
    vTaskDelay(pdMS_TO_TICKS(50));

    //read health
    uart_flush(UART_PORT_NUM);
    ESP_LOGI(TAG,"Sending get health...");
    uart_write_bytes(UART_PORT_NUM, (const char *) LIDAR_PKG_GET_HEALTH, 2);

    ESP_LOGI(TAG,"Receiving health descriptor...");
    int len = uart_read_bytes(UART_PORT_NUM, data_uart, (uart_buffer_size - 1), lidar_init_ms_timeout / portTICK_PERIOD_MS); //(BUF_SIZE - 1)
    ESP_LOG_BUFFER_HEXDUMP(TAG, data_uart, len, ESP_LOG_INFO);
    //ESP_LOGI(TAG, "Len: %d | BUFFER: %02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X", len, data_uart[0],data_uart[1],data_uart[2],data_uart[3],data_uart[4],data_uart[5],data_uart[6],data_uart[7],data_uart[8],data_uart[9]);
    if (len >= 7){
        if((data_uart[0] == 0xA5) && (data_uart[1] == 0x5A) && (data_uart[2] == 0x03) && (data_uart[3] == 0x00) && (data_uart[4] == 0x00) && (data_uart[5] == 0x00) && (data_uart[6] == 0x06)){
            ESP_LOGI(TAG,"Health descriptor is good!");
        } else {
            ESP_LOGE(TAG,"Receive more than 7 bytes but health descriptor is bad...");
        }
    } else {
        ESP_LOGE(TAG,"Error on receiving health descriptor! > Initing attempts...");
        uint8_t attempts = 0;
        while ((data_uart[0] != 0xA5) && (data_uart[1] != 0x5A) && (data_uart[2] != 0x03) && (data_uart[3] != 0x00) && (data_uart[4] != 0x00) && (data_uart[5] != 0x00) && (data_uart[6] != 0x06)){
            ESP_LOGW(TAG,"Trying again: %d", attempts);
            attempts++;
            ESP_LOGI(TAG,"Sending secund get health...");
            uart_flush(UART_PORT_NUM);
            uart_write_bytes(UART_PORT_NUM, (const char *) LIDAR_PKG_GET_HEALTH, 2);

            ESP_LOGI(TAG,"Receiving health descriptor...");
            len = uart_read_bytes(UART_PORT_NUM, data_uart, (uart_buffer_size - 1), lidar_init_ms_timeout / portTICK_PERIOD_MS); //(BUF_SIZE - 1)
            ESP_LOG_BUFFER_HEXDUMP(TAG, data_uart, len, ESP_LOG_INFO);
            //ESP_LOGI(TAG, "Len: %d | BUFFER: %02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X", len, data_uart[0],data_uart[1],data_uart[2],data_uart[3],data_uart[4],data_uart[5],data_uart[6],data_uart[7],data_uart[8],data_uart[9]);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        ESP_LOGI(TAG,"Health descriptor is good! > Continuing...");
    }
    free(data_uart);
}

void start_scan_lidar(){
    uint8_t *data_uart = (uint8_t *) malloc(uart_buffer_size);

    //start scan
    uart_flush(UART_PORT_NUM);
    ESP_LOGI(TAG,"Sending start scan...");
    uart_write_bytes(UART_PORT_NUM, (const char *) LIDAR_PKG_START_SCAN, 2);

    ESP_LOGI(TAG,"Receiving scan descriptor...");
    int len = uart_read_bytes(UART_PORT_NUM, data_uart, (uart_buffer_size - 1), lidar_init_ms_timeout / portTICK_PERIOD_MS); //(BUF_SIZE - 1)
    ESP_LOG_BUFFER_HEXDUMP(TAG, data_uart, len, ESP_LOG_INFO);
    //ESP_LOGI(TAG, "Len: %d | BUFFER: %02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X", len, data_uart[0],data_uart[1],data_uart[2],data_uart[3],data_uart[4],data_uart[5],data_uart[6],data_uart[7],data_uart[8],data_uart[9]);
    if (len >= 7){
        if((data_uart[0] == 0xA5) && (data_uart[1] == 0x5A) && (data_uart[2] == 0x05) && (data_uart[3] == 0x00) && (data_uart[4] == 0x00) && (data_uart[5] == 0x40) && (data_uart[6] == 0x81)){
            ESP_LOGI(TAG,"Scan descriptor is good!");
            ESP_LOGI(TAG,"Scan started!");
        } else {
            ESP_LOGE(TAG,"Receive more than 7 bytes but scan descriptor is bad...");
        }
    } else {
        ESP_LOGE(TAG,"Error on receiving scan descriptor! > Restarting esp32");
        vTaskDelay(pdMS_TO_TICKS(15000));
        esp_restart();
    }
    free(data_uart);
}

void lidar_task(void * arg){
    gpio_reset_pin(GPIO_LIDAR_PWM);
    gpio_set_direction(GPIO_LIDAR_PWM, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(GPIO_LIDAR_PWM, GPIO_FLOATING);
    gpio_set_level(GPIO_LIDAR_PWM, 1);

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
    uart_set_rx_timeout(UART_PORT_NUM, 1); //  set MBB = 1 symbol time on current baudrate
    uart_set_always_rx_timeout(UART_PORT_NUM, true);
    uart_flush(UART_PORT_NUM);

    ESP_LOGW(TAG, "Waiting for semaphore from uROS boot");
    xSemaphoreTake(uros_boot_lidar, portMAX_DELAY);
    ESP_LOGI(TAG, "Resuming semaphore...");

    init_lidar();

    start_scan_lidar();

    uint16_t measures = 0;
    uint8_t *data_scan = (uint8_t *) malloc(10);
    uint8_t *data_scan_pivot = &data_scan[2];
    while(1){
        size_t ring_buffer_len = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_PORT_NUM, &ring_buffer_len));
        if((ring_buffer_len >= 9)){
            int uart_len = 3;
            while((uart_read_bytes(UART_PORT_NUM, data_scan, 2, lidar_ms_timeout / portTICK_PERIOD_MS) == 2) && (uart_len == 3)){
                if ((data_scan[1] & 0x01) & ((data_scan[0] ^ (data_scan[0] >> 1)) & 0x01)){
                    uart_len = uart_read_bytes(UART_PORT_NUM, data_scan_pivot, 3, lidar_ms_timeout / portTICK_PERIOD_MS);
                    uint16_t quality = data_scan[0] >> 2;
                    uint8_t new_scan_flag = data_scan[0] & 0x1;
                    float angle = ((data_scan[1] >> 1) | (data_scan[2] << 7)) / 64.0; //deg
                    uint16_t angle_index = round(angle); //int deg
                    float range = ((data_scan[3] | (data_scan[4] << 8)) / 4000.0); //meters

                    if ((uart_len == 3) && (angle >= 0) && (angle <= 360)){
                        if (((new_scan_flag == 1) && (range >= 0) && (range <= msgs_laserscan.range_max)) || measures >= 360) {
                            //ESP_LOGI(TAG,"Last measure > Flag: %d | Quality: %02d | Angle: %3.8f | Angle_i: %03d | Range: %2.8f", new_scan_flag, quality, angle, angle_index, range);
                            //ESP_LOGI(TAG,"Package > Measures: %3d", measures);
                            msgs_laserscan.ranges.data[angle_index] = range;
                            lidar_pub_callback();
                            new_scan_flag = 0;
                            measures = 0;
                            timestamp_update(&msgs_laserscan);
                        } else if ((new_scan_flag == 0) && (range > 0) && (range <= msgs_laserscan.range_max) && (range >= msgs_laserscan.range_min)) {
                            //ESP_LOGI(TAG,"Measure > Quality: %02d | Angle: %3.8f | Angle_i: %03d | Range: %2.8f", quality, angle, angle_index, range);
                            measures++;
                            msgs_laserscan.ranges.data[angle_index] = range;
                        }
                    }
                }
                taskYIELD();
            }
        }
    }
    ESP_LOGE(TAG, "Task Delete");
    vTaskDelete(NULL);
}