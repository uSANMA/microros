#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_timer.h"

#include "std_msgs/msg/header.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_sntp.h"
#include "driver/gpio.h"

#define MAC_BASE_CUSTOM 0

#define WIFI_CONNECTED_BIT          BIT0
#define WIFI_FAIL_BIT               BIT1
#define GPIO_PIN_LED_GREEN          39 //gpio led pin 32
#define GPIO_PIN_LED_BLUE           40 //gpio led pin 33
#define GPIO_PIN_LED_RED            42 //gpio led pin 35

#define LEDS_LOOP_PERIOD_MS          50 //ms
#define LEDS_LOOP_ID                 0

static EventGroupHandle_t s_wifi_event_group;

SemaphoreHandle_t got_time_semaphore;

static const char *TAG_NET = "NETWORK";
static const char *TAG_MAIN = "APP_MAIN";
static const char *TAG_NTP = "NTP";

static int s_retry_num = 0;

volatile int8_t main_status = 0;
volatile int8_t uros_status = 0;
volatile int8_t lidar_status = 0;
volatile int8_t sensors_status = 0;
volatile uint8_t schedule_flag = 0;

static SemaphoreHandle_t timer_leds_semaphore;
static TimerHandle_t leds_timer;

static void wifi_init_sta(void);
static void event_handler(void*, esp_event_base_t, int32_t, void*);
static void print_time();
static void on_got_time(struct timeval *);
void timestamp_update(void *);
extern void uros_task(void *);
extern void sensors_task(void *);
extern void motorscontrol_task(void *);
extern void lidar_task(void *);
extern void ota_task(void *);
static void leds_loop_cb(TimerHandle_t);
static void leds_init();

static void leds_loop_cb(TimerHandle_t xTimer) {
    xSemaphoreGive(timer_leds_semaphore);
}

static void leds_init(){
    ESP_LOGI(TAG_MAIN,"Initing leds driver...");
    gpio_reset_pin(GPIO_PIN_LED_BLUE);
    gpio_set_direction(GPIO_PIN_LED_BLUE, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(GPIO_PIN_LED_BLUE, GPIO_FLOATING);
    gpio_set_drive_capability(GPIO_PIN_LED_BLUE, GPIO_DRIVE_CAP_3);
    gpio_set_level(GPIO_PIN_LED_BLUE, 0);

    gpio_reset_pin(GPIO_PIN_LED_GREEN);
    gpio_set_direction(GPIO_PIN_LED_GREEN, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(GPIO_PIN_LED_GREEN, GPIO_FLOATING);
    gpio_set_drive_capability(GPIO_PIN_LED_GREEN, GPIO_DRIVE_CAP_3);
    gpio_set_level(GPIO_PIN_LED_GREEN, 0);

    gpio_reset_pin(GPIO_PIN_LED_RED);
    gpio_set_direction(GPIO_PIN_LED_RED, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(GPIO_PIN_LED_RED, GPIO_FLOATING);
    gpio_set_drive_capability(GPIO_PIN_LED_RED, GPIO_DRIVE_CAP_3);
    gpio_set_level(GPIO_PIN_LED_RED, 0);
}

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < CONFIG_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG_NET, "Trying again");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGE(TAG_NET,"Connection fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG_NET, "IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void) {
#if (MAC_BASE_CUSTOM == 1)
    unsigned char cust_mac_base[6] = {0xCC,0xDB,0xA7,0x5A,0x30,0x1C};
    esp_base_mac_addr_set(cust_mac_base);
#endif

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    unsigned char mac_base[6] = {0};
    unsigned char mac_local_base[6] = {0};
    unsigned char mac_uni_base[6] = {0};

    esp_efuse_mac_get_default(mac_base);
    esp_read_mac(mac_base, ESP_MAC_WIFI_STA);
    esp_derive_local_mac(mac_local_base, mac_uni_base);
    ESP_LOGI(TAG_NET, "MAC BASE: %02X:%02X:%02X:%02X:%02X:%02X", mac_base[0],mac_base[1],mac_base[2],mac_base[3],mac_base[4],mac_base[5]);
    ESP_LOGI(TAG_NET, "MAC LOCAL BASE: %02X:%02X:%02X:%02X:%02X:%02X", mac_local_base[0],mac_local_base[1],mac_local_base[2],\
    mac_local_base[3],mac_local_base[4],mac_local_base[5]);
    ESP_LOGI(TAG_NET, "MAC uni: %02X:%02X:%02X:%02X:%02X:%02X", mac_uni_base[0],mac_uni_base[1],mac_uni_base[2],\
    mac_uni_base[3],mac_uni_base[4],mac_uni_base[5]);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG_NET, "Connected in SSID: %s", CONFIG_ESP_WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGW(TAG_NET, "Connection fail: %s", CONFIG_ESP_WIFI_PASSWORD);
    } else {
        ESP_LOGE(TAG_NET, "Error on Wi-fi connection!");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
    }
}

static void print_time(){
    time_t now = 0;
    time(&now);

    struct tm * time_info = localtime(&now);

    char time_buffer[50];

    strftime(time_buffer, sizeof(time_buffer), "%c", time_info);
    ESP_LOGI(TAG_NTP, "%s", time_buffer);
}

static void on_got_time(struct timeval *tv){
    print_time();
    xSemaphoreGive(got_time_semaphore);
}

void timestamp_update(void *arg){
    struct timespec timestamp_raw;

    typedef struct std_msgs__msg__Header_temp {
        std_msgs__msg__Header header;
    } std_msgs__msg__Header_temp;

    std_msgs__msg__Header_temp *temp_data = (std_msgs__msg__Header_temp*) arg;
    if (temp_data == NULL) {
        return;
    }
    clock_gettime(CLOCK_REALTIME, &timestamp_raw);
    temp_data->header.stamp.sec = timestamp_raw.tv_sec;
    temp_data->header.stamp.nanosec = timestamp_raw.tv_nsec;
}

void app_main(void) {

    gpio_set_level(GPIO_PIN_LED_GREEN, 1);
    gpio_set_level(GPIO_PIN_LED_RED,   1);

    leds_init();

    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG_MAIN, "--------------- MCU begin ---------------");
    main_status = 0;

    got_time_semaphore = xSemaphoreCreateBinary();
    timer_leds_semaphore = xSemaphoreCreateBinary();

    setenv("TZ", "<-03>3", 1);
    tzset();

    print_time();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();

#ifdef UCLIENT_PROFILE_UDP
    SP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    esp_sntp_init();
    esp_sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_set_time_sync_notification_cb(on_got_time);

    ESP_LOGI(TAG_MAIN, "Waiting for sync real time");
    xSemaphoreTake(got_time_semaphore, portMAX_DELAY);

    main_status = 1;

    ESP_LOGI(TAG_MAIN, "Creating xTasks");
    xTaskCreate(uros_task, "uROS Task", 1024 * 8, NULL, 5, NULL);
    xTaskCreate(sensors_task, "Sensors Task", 1024 * 4, NULL, 4, NULL);
    xTaskCreate(lidar_task, "Lidar Task", 1024 * 6, NULL, 4, NULL);
    xTaskCreate(ota_task, "OTA Task", 1024 * 6, NULL, 3, NULL);

    ESP_LOGI(TAG_MAIN, "Start leds timer loop");
    leds_timer = xTimerCreate("Leds timer", LEDS_LOOP_PERIOD_MS, pdTRUE, (void *)LEDS_LOOP_ID, &leds_loop_cb);
    if(leds_timer == NULL) {
        ESP_LOGE(TAG_MAIN, "Leds timer create Error!");
    } else {
        ESP_LOGI(TAG_MAIN, "Leds timer created!");
        if(xTimerStart(leds_timer, portMAX_DELAY) != pdPASS) {
            ESP_LOGE(TAG_MAIN, "Leds timer start error!");
        } else {
            ESP_LOGI(TAG_MAIN, "Leds timer started!");
        }
    }

    while(1){
        xSemaphoreTake(timer_leds_semaphore, pdMS_TO_TICKS(500));

        if((main_status == -1) || (uros_status == -1) || (lidar_status == -1) || (sensors_status == -1)){ // Error > Aborting
            gpio_set_level(GPIO_PIN_LED_GREEN, 0);
            gpio_set_level(GPIO_PIN_LED_RED,   1);
        } else if ((main_status == 0) || (uros_status == 0) || (lidar_status == 0) || (sensors_status == 0)){ // Powering On
            gpio_set_level(GPIO_PIN_LED_GREEN, 0);
            gpio_set_level(GPIO_PIN_LED_RED,   0);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(GPIO_PIN_LED_GREEN, 1);
            gpio_set_level(GPIO_PIN_LED_RED,   1);
            vTaskDelay(pdMS_TO_TICKS(200));
        } else if ((main_status == 2) || (uros_status == 2) || (lidar_status == 2) || (sensors_status == 2)){ // Initializing
            gpio_set_level(GPIO_PIN_LED_GREEN, 1);
            gpio_set_level(GPIO_PIN_LED_RED,   1);
        } else if ((main_status == 1) && (uros_status == 1) && (lidar_status == 1) && (sensors_status == 1)){ // Activated
            gpio_set_level(GPIO_PIN_LED_GREEN, 1);
            gpio_set_level(GPIO_PIN_LED_RED,   0);
        }

        if(schedule_flag){
            gpio_set_level(GPIO_PIN_LED_BLUE, 1);
            vTaskDelay(pdMS_TO_TICKS(30));
            gpio_set_level(GPIO_PIN_LED_BLUE, 0);
            vTaskDelay(pdMS_TO_TICKS(30));
            schedule_flag = 0;
        }
    }
}