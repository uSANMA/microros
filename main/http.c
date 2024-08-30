#include <string.h>
#include <stdio.h>
#include <time.h>

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_types.h"
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_system.h>
#include <sys/param.h>
#include <esp_http_server.h>
#include <esp_ota_ops.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

void ota_task(void *argument);
static void init_server(void);
static esp_err_t http_handle(httpd_req_t *req);
static esp_err_t http_handle_ota(httpd_req_t *req);
static void ota_request();
static void restart_request();
static void menu_request();
static void login();
typedef void (*function_t)(void);

static const char *TAG_MAIN = "Task-HTTP";

static volatile int8_t ota_rt = 0;
static volatile int8_t restart_rt = 0;

extern volatile int8_t main_status;
extern volatile int8_t uros_status;
extern volatile int8_t lidar_status;
extern volatile int8_t sensors_status;
extern volatile int8_t motorcontrol_status;

volatile int8_t uros_reset_semaphore;
volatile int8_t lidar_reset_semaphore;
volatile int8_t sensors_reset_semaphore;
volatile int8_t motorcontrol_reset_semaphore;

extern const uint8_t file_home_html_start[] asm("_binary_home_html_start");
extern const uint8_t file_home_html_end[] asm("_binary_home_html_end");
// Style and frameworks
extern const uint8_t file_styles_css_start[] asm("_binary_styles_css_start");
extern const uint8_t file_styles_css_end[] asm("_binary_styles_css_end");
extern const uint8_t file_mini_css_start[] asm("_binary_mini_css_start");
extern const uint8_t file_mini_css_end[] asm("_binary_mini_css_end");
// uwaba png
extern const uint8_t file_uwaba_png_start[] asm("_binary_uwaba_png_start");
extern const uint8_t file_uwaba_png_end[] asm("_binary_uwaba_png_end");
// OTA
extern const uint8_t file_ota_html_start[] asm("_binary_ota_html_start");
extern const uint8_t file_ota_html_end[] asm("_binary_ota_html_end");
// Restart
extern const uint8_t file_restart_html_start[] asm("_binary_restart_html_start");
extern const uint8_t file_restart_html_end[] asm("_binary_restart_html_end");
// Restart
extern const uint8_t file_menu_html_start[] asm("_binary_menu_html_start");
extern const uint8_t file_menu_html_end[] asm("_binary_menu_html_end");

typedef struct file_handle {
   const char *path;
   const uint8_t *file_start;
   const uint8_t *file_end;
   const char *file_type;
   bool is_gzip;
   function_t function;
} file_handle_t;

static httpd_uri_t http_uri = {
   .uri = "/*",
   .method = HTTP_GET,
   .handler = http_handle,
   .user_ctx = NULL
};

static httpd_uri_t http_uri_ota = {
   .uri = "/otarequest",
   .method = HTTP_POST,
   .handler = http_handle_ota,
   .user_ctx = NULL
};

static file_handle_t file_list[] = {
   {"/", file_home_html_start, file_home_html_end, "text/html", 0, &login},
   {"/home.html", file_home_html_start, file_home_html_end, "text/html", 0, &login},
   {"/ota.html", file_ota_html_start, file_ota_html_end, "text/html", 0, &ota_request},
   {"/restart.html", file_restart_html_start, file_restart_html_end, "text/html", 0, &restart_request},
   {"/menu.html", file_menu_html_start, file_menu_html_end, "text/html", 0, &menu_request},
   {"/mini.css", file_mini_css_start, file_mini_css_end, "text/css", 0, NULL},
   {"/styles.css", file_styles_css_start, file_styles_css_end, "text/css", 0, NULL},
   {"/uwaba.png", file_uwaba_png_start, file_uwaba_png_end, "image/x-icon", 0, NULL}
};

static httpd_handle_t server_handle = NULL;

static esp_err_t http_handle(httpd_req_t *req) {
   file_handle_t *pList = NULL;
   for (uint32_t i = 0; i < (sizeof(file_list) / sizeof(file_list[0])); i++) {
      if (strstr(req->uri, file_list[i].path) != NULL) {
         if (strlen(req->uri) == strlen(file_list[i].path)) {
            if (file_list[i].file_start != NULL) {
               pList = &(file_list[i]);
               break;
            }
         }
      }
   }

   if (pList != NULL) {
      if (pList->function != NULL) {
         pList->function();
      }
      
      ESP_ERROR_CHECK(httpd_resp_set_type(req, pList->file_type));
   
      if (pList->is_gzip) {
         ESP_ERROR_CHECK(httpd_resp_set_hdr(req, "list-Encoding", "gzip"));
      }
      
      httpd_resp_send_chunk(req, (const char *) pList->file_start, pList->file_end - pList->file_start);
      httpd_resp_send_chunk(req, NULL, 0);
      return ESP_OK;
   } else if (strstr(req->uri, "/ota_status.json") != NULL) {
      ESP_LOGI(TAG_MAIN, "Sending OTA status");
      char ota_status[100];
      sprintf(ota_status, "{\"status\":%d,\"compile_time\":\"%s\",\"compile_date\":\"%s\"}", ota_rt, __TIME__, __DATE__);
      httpd_resp_sendstr(req, ota_status);
      return ESP_OK;
   } else if (strstr(req->uri, "/uwaba_status.json") != NULL){
      if (main_status == 1 && uros_status == 1 && lidar_status == 1 && sensors_status == 1 && motorcontrol_status == 1) {
         ESP_LOGI(TAG_MAIN, "Sending uWABA status");
      }
      struct timespec timestamp_raw;
      char log[128];
      clock_gettime(CLOCK_REALTIME, &timestamp_raw);
      sprintf(log, "{\"timestamp\":%ld, \"main_status\":%d,\"uros_status\":%d,\"lidar_status\":%d,\"sensors_status\":%d,\"motorcontrol_status\":%d}", timestamp_raw.tv_nsec, main_status, uros_status, lidar_status, sensors_status, motorcontrol_status);
      httpd_resp_sendstr(req, log);
      return ESP_OK;
   }

   httpd_resp_send_404(req);
   return ESP_OK;
}

static esp_err_t http_handle_ota(httpd_req_t *req) {
   ESP_LOGI(TAG_MAIN, "Receiving OTA");
   esp_ota_handle_t ota_handle;

   char ota_buff[2048];

   int ota_length = req->content_len;
   int ota_received = 0;
   int recv_ota_len;
   bool ota_started = 0;
   ota_rt = -1;
   const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);

   do  {
      if ((recv_ota_len = httpd_req_recv(req, ota_buff, MIN(ota_length, sizeof(ota_buff)))) < 0) {
         if (recv_ota_len == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
         }
         return ESP_FAIL;
      }

      ESP_LOGI(TAG_MAIN, "Receive: %d of %d\r", ota_received, ota_length);
      if (!ota_started) {
         ota_started = 1;
         char *pBody_start = strstr(ota_buff, "\r\n\r\n") + 4;
         int body_part_len = recv_ota_len - (pBody_start - ota_buff);

         int body_part_sta = recv_ota_len - body_part_len;
         ESP_LOGI(TAG_MAIN, "Ota size: %d : Start in:%d - End in:%d", ota_length, body_part_sta, body_part_len);
      
         esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
         if (err != ESP_OK) {
            ESP_LOGE(TAG_MAIN, "Error on begin OTA file > Aborting OTA");
            return ESP_FAIL;
         } else {
            ESP_LOGI(TAG_MAIN, "Writing to partition %d at 0x%lx", update_partition->subtype, update_partition->address);
         }

         esp_ota_write(ota_handle, pBody_start, body_part_len);
      } else {
         esp_ota_write(ota_handle, ota_buff, recv_ota_len);
         ota_received += recv_ota_len;
      }
   } while (recv_ota_len > 0 && ota_received < ota_length);

   if (esp_ota_end(ota_handle) == ESP_OK) {
      if (esp_ota_set_boot_partition(update_partition) == ESP_OK) {
         const esp_partition_t *boot_partition = esp_ota_get_boot_partition();
         ESP_LOGI(TAG_MAIN, "Changing to boot partition %d at 0x%lx", boot_partition->subtype, boot_partition->address);
         ESP_LOGI(TAG_MAIN, "Restarting...");
         ota_rt = 1;
         restart_rt = 1;
      } else {
         ESP_LOGE(TAG_MAIN, "Flash Error!");
         ota_rt = -1;
      }
   } else {
      ESP_LOGE(TAG_MAIN, "OTA End Error!");
      ota_rt = -1;
   }
   return ESP_OK;
}

static void init_server(void) {
   ESP_LOGI(TAG_MAIN, "Initializing HTTP Server");
   httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();

      server_config.task_priority = 3;
      server_config.stack_size = (8192);
      server_config.core_id = 0;
      server_config.server_port = 80;
      server_config.uri_match_fn = httpd_uri_match_wildcard;
      server_config.max_resp_headers = 50;
   
   esp_err_t err = httpd_start(&server_handle, &server_config);
   
   if (err == ESP_OK) {
      ESP_ERROR_CHECK(httpd_register_uri_handler(server_handle, &http_uri_ota));
      ESP_ERROR_CHECK(httpd_register_uri_handler(server_handle, &http_uri));
   } else {
    ESP_LOGE(TAG_MAIN, "Erro on http");
   }
}

static void ota_request() {
   ESP_LOGI(TAG_MAIN, "Ota page");
   ota_rt = 0;
}

static void restart_request() {
   ESP_LOGI(TAG_MAIN, "Restart page");
   restart_rt = 1;
}

static void menu_request() {
   ESP_LOGI(TAG_MAIN, "Menu page");
}

static void login() {
   ESP_LOGI(TAG_MAIN, "Login page");
}

void ota_task(void * arg){

   init_server();

   while(1){
      if (restart_rt) {
         ESP_LOGW(TAG_MAIN, "HTTP restart request!");
         lidar_reset_semaphore = 1;
         sensors_reset_semaphore = 1;
         motorcontrol_reset_semaphore = 1;
         uros_reset_semaphore = 1;
         vTaskDelay(pdMS_TO_TICKS(2000));
         httpd_stop(server_handle);
         esp_wifi_stop();
         esp_restart();
      }
      taskYIELD();
   }
   ESP_LOGE(TAG_MAIN, "Task Delete");
   vTaskDelete(NULL);
}