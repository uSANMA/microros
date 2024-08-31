#include "driver/gpio.h"

#include "esp_log.h"

#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include <sensor_msgs/msg/joint_state.h>
#include <geometry_msgs/msg/twist_stamped.h>

#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"

#define MCPWM_TIMER_RESOLUTION_HZ                               10000000 // 10MHz, 1 tick = 0.1us
#define MCPWM_FREQ_HZ                                           16000    // 16KHz PWM
#define MCPWM_DUTY_TICK_MAX                                     (MCPWM_TIMER_RESOLUTION_HZ / MCPWM_FREQ_HZ) - 20 // maximum value we can set for the duty cycle, in ticks
#define PID_LOOP_PERIOD_MS                                      32 //us 0.03125s = 31.25ms = 31250us
#define PID_LOOP_ID                                             0

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

void motorscontrol_task(void *argument);
static void motorcontrol_loop_cb(TimerHandle_t);

extern void motorcontrol_pub_callback();
extern void timestamp_update(void*arg);

static const char *TAG_MAIN = "Task-MotorControl";

static float PID_EXPECT_SPEED_A = 0;  // expected motor speed, in the pulses counted by the rotary encoder
static char orientation_motora = 'F';

static float PID_EXPECT_SPEED_B = 0;  // expected motor speed, in the pulses counted by the rotary encoder
static char orientation_motorb = 'F';

static const float wheels_separation = 0.13607;
static const float reduction_ratio = 18.8;
static const uint8_t encoder_ticks = 10; 

extern volatile int8_t motorcontrol_status;
extern volatile int8_t motorcontrol_reset_semaphore;

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

TimerHandle_t motorcontrol_timer;
SemaphoreHandle_t timer_motorcontrol;

extern SemaphoreHandle_t uros_boot_motorcontrol;
extern sensor_msgs__msg__JointState msgs_encoders;
extern geometry_msgs__msg__TwistStamped msgs_cmdvel;

//CT voltage = MotorCurrent * 0.155

static void motorcontrol_loop_cb(TimerHandle_t xTimer) {
    xSemaphoreGive(timer_motorcontrol);
}

void motorscontrol_task(void *arg){
    ESP_LOGI(TAG_MAIN, "Creating motorscontrol_task");
    vTaskDelay(pdMS_TO_TICKS(200));

    motorcontrol_status = 0;

    timer_motorcontrol = xSemaphoreCreateBinary();

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

    motorcontrol_status = 2;

    ESP_LOGW(TAG_MAIN, "Waiting semaphore from uROS boot");
    xSemaphoreTake(uros_boot_motorcontrol, portMAX_DELAY);
    ESP_LOGI(TAG_MAIN, "Resuming semaphore...");

    ESP_LOGI(TAG_MAIN, "Enable motor A");
    ESP_ERROR_CHECK(bdc_motor_enable(motor_a));

    ESP_LOGI(TAG_MAIN, "Enable motor B");
    ESP_ERROR_CHECK(bdc_motor_enable(motor_b));

    bdc_motor_set_speed(motor_a, (uint32_t)0);
    bdc_motor_set_speed(motor_b, (uint32_t)0);

    ESP_ERROR_CHECK(bdc_motor_forward(motor_a));
    ESP_ERROR_CHECK(bdc_motor_forward(motor_b));

    ESP_LOGI(TAG_MAIN, "Start motor speed timer loop");
    motorcontrol_timer = xTimerCreate("PID Timer", PID_LOOP_PERIOD_MS, pdTRUE, (void *)PID_LOOP_ID, &motorcontrol_loop_cb);
    if(motorcontrol_timer == NULL) {
            ESP_LOGE(TAG_MAIN, "PID Timer create Error!");
        } else {
            ESP_LOGI(TAG_MAIN, "PID Timer created!");
            if(xTimerStart(motorcontrol_timer, portMAX_DELAY) != pdPASS) {
                ESP_LOGE(TAG_MAIN, "PID Timer start error!");
            } else {
                ESP_LOGI(TAG_MAIN, "PID Timer started!");
            }
        }

    motorcontrol_status = 1;

    while(1){
        xSemaphoreTake(timer_motorcontrol, portMAX_DELAY);

        //ESP_LOGI(TAG_MAIN, "PID CallBack");
        static int last_pulse_count_a = 0;
        static int last_pulse_count_b = 0;
        motor_control_context_t *ctx_a = (motor_control_context_t *)&motor_ctrl_ctx_a;
        motor_control_context_t *ctx_b = (motor_control_context_t *)&motor_ctrl_ctx_b;
        pcnt_unit_handle_t pcnt_unit_a = ctx_a->pcnt_encoder;
        pcnt_unit_handle_t pcnt_unit_b = ctx_b->pcnt_encoder;
        pid_ctrl_block_handle_t pid_ctrl_a = ctx_a->pid_ctrl;
        pid_ctrl_block_handle_t pid_ctrl_b = ctx_b->pid_ctrl;
        bdc_motor_handle_t motor_a = ctx_a->motor;
        bdc_motor_handle_t motor_b = ctx_b->motor;

        // encoder reading
        int cur_pulse_count_a = 0;
        int cur_pulse_count_b = 0;
        pcnt_unit_get_count(pcnt_unit_a, &cur_pulse_count_a);
        pcnt_unit_get_count(pcnt_unit_b, &cur_pulse_count_b);
        double real_pulses_a = cur_pulse_count_a - last_pulse_count_a;
        double real_pulses_b = cur_pulse_count_b - last_pulse_count_b;
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

        float error_a = PID_EXPECT_SPEED_A - real_pulses_a;
        float error_b = PID_EXPECT_SPEED_B - real_pulses_b;
        float new_speed_a = 0;
        float new_speed_b = 0;

        pid_compute(pid_ctrl_a, error_a, &new_speed_a);
        pid_compute(pid_ctrl_b, error_b, &new_speed_b);
        bdc_motor_set_speed(motor_a, (uint32_t)new_speed_a);
        bdc_motor_set_speed(motor_b, (uint32_t)new_speed_b);
        
        motorcontrol_pub_callback();
        while(motorcontrol_reset_semaphore){
            motorcontrol_status = 0;
            vTaskDelay(pdMS_TO_TICKS(10000));
            taskYIELD();
        }
        taskYIELD();
    }
}