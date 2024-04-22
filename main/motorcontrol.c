#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_timer.h"

#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <sensor_msgs/msg/joint_state.h>

#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"

static const char *TAG = "MotorControl";

//CT voltage = MotorCurrent * 0.155

#define MCPWM_TIMER_RESOLUTION_HZ   10000000 // 10MHz, 1 tick = 0.1us
#define MCPWM_FREQ_HZ               16000    // 16KHz PWM
#define MCPWM_DUTY_TICK_MAX         (MCPWM_TIMER_RESOLUTION_HZ / MCPWM_FREQ_HZ) - 20 // maximum value we can set for the duty cycle, in ticks
#define PID_LOOP_PERIOD_MS          31250 //us 0.03125s = 31.25ms = 31250us

#define MCPWM_GPIO_A1               3 //forward in2 H brigde
#define MCPWM_GPIO_A2               10 //reverse in1 H bridge
#define ENCODER_GPIO_A1             17 //C2 motor A
#define ENCODER_GPIO_A2             18 //C1 motor A
#define ENCODER_PCNT_HIGH_LIMIT_A   300
#define ENCODER_PCNT_LOW_LIMIT_A    -300
float PID_EXPECT_SPEED_A = 55;  // expected motor speed, in the pulses counted by the rotary encoder

#define MCPWM_GPIO_B1               11 //forward in1 H brigde
#define MCPWM_GPIO_B2               12 //reverse in2 H bridge
#define ENCODER_GPIO_B1             8 //C1 motor A
#define ENCODER_GPIO_B2             13 //C2 motor A
#define ENCODER_PCNT_HIGH_LIMIT_B   300
#define ENCODER_PCNT_LOW_LIMIT_B    -300
float PID_EXPECT_SPEED_B = 55;  // expected motor speed, in the pulses counted by the rotary encoder

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

extern SemaphoreHandle_t got_uros_boot;

extern sensor_msgs__msg__JointState msgs_encoders;

extern void mc_callback();

extern void timestamp_update(void*arg);

static void pid_loop_cb(void *args) {
    //ESP_LOGI(TAG, "PID CallBack");
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
    int real_pulses_a = cur_pulse_count_a - last_pulse_count_a;
    int real_pulses_b = cur_pulse_count_b - last_pulse_count_b;
    last_pulse_count_a = cur_pulse_count_a;
    last_pulse_count_b = cur_pulse_count_b;
    ctx_a->report_pulses = real_pulses_a;
    ctx_b->report_pulses = real_pulses_b;

    // calculate the speed error
    msgs_encoders.velocity.data[0] = real_pulses_a;
    msgs_encoders.velocity.data[1] = real_pulses_b;
    if (real_pulses_a < 0){real_pulses_a = -real_pulses_a;}
    if (real_pulses_b < 0){real_pulses_b = -real_pulses_b;}
    float error_a = PID_EXPECT_SPEED_A - real_pulses_a;
    float error_b = PID_EXPECT_SPEED_B - real_pulses_b;
    float new_speed_a = 0;
    float new_speed_b = 0;

    // set the new speed
    pid_compute(pid_ctrl_a, error_a, &new_speed_a);
    pid_compute(pid_ctrl_b, error_b, &new_speed_b);
    bdc_motor_set_speed(motor_a, (uint32_t)new_speed_a);
    bdc_motor_set_speed(motor_b, (uint32_t)new_speed_b);
    
    timestamp_update(&msgs_encoders);
    mc_callback();
}

void motorscontrol_task(void *arg){

    got_uros_boot = xSemaphoreCreateBinary();

    ESP_LOGI(TAG, "Waiting for uROS boot");
    xSemaphoreTake(got_uros_boot, portMAX_DELAY);
    ESP_LOGI(TAG, "Resuming...");

    ESP_LOGI(TAG, "Motor A Config");
    bdc_motor_config_t motor_config_a = {
        .pwm_freq_hz = MCPWM_FREQ_HZ,
        .pwma_gpio_num = MCPWM_GPIO_A1,
        .pwmb_gpio_num = MCPWM_GPIO_A2,
    };

    ESP_LOGI(TAG, "Motor B Config");
    bdc_motor_config_t motor_config_b = {
        .pwm_freq_hz = MCPWM_FREQ_HZ,
        .pwma_gpio_num = MCPWM_GPIO_B1,
        .pwmb_gpio_num = MCPWM_GPIO_B2,
    };

    ESP_LOGI(TAG, "MCPWM Config");
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = MCPWM_TIMER_RESOLUTION_HZ,
    };

    ESP_LOGI(TAG, "Create new motors devices");
    bdc_motor_handle_t motor_a = NULL;
    bdc_motor_handle_t motor_b = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config_a, &mcpwm_config, &motor_a));
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config_b, &mcpwm_config, &motor_b));
    motor_ctrl_ctx_a.motor = motor_a;
    motor_ctrl_ctx_b.motor = motor_b;

    ESP_LOGI(TAG, "Init pcnt driver to decode A rotary signal");
    pcnt_unit_config_t unit_config_a = {
        .high_limit = ENCODER_PCNT_HIGH_LIMIT_A,
        .low_limit = ENCODER_PCNT_LOW_LIMIT_A,
        .flags.accum_count = true,
    };

    ESP_LOGI(TAG, "Init pcnt driver to decode B rotary signal");
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

    ESP_LOGI(TAG, "Encoder A config channel");
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

    ESP_LOGI(TAG, "Encoder B config channel");
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

    ESP_LOGI(TAG, "Encoder A config edge");
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

    ESP_LOGI(TAG, "Encoder B config edge");
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

    ESP_LOGI(TAG, "PID config constants");
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
    ESP_LOGI(TAG, "Config PID control blocks");
    pid_ctrl_block_handle_t pid_ctrl_a = NULL;
    pid_ctrl_block_handle_t pid_ctrl_b = NULL;
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl_a));
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl_b));
    motor_ctrl_ctx_a.pid_ctrl = pid_ctrl_a;
    motor_ctrl_ctx_b.pid_ctrl = pid_ctrl_b;

    ESP_LOGI(TAG, "Create a high resolution timer to PID calculation for PID A");
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = pid_loop_cb,
        .arg = NULL,
        .name = "pid_loop"
    };

    esp_timer_handle_t pid_loop_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &pid_loop_timer));

    ESP_LOGI(TAG, "Enable motor A");
    ESP_ERROR_CHECK(bdc_motor_enable(motor_a));

    ESP_LOGI(TAG, "Enable motor B");
    ESP_ERROR_CHECK(bdc_motor_enable(motor_b));

    ESP_LOGI(TAG, "Start motor speed timer loop");
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer, PID_LOOP_PERIOD_MS));

    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_ERROR_CHECK(bdc_motor_forward(motor_a));
    vTaskDelay(pdMS_TO_TICKS(1));
    ESP_ERROR_CHECK(bdc_motor_forward(motor_b));

    while (1) {
        //ESP_LOGI(TAG, "Forward motors");
        //ESP_ERROR_CHECK(bdc_motor_forward(motor_a));
        //ESP_ERROR_CHECK(bdc_motor_forward(motor_b));
        //vTaskDelay(pdMS_TO_TICKS(3000));
    
        // ESP_LOGI(TAG, "reverse motors");
        // ESP_ERROR_CHECK(bdc_motor_reverse(motor_a));
        // ESP_ERROR_CHECK(bdc_motor_reverse(motor_b));
        // vTaskDelay(pdMS_TO_TICKS(3000));
    }

    vTaskDelete(NULL);
}