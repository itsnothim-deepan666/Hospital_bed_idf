// ESP-IDF Micro-ROS node for motor control, based on motor_idf/src/main.c
// Node name: motor

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

#define MOTOR1_PWM_PIN   GPIO_NUM_27
#define MOTOR1_DIR_PIN   GPIO_NUM_14
#define MOTOR2_PWM_PIN   GPIO_NUM_25
#define MOTOR2_DIR_PIN   GPIO_NUM_26
#define MOTOR3_PWM_PIN   GPIO_NUM_5
#define MOTOR3_DIR_PIN   GPIO_NUM_4
#define MOTOR_COUNT              3
#define MOTOR_PWM_FREQ_HZ        5000
#define MOTOR_PWM_RESOLUTION     LEDC_TIMER_8_BIT
#define MOTOR_PWM_MAX_DUTY       255
#define MOTOR_LEDC_TIMER         LEDC_TIMER_0
#define MOTOR_LEDC_MODE          LEDC_LOW_SPEED_MODE
#define MOTOR_TEST_STEP_MS       3000

static const char *TAG = "MOTOR_MICROROS";

// Micro-ROS objects
rcl_publisher_t publisher;
std_msgs__msg__String msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Motor struct
typedef struct {
    gpio_num_t pwm_pin;
    gpio_num_t dir_pin;
    ledc_channel_t ledc_channel;
    int speed;
} motor_t;

static motor_t s_motors[MOTOR_COUNT] = {
    { MOTOR1_PWM_PIN, MOTOR1_DIR_PIN, LEDC_CHANNEL_0, 0 },
    { MOTOR2_PWM_PIN, MOTOR2_DIR_PIN, LEDC_CHANNEL_1, 0 },
    { MOTOR3_PWM_PIN, MOTOR3_DIR_PIN, LEDC_CHANNEL_2, 0 },
};

static int clamp_speed(int speed) {
    if (speed > MOTOR_PWM_MAX_DUTY) return MOTOR_PWM_MAX_DUTY;
    if (speed < -MOTOR_PWM_MAX_DUTY) return -MOTOR_PWM_MAX_DUTY;
    return speed;
}

static void publish_motor_event(const char *event) {
    msg.data.data = (char *)event;
    msg.data.size = strlen(event);
    msg.data.capacity = msg.data.size + 1;
    rcl_publish(&publisher, &msg, NULL);
}

static void motor_set_speed(int motor_index, int speed) {
    if (motor_index < 0 || motor_index >= MOTOR_COUNT) return;
    motor_t *motor = &s_motors[motor_index];
    int clamped_speed = clamp_speed(speed);
    uint32_t duty = (uint32_t)abs(clamped_speed);
    int dir_level = (clamped_speed >= 0) ? 1 : 0;
    gpio_set_level(motor->dir_pin, dir_level);
    ledc_set_duty(MOTOR_LEDC_MODE, motor->ledc_channel, duty);
    ledc_update_duty(MOTOR_LEDC_MODE, motor->ledc_channel);
    motor->speed = clamped_speed;
    char buf[128];
    snprintf(buf, sizeof(buf), "Motor %d -> speed=%d duty=%lu dir=%d", motor_index + 1, clamped_speed, (unsigned long)duty, dir_level);
    publish_motor_event(buf);
}

static void motor_stop_all(void) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_set_speed(i, 0);
    }
    publish_motor_event("All motors stopped.");
}

static esp_err_t motors_init(void) {
    ledc_timer_config_t timer_cfg = {
        .speed_mode = MOTOR_LEDC_MODE,
        .timer_num = MOTOR_LEDC_TIMER,
        .duty_resolution = MOTOR_PWM_RESOLUTION,
        .freq_hz = MOTOR_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_cfg);
    for (int i = 0; i < MOTOR_COUNT; i++) {
        ledc_channel_config_t channel_cfg = {
            .gpio_num = s_motors[i].pwm_pin,
            .speed_mode = MOTOR_LEDC_MODE,
            .channel = s_motors[i].ledc_channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = MOTOR_LEDC_TIMER,
            .duty = 0,
            .hpoint = 0,
        };
        ledc_channel_config(&channel_cfg);
        gpio_config_t dir_pin_cfg = {
            .pin_bit_mask = (1ULL << s_motors[i].dir_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&dir_pin_cfg);
        gpio_set_level(s_motors[i].dir_pin, 0);
    }
    return ESP_OK;
}

static void motor_test_task(void *arg) {
    (void)arg;
    publish_motor_event("Starting repeating motor direction test sequence");
    while (1) {
        for (int i = 0; i < MOTOR_COUNT; i++) {
            char buf[64];
            snprintf(buf, sizeof(buf), "M%d forward for %d ms", i + 1, MOTOR_TEST_STEP_MS);
            publish_motor_event(buf);
            motor_set_speed(i, MOTOR_PWM_MAX_DUTY);
            vTaskDelay(pdMS_TO_TICKS(MOTOR_TEST_STEP_MS));
            snprintf(buf, sizeof(buf), "M%d reverse for %d ms", i + 1, MOTOR_TEST_STEP_MS);
            publish_motor_event(buf);
            motor_set_speed(i, -MOTOR_PWM_MAX_DUTY);
            vTaskDelay(pdMS_TO_TICKS(MOTOR_TEST_STEP_MS));
            motor_set_speed(i, 0);
        }
        motor_stop_all();
        publish_motor_event("Motor test cycle complete. Restarting.");
    }
}

void app_main(void) {
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "motor", "", &support);
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "motor_event");
    motors_init();
    motor_stop_all();
    xTaskCreate(motor_test_task, "motor_test_task", 4096, NULL, 5, NULL);
}
