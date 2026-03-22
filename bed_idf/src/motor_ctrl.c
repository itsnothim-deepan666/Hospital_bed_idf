/*
 * motor_ctrl.c - LEDC PWM motor control for 3 DC motors with IMU feedback
 *
 * Uses ESP-IDF LEDC peripheral for PWM generation and GPIO for direction.
 * Motor-to-angle feedback loop reads IMU pitch via imu_driver.
 */

#include "motor_ctrl.h"
#include "imu_driver.h"
#include "config.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MOTOR";

/* Pin and LEDC channel lookup tables */
static const gpio_num_t pwm_pins[3]  = { MOTOR1_PWM_PIN, MOTOR2_PWM_PIN, MOTOR3_PWM_PIN };
static const gpio_num_t dir_pins[3]  = { MOTOR1_DIR_PIN, MOTOR2_DIR_PIN, MOTOR3_DIR_PIN };
static const ledc_channel_t ledc_ch[3] = { MOTOR_LEDC_CH_1, MOTOR_LEDC_CH_2, MOTOR_LEDC_CH_3 };

esp_err_t motor_init(void)
{
    /* Configure LEDC timer (shared by all 3 channels) */
    ledc_timer_config_t timer_conf = {
        .speed_mode      = MOTOR_LEDC_MODE,
        .timer_num       = MOTOR_LEDC_TIMER,
        .duty_resolution = MOTOR_LEDC_RESOLUTION,
        .freq_hz         = MOTOR_LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    /* Configure each motor's LEDC channel and direction GPIO */
    for (int i = 0; i < 3; i++) {
        ledc_channel_config_t ch_conf = {
            .speed_mode = MOTOR_LEDC_MODE,
            .channel    = ledc_ch[i],
            .timer_sel  = MOTOR_LEDC_TIMER,
            .intr_type  = LEDC_INTR_DISABLE,
            .gpio_num   = pwm_pins[i],
            .duty       = 0,
            .hpoint     = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));

        /* Direction pin as simple GPIO output */
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << dir_pins[i]),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&io_conf));
    }

    /* LED pin */
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&led_conf));

    ESP_LOGI(TAG, "Motors and LED initialized");
    return ESP_OK;
}

void motor_stop_all(void)
{
    for (int i = 0; i < 3; i++) {
        ledc_set_duty(MOTOR_LEDC_MODE, ledc_ch[i], 0);
        ledc_update_duty(MOTOR_LEDC_MODE, ledc_ch[i]);
    }
    ESP_LOGI(TAG, "All motors stopped");
}

void motor_set(int motor_idx, bool dir_high, uint32_t duty)
{
    if (motor_idx < 0 || motor_idx > 2) return;

    gpio_set_level(dir_pins[motor_idx], dir_high ? 1 : 0);
    ledc_set_duty(MOTOR_LEDC_MODE, ledc_ch[motor_idx], duty);
    ledc_update_duty(MOTOR_LEDC_MODE, ledc_ch[motor_idx]);
}

bool motor_move_to_angle(int motor_idx, uint8_t mux_channel,
                         float target, bool go_up, bool dir_high,
                         uint32_t timeout_ms)
{
    ESP_LOGI(TAG, "Moving motor %d to %.1f° (go_up=%d dir=%d)",
             motor_idx, target, go_up, dir_high);

    motor_set(motor_idx, dir_high, MOTOR_FULL_SPEED);

    uint32_t start = xTaskGetTickCount();
    uint32_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    while ((xTaskGetTickCount() - start) < timeout_ticks) {
        float pitch = 0.0f;
        esp_err_t ret = imu_read_pitch(mux_channel, &pitch);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "IMU read failed, continuing...");
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (go_up && pitch >= target) {
            ESP_LOGI(TAG, "Motor %d reached target %.1f° (current %.1f°)",
                     motor_idx, target, pitch);
            motor_stop_all();
            return true;
        }
        if (!go_up && pitch <= target) {
            ESP_LOGI(TAG, "Motor %d reached target %.1f° (current %.1f°)",
                     motor_idx, target, pitch);
            motor_stop_all();
            return true;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    ESP_LOGW(TAG, "Motor %d TIMEOUT moving to %.1f°", motor_idx, target);
    motor_stop_all();
    return false;
}
