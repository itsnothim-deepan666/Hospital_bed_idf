/*
 * motor_ctrl.h - LEDC PWM motor control for 3 DC motors
 */

#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize GPIO direction pins and LEDC PWM channels for all 3 motors
 */
esp_err_t motor_init(void);

/**
 * @brief Stop all motors (set PWM duty to 0)
 */
void motor_stop_all(void);

/**
 * @brief Set a specific motor's direction and speed.
 * @param motor_idx  0 = Top, 1 = Bottom, 2 = Side
 * @param dir_high   true = HIGH direction pin, false = LOW
 * @param duty       PWM duty (0-255)
 */
void motor_set(int motor_idx, bool dir_high, uint32_t duty);

/**
 * @brief Move a motor until the IMU pitch reaches the target angle (with timeout).
 * @param motor_idx     0 = Top, 1 = Bottom, 2 = Side
 * @param mux_channel   TCA9548A channel for the corresponding IMU
 * @param target        Target pitch angle (degrees)
 * @param go_up         true = pitch should increase, false = pitch should decrease
 * @param dir_high      Direction pin level for the motor
 * @param timeout_ms    Maximum time to run motor before giving up
 * @return true if target reached, false on timeout
 */
bool motor_move_to_angle(int motor_idx, uint8_t mux_channel,
                         float target, bool go_up, bool dir_high,
                         uint32_t timeout_ms);

#endif /* MOTOR_CTRL_H */
