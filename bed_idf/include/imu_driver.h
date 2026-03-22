/*
 * imu_driver.h - TCA9548A I2C Multiplexer + MPU6050 IMU driver
 */

#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include "esp_err.h"
#include <stdint.h>

/**
 * @brief Initialize I2C master bus (SDA/SCL pins, clock speed from config.h)
 */
esp_err_t imu_i2c_init(void);

/**
 * @brief Select a TCA9548A multiplexer channel (0-7)
 */
esp_err_t imu_select_mux_channel(uint8_t channel);

/**
 * @brief Initialize an MPU6050 sensor behind a specific mux channel.
 *        Wakes it from sleep and configures ±8 g accelerometer range.
 */
esp_err_t imu_mpu6050_init(uint8_t mux_channel);

/**
 * @brief Read raw accelerometer values (in g) from MPU6050 on a mux channel
 */
esp_err_t imu_read_accel(uint8_t mux_channel, float *ax, float *ay, float *az);

/**
 * @brief Calculate pitch angle (degrees) from accelerometer values.
 *        Uses the same formula and offsets as the reference project.
 */
float imu_calc_pitch(float ax, float ay, float az);

/**
 * @brief Convenience: select mux channel, read accel, compute pitch.
 */
esp_err_t imu_read_pitch(uint8_t mux_channel, float *pitch);

#endif /* IMU_DRIVER_H */
