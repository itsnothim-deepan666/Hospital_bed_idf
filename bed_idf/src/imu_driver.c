/*
 * imu_driver.c - TCA9548A I2C Multiplexer + MPU6050 accelerometer driver
 *
 * Provides raw I2C access to MPU6050 sensors behind a TCA9548A multiplexer.
 * Calculates pitch angle from accelerometer data using the same formula
 * as the reference Arduino project.
 */

#include "imu_driver.h"
#include "config.h"

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "IMU";

/* ──────────── low-level I2C helpers ──────────── */

static esp_err_t i2c_write_raw_byte(uint8_t dev_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_write_register(uint8_t dev_addr, uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_registers(uint8_t dev_addr, uint8_t reg,
                                    uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    /* write register address */
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    /* repeated start + read */
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* ──────────── public API ──────────── */

esp_err_t imu_i2c_init(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_SDA_PIN,
        .scl_io_num       = I2C_SCL_PIN,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    esp_err_t ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "I2C master initialized (SDA=%d SCL=%d %d Hz)",
                 I2C_SDA_PIN, I2C_SCL_PIN, I2C_MASTER_FREQ_HZ);
    }
    return ret;
}

esp_err_t imu_select_mux_channel(uint8_t channel)
{
    if (channel > 7) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = i2c_write_raw_byte(TCA9548A_ADDR, 1 << channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select mux channel %d", channel);
    }
    vTaskDelay(pdMS_TO_TICKS(1));   /* let bus settle */
    return ret;
}

esp_err_t imu_mpu6050_init(uint8_t mux_channel)
{
    esp_err_t ret = imu_select_mux_channel(mux_channel);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(10));

    /* Wake up MPU6050 (clear SLEEP bit in PWR_MGMT_1) */
    ret = i2c_write_register(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 NOT found on mux channel %d (err %d)",
                 mux_channel, ret);
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Set accelerometer range to ±8 g  (AFS_SEL = 2 → bits [4:3] = 10) */
    ret = i2c_write_register(MPU6050_ADDR, MPU6050_REG_ACCEL_CONFIG, 0x10);
    if (ret != ESP_OK) return ret;

    /* Set DLPF to ~21 Hz bandwidth (DLPF_CFG = 4) */
    ret = i2c_write_register(MPU6050_ADDR, MPU6050_REG_CONFIG, 0x04);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "MPU6050 initialized on mux channel %d", mux_channel);
    return ESP_OK;
}

esp_err_t imu_read_accel(uint8_t mux_channel, float *ax, float *ay, float *az)
{
    esp_err_t ret = imu_select_mux_channel(mux_channel);
    if (ret != ESP_OK) return ret;

    uint8_t data[6];
    ret = i2c_read_registers(MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, data, 6);
    if (ret != ESP_OK) return ret;

    int16_t raw_x = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_y = (int16_t)((data[2] << 8) | data[3]);
    int16_t raw_z = (int16_t)((data[4] << 8) | data[5]);

    *ax = (float)raw_x / MPU6050_ACCEL_SENSITIVITY;
    *ay = (float)raw_y / MPU6050_ACCEL_SENSITIVITY;
    *az = (float)raw_z / MPU6050_ACCEL_SENSITIVITY;

    return ESP_OK;
}

float imu_calc_pitch(float ax, float ay, float az)
{
    /* Apply the same offsets used in the reference project */
    ax -= 0.05f;
    ay -= (-0.01f);
    az -= 0.03f;

    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / (float)M_PI;
    return pitch;
}

esp_err_t imu_read_pitch(uint8_t mux_channel, float *pitch)
{
    float ax, ay, az;
    esp_err_t ret = imu_read_accel(mux_channel, &ax, &ay, &az);
    if (ret != ESP_OK) return ret;

    *pitch = imu_calc_pitch(ax, ay, az);
    return ESP_OK;
}
