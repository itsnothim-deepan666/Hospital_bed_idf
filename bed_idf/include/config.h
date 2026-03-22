/*
 * config.h - Project-wide configuration for the BED ESP-IDF application
 *
 * Pin assignments, WiFi credentials, I2C addresses, motor/LEDC config,
 * Nextion UART config, and IMU mux channel mapping.
 * All values match the reference Arduino "bed" project.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "driver/uart.h"

/* ────────────────── WiFi ────────────────── */
#define WIFI_SSID       "deepan"
#define WIFI_PASS       "hehehehe"
#define WIFI_MAX_RETRY  10

/* ────────────────── UDP ────────────────── */
#define UDP_PORT        12345
#define UDP_RX_BUF_SIZE 256

/* ────────────────── I2C ────────────────── */
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_SDA_PIN         GPIO_NUM_21
#define I2C_SCL_PIN         GPIO_NUM_22
#define I2C_MASTER_FREQ_HZ  100000      /* 100 kHz */

/* ────────────────── TCA9548A Multiplexer ────────────────── */
#define TCA9548A_ADDR   0x70

/* ────────────────── MPU6050 IMU ────────────────── */
#define MPU6050_ADDR                0x68
#define MPU6050_REG_PWR_MGMT_1     0x6B
#define MPU6050_REG_ACCEL_CONFIG   0x1C
#define MPU6050_REG_CONFIG         0x1A
#define MPU6050_REG_ACCEL_XOUT_H   0x3B
#define MPU6050_ACCEL_SENSITIVITY  4096.0f  /* ±8 g range */

/* IMU mux channel assignments */
#define IMU_MUX_CH_TOP      0
#define IMU_MUX_CH_BOTTOM   2
#define IMU_MUX_CH_SIDES    3

/* ────────────────── Motor Pins ────────────────── */
/* Motor 1 - Top section */
#define MOTOR1_PWM_PIN  GPIO_NUM_32
#define MOTOR1_DIR_PIN  GPIO_NUM_33
/* Motor 2 - Bottom section */
#define MOTOR2_PWM_PIN  GPIO_NUM_18
#define MOTOR2_DIR_PIN  GPIO_NUM_19
/* Motor 3 - Side section */
#define MOTOR3_PWM_PIN  GPIO_NUM_5
#define MOTOR3_DIR_PIN  GPIO_NUM_4

/* ────────────────── LEDC (PWM) Config ────────────────── */
#define MOTOR_LEDC_TIMER        LEDC_TIMER_0
#define MOTOR_LEDC_MODE         LEDC_LOW_SPEED_MODE
#define MOTOR_LEDC_FREQ_HZ      5000
#define MOTOR_LEDC_RESOLUTION   LEDC_TIMER_8_BIT   /* 0-255 */
#define MOTOR_LEDC_CH_1         LEDC_CHANNEL_0
#define MOTOR_LEDC_CH_2         LEDC_CHANNEL_1
#define MOTOR_LEDC_CH_3         LEDC_CHANNEL_2
#define MOTOR_FULL_SPEED        255
#define MOTOR_TIMEOUT_MS        10000

/* ────────────────── Nextion Display (UART2) ────────────────── */
#define NEXTION_UART_NUM    UART_NUM_2
#define NEXTION_TX_PIN      GPIO_NUM_17
#define NEXTION_RX_PIN      GPIO_NUM_16
#define NEXTION_BAUD        9600
#define NEXTION_BUF_SIZE    256

/* ────────────────── LED ────────────────── */
#define LED_PIN     GPIO_NUM_2

/* ────────────────── Menu Config ────────────────── */
#define MENU_PAGE0_COUNT    4       /* TOP, RIGHT, BOTTOM, LEFT */
#define MENU_PAGE1_COUNT    5       /* 0°, 30°, 45°, 60°, BACK */

#endif /* CONFIG_H */
