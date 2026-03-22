/*
 * main.c - BED ESP-IDF Application
 *
 * Entry point and blink-driven menu state machine.
 *
 * Flow:
 *   1. Python detects blinks via MediaPipe and sends UDP commands:
 *      "0" = short blink  → CYCLE through menu options
 *      "1" = long  blink  → SELECT / CONFIRM current option
 *
 *   2. ESP32 receives commands and navigates a two-page menu:
 *      Page 0: Select axis   → TOP | RIGHT | BOTTOM | LEFT
 *      Page 1: Select angle  → 0° | 30° | 45° | 60° | BACK
 *
 *   3. On confirmation, the corresponding motor moves to the chosen
 *      angle using closed-loop IMU pitch feedback.
 *
 *   4. Nextion display mirrors the menu state via UART commands.
 *
 * Hardware:
 *   - ESP32-WROVER or ESP32-DevKitC
 *   - TCA9548A I2C multiplexer (addr 0x70)
 *   - 3 × MPU6050 IMUs on mux channels 0 (top), 2 (bottom), 3 (sides)
 *   - 3 × DC motors with H-bridge drivers (PWM + DIR pins)
 *   - Nextion display on UART2
 *   - Built-in LED on GPIO 2
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "config.h"
#include "imu_driver.h"
#include "motor_ctrl.h"
#include "wifi_udp.h"
#include "nextion_uart.h"

static const char *TAG = "MAIN";

/* ═══════════════════════════════════════════════════════════════════
 *  Menu state
 * ═══════════════════════════════════════════════════════════════════ */

static int current_page  = 0;   /* 0 = axis selection, 1 = angle selection */
static int sel_page0     = 0;   /* 0..3 index on page 0 */
static int sel_page1     = 0;   /* 0..4 index on page 1 */

/* IMU-initialised flags */
static bool imu_top_ok    = false;
static bool imu_bottom_ok = false;
static bool imu_sides_ok  = false;

/* Axis names for logging */
static const char *axis_names[MENU_PAGE0_COUNT] = {
    "TOP", "RIGHT", "BOTTOM", "LEFT"
};

/* Target angles selectable on page 1 */
static const float angle_values[MENU_PAGE1_COUNT] = {
    0.0f, 30.0f, 45.0f, 60.0f, 0.0f  /* last entry is BACK, angle unused */
};

/* Nextion button component names (must match HMI layout) */
static const char *page0_btn[MENU_PAGE0_COUNT] = { "b0", "b1", "b3", "b2" };
static const char *page1_btn[MENU_PAGE1_COUNT] = { "b1", "b2", "b3", "b4", "b0" };

/* Nextion colour values: highlighted vs default */
#define NX_COLOR_HIGHLIGHT  65504   /* yellow-ish */
#define NX_COLOR_DEFAULT    8200    /* dark grey */

/* ──────────── Nextion helpers ──────────── */

static void nextion_update_page0_buttons(void)
{
    char buf[48];
    for (int i = 0; i < MENU_PAGE0_COUNT; i++) {
        snprintf(buf, sizeof(buf), "%s.bco=%d",
                 page0_btn[i],
                 (i == sel_page0) ? NX_COLOR_HIGHLIGHT : NX_COLOR_DEFAULT);
        nextion_send_command(buf);
    }
}

static void nextion_update_page1_buttons(void)
{
    char buf[48];
    for (int i = 0; i < MENU_PAGE1_COUNT; i++) {
        snprintf(buf, sizeof(buf), "%s.bco=%d",
                 page1_btn[i],
                 (i == sel_page1) ? NX_COLOR_HIGHLIGHT : NX_COLOR_DEFAULT);
        nextion_send_command(buf);
    }
}

/* ──────────── Motor movement executors ──────────── */

/*
 * Move a motor+IMU pair to a target angle.
 * Direction logic matches the reference project exactly:
 *   TOP    (motor 0, mux 0): up = DIR HIGH, down = DIR LOW
 *   BOTTOM (motor 1, mux 2): up = DIR LOW,  down = DIR HIGH
 *   SIDE   (motor 2, mux 3): left = DIR LOW, right = DIR HIGH
 */

static void execute_top_move(float target)
{
    float pitch = 0;
    imu_read_pitch(IMU_MUX_CH_TOP, &pitch);
    ESP_LOGI(TAG, "TOP current pitch=%.1f° target=%.1f°", pitch, target);

    if (pitch < target) {
        /* Top Up: DIR HIGH */
        motor_move_to_angle(0, IMU_MUX_CH_TOP, target,
                            true, true, MOTOR_TIMEOUT_MS);
    } else {
        /* Top Down: DIR LOW */
        motor_move_to_angle(0, IMU_MUX_CH_TOP, target,
                            false, false, MOTOR_TIMEOUT_MS);
    }
}

static void execute_bottom_move(float target)
{
    float pitch = 0;
    imu_read_pitch(IMU_MUX_CH_BOTTOM, &pitch);
    ESP_LOGI(TAG, "BOTTOM current pitch=%.1f° target=%.1f°", pitch, target);

    if (pitch < target) {
        /* Bottom Up: DIR LOW */
        motor_move_to_angle(1, IMU_MUX_CH_BOTTOM, target,
                            true, false, MOTOR_TIMEOUT_MS);
    } else {
        /* Bottom Down: DIR HIGH */
        motor_move_to_angle(1, IMU_MUX_CH_BOTTOM, target,
                            false, true, MOTOR_TIMEOUT_MS);
    }
}

static void execute_side_move(float target, int axis)
{
    float pitch = 0;
    imu_read_pitch(IMU_MUX_CH_SIDES, &pitch);
    ESP_LOGI(TAG, "SIDE (%s) current pitch=%.1f° target=%.1f°",
             (axis == 1) ? "RIGHT" : "LEFT", pitch, target);

    if (axis == 1) {  /* RIGHT */
        if (pitch < target) {
            motor_move_to_angle(2, IMU_MUX_CH_SIDES, target,
                                true, true, MOTOR_TIMEOUT_MS);
        } else {
            motor_move_to_angle(2, IMU_MUX_CH_SIDES, -target,
                                false, false, MOTOR_TIMEOUT_MS);
        }
    } else {  /* LEFT */
        if (pitch < target) {
            motor_move_to_angle(2, IMU_MUX_CH_SIDES, target,
                                true, true, MOTOR_TIMEOUT_MS);
        } else {
            motor_move_to_angle(2, IMU_MUX_CH_SIDES, -target,
                                false, false, MOTOR_TIMEOUT_MS);
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════
 *  Blink command handler  (called from the UDP task)
 *
 *  "0" = short blink → CYCLE selection
 *  "1" = long  blink → CONFIRM selection
 * ═══════════════════════════════════════════════════════════════════ */

static void blink_handler(const char *cmd)
{
    /* Blink LED for visual feedback */
    if (strcmp(cmd, "0") == 0) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(LED_PIN, 0);
    } else if (strcmp(cmd, "1") == 0) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(800));
        gpio_set_level(LED_PIN, 0);
    }

    /* ── CYCLE ── */
    if (strcmp(cmd, "0") == 0) {
        if (current_page == 0) {
            sel_page0 = (sel_page0 + 1) % MENU_PAGE0_COUNT;
            ESP_LOGI(TAG, "Page0 → %s (idx %d)", axis_names[sel_page0], sel_page0);
            nextion_update_page0_buttons();
        } else {
            sel_page1 = (sel_page1 + 1) % MENU_PAGE1_COUNT;
            ESP_LOGI(TAG, "Page1 → idx %d", sel_page1);
            nextion_update_page1_buttons();
        }
        return;
    }

    /* ── CONFIRM ── */
    if (strcmp(cmd, "1") == 0) {
        if (current_page == 0) {
            /* Axis confirmed → go to angle page */
            ESP_LOGI(TAG, "Confirmed axis: %s", axis_names[sel_page0]);
            nextion_send_command("page 1");
            vTaskDelay(pdMS_TO_TICKS(50));
            current_page = 1;
            sel_page1 = 0;
            nextion_update_page1_buttons();
            return;
        }

        /* current_page == 1 */
        ESP_LOGI(TAG, "Confirmed page1 selection: idx %d", sel_page1);

        if (sel_page1 == 4) {
            /* BACK selected → return to axis page */
            nextion_send_command("page 0");
            vTaskDelay(pdMS_TO_TICKS(50));
            current_page = 0;
            sel_page0 = 0;
            nextion_update_page0_buttons();
            return;
        }

        /* An angle was selected – move the motor */
        float target = angle_values[sel_page1];
        int axis = sel_page0;

        ESP_LOGI(TAG, "Moving %s to %.0f°", axis_names[axis], target);

        switch (axis) {
        case 0: execute_top_move(target);             break;
        case 2: execute_bottom_move(target);          break;
        case 1: /* fall-through */
        case 3: execute_side_move(target, axis);      break;
        default:
            ESP_LOGW(TAG, "Invalid axis index %d", axis);
            break;
        }

        nextion_send_command("t1.txt=\"Done\"");
    }
}

/* ═══════════════════════════════════════════════════════════════════
 *  app_main
 * ═══════════════════════════════════════════════════════════════════ */

void app_main(void)
{
    ESP_LOGI(TAG, "======== BED ESP-IDF starting ========");

    /* ── I2C + IMU ── */
    ESP_ERROR_CHECK(imu_i2c_init());

    imu_top_ok    = (imu_mpu6050_init(IMU_MUX_CH_TOP)    == ESP_OK);
    imu_bottom_ok = (imu_mpu6050_init(IMU_MUX_CH_BOTTOM) == ESP_OK);
    imu_sides_ok  = (imu_mpu6050_init(IMU_MUX_CH_SIDES)  == ESP_OK);

    ESP_LOGI(TAG, "IMU status: TOP=%s  BOTTOM=%s  SIDES=%s",
             imu_top_ok    ? "OK" : "FAIL",
             imu_bottom_ok ? "OK" : "FAIL",
             imu_sides_ok  ? "OK" : "FAIL");

    /* ── Motors + LED ── */
    ESP_ERROR_CHECK(motor_init());
    motor_stop_all();

    /* ── Nextion display ── */
    nextion_init();
    nextion_send_command("page 0");
    vTaskDelay(pdMS_TO_TICKS(50));
    current_page = 0;
    sel_page0 = 0;
    nextion_update_page0_buttons();

    /* ── WiFi (blocks until connected) ── */
    wifi_init_sta();

    /* ── Start UDP listener → calls blink_handler() on each packet ── */
    udp_server_start(blink_handler);

    ESP_LOGI(TAG, "======== BED ESP-IDF ready ========");

    /* app_main can return; FreeRTOS keeps running the UDP task */
}
