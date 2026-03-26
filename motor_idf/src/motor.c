#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Motor 1 (Top) */
#define MOTOR1_PWM_PIN   GPIO_NUM_27
#define MOTOR1_DIR_PIN   GPIO_NUM_14
/* Motor 2 (Bottom) */
#define MOTOR2_PWM_PIN   GPIO_NUM_25
#define MOTOR2_DIR_PIN   GPIO_NUM_26
/* Motor 3 (Side) */
#define MOTOR3_PWM_PIN   GPIO_NUM_5
#define MOTOR3_DIR_PIN   GPIO_NUM_4

#define MOTOR_COUNT              3
#define MOTOR_PWM_FREQ_HZ        5000
#define MOTOR_PWM_RESOLUTION     LEDC_TIMER_8_BIT
#define MOTOR_PWM_MAX_DUTY       255
#define MOTOR_LEDC_TIMER         LEDC_TIMER_0
#define MOTOR_LEDC_MODE          LEDC_LOW_SPEED_MODE
#define MOTOR_TEST_STEP_MS       3000

#define CMD_UART_PORT            UART_NUM_0
#define CMD_UART_BAUDRATE        115200
#define CMD_UART_RX_BUF_SIZE     2048
#define CMD_LINE_MAX_LEN         96

typedef struct {
	gpio_num_t pwm_pin;
	gpio_num_t dir_pin;
	ledc_channel_t ledc_channel;
	int speed;
} motor_t;

static const char *TAG = "MOTOR_APP";

static motor_t s_motors[MOTOR_COUNT] = {
	{
		.pwm_pin = MOTOR1_PWM_PIN,
		.dir_pin = MOTOR1_DIR_PIN,
		.ledc_channel = LEDC_CHANNEL_0,
		.speed = 0,
	},
	{
		.pwm_pin = MOTOR2_PWM_PIN,
		.dir_pin = MOTOR2_DIR_PIN,
		.ledc_channel = LEDC_CHANNEL_1,
		.speed = 0,
	},
	{
		.pwm_pin = MOTOR3_PWM_PIN,
		.dir_pin = MOTOR3_DIR_PIN,
		.ledc_channel = LEDC_CHANNEL_2,
		.speed = 0,
	},
};

static int clamp_speed(int speed)
{
	if (speed > MOTOR_PWM_MAX_DUTY) {
		return MOTOR_PWM_MAX_DUTY;
	}
	if (speed < -MOTOR_PWM_MAX_DUTY) {
		return -MOTOR_PWM_MAX_DUTY;
	}
	return speed;
}

static void motor_set_speed(int motor_index, int speed)
{
	if (motor_index < 0 || motor_index >= MOTOR_COUNT) {
		ESP_LOGW(TAG, "Invalid motor index: %d", motor_index);
		return;
	}

	motor_t *motor = &s_motors[motor_index];
	int clamped_speed = clamp_speed(speed);
	uint32_t duty = (uint32_t)abs(clamped_speed);
	int dir_level = (clamped_speed >= 0) ? 1 : 0;

	gpio_set_level(motor->dir_pin, dir_level);
	ESP_ERROR_CHECK(ledc_set_duty(MOTOR_LEDC_MODE, motor->ledc_channel, duty));
	ESP_ERROR_CHECK(ledc_update_duty(MOTOR_LEDC_MODE, motor->ledc_channel));

	motor->speed = clamped_speed;

	ESP_LOGI(TAG,
			 "Motor %d -> speed=%d duty=%lu dir=%d",
			 motor_index + 1,
			 clamped_speed,
			 (unsigned long)duty,
			 dir_level);
}

static void motor_stop_all(void)
{
	for (int i = 0; i < MOTOR_COUNT; i++) {
		motor_set_speed(i, 0);
	}
}

static esp_err_t motors_init(void)
{
	ledc_timer_config_t timer_cfg = {
		.speed_mode = MOTOR_LEDC_MODE,
		.timer_num = MOTOR_LEDC_TIMER,
		.duty_resolution = MOTOR_PWM_RESOLUTION,
		.freq_hz = MOTOR_PWM_FREQ_HZ,
		.clk_cfg = LEDC_AUTO_CLK,
	};
	ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

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
		ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));

		gpio_config_t dir_pin_cfg = {
			.pin_bit_mask = (1ULL << s_motors[i].dir_pin),
			.mode = GPIO_MODE_OUTPUT,
			.pull_up_en = GPIO_PULLUP_DISABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.intr_type = GPIO_INTR_DISABLE,
		};
		ESP_ERROR_CHECK(gpio_config(&dir_pin_cfg));

		gpio_set_level(s_motors[i].dir_pin, 0);
	}

	return ESP_OK;
}

static void print_help(void)
{
	printf("\n--- 3 Motor Control Commands ---\n");
	printf("help                 : Show this help\n");
	printf("status               : Show all motor speeds\n");
	printf("stop                 : Stop all motors\n");
	printf("m <id> <speed>       : Set one motor (id=1..3, speed=-255..255)\n");
	printf("all <s1> <s2> <s3>   : Set all motor speeds (-255..255)\n");
	printf("\nSpeed sign controls direction: positive=DIR HIGH, negative=DIR LOW\n\n");
}

static void print_status(void)
{
	printf("Motor states:\n");
	for (int i = 0; i < MOTOR_COUNT; i++) {
		printf("  M%d: speed=%d pwm_pin=%d dir_pin=%d\n",
			   i + 1,
			   s_motors[i].speed,
			   s_motors[i].pwm_pin,
			   s_motors[i].dir_pin);
	}
}

static void handle_command(char *line)
{
	int id = 0;
	int speed = 0;
	int s1 = 0;
	int s2 = 0;
	int s3 = 0;

	if (strcmp(line, "help") == 0) {
		print_help();
		return;
	}

	if (strcmp(line, "status") == 0) {
		print_status();
		return;
	}

	if (strcmp(line, "stop") == 0) {
		motor_stop_all();
		printf("All motors stopped.\n");
		return;
	}

	if (sscanf(line, "m %d %d", &id, &speed) == 2) {
		if (id < 1 || id > MOTOR_COUNT) {
			printf("Invalid motor id. Use 1..3.\n");
			return;
		}
		motor_set_speed(id - 1, speed);
		return;
	}

	if (sscanf(line, "all %d %d %d", &s1, &s2, &s3) == 3) {
		motor_set_speed(0, s1);
		motor_set_speed(1, s2);
		motor_set_speed(2, s3);
		return;
	}

	printf("Unknown command: %s\n", line);
	printf("Type 'help' for usage.\n");
}

static esp_err_t command_uart_init(void)
{
	uart_config_t uart_cfg = {
		.baud_rate = CMD_UART_BAUDRATE,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};

	esp_err_t err = uart_driver_install(CMD_UART_PORT, CMD_UART_RX_BUF_SIZE, 0, 0, NULL, 0);
	if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
		return err;
	}

	ESP_ERROR_CHECK(uart_param_config(CMD_UART_PORT, &uart_cfg));
	return ESP_OK;
}

static void command_task(void *arg)
{
	(void)arg;
	uint8_t rx_char = 0;
	char cmd_line[CMD_LINE_MAX_LEN];
	size_t pos = 0;

	printf("\n3-motor controller ready. Type 'help'.\n> ");
	fflush(stdout);

	while (1) {
		int read = uart_read_bytes(CMD_UART_PORT, &rx_char, 1, pdMS_TO_TICKS(100));
		if (read <= 0) {
			continue;
		}

		if (rx_char == '\r' || rx_char == '\n') {
			if (pos == 0) {
				continue;
			}

			cmd_line[pos] = '\0';
			printf("\n");
			handle_command(cmd_line);
			pos = 0;
			printf("> ");
			fflush(stdout);
			continue;
		}

		if (pos < (CMD_LINE_MAX_LEN - 1)) {
			cmd_line[pos++] = (char)rx_char;
		} else {
			cmd_line[CMD_LINE_MAX_LEN - 1] = '\0';
			printf("\nCommand too long, max %d chars\n> ", CMD_LINE_MAX_LEN - 1);
			pos = 0;
			fflush(stdout);
		}
	}
}

static void motor_test_task(void *arg)
{
	(void)arg;

	ESP_LOGI(TAG, "Starting repeating motor direction test sequence");

	while (1) {
		for (int i = 0; i < MOTOR_COUNT; i++) {
			ESP_LOGI(TAG, "M%d forward for %d ms", i + 1, MOTOR_TEST_STEP_MS);
			motor_set_speed(i, MOTOR_PWM_MAX_DUTY);
			vTaskDelay(pdMS_TO_TICKS(MOTOR_TEST_STEP_MS));

			ESP_LOGI(TAG, "M%d reverse for %d ms", i + 1, MOTOR_TEST_STEP_MS);
			motor_set_speed(i, -MOTOR_PWM_MAX_DUTY);
			vTaskDelay(pdMS_TO_TICKS(MOTOR_TEST_STEP_MS));

			motor_set_speed(i, 0);
		}

		motor_stop_all();
		ESP_LOGI(TAG, "Motor test cycle complete. Restarting.");
	}
}

void app_main(void)
{
	ESP_LOGI(TAG, "Starting 3-motor test app");

	ESP_ERROR_CHECK(motors_init());
	motor_stop_all();

	xTaskCreate(motor_test_task, "motor_test_task", 4096, NULL, 5, NULL);
}