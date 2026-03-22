/*
 * nextion_uart.c - Nextion display communication over UART2
 *
 * Sends commands terminated with three 0xFF bytes, matching the
 * Nextion serial protocol.
 */

#include "nextion_uart.h"
#include "config.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "NEXTION";

void nextion_init(void)
{
    uart_config_t uart_config = {
        .baud_rate  = NEXTION_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_param_config(NEXTION_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(NEXTION_UART_NUM,
                                 NEXTION_TX_PIN, NEXTION_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(NEXTION_UART_NUM,
                                        NEXTION_BUF_SIZE, 0, 0, NULL, 0));

    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Nextion UART%d initialized (TX=%d RX=%d %d baud)",
             NEXTION_UART_NUM, NEXTION_TX_PIN, NEXTION_RX_PIN, NEXTION_BAUD);
}

void nextion_send_command(const char *cmd)
{
    /* Send the command string */
    uart_write_bytes(NEXTION_UART_NUM, cmd, strlen(cmd));
    /* Three 0xFF termination bytes (Nextion protocol) */
    const uint8_t term[3] = { 0xFF, 0xFF, 0xFF };
    uart_write_bytes(NEXTION_UART_NUM, (const char *)term, 3);
    uart_wait_tx_done(NEXTION_UART_NUM, pdMS_TO_TICKS(100));
}
