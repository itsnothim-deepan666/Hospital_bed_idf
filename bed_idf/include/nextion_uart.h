/*
 * nextion_uart.h - Nextion display UART communication
 */

#ifndef NEXTION_UART_H
#define NEXTION_UART_H

/**
 * @brief Initialize UART2 for the Nextion display (9600 baud, pins from config.h)
 */
void nextion_init(void);

/**
 * @brief Send a raw command string to the Nextion display.
 *        Automatically appends the three 0xFF termination bytes.
 */
void nextion_send_command(const char *cmd);

#endif /* NEXTION_UART_H */
