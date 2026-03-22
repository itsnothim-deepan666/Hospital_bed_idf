/*
 * wifi_udp.h - WiFi STA connection and UDP socket server
 */

#ifndef WIFI_UDP_H
#define WIFI_UDP_H

#include "esp_err.h"

/**
 * @brief Initialize WiFi in STA mode and connect to the configured AP.
 *        Blocks until connected or max retries exceeded.
 */
void wifi_init_sta(void);

/**
 * @brief Start the UDP listener task (FreeRTOS task).
 *        Receives blink commands and forwards them to the menu handler.
 *
 * @param handler  Callback function: void handler(const char *cmd)
 *                 called for each received UDP packet.
 */
void udp_server_start(void (*handler)(const char *cmd));

#endif /* WIFI_UDP_H */
