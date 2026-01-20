/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : uart.h
  * @brief          : UART spectrum RX (USART1) interface
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef DRIVERS_UART_H
#define DRIVERS_UART_H

#include "stm32g0xx_hal.h"

void uart_init(UART_HandleTypeDef *uart1, UART_HandleTypeDef *uart2);
void uart_tick(void);
void uart_get_bands(uint8_t bands[12], uint32_t *last_ms, uint8_t *src_port);
uint32_t uart_get_rx_count(void);

#endif /* DRIVERS_UART_H */
