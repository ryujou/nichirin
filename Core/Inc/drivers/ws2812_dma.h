/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : ws2812_dma.h
  * @brief          : WS2812B driver using TIM1 PWM + DMA (auto refresh)
  ******************************************************************************
  * Notes:
  * - PWM clock 64MHz, ARR=79 -> 800kHz bit rate (1.25us per bit)
  * - DMA Normal mode (one frame per transfer)
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef DRIVERS_WS2812_DMA_H
#define DRIVERS_WS2812_DMA_H

#include "stm32g0xx_hal.h"
#include <stdint.h>

/* Tunable constants */
#define WS2812_LED_COUNT 60U
#define WS2812_RESET_SLOTS 64U /* 64 * 1.25us = 80us */
#define WS2812_TARGET_FPS 200U
#define WS2812_CCR_0 22U
#define WS2812_CCR_1 45U
#define WS2812_ARR 79U

void ws2812_init(TIM_HandleTypeDef *htim, uint32_t channel);
void ws2812_start(void);
void ws2812_stop(void);

void ws2812_set_brightness(uint8_t br);
uint8_t ws2812_is_busy(void);
uint16_t *ws2812_pwm_buf(void);
uint32_t ws2812_pwm_len(void);
void ws2812_start_frame(void);

#endif /* DRIVERS_WS2812_DMA_H */
