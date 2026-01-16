/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : tlc59116.h
  * @brief          : TLC59116 16 路 LED 驱动接口 (I2C)
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef DRIVERS_TLC59116_H
#define DRIVERS_TLC59116_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32g0xx_hal.h"

#define TLC59116_I2C_ADDR_7BIT 0x60U
#define TLC59116_I2C_ADDR (TLC59116_I2C_ADDR_7BIT << 1)
#define TLC59116_I2C_TIMEOUT_MS 10U

bool TLC59116_Init(I2C_HandleTypeDef *hi2c);
bool TLC59116_SetPWM(uint8_t ch, uint8_t val);
bool TLC59116_SetPWM12(const uint8_t pwm12[12]);
void TLC59116_AllOff(void);

#endif /* DRIVERS_TLC59116_H */
