/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : tlc59116.h
   * @brief          : TLC59116 16-channel LED driver interface (soft I2C)
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef DRIVERS_TLC59116_H
#define DRIVERS_TLC59116_H

#include <stdbool.h>
#include <stdint.h>
#define TLC_ADDR_7B 0x60U

bool TLC59116_InitSoft(void);
bool TLC59116_SetPWM(uint8_t ch, uint8_t val);
bool TLC59116_SetPWM12(const uint8_t pwm12[12]);
bool TLC59116_SetPWM12_Single(const uint8_t pwm12[12]);
bool TLC59116_SetAllPWM(uint8_t pwm);
bool TLC59116_SetAllPWMMode(void);
bool TLC59116_SetAllOn(void);
bool TLC59116_SetAllOff(void);

#endif /* DRIVERS_TLC59116_H */
