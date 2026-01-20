/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : encoder.h
  * @brief          : Rotary encoder sampling and key event interface
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef DRIVERS_ENCODER_H
#define DRIVERS_ENCODER_H

#include <stdint.h>
#include "stm32g0xx_hal.h"

typedef enum
{
  ENC_EVENT_NONE = 0,
  ENC_EVENT_CLICK,
  ENC_EVENT_DOUBLE_CLICK,
  ENC_EVENT_TRIPLE_CLICK,
  ENC_EVENT_LONGPRESS,
  ENC_EVENT_SUPER_LONGPRESS
} EncoderEvent;

void Encoder_Init(GPIO_TypeDef *a_port, uint16_t a_pin,
                  GPIO_TypeDef *b_port, uint16_t b_pin,
                  GPIO_TypeDef *k_port, uint16_t k_pin);
void Encoder_1msTick(void);
int8_t Encoder_GetDelta(void);
int16_t Encoder_GetDeltaAccel(void);
EncoderEvent Encoder_GetKeyEvent(void);

#endif /* DRIVERS_ENCODER_H */
