/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : app.h
  * @brief          : Application interface for modes, params, and effects
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef APP_APP_H
#define APP_APP_H

#include <stdint.h>
#include "storage/flash_cfg.h"

#define EFFECT_TICK_MS 10U

void App_Init(void);
void Apply_Config(const Config *cfg);
void Effect_Tick(void);
void App_SetSafeMode(uint8_t enable);
uint8_t App_IsSafeMode(void);

#endif /* APP_APP_H */
