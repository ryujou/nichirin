/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : app.h
  * @brief          : 模式/参数/效果的应用接口
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

#endif /* APP_APP_H */
