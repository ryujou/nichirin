/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : flash_cfg.h
  * @brief          : Flash config log storage interface
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef FLASH_CFG_H
#define FLASH_CFG_H

#include <stdbool.h>
#include <stdint.h>

enum
{
  FLASH_CFG_FORMAT_TAG = 0xA5U
};

typedef struct
{
  uint8_t mode;
  uint8_t param_speed_idx;
  uint8_t param_level[10];
  uint16_t flash_period;
  uint8_t format_tag;
  uint8_t reserved[18];
} Config;

bool FlashCfg_Load(Config *out);
bool FlashCfg_Append(const Config *in);
void FlashCfg_InitDefaults(Config *out);

#endif /* FLASH_CFG_H */
