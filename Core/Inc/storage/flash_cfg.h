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
  uint8_t param_layer;
  uint8_t lock_enabled;
  uint8_t global_brightness;
  uint8_t param_speed_idx;
  uint8_t param_level[10];
  uint16_t flash_period;
  uint8_t adv_gamma;
  uint8_t adv_breath_shape;
  uint8_t adv_phase_offset;
  uint8_t gamma_profile;
  uint8_t format_tag;
  uint8_t reserved[29];
  uint32_t cfg_version;
  uint32_t created_by_fw;
  uint32_t last_migrated_by_fw;
} Config;

bool FlashCfg_Load(Config *out);
bool FlashCfg_Append(const Config *in);
void FlashCfg_InitDefaults(Config *out);

#endif /* FLASH_CFG_H */
