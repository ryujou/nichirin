/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : flash_cfg.c
  * @brief          : Compatibility wrapper for cfg_store
  ******************************************************************************
  */
/* USER CODE END Header */

#include "storage/flash_cfg.h"
#include "storage/cfg_store.h"

bool FlashCfg_Load(Config *out)
{
  return Cfg_Load(out);
}

bool FlashCfg_Append(const Config *in)
{
  return Cfg_SaveAtomic(in);
}

void FlashCfg_InitDefaults(Config *out)
{
  Cfg_ResetToDefault(out);
}
