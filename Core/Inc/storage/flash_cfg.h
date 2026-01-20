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

#define FLASH_CFG_VERSION 0x00010001UL

typedef struct
{
  uint8_t format_tag;      /* = FLASH_CFG_FORMAT_TAG */
  uint8_t mode;            /* 1,2,3,4,6 */
  uint8_t flow_speed;      /* 0..255 */
  uint8_t strobe_period;   /* 0..255 */
  uint8_t steady_bright;   /* 0..255 */
  uint16_t hue;            /* 0..359 */
  uint8_t sat;             /* 0..255 */
  uint8_t val;             /* 0..255 */
  uint8_t spectrum_gain;   /* 0..255 */
  uint8_t breath_speed;    /* 0..255 */
  uint8_t reserved0;
  uint32_t cfg_version;    /* = FLASH_CFG_VERSION */
  uint32_t created_by_fw;
  uint32_t last_migrated_by_fw;
  uint8_t reserved[36];
} Config;

_Static_assert(sizeof(Config) == 64U, "Config size must be 64 bytes");

bool FlashCfg_Load(Config *out);
bool FlashCfg_Append(const Config *in);
void FlashCfg_InitDefaults(Config *out);

#endif /* FLASH_CFG_H */
