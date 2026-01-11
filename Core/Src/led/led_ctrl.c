#include "led/led_ctrl.h"
#include "led/led_effects.h"
#include "led/led_engine.h"
#include "storage/config_store.h"
#include "main.h"
#include <string.h>

#define LED_CTRL_TICK_MS 10U
#define LED_CTRL_SAVE_DELAY_MS 500U

extern TIM_HandleTypeDef htim6;

static uint8_t s_mode = 1U;
static uint8_t s_levels[10] = {0};
static uint8_t s_dirty = 0U;
static uint8_t s_save_pending = 0U;
static uint16_t s_save_delay_ms = 0U;
static uint8_t s_timer_running = 0U;
static uint8_t s_output_enabled = 1U;

static uint8_t LedCtrl_ClampMode(uint8_t mode)
{
  if ((mode < 1U) || (mode > 9U))
  {
    return 1U;
  }
  return mode;
}

static uint8_t LedCtrl_ClampLevel(uint8_t level)
{
  if ((level < 1U) || (level > 7U))
  {
    return 4U;
  }
  return level;
}

static void LedCtrl_FillConfig(Config *cfg)
{
  memset(cfg, 0, sizeof(*cfg));
  cfg->mode = s_mode;
  cfg->run_state = 0U;
  for (uint8_t i = 1U; i <= 9U; i++)
  {
    cfg->param_level[i] = s_levels[i];
  }
}

static void LedCtrl_ApplyConfig(const Config *cfg)
{
  s_mode = LedCtrl_ClampMode(cfg->mode);
  s_levels[0] = 0U;
  for (uint8_t i = 1U; i <= 9U; i++)
  {
    s_levels[i] = LedCtrl_ClampLevel(cfg->param_level[i]);
  }

  LedEffects_Init();
  LedEffects_SetMode(s_mode);
  for (uint8_t i = 1U; i <= 9U; i++)
  {
    LedEffects_SetLevel(i, s_levels[i]);
  }
}

void LedCtrl_Init(void)
{
  LedEngine_Init();

  Config cfg;
  if (!ConfigStore_Load(&cfg))
  {
    ConfigStore_InitDefaults(&cfg);
    (void)ConfigStore_Save(&cfg);
  }

  LedCtrl_ApplyConfig(&cfg);

  s_dirty = 0U;
  s_save_pending = 0U;
  s_save_delay_ms = 0U;
  s_output_enabled = 1U;

  {
    uint8_t cmd[LED_COUNT];
    LedEffects_Tick10ms(cmd);
    LedEngine_UpdateCmd(cmd);
  }
}

void LedCtrl_Start(void)
{
  (void)HAL_TIM_Base_Start_IT(&htim6);
  s_timer_running = 1U;
}

void LedCtrl_10msTick(void)
{
  uint8_t cmd[LED_COUNT];
  LedEffects_Tick10ms(cmd);
  if (s_output_enabled != 0U)
  {
    LedEngine_UpdateCmd(cmd);
  }

  if (s_save_pending != 0U)
  {
    s_save_delay_ms += LED_CTRL_TICK_MS;
    if (s_save_delay_ms >= LED_CTRL_SAVE_DELAY_MS)
    {
      LedCtrl_SaveNow();
    }
  }
}

void LedCtrl_SetMode(uint8_t mode_1_9)
{
  uint8_t mode = LedCtrl_ClampMode(mode_1_9);
  if (s_mode != mode)
  {
    s_mode = mode;
    LedEffects_SetMode(mode);
    s_dirty = 1U;
  }
}

uint8_t LedCtrl_GetMode(void)
{
  return s_mode;
}

void LedCtrl_SetLevel(uint8_t mode_1_9, uint8_t level_1_7)
{
  uint8_t mode = LedCtrl_ClampMode(mode_1_9);
  uint8_t level = LedCtrl_ClampLevel(level_1_7);
  if (s_levels[mode] != level)
  {
    s_levels[mode] = level;
    LedEffects_SetLevel(mode, level);
    s_dirty = 1U;
  }
}

uint8_t LedCtrl_GetLevel(uint8_t mode_1_9)
{
  uint8_t mode = LedCtrl_ClampMode(mode_1_9);
  uint8_t level = s_levels[mode];
  return LedCtrl_ClampLevel(level);
}

void LedCtrl_RequestSave(void)
{
  s_save_pending = 1U;
  s_save_delay_ms = 0U;
}

void LedCtrl_SaveNow(void)
{
  Config cfg;
  LedCtrl_FillConfig(&cfg);

  if (s_timer_running != 0U)
  {
    (void)HAL_TIM_Base_Stop_IT(&htim6);
  }
  bool ok = ConfigStore_Save(&cfg);
  if (s_timer_running != 0U)
  {
    (void)HAL_TIM_Base_Start_IT(&htim6);
  }

  if (ok)
  {
    s_dirty = 0U;
  }
  s_save_pending = 0U;
  s_save_delay_ms = 0U;
}

bool LedCtrl_IsDirty(void)
{
  return (s_dirty != 0U);
}

void LedCtrl_SetOutputEnabled(uint8_t enabled)
{
  s_output_enabled = (enabled != 0U) ? 1U : 0U;
}
