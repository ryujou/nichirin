/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : app.c
  * @brief          : Simplified multi-mode effects (1..8) + encoder param adjust
  ******************************************************************************
  */
/* USER CODE END Header */
#include "app/app.h"

#include "storage/cfg_store.h"
#include "drivers/encoder.h"
#include "drivers/tlc59116.h"
#include "stm32g0xx_hal.h"

#define LED_COUNT 12U
#define MODE_MIN 1U
#define MODE_MAX 8U

#define FLOW_MIN_MS 10U
#define FLOW_MAX_MS 300U
#define FLASH_MIN_MS 50U
#define FLASH_MAX_MS 1000U
#define BREATH_MIN_MS 200U
#define BREATH_MAX_MS 2000U

#define FULL_BRIGHT_DERATE_THRESHOLD 200U
#define FULL_BRIGHT_DERATE_LEVEL 128U
#define FULL_BRIGHT_DERATE_MS 60000U
#define FULL_BRIGHT_DERATE_TICKS (FULL_BRIGHT_DERATE_MS / EFFECT_TICK_MS)

#define ENV_DELTA 4U
#define BREATH_STEPS 256U
#define BREATH_CYCLE (BREATH_STEPS * 2U)

static uint8_t mode = MODE_MIN;
static uint8_t pos = 0U;
static uint8_t group = 0U;
static uint16_t step_accum_ms = 0U;
static uint16_t flash_accum_ms = 0U;
static uint8_t flash_on = 0U;
static uint16_t breath_phase = 0U;
static uint32_t breath_phase_accum = 0U;
static uint8_t env = 0U;
static uint8_t breath_lut_cur = 0U;
static uint8_t save_flash_remain = 0U;
static uint8_t save_flash_phase = 0U;
static uint16_t full_bright_derate_ticks = 0U;
static uint8_t full_on_derated = 0U;

static uint8_t param_level[10] = {0};
static Config current_cfg;

static void Apply_CmdToDutyNext(const uint8_t cmd[LED_COUNT])
{
  (void)TLC59116_SetPWM12_Single(cmd);
}

static uint16_t Map_Param_To_Ms(uint8_t param, uint16_t min_ms, uint16_t max_ms)
{
  if (max_ms <= min_ms)
  {
    return min_ms;
  }
  {
    uint32_t range = (uint32_t)(max_ms - min_ms);
    uint32_t scaled = ((uint32_t)(255U - param) * range + 127U) / 255U;
    return (uint16_t)(min_ms + scaled);
  }
}

static uint8_t Breath_Triangle(uint16_t phase)
{
  uint16_t tri = (phase < BREATH_STEPS) ? phase : (uint16_t)(BREATH_CYCLE - 1U - phase);
  return (uint8_t)tri;
}

void App_Init(void)
{
  Config cfg;
  if (!CfgStore_Load(&cfg))
  {
    Cfg_ResetToDefault(&cfg);
    (void)Cfg_SaveAtomic(&cfg);
  }

  current_cfg = cfg;
  Apply_Config(&cfg);
}

void Apply_Config(const Config *cfg)
{
  if (cfg == NULL)
  {
    return;
  }

  mode = cfg->mode;
  if ((mode < MODE_MIN) || (mode > MODE_MAX))
  {
    mode = MODE_MIN;
  }

  param_level[0] = 0U;
  for (uint8_t i = MODE_MIN; i <= MODE_MAX; i++)
  {
    param_level[i] = cfg->param_level[i];
  }

  pos = 0U;
  group = 0U;
  step_accum_ms = 0U;
  flash_accum_ms = 0U;
  flash_on = 0U;
  breath_phase = 0U;
  breath_phase_accum = 0U;
  env = 0U;
  breath_lut_cur = 0U;
  full_bright_derate_ticks = 0U;
  full_on_derated = 0U;

  current_cfg.mode = mode;
  for (uint8_t i = MODE_MIN; i <= MODE_MAX; i++)
  {
    current_cfg.param_level[i] = param_level[i];
  }
  current_cfg.param_level[0] = 0U;
}

void Effect_Tick(void)
{
  int16_t delta_sum = 0;
  int16_t d;

  do
  {
    d = Encoder_GetDeltaAccel();
    delta_sum = (int16_t)(delta_sum + d);
  } while (d != 0);

  if (delta_sum == 0)
  {
    int8_t d_raw;
    do
    {
      d_raw = Encoder_GetDelta();
      delta_sum = (int16_t)(delta_sum + d_raw);
    } while (d_raw != 0);
  }

  EncoderEvent ev = Encoder_GetKeyEvent();
  if (ev == ENC_EVENT_CLICK)
  {
    mode++;
    if (mode > MODE_MAX)
    {
      mode = MODE_MIN;
    }
    current_cfg.mode = mode;
  }

  if (delta_sum != 0)
  {
    int32_t next = (int32_t)param_level[mode] + delta_sum;
    if (next < 0)
    {
      next = 0;
    }
    if (next > 255)
    {
      next = 255;
    }
    param_level[mode] = (uint8_t)next;
    current_cfg.param_level[mode] = param_level[mode];
  }

  if ((ev == ENC_EVENT_LONGPRESS) || (ev == ENC_EVENT_SUPER_LONGPRESS))
  {
    uint8_t saved = 0U;
    uint8_t ok = 1U;
    __disable_irq();
    saved = (uint8_t)Cfg_SaveAtomic(&current_cfg);
    __enable_irq();
    if (saved != 0U)
    {
      Config verify;
      if (!Cfg_Load(&verify))
      {
        ok = 0U;
      }
      else
      {
        if ((verify.mode != current_cfg.mode))
        {
          ok = 0U;
        }
        else
        {
          for (uint8_t i = MODE_MIN; i <= MODE_MAX; i++)
          {
            if (verify.param_level[i] != current_cfg.param_level[i])
            {
              ok = 0U;
              break;
            }
          }
        }
      }

      if (ok != 0U)
      {
        CfgStore_OnCommit();
        save_flash_remain = 1U;
        save_flash_phase = 0U;
      }
      else
      {
        save_flash_remain = 3U;
        save_flash_phase = 0U;
      }
    }
    else
    {
      save_flash_remain = 3U;
      save_flash_phase = 0U;
    }
  }

  /* INSERT: full-bright derate logic (mode 7 only, continuous >200 for 10s). */
  if ((mode == 7U) && (param_level[7] > FULL_BRIGHT_DERATE_THRESHOLD))
  {
    if (full_on_derated == 0U)
    {
      if (full_bright_derate_ticks < FULL_BRIGHT_DERATE_TICKS)
      {
        full_bright_derate_ticks++;
      }
      if (full_bright_derate_ticks >= FULL_BRIGHT_DERATE_TICKS)
      {
        param_level[7] = FULL_BRIGHT_DERATE_LEVEL;
        current_cfg.param_level[7] = param_level[7];
        full_on_derated = 1U;
      }
    }
  }
  else
  {
    full_bright_derate_ticks = 0U;
    full_on_derated = 0U;
  }

  {
    uint16_t step_ms = FLOW_MAX_MS;
    uint16_t flash_ms = FLASH_MAX_MS;
    uint16_t breath_ms = BREATH_MAX_MS;

    if ((mode >= 1U) && (mode <= 5U))
    {
      step_ms = Map_Param_To_Ms(param_level[mode], FLOW_MIN_MS, FLOW_MAX_MS);
    }
    if (mode == 6U)
    {
      flash_ms = Map_Param_To_Ms(param_level[6], FLASH_MIN_MS, FLASH_MAX_MS);
    }
    if (mode == 8U)
    {
      breath_ms = Map_Param_To_Ms(param_level[8], BREATH_MIN_MS, BREATH_MAX_MS);
    }

    if (mode == 8U)
    {
      uint32_t delta_q8 = ((uint32_t)BREATH_CYCLE << 8) * EFFECT_TICK_MS / breath_ms;
      breath_phase_accum += delta_q8;
      if (breath_phase_accum >= ((uint32_t)BREATH_CYCLE << 8))
      {
        breath_phase_accum -= ((uint32_t)BREATH_CYCLE << 8);
      }
      breath_phase = (uint16_t)(breath_phase_accum >> 8);
    }
    else
    {
      breath_phase = (uint16_t)(breath_phase + ENV_DELTA);
      if (breath_phase >= BREATH_CYCLE)
      {
        breath_phase = (uint16_t)(breath_phase - BREATH_CYCLE);
      }
    }

    env = Breath_Triangle(breath_phase);
    breath_lut_cur = env;

    step_accum_ms = (uint16_t)(step_accum_ms + EFFECT_TICK_MS);
    if (step_accum_ms >= step_ms)
    {
      step_accum_ms = (uint16_t)(step_accum_ms - step_ms);
      switch (mode)
      {
        case 1U:
        case 4U:
          pos = (uint8_t)((pos + 1U) % LED_COUNT);
          break;
        case 2U:
        case 5U:
          pos = (uint8_t)((pos + LED_COUNT - 1U) % LED_COUNT);
          break;
        case 3U:
          group = (uint8_t)((group + 1U) % 6U);
          break;
        default:
          break;
      }
    }

    flash_accum_ms = (uint16_t)(flash_accum_ms + EFFECT_TICK_MS);
    if (flash_accum_ms >= flash_ms)
    {
      flash_accum_ms = (uint16_t)(flash_accum_ms - flash_ms);
      flash_on = (uint8_t)!flash_on;
    }
  }

  {
    uint8_t cmd[LED_COUNT] = {0};

    if (save_flash_remain > 0U)
    {
      if (save_flash_phase == 0U)
      {
        save_flash_phase = 1U;
        for (uint8_t i = 0; i < LED_COUNT; i++)
        {
          cmd[i] = 255U;
        }
        Apply_CmdToDutyNext(cmd);
        return;
      }
      save_flash_phase = 0U;
      save_flash_remain--;
    }

    switch (mode)
    {
      case 1U:
      case 2U:
        cmd[pos] = 255U;
        break;
      case 3U:
        {
          static const uint8_t pairs[6][2] = {
            {5U, 6U}, {4U, 7U}, {3U, 8U}, {2U, 9U}, {1U, 10U}, {0U, 11U}
          };
          uint8_t idx = (uint8_t)(group % 6U);
          cmd[pairs[idx][0]] = 255U;
          cmd[pairs[idx][1]] = 255U;
        }
        break;
      case 4U:
      case 5U:
        {
          uint8_t env_breath = env;
          if (env_breath < 100U)
          {
            env_breath = 100U;
          }
          for (uint8_t i = 0; i < LED_COUNT; i++)
          {
            uint8_t dist = (i > pos) ? (uint8_t)(i - pos) : (uint8_t)(pos - i);
            if (dist > (LED_COUNT / 2U))
            {
              dist = (uint8_t)(LED_COUNT - dist);
            }
            uint8_t base = 0U;
            if (dist == 0U)
            {
              base = 255U;
            }
            else if (dist == 1U)
            {
              base = 220U;
            }
            else if (dist == 2U)
            {
              base = 128U;
            }
            cmd[i] = (uint8_t)(((uint16_t)base * env_breath) / 255U);
          }
        }
        break;
      case 6U:
        if (flash_on != 0U)
        {
          for (uint8_t i = 0; i < LED_COUNT; i++)
          {
            cmd[i] = 255U;
          }
        }
        break;
      case 8U:
        for (uint8_t i = 0; i < LED_COUNT; i++)
        {
          cmd[i] = breath_lut_cur;
        }
        break;
      case 7U:
        for (uint8_t i = 0; i < LED_COUNT; i++)
        {
          cmd[i] = param_level[7];
        }
        break;
      default:
        break;
    }

    Apply_CmdToDutyNext(cmd);
  }

}
