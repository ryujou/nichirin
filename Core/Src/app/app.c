/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : app.c
  * @brief          : Application logic for modes, params, and effects
  ******************************************************************************
  */
/* USER CODE END Header */
#include "app/app.h"

#include "drivers/encoder.h"
#include "drivers/tlc59116.h"
#include "watchdog.h"
#include "storage/cfg_store.h"
#include <string.h>

#define LED_COUNT 12U
#define MODE_MIN 1U
#define MODE_MAX 9U
#define SAVE_DELAY_MS 500U
#define ENV_DELTA 4U
#define BREATH_STEPS 256U
#define BREATH_CYCLE (BREATH_STEPS * 2U)

#define STEP_MIN_MS 70U
#define STEP_MAX_MS 240U
#define FLASH_MIN_MS 50U
#define FLASH_MAX_MS 1000U
#define BREATH_MIN_MS 800U
#define BREATH_MAX_MS 6000U

#define PARAM_LAYER_BASIC 0U
#define PARAM_LAYER_ADV 1U

#define BASIC_ITEM_BRIGHTNESS 0U
#define BASIC_ITEM_SPEED 1U
#define BASIC_ITEM_COUNT 2U

#define ADV_ITEM_GAMMA 0U
#define ADV_ITEM_BREATH_SHAPE 1U
#define ADV_ITEM_FLASH_PERIOD 2U
#define ADV_ITEM_PHASE_OFFSET 3U
#define ADV_ITEM_COUNT 4U

#define FLASH_PERIOD_STEP_MS 10U

typedef enum
{
  RUN_STATE_RUN = 0,
  RUN_STATE_SETUP = 1
} RunState_t;

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
static uint8_t save_pending = 0U;
static uint16_t save_delay_ms = 0U;
static RunState_t run_state = RUN_STATE_RUN;
static uint8_t param_layer = PARAM_LAYER_BASIC;
static uint8_t setup_item = 0U;
static uint8_t locked = 0U;
static uint8_t global_brightness = 255U;
static uint8_t adv_gamma = 0U;
static uint8_t adv_breath_shape = 0U;
static uint8_t adv_phase_offset = 0U;
static uint8_t last_mode = MODE_MIN;

static uint8_t param_level[10] = {0};
static uint8_t param_speed_idx = 0U;
static uint16_t flash_period = 0U;

/* Function: Map_Param_To_Ms
 * Purpose: Map a 0..255 parameter to a time range (0 -> max_ms, 255 -> min_ms).
 * Inputs: param - 0..255 parameter; min_ms/max_ms - range bounds.
 * Outputs: Mapped time in milliseconds.
 */
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

/* Function: Clamp_U8
 * Purpose: Clamp a signed value to the 0..255 range.
 * Inputs: value - signed integer to clamp.
 * Outputs: uint8_t clamped to [0, 255].
 */
static uint8_t Clamp_U8(int32_t value)
{
  if (value < 0)
  {
    return 0U;
  }
  if (value > 255)
  {
    return 255U;
  }
  return (uint8_t)value;
}

/* Function: Clamp_U16
 * Purpose: Clamp a signed value to the 0..65535 range.
 * Inputs: value - signed integer to clamp.
 * Outputs: uint16_t clamped to [0, 65535].
 */
static uint16_t Clamp_U16(int32_t value)
{
  if (value < 0)
  {
    return 0U;
  }
  if (value > 65535)
  {
    return 65535U;
  }
  return (uint16_t)value;
}

/* Function: Breath_Triangle
 * Purpose: Generate a 0..255 triangle wave from a phase in [0, BREATH_CYCLE).
 * Inputs: phase - triangle wave phase.
 * Outputs: uint8_t triangle amplitude.
 */
static uint8_t Breath_Triangle(uint16_t phase)
{
  uint16_t tri = (phase < BREATH_STEPS) ? phase : (uint16_t)(BREATH_CYCLE - 1U - phase);
  return (uint8_t)tri;
}

/* Function: Apply_Curve
 * Purpose: Blend linear and quadratic curve based on shape (0..255).
 * Inputs: value - 0..255, shape - 0..255 (0=linear, 255=quadratic).
 * Outputs: Shaped 0..255 value.
 */
static uint8_t Apply_Curve(uint8_t value, uint8_t shape)
{
  uint16_t square = (uint16_t)(((uint32_t)value * (uint32_t)value + 255U) / 255U);
  uint16_t out = (uint16_t)(((uint32_t)(255U - shape) * value + (uint32_t)shape * square + 127U) / 255U);
  return (uint8_t)out;
}

/* Function: Apply_BrightnessGamma
 * Purpose: Apply global brightness and gamma shaping.
 * Inputs: value - 0..255 input.
 * Outputs: Brightness/gamma adjusted value.
 */
static uint8_t Apply_BrightnessGamma(uint8_t value)
{
  uint16_t scaled = (uint16_t)(((uint32_t)value * global_brightness + 127U) / 255U);
  return Apply_Curve((uint8_t)scaled, adv_gamma);
}

/* Function: Apply_CmdToDutyNext
 * Purpose: Send current 12-channel PWM command to the TLC59116.
 * Inputs: cmd - array of LED PWM values (0..255).
 * Outputs: None (I2C write side effects).
 */
static void Apply_CmdToDutyNext(const uint8_t cmd[LED_COUNT])
{
  (void)TLC59116_SetPWM12(cmd);
  WDG_TaskKick_Notify(WDG_TASK_TLC_UPDATE);
}

/* Function: Save_Config_Now
 * Purpose: Pack and append the current config to flash log.
 * Inputs: None (uses current state variables).
 * Outputs: None (flash write side effects).
 */
static void Save_Config_Now(void)
{
  Config cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.mode = mode;
  cfg.param_layer = param_layer;
  cfg.lock_enabled = locked;
  cfg.global_brightness = global_brightness;
  cfg.param_speed_idx = param_speed_idx;
  cfg.flash_period = flash_period;
  cfg.adv_gamma = adv_gamma;
  cfg.adv_breath_shape = adv_breath_shape;
  cfg.adv_phase_offset = adv_phase_offset;
  cfg.format_tag = FLASH_CFG_FORMAT_TAG;
  cfg.param_level[0] = 0U;
  for (uint8_t i = 1U; i <= MODE_MAX; i++)
  {
    cfg.param_level[i] = param_level[i];
  }

  __disable_irq();
  (void)Cfg_SaveAtomic(&cfg);
  __enable_irq();
  WDG_TaskKick_Notify(WDG_TASK_FLASH_SAVE);
}

/* Function: App_Init
 * Purpose: Initialize runtime state to defaults before loading flash config.
 * Inputs: None.
 * Outputs: None (initializes module state).
 */
void App_Init(void)
{
  mode = MODE_MIN;
  pos = 0U;
  group = 0U;
  step_accum_ms = 0U;
  flash_accum_ms = 0U;
  flash_on = 0U;
  breath_phase = 0U;
  breath_phase_accum = 0U;
  env = 0U;
  breath_lut_cur = 0U;
  save_pending = 0U;
  save_delay_ms = 0U;
  run_state = RUN_STATE_RUN;
  param_layer = PARAM_LAYER_BASIC;
  setup_item = 0U;
  locked = 0U;
  global_brightness = 255U;
  adv_gamma = 0U;
  adv_breath_shape = 0U;
  adv_phase_offset = 0U;
  last_mode = MODE_MIN;
  param_speed_idx = 0U;
  flash_period = 0U;
  for (uint8_t i = 1U; i <= MODE_MAX; i++)
  {
    param_level[i] = 128U;
  }
  param_level[0] = 0U;
}

/* Function: Apply_Config
 * Purpose: Apply persisted config values to runtime state.
 * Inputs: cfg - pointer to configuration loaded from flash.
 * Outputs: None (updates module state).
 */
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
  last_mode = mode;
  param_layer = (cfg->param_layer == PARAM_LAYER_ADV) ? PARAM_LAYER_ADV : PARAM_LAYER_BASIC;
  locked = (cfg->lock_enabled != 0U) ? 1U : 0U;
  global_brightness = cfg->global_brightness;
  param_speed_idx = cfg->param_speed_idx;
  for (uint8_t i = 1U; i <= MODE_MAX; i++)
  {
    uint8_t level = cfg->param_level[i];
    if (cfg->format_tag == FLASH_CFG_FORMAT_TAG)
    {
      param_level[i] = level;
    }
    else
    {
      if (level < 1U)
      {
        level = 1U;
      }
      if (level > 7U)
      {
        level = 7U;
      }
      param_level[i] = (uint8_t)(((uint32_t)(level - 1U) * 255U) / 6U);
    }
  }
  param_level[0] = 0U;

  if ((cfg->flash_period >= FLASH_MIN_MS) && (cfg->flash_period <= FLASH_MAX_MS))
  {
    flash_period = cfg->flash_period;
  }
  else
  {
    flash_period = Map_Param_To_Ms(param_level[7], FLASH_MIN_MS, FLASH_MAX_MS);
  }

  adv_gamma = cfg->adv_gamma;
  adv_breath_shape = cfg->adv_breath_shape;
  adv_phase_offset = cfg->adv_phase_offset;
}

/* Function: Effect_Tick
 * Purpose: 10 ms tick handler to update input state and LED effects.
 * Inputs: None.
 * Outputs: None (updates mode state and pushes PWM to TLC59116).
 */
void Effect_Tick(void)
{
  int16_t delta = Encoder_GetDeltaAccel();
  EncoderEvent event = Encoder_GetKeyEvent();

  if (locked != 0U)
  {
    delta = 0;
    if ((event == ENC_EVENT_SUPER_LONGPRESS) && (run_state == RUN_STATE_RUN))
    {
      locked = 0U;
      Save_Config_Now();
    }
    event = ENC_EVENT_NONE;
  }

  if ((event == ENC_EVENT_SUPER_LONGPRESS) && (run_state == RUN_STATE_RUN))
  {
    locked = 1U;
    run_state = RUN_STATE_RUN;
    save_pending = 0U;
    save_delay_ms = 0U;
    Save_Config_Now();
    event = ENC_EVENT_NONE;
  }

  if (event == ENC_EVENT_TRIPLE_CLICK)
  {
    param_layer = (param_layer == PARAM_LAYER_BASIC) ? PARAM_LAYER_ADV : PARAM_LAYER_BASIC;
    setup_item = 0U;
    save_pending = 1U;
    save_delay_ms = 0U;
    event = ENC_EVENT_NONE;
  }

  if (event == ENC_EVENT_DOUBLE_CLICK)
  {
    if (run_state == RUN_STATE_RUN)
    {
      if (mode != MODE_MIN)
      {
        last_mode = mode;
        mode = MODE_MIN;
      }
      else
      {
        mode = last_mode;
      }
      save_pending = 1U;
      save_delay_ms = 0U;
    }
    event = ENC_EVENT_NONE;
  }

  if (event == ENC_EVENT_LONGPRESS)
  {
    if (run_state == RUN_STATE_RUN)
    {
      run_state = RUN_STATE_SETUP;
      setup_item = 0U;
      save_pending = 0U;
      save_delay_ms = 0U;
    }
    else
    {
      run_state = RUN_STATE_RUN;
      save_pending = 0U;
      save_delay_ms = 0U;
      Save_Config_Now();
    }
    event = ENC_EVENT_NONE;
  }

  if (event == ENC_EVENT_CLICK)
  {
    if (run_state == RUN_STATE_RUN)
    {
      last_mode = mode;
      mode++;
      if (mode > MODE_MAX)
      {
        mode = MODE_MIN;
      }
      save_pending = 1U;
      save_delay_ms = 0U;
    }
    else
    {
      uint8_t count = (param_layer == PARAM_LAYER_BASIC) ? BASIC_ITEM_COUNT : ADV_ITEM_COUNT;
      setup_item = (uint8_t)((setup_item + 1U) % count);
    }
    event = ENC_EVENT_NONE;
  }

  if (run_state == RUN_STATE_SETUP)
  {
    if (delta != 0)
    {
      if (param_layer == PARAM_LAYER_BASIC)
      {
        if (setup_item == BASIC_ITEM_BRIGHTNESS)
        {
          int32_t next = (int32_t)global_brightness + delta;
          global_brightness = Clamp_U8(next);
        }
        else
        {
          uint8_t idx = mode;
          if ((idx < MODE_MIN) || (idx > MODE_MAX))
          {
            idx = MODE_MIN;
          }
          {
            int32_t next = (int32_t)param_level[idx] + delta;
            param_level[idx] = Clamp_U8(next);
          }
          if (idx == 7U)
          {
            flash_period = Map_Param_To_Ms(param_level[7], FLASH_MIN_MS, FLASH_MAX_MS);
          }
        }
      }
      else
      {
        switch (setup_item)
        {
          case ADV_ITEM_GAMMA:
            adv_gamma = Clamp_U8((int32_t)adv_gamma + delta);
            break;
          case ADV_ITEM_BREATH_SHAPE:
            adv_breath_shape = Clamp_U8((int32_t)adv_breath_shape + delta);
            break;
          case ADV_ITEM_FLASH_PERIOD:
            {
              int32_t next = (int32_t)flash_period + (int32_t)delta * FLASH_PERIOD_STEP_MS;
              if (next < (int32_t)FLASH_MIN_MS)
              {
                next = FLASH_MIN_MS;
              }
              if (next > (int32_t)FLASH_MAX_MS)
              {
                next = FLASH_MAX_MS;
              }
              flash_period = Clamp_U16(next);
            }
            break;
          case ADV_ITEM_PHASE_OFFSET:
            adv_phase_offset = Clamp_U8((int32_t)adv_phase_offset + delta);
            break;
          default:
            break;
        }
      }
    }
  }

  {
    uint16_t step_ms = STEP_MAX_MS;
    uint16_t flash_ms = flash_period;
    uint16_t breath_ms = Map_Param_To_Ms(param_level[9], BREATH_MIN_MS, BREATH_MAX_MS);

    if ((mode >= 1U) && (mode <= 6U))
    {
      step_ms = Map_Param_To_Ms(param_level[mode], STEP_MIN_MS, STEP_MAX_MS);
    }
    if ((flash_ms < FLASH_MIN_MS) || (flash_ms > FLASH_MAX_MS))
    {
      flash_ms = Map_Param_To_Ms(param_level[7], FLASH_MIN_MS, FLASH_MAX_MS);
      flash_period = flash_ms;
    }

    if (mode == 9U)
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
    env = Apply_Curve(env, adv_breath_shape);
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
        case 6U:
          group = (uint8_t)((group + 1U) % 4U);
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

  if ((save_pending != 0U) && (run_state == RUN_STATE_RUN))
  {
    save_delay_ms = (uint16_t)(save_delay_ms + EFFECT_TICK_MS);
    if (save_delay_ms >= SAVE_DELAY_MS)
    {
      save_delay_ms = 0U;
      save_pending = 0U;
      Save_Config_Now();
    }
  }

  {
    uint8_t cmd[LED_COUNT] = {0};
    uint8_t phase_offset = (uint8_t)(((uint16_t)adv_phase_offset * LED_COUNT + 127U) / 255U);

    if (run_state == RUN_STATE_SETUP)
    {
      uint8_t ui_level = param_level[mode];
      if (param_layer == PARAM_LAYER_BASIC)
      {
        if (setup_item == BASIC_ITEM_BRIGHTNESS)
        {
          ui_level = global_brightness;
        }
        else
        {
          uint8_t idx = mode;
          if ((idx < MODE_MIN) || (idx > MODE_MAX))
          {
            idx = MODE_MIN;
          }
          ui_level = param_level[idx];
        }
      }
      else
      {
        switch (setup_item)
        {
          case ADV_ITEM_GAMMA:
            ui_level = adv_gamma;
            break;
          case ADV_ITEM_BREATH_SHAPE:
            ui_level = adv_breath_shape;
            break;
          case ADV_ITEM_FLASH_PERIOD:
            if ((flash_period >= FLASH_MIN_MS) && (flash_period <= FLASH_MAX_MS))
            {
              uint32_t span = (uint32_t)(FLASH_MAX_MS - FLASH_MIN_MS);
              uint32_t scaled = (uint32_t)(flash_period - FLASH_MIN_MS) * 255U / span;
              ui_level = (uint8_t)scaled;
            }
            else
            {
              ui_level = param_level[7];
            }
            break;
          case ADV_ITEM_PHASE_OFFSET:
            ui_level = adv_phase_offset;
            break;
          default:
            break;
        }
      }
      for (uint8_t i = 0; i < LED_COUNT; i++)
      {
        cmd[i] = ui_level;
      }
      for (uint8_t i = 0; i < LED_COUNT; i++)
      {
        cmd[i] = Apply_BrightnessGamma(cmd[i]);
      }
      Apply_CmdToDutyNext(cmd);
      WDG_TaskKick_Notify(WDG_TASK_EFFECT_10MS);
      return;
    }

    switch (mode)
    {
      case 1U:
      case 2U:
        {
          uint8_t pos_offset = (uint8_t)((pos + phase_offset) % LED_COUNT);
          cmd[pos_offset] = 255U;
        }
        break;
      case 3U:
        {
          static const uint8_t pairs[6][2] = {
            {5U, 6U}, {4U, 7U}, {3U, 8U}, {2U, 9U}, {1U, 10U}, {0U, 11U}
          };
          uint8_t idx = (uint8_t)((group + (phase_offset % 6U)) % 6U);
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
            uint8_t pos_offset = (uint8_t)((pos + phase_offset) % LED_COUNT);
            uint8_t dist = (i > pos_offset) ? (uint8_t)(i - pos_offset) : (uint8_t)(pos_offset - i);
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
        {
          uint8_t env_breath = env;
          if (env_breath < 100U)
          {
            env_breath = 100U;
          }
          {
            uint8_t group_idx = (uint8_t)((group + (phase_offset % 4U)) % 4U);
            uint8_t base = (uint8_t)(group_idx * 3U);
            for (uint8_t i = 0; i < 3U; i++)
            {
              cmd[base + i] = env_breath;
            }
          }
        }
        break;
      case 7U:
        if (flash_on != 0U)
        {
          for (uint8_t i = 0; i < LED_COUNT; i++)
          {
            cmd[i] = 255U;
          }
        }
        break;
      case 9U:
        for (uint8_t i = 0; i < LED_COUNT; i++)
        {
          cmd[i] = breath_lut_cur;
        }
        break;
      default:
        for (uint8_t i = 0; i < LED_COUNT; i++)
        {
          cmd[i] = param_level[8];
        }
        break;
    }

    for (uint8_t i = 0; i < LED_COUNT; i++)
    {
      cmd[i] = Apply_BrightnessGamma(cmd[i]);
    }
    Apply_CmdToDutyNext(cmd);
  }

  WDG_TaskKick_Notify(WDG_TASK_EFFECT_10MS);
}
