#include "led/led_effects.h"
#include <math.h>
#include <string.h>

#define EFFECT_TICK_MS 10U
#define STEP_MS 120U
#define ENV_DELTA 4U
#define BREATH_STEPS 256U
#define BREATH_CYCLE (BREATH_STEPS * 2U)
#define FLASH_TOGGLE_MS 100U
#define GROUP_COUNT ((LED_COUNT + 1U) / 2U)
static const uint16_t s_step_ms_table[7] = {240, 200, 160, 130, 110, 90, 70};
static const uint16_t s_flash_ms_table[7] = {300, 240, 200, 160, 130, 110, 90};
static const uint8_t s_bright_cmd_table[7] = {100, 120, 140, 170, 200, 230, 255};
static const uint16_t s_breath_delta_q8_table[7] = {218, 246, 281, 328, 393, 492, 655};

static uint8_t s_gamma_lut[256];

static uint8_t s_mode = 1U;
static uint8_t s_pos = 0U;
static uint8_t s_group = 0U;
static uint16_t s_step_accum_ms = 0U;
static uint16_t s_flash_accum_ms = 0U;
static uint8_t s_flash_on = 0U;
static uint8_t s_env = 0U;
static uint16_t s_breath_phase = 0U;
static uint32_t s_breath_phase_accum = 0U;
static uint8_t s_breath_lut_cur = 0U;
static uint8_t s_param_level[10] = {0};

static void LedEffects_GammaInit(void)
{
  const float gamma = 2.2f;
  for (uint16_t i = 0U; i < 256U; i++)
  {
    float x = (float)i / 255.0f;
    float y = powf(x, gamma);
    uint32_t duty = (uint32_t)(y * 255.0f + 0.5f);
    if (duty > 255U)
    {
      duty = 255U;
    }
    s_gamma_lut[i] = (uint8_t)duty;
  }
}

static void LedEffects_ResetAnim(void)
{
  s_pos = 0U;
  s_group = 0U;
  s_step_accum_ms = 0U;
  s_flash_accum_ms = 0U;
  s_flash_on = 0U;
  s_env = 0U;
  s_breath_phase = 0U;
  s_breath_phase_accum = 0U;
  s_breath_lut_cur = 0U;
}

static void Set_CenterGroup(uint8_t cmd[LED_COUNT], uint8_t group_idx, uint8_t value)
{
  uint8_t center = LED_COUNT / 2U;
  if ((LED_COUNT % 2U) == 0U)
  {
    uint8_t left_center = (uint8_t)(center - 1U);
    uint8_t right_center = center;
    if (group_idx == 0U)
    {
      cmd[left_center] = value;
      cmd[right_center] = value;
    }
    else
    {
      uint8_t left = (uint8_t)(left_center - group_idx);
      uint8_t right = (uint8_t)(right_center + group_idx);
      if (left < LED_COUNT)
      {
        cmd[left] = value;
      }
      if (right < LED_COUNT)
      {
        cmd[right] = value;
      }
    }
  }
  else
  {
    if (group_idx == 0U)
    {
      cmd[center] = value;
    }
    else
    {
      uint8_t left = (uint8_t)(center - group_idx);
      uint8_t right = (uint8_t)(center + group_idx);
      if (left < LED_COUNT)
      {
        cmd[left] = value;
      }
      if (right < LED_COUNT)
      {
        cmd[right] = value;
      }
    }
  }
}

static uint8_t Breath_FromPhase(uint16_t phase)
{
  uint16_t tri = (phase < BREATH_STEPS) ? phase : (uint16_t)(BREATH_CYCLE - 1U - phase);
  return s_gamma_lut[tri];
}

static uint8_t Breath_Triangle(uint16_t phase)
{
  uint16_t tri = (phase < BREATH_STEPS) ? phase : (uint16_t)(BREATH_CYCLE - 1U - phase);
  return s_gamma_lut[tri];
}

void LedEffects_Init(void)
{
  LedEffects_GammaInit();
  LedEffects_ResetAnim();

  s_mode = 1U;
  s_param_level[0] = 0U;
  for (uint8_t i = 1U; i <= 9U; i++)
  {
    s_param_level[i] = 4U;
  }
}

void LedEffects_SetMode(uint8_t mode)
{
  if ((mode < 1U) || (mode > 9U))
  {
    mode = 1U;
  }
  if (s_mode != mode)
  {
    s_mode = mode;
  }
}

void LedEffects_SetLevel(uint8_t mode, uint8_t level_1_to_7)
{
  if ((mode < 1U) || (mode > 9U))
  {
    return;
  }
  uint8_t level = level_1_to_7;
  if ((level < 1U) || (level > 7U))
  {
    level = 4U;
  }
  if (s_param_level[mode] != level)
  {
    s_param_level[mode] = level;
  }
}
void LedEffects_Tick10ms(uint8_t out_cmd[LED_COUNT])
{
  if (out_cmd == NULL)
  {
    return;
  }
  uint8_t level_idx = (s_param_level[s_mode] > 0U) ? (uint8_t)(s_param_level[s_mode] - 1U) : 0U;
  uint16_t step_ms = STEP_MS;
  uint16_t flash_ms = FLASH_TOGGLE_MS;

  if ((s_mode >= 1U) && (s_mode <= 6U))
  {
    step_ms = s_step_ms_table[level_idx];
  }
  else if (s_mode == 7U)
  {
    flash_ms = s_flash_ms_table[level_idx];
  }

  if (s_mode == 9U)
  {
    uint32_t delta_q8 = s_breath_delta_q8_table[level_idx];
    uint32_t scaled = (delta_q8 * EFFECT_TICK_MS) / EFFECT_TICK_MS;
    s_breath_phase_accum += scaled;
    if (s_breath_phase_accum >= ((uint32_t)BREATH_CYCLE << 8))
    {
      s_breath_phase_accum -= (uint32_t)BREATH_CYCLE << 8;
    }
    s_breath_phase = (uint16_t)(s_breath_phase_accum >> 8);
  }
  else
  {
    uint32_t delta = ((uint32_t)ENV_DELTA * EFFECT_TICK_MS) / EFFECT_TICK_MS;
    s_breath_phase = (uint16_t)(s_breath_phase + delta);
    if (s_breath_phase >= BREATH_CYCLE)
    {
      s_breath_phase = (uint16_t)(s_breath_phase - BREATH_CYCLE);
    }
  }

  {
    uint16_t breath = Breath_FromPhase(s_breath_phase);
    uint16_t boosted = (uint16_t)((breath * 255U) / 32U);
    if (boosted > 255U)
    {
      boosted = 255U;
    }
    s_env = (uint8_t)boosted;
    if (s_mode == 9U)
    {
      s_breath_lut_cur = Breath_Triangle(s_breath_phase);
    }
    else
    {
      s_breath_lut_cur = (uint8_t)breath;
    }
  }

  s_step_accum_ms += EFFECT_TICK_MS;
  if (s_step_accum_ms >= step_ms)
  {
    s_step_accum_ms = (uint16_t)(s_step_accum_ms - step_ms);
    switch (s_mode)
    {
      case 1U:
      case 4U:
        s_pos = (uint8_t)((s_pos + 1U) % LED_COUNT);
        break;
      case 2U:
      case 5U:
        s_pos = (uint8_t)((s_pos + LED_COUNT - 1U) % LED_COUNT);
        break;
      case 3U:
      case 6U:
        s_group = (uint8_t)((s_group + 1U) % GROUP_COUNT);
        break;
      default:
        break;
    }
  }

  s_flash_accum_ms += EFFECT_TICK_MS;
  if (s_flash_accum_ms >= flash_ms)
  {
    s_flash_accum_ms = (uint16_t)(s_flash_accum_ms - flash_ms);
    s_flash_on = (uint8_t)!s_flash_on;
  }

  memset(out_cmd, 0, LED_COUNT);

  /* Mode meanings:
   * 1: single dot forward
   * 2: single dot reverse
   * 3: center pair expands outward
   * 4: comet forward with breathing envelope
   * 5: comet reverse with breathing envelope
   * 6: center group breathing
   * 7: global flash toggle
   * 8: steady full brightness level
   * 9: global breathing
   */
  switch (s_mode)
  {
    case 1U:
    case 2U:
      out_cmd[s_pos] = 255U;
      break;
    case 3U:
      Set_CenterGroup(out_cmd, s_group, 255U);
      break;
    case 4U:
    case 5U:
      {
        uint8_t env_breath = s_env;
        if (env_breath < 100U)
        {
          env_breath = 100U;
        }
        for (uint8_t i = 0U; i < LED_COUNT; i++)
        {
          uint8_t dist = (i > s_pos) ? (i - s_pos) : (s_pos - i);
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
          out_cmd[i] = (uint8_t)(((uint16_t)base * env_breath) / 255U);
        }
      }
      break;
    case 6U:
      {
        uint8_t env_breath = s_env;
        if (env_breath < 100U)
        {
          env_breath = 100U;
        }
        Set_CenterGroup(out_cmd, s_group, env_breath);
      }
      break;
    case 7U:
      if (s_flash_on != 0U)
      {
        for (uint8_t i = 0U; i < LED_COUNT; i++)
        {
          out_cmd[i] = 255U;
        }
      }
      break;
    case 8U:
      for (uint8_t i = 0U; i < LED_COUNT; i++)
      {
        uint8_t lvl = s_param_level[8];
        uint8_t idx = (lvl > 0U) ? (uint8_t)(lvl - 1U) : 0U;
        out_cmd[i] = s_bright_cmd_table[idx];
      }
      break;
    case 9U:
      for (uint8_t i = 0U; i < LED_COUNT; i++)
      {
        out_cmd[i] = s_breath_lut_cur;
      }
      break;
    default:
      break;
  }
}
