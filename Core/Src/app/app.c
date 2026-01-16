/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : app.c
  * @brief          : 模式/参数/效果的应用逻辑
  ******************************************************************************
  */
/* USER CODE END Header */
#include "app/app.h"

#include "drivers/encoder.h"
#include "drivers/tlc59116.h"
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

static uint8_t param_level[10] = {0};
static uint8_t param_speed_idx = 0U;
static uint16_t flash_period = 0U;

/* 函数: Map_Param_To_Ms
 * 功能: 将 0..255 参数映射到时间范围 (0 -> max_ms, 255 -> min_ms)。
 * 输入: param - 0..255 参数; min_ms/max_ms - 时间边界。
 * 输出: 映射后的毫秒值。
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

/* 函数: Clamp_U8
 * 功能: 将有符号值钳位到 0..255 范围。
 * 输入: value - 需要钳位的有符号整数。
 * 输出: 0..255 的 uint8_t。
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

/* 函数: Breath_Triangle
 * 功能: 将相位 [0, BREATH_CYCLE) 映射为 0..255 三角波。
 * 输入: phase - 三角波相位。
 * 输出: 0..255 幅度。
 */
static uint8_t Breath_Triangle(uint16_t phase)
{
  uint16_t tri = (phase < BREATH_STEPS) ? phase : (uint16_t)(BREATH_CYCLE - 1U - phase);
  return (uint8_t)tri;
}

/* 函数: Apply_CmdToDutyNext
 * 功能: 将当前 12 路 PWM 命令发送给 TLC59116。
 * 输入: cmd - 12 路 LED PWM 值 (0..255)。
 * 输出: 无 (I2C 写入副作用)。
 */
static void Apply_CmdToDutyNext(const uint8_t cmd[LED_COUNT])
{
  (void)TLC59116_SetPWM12(cmd);
}

/* 函数: Save_Config_Now
 * 功能: 打包并追加当前配置到 Flash 日志。
 * 输入: 无 (使用当前状态变量)。
 * 输出: 无 (Flash 写入副作用)。
 */
static void Save_Config_Now(void)
{
  Config cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.mode = mode;
  cfg.param_speed_idx = param_speed_idx;
  cfg.flash_period = flash_period;
  cfg.format_tag = FLASH_CFG_FORMAT_TAG;
  cfg.param_level[0] = 0U;
  for (uint8_t i = 1U; i <= MODE_MAX; i++)
  {
    cfg.param_level[i] = param_level[i];
  }

  __disable_irq();
  (void)FlashCfg_Append(&cfg);
  __enable_irq();
}

/* 函数: App_Init
 * 功能: 加载 Flash 前初始化运行时默认状态。
 * 输入: 无。
 * 输出: 无 (初始化模块状态)。
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
  param_speed_idx = 0U;
  flash_period = 0U;
  for (uint8_t i = 1U; i <= MODE_MAX; i++)
  {
    param_level[i] = 128U;
  }
  param_level[0] = 0U;
}

/* 函数: Apply_Config
 * 功能: 将持久化配置应用到运行时状态。
 * 输入: cfg - 从 Flash 读取的配置指针。
 * 输出: 无 (更新模块状态)。
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
}

/* 函数: Effect_Tick
 * 功能: 10 ms 节拍更新输入状态与 LED 效果。
 * 输入: 无。
 * 输出: 无 (更新模式状态并输出 PWM 到 TLC59116)。
 */
void Effect_Tick(void)
{
  int8_t delta = Encoder_GetDelta();
  EncoderEvent event = Encoder_GetKeyEvent();

  if (run_state == RUN_STATE_RUN)
  {
    if (event == ENC_EVENT_LONGPRESS)
    {
      run_state = RUN_STATE_SETUP;
      save_pending = 0U;
      save_delay_ms = 0U;
    }
    else if (event == ENC_EVENT_CLICK)
    {
      mode++;
      if (mode > MODE_MAX)
      {
        mode = MODE_MIN;
      }
      save_pending = 1U;
      save_delay_ms = 0U;
    }
  }
  else
  {
    if (event == ENC_EVENT_LONGPRESS)
    {
      run_state = RUN_STATE_RUN;
      save_pending = 0U;
      save_delay_ms = 0U;
      Save_Config_Now();
    }
  }

  if (run_state == RUN_STATE_SETUP)
  {
    if (delta != 0)
    {
      int32_t next = (int32_t)param_level[mode] + delta;
      param_level[mode] = Clamp_U8(next);
    }
  }

  {
    uint16_t step_ms = STEP_MAX_MS;
    uint16_t flash_ms = Map_Param_To_Ms(param_level[7], FLASH_MIN_MS, FLASH_MAX_MS);
    uint16_t breath_ms = Map_Param_To_Ms(param_level[9], BREATH_MIN_MS, BREATH_MAX_MS);

    if ((mode >= 1U) && (mode <= 6U))
    {
      step_ms = Map_Param_To_Ms(param_level[mode], STEP_MIN_MS, STEP_MAX_MS);
    }
    flash_period = flash_ms;

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
    if (mode == 9U)
    {
      breath_lut_cur = env;
    }
    else
    {
      breath_lut_cur = env;
    }

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

    if (run_state == RUN_STATE_SETUP)
    {
      for (uint8_t i = 0; i < LED_COUNT; i++)
      {
        cmd[i] = param_level[mode];
      }
      Apply_CmdToDutyNext(cmd);
      return;
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
        {
          uint8_t env_breath = env;
          if (env_breath < 100U)
          {
            env_breath = 100U;
          }
          {
            uint8_t base = (uint8_t)(group * 3U);
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

    Apply_CmdToDutyNext(cmd);
  }
}
