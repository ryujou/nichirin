/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : app.c
  * @brief          : Grouped WS2812 effects + UI + 200Hz scheduler
  ******************************************************************************
  * 200Hz scheduling:
  * - app_loop() is called in the main while(1).
  * - A 1ms software tick updates encoder and a 5ms frame timer.
  * - When 5ms elapsed and DMA is idle, render a full frame and start DMA.
  * - If DMA is busy, the frame is marked pending and sent as soon as DMA ends.
  ******************************************************************************
  * UI controls (simplified):
  * - Normal: CLICK -> next mode (1,2,3,4,6), LONGPRESS -> enter setting.
  * - Setting: CLICK -> toggle page (Color <-> ModeParam), DOUBLE_CLICK -> next item,
  *           ROTATE -> adjust item, LONGPRESS -> exit.
  ******************************************************************************
  */
/* USER CODE END Header */
#include "app.h"
#include "drivers/ws2812_dma.h"
#include "drivers/encoder.h"
#include "drivers/uart.h"
#include "utils/led_utils.h"
#include "main.h"
#include "storage/cfg_store.h"
#include "stm32g0xx_hal.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

#define GROUP_COUNT 12U
#define LEDS_PER_GROUP 5U
#define LED_COUNT (GROUP_COUNT * LEDS_PER_GROUP)

#define FRAME_PERIOD_MS 2U
#define FLASH_MS 100U
#define SPECTRUM_TIMEOUT_MS 200U
#define SPECTRUM_SMOOTH_SHIFT 3U
#define SPECTRUM_HUE_RANGE 300U

#define FLOW_INTERVAL_MIN_MS 40U
#define FLOW_INTERVAL_MAX_MS 320U

typedef struct
{
  uint8_t mode;       /* 1..6 */
  uint16_t phase;     /* effect phase accumulator */
  uint8_t flash_ms;   /* selection flash countdown */
} GroupState;

typedef struct
{
  uint16_t hue;   /* 0..359 */
  uint8_t sat;    /* 0..255 */
  uint8_t val;    /* 0..255 */
} GlobalColor;

typedef struct
{
  uint8_t flow_speed;     /* 0..255, larger = slower */
  uint8_t strobe_period;  /* 0..255, larger = slower */
  uint8_t steady_bright;  /* 0..255 */
  uint8_t breath_speed;   /* 0..255, larger = slower */
  uint8_t spectrum_gain;  /* 0..255 */
} ModeParams;

static GroupState s_groups[GROUP_COUNT];
static GlobalColor s_color;
static ModeParams s_params;

static uint8_t s_ui_in_setting = 0U;
static uint8_t s_ui_page = 0U;     /* 0=color, 1=modeparam */
static uint8_t s_ui_item = 0U;     /* item index in page */

static uint32_t s_last_ms = 0U;
static uint32_t s_frame_accum_ms = 0U;
static uint8_t s_frame_pending = 0U;

static uint16_t s_global_phase = 0U;
static uint16_t s_bands_smooth[GROUP_COUNT];
static uint32_t s_flow_accum_ms = 0U;
static uint8_t s_ui_blink = 0U;

static char s_dbg_buf[128];

static uint8_t s_cfg_dirty = 0U;
static uint8_t s_cfg_force_save = 0U;
static uint32_t s_cfg_last_change_ms = 0U;

static void App_MarkCfgDirty(uint32_t now_ms)
{
  s_cfg_dirty = 1U;
  s_cfg_force_save = 0U;
  s_cfg_last_change_ms = now_ms;
}

static void App_FillCfgFromRuntime(Config *cfg)
{
  if (cfg == NULL)
  {
    return;
  }

  memset(cfg, 0, sizeof(*cfg));
  cfg->format_tag = FLASH_CFG_FORMAT_TAG;
  cfg->mode = s_groups[0].mode;
  cfg->flow_speed = s_params.flow_speed;
  cfg->strobe_period = s_params.strobe_period;
  cfg->steady_bright = s_params.steady_bright;
  cfg->hue = s_color.hue;
  cfg->sat = s_color.sat;
  cfg->val = s_color.val;
  cfg->spectrum_gain = s_params.spectrum_gain;
  cfg->breath_speed = s_params.breath_speed;
  cfg->cfg_version = FLASH_CFG_VERSION;
  cfg->created_by_fw = 0U;
  cfg->last_migrated_by_fw = 0U;
}

static void Debug_Printf(const char *fmt, ...)
{
  va_list ap;
  int len;
  size_t used;

  va_start(ap, fmt);
  len = vsnprintf(s_dbg_buf, sizeof(s_dbg_buf), fmt, ap);
  va_end(ap);

  if (len <= 0)
  {
    return;
  }
  if ((size_t)len >= sizeof(s_dbg_buf))
  {
    len = (int)(sizeof(s_dbg_buf) - 1U);
  }
  used = (size_t)len;
  if (used < (sizeof(s_dbg_buf) - 2U))
  {
    if ((used < 2U) || (s_dbg_buf[used - 1U] != '\n'))
    {
      s_dbg_buf[used++] = '\r';
      s_dbg_buf[used++] = '\n';
      s_dbg_buf[used] = '\0';
    }
  }
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)s_dbg_buf, (uint16_t)used, 50U);
}

static uint8_t App_ClampU8(int16_t v)
{
  if (v < 0)
  {
    return 0U;
  }
  if (v > 255)
  {
    return 255U;
  }
  return (uint8_t)v;
}

static uint32_t App_MapFlowIntervalMs(uint8_t flow_speed)
{
  return FLOW_INTERVAL_MIN_MS + ((uint32_t)flow_speed * (FLOW_INTERVAL_MAX_MS - FLOW_INTERVAL_MIN_MS)) / 255U;
}

static uint32_t App_MapStrobePeriodMs(uint8_t period)
{
  return 80U + ((uint32_t)period * (2000U - 80U)) / 255U;
}

static uint16_t App_MapBreathStep(uint8_t breath_speed)
{
  uint16_t step = 1U + ((uint16_t)(255U - breath_speed) * 4U) / 255U;
  return step;
}

static void App_FlashAll(uint8_t times)
{
  uint8_t g;
  uint16_t ms = (uint16_t)FLASH_MS * (uint16_t)times;
  if (ms > 255U)
  {
    ms = 255U;
  }
  for (g = 0U; g < GROUP_COUNT; g++)
  {
    s_groups[g].flash_ms = (uint8_t)ms;
  }
}

static void App_UpdateSpectrum(uint32_t now_ms)
{
  uint8_t bands[GROUP_COUNT];
  uint32_t last_ms = 0U;
  uint8_t src = 0U;
  uint8_t g;
  uint8_t gain = s_params.spectrum_gain;

  uart_get_bands(bands, &last_ms, &src);
  if ((last_ms == 0U) || ((now_ms - last_ms) > SPECTRUM_TIMEOUT_MS))
  {
    for (g = 0U; g < GROUP_COUNT; g++)
    {
      bands[g] = 0U;
    }
  }

  for (g = 0U; g < GROUP_COUNT; g++)
  {
    uint16_t scaled = ((uint16_t)bands[g] * (uint16_t)(64U + gain)) >> 7U;
    int16_t smooth;
    int16_t diff;

    if (scaled > 255U)
    {
      scaled = 255U;
    }
    smooth = (int16_t)s_bands_smooth[g];
    diff = (int16_t)scaled - smooth;
    smooth = (int16_t)(smooth + (diff >> SPECTRUM_SMOOTH_SHIFT));
    if (smooth < 0)
    {
      smooth = 0;
    }
    if (smooth > 255)
    {
      smooth = 255;
    }
    s_bands_smooth[g] = (uint16_t)smooth;
  }

  (void)src;
}

static void App_RenderGroup(uint8_t group, uint16_t *pwm, uint32_t *idx)
{
  GroupState *gs = &s_groups[group];
  uint8_t r = 0U;
  uint8_t g = 0U;
  uint8_t b = 0U;
  uint8_t i;
  uint8_t base_v = s_color.val;
  uint8_t base_s = s_color.sat;

  if (gs->flash_ms > 0U)
  {
    if (base_v < 220U)
    {
      base_v = (uint8_t)(base_v + 35U);
    }
  }
  else if ((s_ui_in_setting != 0U) && (group == 0U) && (s_ui_blink < 10U))
  {
    if (base_v < 235U)
    {
      base_v = (uint8_t)(base_v + 20U);
    }
  }

  switch (gs->mode)
  {
    case 1: /* forward group flow */
    {
      uint8_t pos = (uint8_t)(s_global_phase % GROUP_COUNT);
      uint8_t v = (group == pos) ? base_v : 0U;
      led_hsv_to_rgb(s_color.hue, base_s, v, &r, &g, &b);
      for (i = 0U; i < LEDS_PER_GROUP; i++)
      {
        led_ws2812_encode_rgb(r, g, b, pwm, idx, ws2812_pwm_len());
      }
      break;
    }
    case 2: /* strobe */
    {
      uint8_t on = (gs->phase & 0x100U) ? 1U : 0U;
      uint8_t v = on ? base_v : 0U;
      led_hsv_to_rgb(s_color.hue, base_s, v, &r, &g, &b);
      for (i = 0U; i < LEDS_PER_GROUP; i++)
      {
        led_ws2812_encode_rgb(r, g, b, pwm, idx, ws2812_pwm_len());
      }
      break;
    }
    case 3: /* steady */
    {
      uint8_t v = s_params.steady_bright;
      led_hsv_to_rgb(s_color.hue, base_s, v, &r, &g, &b);
      for (i = 0U; i < LEDS_PER_GROUP; i++)
      {
        led_ws2812_encode_rgb(r, g, b, pwm, idx, ws2812_pwm_len());
      }
      break;
    }
    case 4: /* breathing */
    {
      uint8_t v = (uint8_t)(((uint16_t)base_v * led_breath_from_phase(gs->phase)) / 255U);
      led_hsv_to_rgb(s_color.hue, base_s, v, &r, &g, &b);
      for (i = 0U; i < LEDS_PER_GROUP; i++)
      {
        led_ws2812_encode_rgb(r, g, b, pwm, idx, ws2812_pwm_len());
      }
      break;
    }
    case 6: /* audio spectrum */
    {
      uint16_t base_h = (uint16_t)((group * SPECTRUM_HUE_RANGE) / (GROUP_COUNT - 1U));
      uint16_t hue = (uint16_t)((base_h + s_color.hue) % 360U);
      uint8_t lit = (uint8_t)((s_bands_smooth[group] * (LEDS_PER_GROUP + 1U)) >> 8U);

      if (lit > LEDS_PER_GROUP)
      {
        lit = LEDS_PER_GROUP;
      }
      for (i = 0U; i < LEDS_PER_GROUP; i++)
      {
        uint8_t v = 0U;
        if (i < lit)
        {
          v = (uint8_t)(((uint16_t)base_v * (uint16_t)(i + 1U)) / LEDS_PER_GROUP);
        }
        led_hsv_to_rgb(hue, base_s, v, &r, &g, &b);
        led_ws2812_encode_rgb(r, g, b, pwm, idx, ws2812_pwm_len());
      }
      break;
    }
    default:
    {
      led_hsv_to_rgb(s_color.hue, base_s, base_v, &r, &g, &b);
      for (i = 0U; i < LEDS_PER_GROUP; i++)
      {
        led_ws2812_encode_rgb(r, g, b, pwm, idx, ws2812_pwm_len());
      }
      break;
    }
  }
}

static void App_RenderFrame(void)
{
  uint16_t *pwm = ws2812_pwm_buf();
  uint32_t idx = 0U;
  uint8_t group;

  for (group = 0U; group < GROUP_COUNT; group++)
  {
    App_RenderGroup(group, pwm, &idx);
  }

  /* fill rest of PWM buffer */
  while (idx < ws2812_pwm_len())
  {
    pwm[idx++] = 0U;
  }
}

void led_group_set_rgb(uint8_t group, uint8_t r, uint8_t g, uint8_t b)
{
  uint16_t h;
  uint8_t s;
  uint8_t v;

  if (group >= GROUP_COUNT)
  {
    return;
  }
  led_rgb_to_hsv(r, g, b, &h, &s, &v);
  s_color.hue = h;
  s_color.sat = s;
  s_color.val = v;
}

void led_group_set_hsv(uint8_t group, uint16_t h, uint8_t s, uint8_t v)
{
  if (group >= GROUP_COUNT)
  {
    return;
  }
  if (h >= 360U)
  {
    h %= 360U;
  }
  s_color.hue = h;
  s_color.sat = s;
  s_color.val = v;
}

void led_group_set_brightness(uint8_t group, uint8_t br)
{
  (void)group;
  s_color.val = br;
}

void led_group_fill(uint8_t group, uint8_t r, uint8_t g, uint8_t b)
{
  led_group_set_rgb(group, r, g, b);
}

void app_init(void)
{
  uint8_t i;
  Config cfg;
  uint8_t loaded;
  uint8_t default_saved = 0U;

  ws2812_init(&htim1, TIM_CHANNEL_1);
  ws2812_start();

  Encoder_Init(ENC_A_GPIO_Port, ENC_A_Pin,
               ENC_B_GPIO_Port, ENC_B_Pin,
               ENC_KEY_GPIO_Port, ENC_KEY_Pin);
  uart_init(&huart1, &huart2);

  loaded = (uint8_t)(Cfg_Load(&cfg) ? 1U : 0U);
  if (loaded == 0U)
  {
    Cfg_ResetToDefault(&cfg);
    if (ws2812_is_busy() == 0U)
    {
      if (Cfg_SaveAtomic(&cfg))
      {
        default_saved = 1U;
      }
    }
    Debug_Printf("CFG load: none -> default");
  }

  if (!((cfg.mode == 1U) || (cfg.mode == 2U) || (cfg.mode == 3U) || (cfg.mode == 4U) || (cfg.mode == 6U)))
  {
    cfg.mode = 1U;
  }

  for (i = 0U; i < GROUP_COUNT; i++)
  {
    s_groups[i].mode = cfg.mode;
    s_groups[i].phase = 0U;
    s_groups[i].flash_ms = 0U;
    s_bands_smooth[i] = 0U;
  }

  s_color.hue = cfg.hue;
  s_color.sat = cfg.sat;
  s_color.val = cfg.val;

  s_params.flow_speed = cfg.flow_speed;
  s_params.strobe_period = cfg.strobe_period;
  s_params.steady_bright = cfg.steady_bright;
  s_params.breath_speed = cfg.breath_speed;
  s_params.spectrum_gain = cfg.spectrum_gain;
  if (loaded != 0U)
  {
    Debug_Printf("CFG load: ok mode=%u hue=%u sat=%u val=%u flow=%u strobe=%u steady=%u breath=%u gain=%u",
                 (unsigned)cfg.mode, (unsigned)cfg.hue, (unsigned)cfg.sat, (unsigned)cfg.val,
                 (unsigned)cfg.flow_speed, (unsigned)cfg.strobe_period, (unsigned)cfg.steady_bright,
                 (unsigned)cfg.breath_speed, (unsigned)cfg.spectrum_gain);
  }

  s_ui_in_setting = 0U;
  s_ui_page = 0U;
  s_ui_item = 0U;

  s_last_ms = HAL_GetTick();
  s_frame_accum_ms = 0U;
  s_frame_pending = 1U;
  s_global_phase = 0U;
  s_flow_accum_ms = 0U;
  s_ui_blink = 0U;

  s_cfg_dirty = 0U;
  s_cfg_force_save = 0U;
  s_cfg_last_change_ms = s_last_ms;
  if ((loaded == 0U) && (default_saved == 0U))
  {
    s_cfg_dirty = 1U;
    s_cfg_force_save = 1U;
    s_cfg_last_change_ms = s_last_ms;
  }
}

void app_loop(void)
{
  uint32_t now_ms = HAL_GetTick();
  uint32_t delta_ms = now_ms - s_last_ms;
  uint32_t step;

  uart_tick();

  for (step = 0U; step < delta_ms; step++)
  {
    uint8_t g;
    s_last_ms++;
    Encoder_1msTick();

    for (g = 0U; g < GROUP_COUNT; g++)
    {
      if (s_groups[g].flash_ms > 0U)
      {
        s_groups[g].flash_ms--;
      }
    }

    s_frame_accum_ms++;
    if (s_frame_accum_ms >= FRAME_PERIOD_MS)
    {
      s_frame_accum_ms -= FRAME_PERIOD_MS;
      s_frame_pending = 1U;

      App_UpdateSpectrum(s_last_ms);

      s_ui_blink++;
      if (s_ui_blink >= 40U)
      {
        s_ui_blink = 0U;
      }

      if (s_groups[0].mode == 1U)
      {
        uint32_t interval = App_MapFlowIntervalMs(s_params.flow_speed);
        s_flow_accum_ms += FRAME_PERIOD_MS;
        if (s_flow_accum_ms >= interval)
        {
          s_flow_accum_ms -= interval;
          s_global_phase = (uint16_t)(s_global_phase + 1U);
        }
      }
      else
      {
        s_flow_accum_ms = 0U;
      }

      for (g = 0U; g < GROUP_COUNT; g++)
      {
        uint16_t step_phase = 0U;
        if (s_groups[g].mode == 2U)
        {
          uint32_t period_ms = App_MapStrobePeriodMs(s_params.strobe_period);
          uint32_t step_calc = (512U * FRAME_PERIOD_MS + (period_ms / 2U)) / period_ms;
          if (step_calc < 1U)
          {
            step_calc = 1U;
          }
          if (step_calc > 255U)
          {
            step_calc = 255U;
          }
          step_phase = (uint16_t)step_calc;
        }
        else if (s_groups[g].mode == 4U)
        {
          step_phase = App_MapBreathStep(s_params.breath_speed);
        }
        else
        {
          step_phase = 0U;
        }

        s_groups[g].phase = (uint16_t)(s_groups[g].phase + step_phase);
      }
    }
  }

  {
    EncoderEvent ev = Encoder_GetKeyEvent();
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

    if ((delta_sum != 0) && (s_ui_in_setting != 0U))
    {
      uint16_t prev_hue = s_color.hue;
      uint8_t prev_sat = s_color.sat;
      uint8_t prev_val = s_color.val;
      uint8_t prev_flow = s_params.flow_speed;
      uint8_t prev_strobe = s_params.strobe_period;
      uint8_t prev_steady = s_params.steady_bright;
      uint8_t prev_breath = s_params.breath_speed;
      uint8_t prev_gain = s_params.spectrum_gain;

      if (s_ui_page == 0U)
      {
        if (s_ui_item == 0U)
        {
          int16_t hue_delta = delta_sum;
          if (hue_delta > 5)
          {
            hue_delta = 5;
          }
          if (hue_delta < -5)
          {
            hue_delta = -5;
          }
          int16_t h = (int16_t)s_color.hue + hue_delta;
          if (h < 0)
          {
            h += 360;
          }
          if (h >= 360)
          {
            h -= 360;
          }
          s_color.hue = (uint16_t)h;
        }
        else if (s_ui_item == 1U)
        {
          s_color.sat = App_ClampU8((int16_t)s_color.sat + delta_sum);
        }
        else
        {
          s_color.val = App_ClampU8((int16_t)s_color.val + delta_sum);
        }
      }
      else
      {
        switch (s_groups[0].mode)
        {
          case 1:
            s_params.flow_speed = App_ClampU8((int16_t)s_params.flow_speed + delta_sum);
            break;
          case 2:
            s_params.strobe_period = App_ClampU8((int16_t)s_params.strobe_period + delta_sum);
            break;
          case 3:
            s_params.steady_bright = App_ClampU8((int16_t)s_params.steady_bright + delta_sum);
            break;
          case 4:
            s_params.breath_speed = App_ClampU8((int16_t)s_params.breath_speed + delta_sum);
            break;
          case 6:
            s_params.spectrum_gain = App_ClampU8((int16_t)s_params.spectrum_gain + delta_sum);
            break;
          default:
            break;
        }
      }
      s_frame_pending = 1U;
      if ((s_color.hue != prev_hue) || (s_color.sat != prev_sat) || (s_color.val != prev_val) ||
          (s_params.flow_speed != prev_flow) || (s_params.strobe_period != prev_strobe) ||
          (s_params.steady_bright != prev_steady) || (s_params.breath_speed != prev_breath) ||
          (s_params.spectrum_gain != prev_gain))
      {
        App_MarkCfgDirty(now_ms);
      }

      {
        uint16_t val = 0U;
        if (s_ui_page == 0U)
        {
          if (s_ui_item == 0U) { val = s_color.hue; }
          else if (s_ui_item == 1U) { val = s_color.sat; }
          else { val = s_color.val; }
        }
        else
        {
          switch (s_groups[0].mode)
          {
            case 1: val = s_params.flow_speed; break;
            case 2: val = s_params.strobe_period; break;
            case 3: val = s_params.steady_bright; break;
            case 4: val = s_params.breath_speed; break;
            case 6: val = s_params.spectrum_gain; break;
            default: val = 0U; break;
          }
        }
        Debug_Printf("SET page=%u item=%u mode=%u val=%u", (unsigned)s_ui_page, (unsigned)s_ui_item, (unsigned)s_groups[0].mode, (unsigned)val);
      }
    }

    if (ev == ENC_EVENT_CLICK)
    {
      uint8_t g;
      if (s_ui_in_setting == 0U)
      {
        static const uint8_t k_modes[] = { 1U, 2U, 3U, 4U, 6U };
        uint8_t old_mode = s_groups[0].mode;
        uint8_t next_mode = k_modes[0];
        for (g = 0U; g < (uint8_t)(sizeof(k_modes) / sizeof(k_modes[0])); g++)
        {
          if (s_groups[0].mode == k_modes[g])
          {
            uint8_t next = (uint8_t)(g + 1U);
            if (next >= (uint8_t)(sizeof(k_modes) / sizeof(k_modes[0])))
            {
              next = 0U;
            }
            next_mode = k_modes[next];
            break;
          }
        }
        for (g = 0U; g < GROUP_COUNT; g++)
        {
          s_groups[g].mode = next_mode;
          s_groups[g].phase = 0U;
          s_groups[g].flash_ms = 0U;
        }
        App_FlashAll(1U);
        if (next_mode != old_mode)
        {
          App_MarkCfgDirty(now_ms);
        }
      }
      else
      {
        s_ui_page = (uint8_t)(1U - s_ui_page);
        s_ui_item = 0U;
        App_FlashAll(1U);
      }
      s_frame_pending = 1U;
    }
    else if (ev == ENC_EVENT_DOUBLE_CLICK)
    {
      if (s_ui_in_setting != 0U)
      {
        if (s_ui_page == 0U)
        {
          s_ui_item = (uint8_t)((s_ui_item + 1U) % 3U);
        }
        else
        {
          s_ui_item = 0U;
        }
        App_FlashAll(1U);
        s_frame_pending = 1U;
      }
    }
    else if (ev == ENC_EVENT_LONGPRESS)
    {
      if (s_ui_in_setting == 0U)
      {
        if (s_groups[0].mode != 6U)
        {
          s_ui_in_setting = 1U;
          s_ui_page = 0U;
          s_ui_item = 0U;
          App_FlashAll(1U);
        }
      }
      else
      {
        s_ui_in_setting = 0U;
        App_FlashAll(2U);
        s_cfg_dirty = 1U;
        s_cfg_force_save = 1U;
        s_cfg_last_change_ms = now_ms;
      }
      s_frame_pending = 1U;
    }
  }

  if (s_cfg_dirty != 0U)
  {
    uint8_t due = s_cfg_force_save;
    if (due == 0U)
    {
      if ((uint32_t)(now_ms - s_cfg_last_change_ms) >= 500U)
      {
        due = 1U;
      }
    }
    if ((due != 0U) && (ws2812_is_busy() == 0U))
    {
      Config cfg;
      App_FillCfgFromRuntime(&cfg);
      if (Cfg_SaveAtomic(&cfg))
      {
        if (s_cfg_force_save != 0U)
        {
          Debug_Printf("CFG saved");
        }
        s_cfg_dirty = 0U;
        s_cfg_force_save = 0U;
      }
      else
      {
        Debug_Printf("CFG save failed, will retry");
        s_cfg_dirty = 1U;
        s_cfg_force_save = 0U;
        s_cfg_last_change_ms = now_ms;
      }
    }
  }

  if ((s_frame_pending != 0U) && (ws2812_is_busy() == 0U))
  {
    App_RenderFrame();
    ws2812_start_frame();
    if (ws2812_is_busy() != 0U)
    {
      s_frame_pending = 0U;
    }
  }
}
