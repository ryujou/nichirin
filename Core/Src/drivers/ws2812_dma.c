/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : ws2812_dma.c
  * @brief          : WS2812B driver using TIM1 PWM + DMA (auto refresh)
  ******************************************************************************
  * CubeMX notes:
  * - TIM1 CH1 configured for PWM, PSC=0, ARR=79 (800 kHz)
  * - TIM1 CHx DMA: Memory->Peripheral, Normal mode, HalfWord alignment
  * - TIM1_CH1 on PA8/PB0 (AF2) as configured
  ******************************************************************************
  */
/* USER CODE END Header */
#include "drivers/ws2812_dma.h"
#include <string.h>

#define WS2812_BITS_PER_LED 24U
/* PWM buffer length must match actual LED count; otherwise it will underrun and cause flicker. */
#define WS2812_PWM_LEN (WS2812_LED_COUNT * WS2812_BITS_PER_LED + WS2812_RESET_SLOTS)

#if (WS2812_PWM_LEN < (WS2812_LED_COUNT * WS2812_BITS_PER_LED + WS2812_RESET_SLOTS))
#error "WS2812_PWM_LEN too small: must be >= LED_COUNT*24 + RESET_SLOTS"
#endif

static TIM_HandleTypeDef *s_htim = NULL;
static uint32_t s_channel = 0U;
static uint32_t s_active_channel = 0U;

/* RAM estimate (driver only):
 * - pwm_buf: WS2812_PWM_LEN * 2 bytes = (60*24 + 64) * 2 = 3008 bytes
 * - total (driver): ~3008 bytes + small state
 */
static uint16_t s_pwm_buf[WS2812_PWM_LEN];

static volatile uint8_t s_busy = 0U;
static uint8_t s_running = 0U;

static uint8_t s_brightness = 255U;

static uint32_t WS2812_HalChannel(uint32_t channel)
{
  switch (channel)
  {
    case TIM_CHANNEL_1:
      return HAL_TIM_ACTIVE_CHANNEL_1;
    case TIM_CHANNEL_2:
      return HAL_TIM_ACTIVE_CHANNEL_2;
    case TIM_CHANNEL_3:
      return HAL_TIM_ACTIVE_CHANNEL_3;
    case TIM_CHANNEL_4:
      return HAL_TIM_ACTIVE_CHANNEL_4;
    default:
      return HAL_TIM_ACTIVE_CHANNEL_1;
  }
}

static void WS2812_StopDma(void)
{
  if (s_htim == NULL)
  {
    return;
  }
  (void)HAL_TIM_PWM_Stop_DMA(s_htim, s_channel);
  __HAL_TIM_SET_COMPARE(s_htim, s_channel, 0U);
  s_busy = 0U;
}

static void WS2812_StartDma(void)
{
  HAL_StatusTypeDef st;

  st = HAL_TIM_PWM_Start_DMA(s_htim, s_channel,
                             (uint32_t *)s_pwm_buf,
                             WS2812_PWM_LEN);
  if (st != HAL_OK)
  {
    s_busy = 0U;
    return;
  }
  s_busy = 1U;
}

void ws2812_init(TIM_HandleTypeDef *htim, uint32_t channel)
{
  s_htim = htim;
  s_channel = channel;
  s_active_channel = WS2812_HalChannel(channel);
  memset(s_pwm_buf, 0, sizeof(s_pwm_buf));
  s_busy = 0U;
  s_running = 0U;
  s_brightness = 255U;
  __HAL_TIM_SET_COMPARE(s_htim, s_channel, 0U);
}

void ws2812_start(void)
{
  s_running = 1U;
}

void ws2812_stop(void)
{
  s_running = 0U;
  if (s_busy != 0U)
  {
    WS2812_StopDma();
  }
}

void ws2812_set_brightness(uint8_t br)
{
  s_brightness = br;
}

uint8_t ws2812_is_busy(void)
{
  return s_busy;
}

uint16_t *ws2812_pwm_buf(void)
{
  return s_pwm_buf;
}

uint32_t ws2812_pwm_len(void)
{
  return WS2812_PWM_LEN;
}

void ws2812_start_frame(void)
{
  if ((s_htim == NULL) || (s_running == 0U))
  {
    return;
  }
  if (s_busy != 0U)
  {
    return;
  }
  WS2812_StartDma();
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if ((s_htim != NULL) &&
      (htim->Instance == s_htim->Instance) &&
      (htim->Channel == s_active_channel))
  {
    WS2812_StopDma();
  }
}
