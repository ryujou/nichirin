#include "watchdog.h"

#define WDG_LSI_HZ          32000U
#define WDG_PRESCALER       IWDG_PRESCALER_64
#define WDG_PRESCALER_DIV   64U
#define WDG_TIMEOUT_MS      4000U
#define WDG_RELOAD          (((WDG_LSI_HZ / WDG_PRESCALER_DIV) * WDG_TIMEOUT_MS) / 1000U - 1U)
#define WDG_WINDOW          IWDG_WINDOW_DISABLE

#define WDG_TICK_MS         10U
#define WDG_WINDOW_TICKS    (1000U / WDG_TICK_MS)

#define WDG_TASK_BIT(id)    (1UL << (id))

#define WDG_REQUIRED_MASK   (WDG_TASK_BIT(WDG_TASK_ENCODER_1MS) | \
                             WDG_TASK_BIT(WDG_TASK_EFFECT_10MS) | \
                             WDG_TASK_BIT(WDG_TASK_TLC_UPDATE))

extern void Error_Handler(void);

static IWDG_HandleTypeDef *s_hiwdg;
static volatile uint32_t s_task_seen_mask;
static uint32_t s_window_ticks;
static uint8_t s_faulted;
static uint8_t s_force_reset;
volatile WDG_ResetReason wdg_last_reset_reason = WDG_RESET_UNKNOWN;

static void WDG_ReadResetFlags(void)
{
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
  {
    wdg_last_reset_reason = WDG_RESET_IWDG;
  }
  else
  {
    wdg_last_reset_reason = WDG_RESET_OTHER;
  }

  __HAL_RCC_CLEAR_RESET_FLAGS();
}

void WDG_Init(IWDG_HandleTypeDef *hiwdg)
{
  s_hiwdg = hiwdg;
  s_task_seen_mask = 0U;
  s_window_ticks = 0U;
  s_faulted = 0U;
  s_force_reset = 0U;

  WDG_ReadResetFlags();

  if (s_hiwdg == NULL)
  {
    return;
  }

  /*
   * IWDG timeout formula:
   *   Tout = (Reload + 1) * Prescaler / LSI
   * Using LSI ~= 32 kHz, Prescaler=64, Reload=1999:
   *   Tout ~= (1999 + 1) * 64 / 32000 ~= 4.0 s
   */
  s_hiwdg->Init.Prescaler = WDG_PRESCALER;
  s_hiwdg->Init.Reload = WDG_RELOAD;
  s_hiwdg->Init.Window = WDG_WINDOW;

  if (HAL_IWDG_Init(s_hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
}

void WDG_TaskKick_Notify(uint32_t task_id)
{
  if (task_id >= (uint32_t)WDG_TASK_COUNT)
  {
    return;
  }
  s_task_seen_mask |= WDG_TASK_BIT(task_id);
}

void WDG_Tick10ms(void)
{
  if (s_hiwdg == NULL)
  {
    return;
  }

  if ((s_faulted != 0U) || (s_force_reset != 0U))
  {
    return;
  }

  s_window_ticks++;
  if (s_window_ticks < WDG_WINDOW_TICKS)
  {
    return;
  }

  s_window_ticks = 0U;

  __disable_irq();
  uint32_t seen = s_task_seen_mask;
  s_task_seen_mask = 0U;
  __enable_irq();

  if ((seen & WDG_REQUIRED_MASK) == WDG_REQUIRED_MASK)
  {
    HAL_IWDG_Refresh(s_hiwdg);
  }
  else
  {
    s_faulted = 1U;
  }
}

void WDG_ForceReset_Test(void)
{
  s_force_reset = 1U;
}

WDG_ResetReason WDG_GetLastResetReason(void)
{
  return wdg_last_reset_reason;
}

uint8_t WDG_WasIwdgReset(void)
{
  return (wdg_last_reset_reason == WDG_RESET_IWDG) ? 1U : 0U;
}
