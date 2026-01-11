#include "app/app.h"
#include "app/mode_manager.h"
#include "led/led_ctrl.h"
#include "stream/lcd_streamer.h"
#include "stream/usb_stream_rx.h"
#include "ui/ui.h"
#include "main.h"

#ifndef APP_EFFECT_TICK_MS
#define APP_EFFECT_TICK_MS 10U
#endif

#ifndef APP_MIN_SLEEP_MS
#define APP_MIN_SLEEP_MS 1U
#endif

#ifndef APP_MAX_SLEEP_MS
#define APP_MAX_SLEEP_MS 5U
#endif

static uint32_t s_last_tick_ms = 0U;

void App_Init(void)
{
  HAL_NVIC_SetPriority(SysTick_IRQn, 0U, 0U);

  LedCtrl_Init();
  ModeManager_Init();
  UsbStreamRx_Init();
  Ui_Init();
  LedCtrl_Start();
  s_last_tick_ms = HAL_GetTick();
}

void App_Loop(void)
{
  uint32_t now = HAL_GetTick();
  while ((now - s_last_tick_ms) >= APP_EFFECT_TICK_MS)
  {
    s_last_tick_ms += APP_EFFECT_TICK_MS;
    LedCtrl_10msTick();
  }

  Ui_LoopOnce();
  ModeManager_Poll();
  UsbStreamRx_Poll();
  LcdStreamer_Poll();

  uint32_t wait = Ui_GetSleepHintMs();
  if (wait < APP_MIN_SLEEP_MS)
  {
    wait = APP_MIN_SLEEP_MS;
  }
  if (wait > APP_MAX_SLEEP_MS)
  {
    wait = APP_MAX_SLEEP_MS;
  }
  HAL_Delay(wait);
}
