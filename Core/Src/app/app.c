#include "app/app.h"
#include "led/led_ctrl.h"
#include "ui/simple_menu.h"
#include "main.h"

#ifndef APP_MIN_SLEEP_MS
#define APP_MIN_SLEEP_MS 1U
#endif

void App_Init(void)
{
  HAL_NVIC_SetPriority(SysTick_IRQn, 0U, 0U);

  LedCtrl_Init();
  LedCtrl_Start();
  SimpleMenu_Init();
}

void App_Loop(void)
{
  static uint32_t last_tick_ms = 0U;

  SimpleMenu_Loop();

  {
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_tick_ms) >= 10U)
    {
      last_tick_ms = now;
      LedCtrl_10msTick();
    }
  }

  HAL_Delay(APP_MIN_SLEEP_MS);
}
