#include "ui/ui.h"
#include "lvgl.h"
#include "st7735.h"
#include "ui/keypad_port.h"
#include "ui/lcd_port.h"
#include "ui/ui_led_menu.h"

static uint32_t s_last_wait_ms = 5U;

void Ui_Init(void)
{
  lv_init();
  ST7735_Init();
  /* Minimal visibility check: fill white before LVGL starts drawing. */
  ST7735_FillColor(0xFFFFU);
  LcdPort_Init();
  KeypadPort_Init();
  UiLedMenu_Init();
}

void Ui_LoopOnce(void)
{
  s_last_wait_ms = lv_timer_handler();
}

uint32_t Ui_GetSleepHintMs(void)
{
  return s_last_wait_ms;
}

void Ui_TickInc1ms(void)
{
  lv_tick_inc(1);
}
