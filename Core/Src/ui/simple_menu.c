#include "ui/simple_menu.h"
#include "lcd_st7735.h"
#include "led/led_ctrl.h"
#include "stream/usb_stream_rx.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

#define MENU_DEBOUNCE_MS 20U
#define MENU_RENDER_THROTTLE_MS 50U

typedef enum
{
  BTN_UP = 0,
  BTN_DOWN = 1,
  BTN_LEFT = 2,
  BTN_RIGHT = 3,
  BTN_CENTER = 4
} MenuButton;

static uint8_t s_selected = 0U;
static uint8_t s_menu_mode = 1U;
static uint8_t s_stream_active = 0U;
static uint32_t s_last_render_ms = 0U;
static uint8_t s_dirty = 1U;

static uint8_t s_raw = 0U;
static uint8_t s_stable = 0U;
static uint32_t s_last_change_ms = 0U;

static uint8_t read_buttons_raw(void)
{
  uint8_t mask = 0U;
  if (HAL_GPIO_ReadPin(UP_GPIO_Port, UP_Pin) == GPIO_PIN_RESET)
  {
    mask |= (1U << BTN_UP);
  }
  if (HAL_GPIO_ReadPin(DOWN_GPIO_Port, DOWN_Pin) == GPIO_PIN_RESET)
  {
    mask |= (1U << BTN_DOWN);
  }
  if (HAL_GPIO_ReadPin(LEFT_GPIO_Port, LEFT_Pin) == GPIO_PIN_RESET)
  {
    mask |= (1U << BTN_LEFT);
  }
  if (HAL_GPIO_ReadPin(RIGHT_GPIO_Port, RIGHT_Pin) == GPIO_PIN_RESET)
  {
    mask |= (1U << BTN_RIGHT);
  }
  if (HAL_GPIO_ReadPin(CENTER_GPIO_Port, CENTER_Pin) == GPIO_PIN_RESET)
  {
    mask |= (1U << BTN_CENTER);
  }
  return mask;
}

static uint8_t buttons_poll_edges(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t raw = read_buttons_raw();

  if (raw != s_raw)
  {
    s_raw = raw;
    s_last_change_ms = now;
  }

  if ((now - s_last_change_ms) >= MENU_DEBOUNCE_MS)
  {
    uint8_t prev = s_stable;
    if (prev != s_raw)
    {
      s_stable = s_raw;
      return (uint8_t)(s_stable & (uint8_t)~prev);
    }
  }

  return 0U;
}

static void draw_text_line(uint16_t y, const char *text, uint16_t fg, uint16_t bg)
{
  char buf[24];
  (void)snprintf(buf, sizeof(buf), "%-12s", text);
  LCD_ShowCharStr(0U, y, 80U, buf, bg, fg, 16U);
}

static void render_menu(void)
{
  char buf[24];
  uint8_t mode = s_menu_mode;
  uint8_t level = (mode <= 9U) ? LedCtrl_GetLevel(mode) : 0U;
  uint16_t bg = BLACK;
  uint16_t fg = WHITE;
  uint16_t sel_bg = YELLOW;
  uint16_t sel_fg = BLUE;

  lcd_clear(bg);

  (void)snprintf(buf, sizeof(buf), "MODE: %u", mode);
  draw_text_line(0U, buf, (s_selected == 0U) ? sel_fg : fg, (s_selected == 0U) ? sel_bg : bg);

  if (mode <= 9U)
  {
    (void)snprintf(buf, sizeof(buf), "LEVEL: %u", level);
  }
  else
  {
    (void)snprintf(buf, sizeof(buf), "LEVEL: --");
  }
  draw_text_line(16U, buf, (s_selected == 1U) ? sel_fg : fg, (s_selected == 1U) ? sel_bg : bg);

  draw_text_line(32U, "SAVE", (s_selected == 2U) ? sel_fg : fg, (s_selected == 2U) ? sel_bg : bg);

  if (mode == 10U)
  {
    (void)snprintf(buf, sizeof(buf), "STREAM: %s", s_stream_active ? "ON" : "OFF");
  }
  else
  {
    (void)snprintf(buf, sizeof(buf), "STREAM: --");
  }
  draw_text_line(48U, buf, (s_selected == 3U) ? sel_fg : fg, (s_selected == 3U) ? sel_bg : bg);

  (void)snprintf(buf, sizeof(buf), "DIRTY:%s", LedCtrl_IsDirty() ? "Y" : "N");
  draw_text_line(80U, buf, fg, bg);
}

static void stream_set_active(uint8_t active)
{
  if (active != 0U)
  {
    if (s_stream_active == 0U)
    {
      s_selected = 0U;
      LedCtrl_SetOutputEnabled(0U);
      UsbStreamRx_Reset();
      s_stream_active = 1U;
    }
  }
  else if (s_stream_active != 0U)
  {
    LedCtrl_SetOutputEnabled(1U);
    s_stream_active = 0U;
  }
}

static void menu_set_mode(uint8_t mode)
{
  uint8_t prev = s_menu_mode;
  if (mode < 1U)
  {
    mode = 1U;
  }
  if (mode > 10U)
  {
    mode = 10U;
  }
  if (mode == s_menu_mode)
  {
    return;
  }

  s_menu_mode = mode;
  if (mode != 10U)
  {
    LedCtrl_SetMode(mode);
    LedCtrl_RequestSave();
  }
  else
  {
    stream_set_active(1U);
  }
  if ((prev == 10U) && (mode != 10U))
  {
    stream_set_active(0U);
  }
  s_dirty = 1U;
}

static void menu_apply_delta(int8_t delta)
{
  uint8_t mode = s_menu_mode;
  uint8_t level = (mode <= 9U) ? LedCtrl_GetLevel(mode) : 0U;

  if (s_selected == 0U)
  {
    int16_t next = (int16_t)mode + delta;
    menu_set_mode((uint8_t)next);
  }
  else if ((s_selected == 1U) && (mode <= 9U))
  {
    int16_t next = (int16_t)level + delta;
    if (next < 1)
    {
      next = 1;
    }
    if (next > 7)
    {
      next = 7;
    }
    if ((uint8_t)next != level)
    {
      LedCtrl_SetLevel(mode, (uint8_t)next);
      LedCtrl_RequestSave();
      s_dirty = 1U;
    }
  }
}

void SimpleMenu_Init(void)
{
  tft_init();
  lcd_clear(BLACK);
  s_selected = 0U;
  s_menu_mode = LedCtrl_GetMode();
  s_stream_active = 0U;
  s_last_render_ms = 0U;
  s_dirty = 1U;
  s_raw = 0U;
  s_stable = 0U;
  s_last_change_ms = HAL_GetTick();
  if (s_menu_mode == 10U)
  {
    stream_set_active(1U);
  }
}

void SimpleMenu_Loop(void)
{
  if (s_stream_active != 0U)
  {
    UsbStreamRx_Poll();
  }

  uint8_t edges = buttons_poll_edges();
  if (edges & (1U << BTN_UP))
  {
    s_selected = (s_selected == 0U) ? 3U : (uint8_t)(s_selected - 1U);
    s_dirty = 1U;
  }
  if (edges & (1U << BTN_DOWN))
  {
    s_selected = (uint8_t)((s_selected + 1U) % 4U);
    s_dirty = 1U;
  }
  if (edges & (1U << BTN_LEFT))
  {
    menu_apply_delta(-1);
  }
  if (edges & (1U << BTN_RIGHT))
  {
    menu_apply_delta(1);
  }
  if (edges & (1U << BTN_CENTER))
  {
    if (s_menu_mode == 10U)
    {
      stream_set_active((s_stream_active == 0U) ? 1U : 0U);
      s_dirty = 1U;
    }
    else if (s_selected == 2U)
    {
      LedCtrl_SaveNow();
      s_dirty = 1U;
    }
    else if (s_selected == 3U)
    {
      if (s_menu_mode == 10U)
      {
        stream_set_active((s_stream_active == 0U) ? 1U : 0U);
        s_dirty = 1U;
      }
    }
  }

  uint32_t now = HAL_GetTick();
  if ((s_stream_active == 0U) && (s_dirty != 0U) && (now - s_last_render_ms) >= MENU_RENDER_THROTTLE_MS)
  {
    s_dirty = 0U;
    s_last_render_ms = now;
    render_menu();
  }
}
