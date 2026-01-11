#include "ui/ui_led_menu.h"
#include "ui/keypad_port.h"
#include "app/mode_manager.h"
#include "led/led_ctrl.h"
#include "stream/lcd_streamer.h"
#include "lvgl.h"
#include <stdio.h>

typedef enum
{
  UI_PANEL_HOME = 0,
  UI_PANEL_MENU = 1,
  UI_PANEL_MODE = 2,
  UI_PANEL_LEVEL = 3,
  UI_PANEL_STREAM = 4
} UiPanel;

static lv_obj_t *s_panel_home = NULL;
static lv_obj_t *s_panel_menu = NULL;
static lv_obj_t *s_panel_mode = NULL;
static lv_obj_t *s_panel_level = NULL;
static lv_obj_t *s_panel_stream = NULL;

static lv_obj_t *s_label_mode_home = NULL;
static lv_obj_t *s_label_level_home = NULL;
static lv_obj_t *s_label_dirty_home = NULL;

static lv_obj_t *s_label_mode_menu = NULL;
static lv_obj_t *s_label_level_menu = NULL;
static lv_obj_t *s_label_dirty_menu = NULL;

static lv_obj_t *s_label_mode_select = NULL;
static lv_obj_t *s_label_level_value = NULL;
static lv_obj_t *s_label_stream_seq = NULL;
static lv_obj_t *s_label_stream_state = NULL;
static lv_obj_t *s_label_stream_fifo = NULL;

static lv_obj_t *s_btn_menu = NULL;
static lv_obj_t *s_btn_mode = NULL;
static lv_obj_t *s_btn_level = NULL;
static lv_obj_t *s_btn_stream = NULL;
static lv_obj_t *s_btn_save = NULL;
static lv_obj_t *s_btn_back = NULL;
static lv_obj_t *s_btn_mode_select = NULL;
static lv_obj_t *s_btn_level_adjust = NULL;
static lv_obj_t *s_btn_stream_stop = NULL;

static uint8_t s_mode_select = 1U;
static UiPanel s_current_panel = UI_PANEL_MENU;

static void UiLedMenu_UpdateStatus(void)
{
  char buf[32];
  uint8_t mode = ModeManager_GetMode();
  uint8_t level = 0U;
  const char *dirty = LedCtrl_IsDirty() ? "YES" : "NO";

  (void)snprintf(buf, sizeof(buf), "Mode: %u", mode);
  if (s_label_mode_home != NULL)
  {
    lv_label_set_text(s_label_mode_home, buf);
  }
  if (s_label_mode_menu != NULL)
  {
    lv_label_set_text(s_label_mode_menu, buf);
  }

  if (mode == 10U)
  {
    (void)snprintf(buf, sizeof(buf), "Level: --");
  }
  else
  {
    level = LedCtrl_GetLevel(mode);
    (void)snprintf(buf, sizeof(buf), "Level: %u", level);
  }
  if (s_label_level_home != NULL)
  {
    lv_label_set_text(s_label_level_home, buf);
  }
  if (s_label_level_menu != NULL)
  {
    lv_label_set_text(s_label_level_menu, buf);
  }

  (void)snprintf(buf, sizeof(buf), "Dirty: %s", dirty);
  if (s_label_dirty_home != NULL)
  {
    lv_label_set_text(s_label_dirty_home, buf);
  }
  if (s_label_dirty_menu != NULL)
  {
    lv_label_set_text(s_label_dirty_menu, buf);
  }
}

static void UiLedMenu_UpdateModeSelect(void)
{
  char buf[32];
  (void)snprintf(buf, sizeof(buf), "Select: %u", s_mode_select);
  if (s_label_mode_select != NULL)
  {
    lv_label_set_text(s_label_mode_select, buf);
  }
}

static void UiLedMenu_UpdateLevelValue(uint8_t level)
{
  char buf[32];
  (void)snprintf(buf, sizeof(buf), "Level: %u", level);
  if (s_label_level_value != NULL)
  {
    lv_label_set_text(s_label_level_value, buf);
  }
}

static void UiLedMenu_UpdateStreamStats(void)
{
  if (s_label_stream_seq == NULL)
  {
    return;
  }

  char buf[32];
  uint16_t seq = LcdStreamer_GetLastSeqOk();
  uint16_t fifo = LcdStreamer_GetFifoFill();
  const char *state = LcdStreamer_IsNoSignal() ? "No signal" : "OK";

  (void)snprintf(buf, sizeof(buf), "Seq: %u", seq);
  lv_label_set_text(s_label_stream_seq, buf);

  (void)snprintf(buf, sizeof(buf), "Status: %s", state);
  if (s_label_stream_state != NULL)
  {
    lv_label_set_text(s_label_stream_state, buf);
  }

  (void)snprintf(buf, sizeof(buf), "FIFO: %u/%u", fifo, (unsigned)LCD_STREAMER_FIFO_SIZE);
  if (s_label_stream_fifo != NULL)
  {
    lv_label_set_text(s_label_stream_fifo, buf);
  }
}

static void UiLedMenu_SetGroupSingle(lv_obj_t *obj)
{
  lv_group_t *group = KeypadPort_GetGroup();
  if (group == NULL)
  {
    return;
  }
  lv_group_remove_all_objs(group);
  if (obj != NULL)
  {
    lv_group_add_obj(group, obj);
    lv_group_focus_obj(obj);
  }
}

static void UiLedMenu_SetGroupMenu(void)
{
  lv_group_t *group = KeypadPort_GetGroup();
  if (group == NULL)
  {
    return;
  }
  lv_group_remove_all_objs(group);
  if (s_btn_mode != NULL)
  {
    lv_group_add_obj(group, s_btn_mode);
  }
  if (s_btn_level != NULL)
  {
    lv_group_add_obj(group, s_btn_level);
  }
  if (s_btn_stream != NULL)
  {
    lv_group_add_obj(group, s_btn_stream);
  }
  if (s_btn_save != NULL)
  {
    lv_group_add_obj(group, s_btn_save);
  }
  if (s_btn_back != NULL)
  {
    lv_group_add_obj(group, s_btn_back);
  }
  if (s_btn_mode != NULL)
  {
    lv_group_focus_obj(s_btn_mode);
  }
}

static void UiLedMenu_ShowPanel(UiPanel panel)
{
  if (s_panel_home != NULL)
  {
    lv_obj_add_flag(s_panel_home, LV_OBJ_FLAG_HIDDEN);
  }
  if (s_panel_menu != NULL)
  {
    lv_obj_add_flag(s_panel_menu, LV_OBJ_FLAG_HIDDEN);
  }
  if (s_panel_mode != NULL)
  {
    lv_obj_add_flag(s_panel_mode, LV_OBJ_FLAG_HIDDEN);
  }
  if (s_panel_level != NULL)
  {
    lv_obj_add_flag(s_panel_level, LV_OBJ_FLAG_HIDDEN);
  }
  if (s_panel_stream != NULL)
  {
    lv_obj_add_flag(s_panel_stream, LV_OBJ_FLAG_HIDDEN);
  }

  UiLedMenu_UpdateStatus();
  s_current_panel = panel;

  switch (panel)
  {
    case UI_PANEL_HOME:
      lv_obj_clear_flag(s_panel_home, LV_OBJ_FLAG_HIDDEN);
      UiLedMenu_SetGroupSingle(s_btn_menu);
      break;
    case UI_PANEL_MENU:
      lv_obj_clear_flag(s_panel_menu, LV_OBJ_FLAG_HIDDEN);
      UiLedMenu_SetGroupMenu();
      break;
    case UI_PANEL_MODE:
      s_mode_select = ModeManager_GetLedMode();
      UiLedMenu_UpdateModeSelect();
      lv_obj_clear_flag(s_panel_mode, LV_OBJ_FLAG_HIDDEN);
      UiLedMenu_SetGroupSingle(s_btn_mode_select);
      break;
    case UI_PANEL_LEVEL:
      {
        uint8_t mode = LedCtrl_GetMode();
        uint8_t level = LedCtrl_GetLevel(mode);
        UiLedMenu_UpdateLevelValue(level);
      }
      lv_obj_clear_flag(s_panel_level, LV_OBJ_FLAG_HIDDEN);
      UiLedMenu_SetGroupSingle(s_btn_level_adjust);
      break;
    case UI_PANEL_STREAM:
      lv_obj_clear_flag(s_panel_stream, LV_OBJ_FLAG_HIDDEN);
      UiLedMenu_SetGroupSingle(s_btn_stream_stop);
      UiLedMenu_UpdateStreamStats();
      break;
    default:
      break;
  }
}

static void UiLedMenu_StatusTimerCb(lv_timer_t *timer)
{
  (void)timer;
  if (ModeManager_IsStreamActive())
  {
    if (s_current_panel != UI_PANEL_STREAM)
    {
      UiLedMenu_ShowPanel(UI_PANEL_STREAM);
      return;
    }
    UiLedMenu_UpdateStreamStats();
  }
  else if (s_current_panel == UI_PANEL_STREAM)
  {
    UiLedMenu_ShowPanel(UI_PANEL_MENU);
    return;
  }
  UiLedMenu_UpdateStatus();
}

static void UiLedMenu_MenuBtnCb(lv_event_t *e)
{
  (void)e;
  UiLedMenu_ShowPanel(UI_PANEL_MENU);
}

static void UiLedMenu_ModeBtnCb(lv_event_t *e)
{
  (void)e;
  UiLedMenu_ShowPanel(UI_PANEL_MODE);
}

static void UiLedMenu_LevelBtnCb(lv_event_t *e)
{
  (void)e;
  if (ModeManager_IsStreamActive())
  {
    return;
  }
  UiLedMenu_ShowPanel(UI_PANEL_LEVEL);
}

static void UiLedMenu_StreamBtnCb(lv_event_t *e)
{
  (void)e;
  ModeManager_EnterStream();
  UiLedMenu_ShowPanel(UI_PANEL_STREAM);
}

static void UiLedMenu_SaveBtnCb(lv_event_t *e)
{
  (void)e;
  LedCtrl_SaveNow();
  UiLedMenu_UpdateStatus();
}

static void UiLedMenu_BackBtnCb(lv_event_t *e)
{
  (void)e;
  UiLedMenu_ShowPanel(UI_PANEL_HOME);
}

static void UiLedMenu_ModeSelectKeyCb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_KEY)
  {
    return;
  }

  uint32_t key = lv_event_get_key(e);
  if (key == LV_KEY_UP)
  {
    s_mode_select = (s_mode_select < 9U) ? (uint8_t)(s_mode_select + 1U) : 1U;
    UiLedMenu_UpdateModeSelect();
  }
  else if (key == LV_KEY_DOWN)
  {
    s_mode_select = (s_mode_select > 1U) ? (uint8_t)(s_mode_select - 1U) : 9U;
    UiLedMenu_UpdateModeSelect();
  }
  else if (key == LV_KEY_ENTER)
  {
    ModeManager_SetLedMode(s_mode_select);
    UiLedMenu_ShowPanel(UI_PANEL_MENU);
  }
}

static void UiLedMenu_LevelAdjustKeyCb(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_KEY)
  {
    return;
  }

  uint32_t key = lv_event_get_key(e);
  if ((key == LV_KEY_LEFT) || (key == LV_KEY_RIGHT) || (key == LV_KEY_UP) || (key == LV_KEY_DOWN))
  {
    uint8_t mode = LedCtrl_GetMode();
    uint8_t level = LedCtrl_GetLevel(mode);
    if ((key == LV_KEY_RIGHT) || (key == LV_KEY_UP))
    {
      level = (level < 7U) ? (uint8_t)(level + 1U) : 1U;
    }
    else
    {
      level = (level > 1U) ? (uint8_t)(level - 1U) : 7U;
    }
    LedCtrl_SetLevel(mode, level);
    LedCtrl_RequestSave();
    UiLedMenu_UpdateLevelValue(level);
  }
  else if (key == LV_KEY_ENTER)
  {
    UiLedMenu_ShowPanel(UI_PANEL_MENU);
  }
}

static void UiLedMenu_StreamStopKeyCb(lv_event_t *e)
{
  if (lv_event_get_code(e) == LV_EVENT_KEY)
  {
    uint32_t key = lv_event_get_key(e);
    if (key == LV_KEY_ENTER)
    {
      ModeManager_ExitStream();
      UiLedMenu_ShowPanel(UI_PANEL_MENU);
    }
  }
  else if (lv_event_get_code(e) == LV_EVENT_CLICKED)
  {
    ModeManager_ExitStream();
    UiLedMenu_ShowPanel(UI_PANEL_MENU);
  }
}

void UiLedMenu_Init(void)
{
  lv_obj_t *scr = lv_screen_active();

  s_panel_home = lv_obj_create(scr);
  lv_obj_set_size(s_panel_home, 80, 160);

  s_label_mode_home = lv_label_create(s_panel_home);
  lv_obj_set_pos(s_label_mode_home, 4, 4);
  s_label_level_home = lv_label_create(s_panel_home);
  lv_obj_set_pos(s_label_level_home, 4, 20);
  s_label_dirty_home = lv_label_create(s_panel_home);
  lv_obj_set_pos(s_label_dirty_home, 4, 36);

  s_btn_menu = lv_button_create(s_panel_home);
  lv_obj_set_size(s_btn_menu, 60, 28);
  lv_obj_set_pos(s_btn_menu, 10, 90);
  lv_obj_add_event_cb(s_btn_menu, UiLedMenu_MenuBtnCb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *menu_label = lv_label_create(s_btn_menu);
  lv_label_set_text(menu_label, "Menu");
  lv_obj_center(menu_label);

  s_panel_menu = lv_obj_create(scr);
  lv_obj_set_size(s_panel_menu, 80, 160);
  lv_obj_add_flag(s_panel_menu, LV_OBJ_FLAG_HIDDEN);

  s_label_mode_menu = lv_label_create(s_panel_menu);
  lv_obj_set_pos(s_label_mode_menu, 4, 4);
  s_label_level_menu = lv_label_create(s_panel_menu);
  lv_obj_set_pos(s_label_level_menu, 4, 20);
  s_label_dirty_menu = lv_label_create(s_panel_menu);
  lv_obj_set_pos(s_label_dirty_menu, 4, 36);

  s_btn_mode = lv_button_create(s_panel_menu);
  lv_obj_set_size(s_btn_mode, 70, 20);
  lv_obj_set_pos(s_btn_mode, 5, 50);
  lv_obj_add_event_cb(s_btn_mode, UiLedMenu_ModeBtnCb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *mode_label = lv_label_create(s_btn_mode);
  lv_label_set_text(mode_label, "Mode");
  lv_obj_center(mode_label);

  s_btn_level = lv_button_create(s_panel_menu);
  lv_obj_set_size(s_btn_level, 70, 20);
  lv_obj_set_pos(s_btn_level, 5, 72);
  lv_obj_add_event_cb(s_btn_level, UiLedMenu_LevelBtnCb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *level_label = lv_label_create(s_btn_level);
  lv_label_set_text(level_label, "Level");
  lv_obj_center(level_label);

  s_btn_stream = lv_button_create(s_panel_menu);
  lv_obj_set_size(s_btn_stream, 70, 20);
  lv_obj_set_pos(s_btn_stream, 5, 94);
  lv_obj_add_event_cb(s_btn_stream, UiLedMenu_StreamBtnCb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *stream_label = lv_label_create(s_btn_stream);
  lv_label_set_text(stream_label, "Mode 10");
  lv_obj_center(stream_label);

  s_btn_save = lv_button_create(s_panel_menu);
  lv_obj_set_size(s_btn_save, 70, 20);
  lv_obj_set_pos(s_btn_save, 5, 116);
  lv_obj_add_event_cb(s_btn_save, UiLedMenu_SaveBtnCb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *save_label = lv_label_create(s_btn_save);
  lv_label_set_text(save_label, "Save Now");
  lv_obj_center(save_label);

  s_btn_back = lv_button_create(s_panel_menu);
  lv_obj_set_size(s_btn_back, 70, 20);
  lv_obj_set_pos(s_btn_back, 5, 138);
  lv_obj_add_event_cb(s_btn_back, UiLedMenu_BackBtnCb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *back_label = lv_label_create(s_btn_back);
  lv_label_set_text(back_label, "Back");
  lv_obj_center(back_label);

  s_panel_mode = lv_obj_create(scr);
  lv_obj_set_size(s_panel_mode, 80, 160);
  lv_obj_add_flag(s_panel_mode, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t *mode_title = lv_label_create(s_panel_mode);
  lv_label_set_text(mode_title, "Select Mode");
  lv_obj_set_pos(mode_title, 4, 4);

  s_label_mode_select = lv_label_create(s_panel_mode);
  lv_obj_set_pos(s_label_mode_select, 4, 28);

  s_btn_mode_select = lv_button_create(s_panel_mode);
  lv_obj_set_size(s_btn_mode_select, 70, 24);
  lv_obj_set_pos(s_btn_mode_select, 5, 90);
  lv_obj_add_event_cb(s_btn_mode_select, UiLedMenu_ModeSelectKeyCb, LV_EVENT_KEY, NULL);
  lv_obj_t *mode_hint = lv_label_create(s_btn_mode_select);
  lv_label_set_text(mode_hint, "UP/DN OK");
  lv_obj_center(mode_hint);

  s_panel_level = lv_obj_create(scr);
  lv_obj_set_size(s_panel_level, 80, 160);
  lv_obj_add_flag(s_panel_level, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t *level_title = lv_label_create(s_panel_level);
  lv_label_set_text(level_title, "Adjust Level");
  lv_obj_set_pos(level_title, 4, 4);

  s_label_level_value = lv_label_create(s_panel_level);
  lv_obj_set_pos(s_label_level_value, 4, 28);

  s_btn_level_adjust = lv_button_create(s_panel_level);
  lv_obj_set_size(s_btn_level_adjust, 70, 24);
  lv_obj_set_pos(s_btn_level_adjust, 5, 90);
  lv_obj_add_event_cb(s_btn_level_adjust, UiLedMenu_LevelAdjustKeyCb, LV_EVENT_KEY, NULL);
  lv_obj_t *level_hint = lv_label_create(s_btn_level_adjust);
  lv_label_set_text(level_hint, "L/R OK");
  lv_obj_center(level_hint);

  s_panel_stream = lv_obj_create(scr);
  lv_obj_set_size(s_panel_stream, 80, 160);
  lv_obj_add_flag(s_panel_stream, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t *stream_title = lv_label_create(s_panel_stream);
  lv_label_set_text(stream_title, "Streaming...");
  lv_obj_set_pos(stream_title, 4, 10);

  s_label_stream_seq = lv_label_create(s_panel_stream);
  lv_obj_set_pos(s_label_stream_seq, 4, 32);

  s_label_stream_state = lv_label_create(s_panel_stream);
  lv_obj_set_pos(s_label_stream_state, 4, 48);

  s_label_stream_fifo = lv_label_create(s_panel_stream);
  lv_obj_set_pos(s_label_stream_fifo, 4, 64);

  s_btn_stream_stop = lv_button_create(s_panel_stream);
  lv_obj_set_size(s_btn_stream_stop, 70, 28);
  lv_obj_set_pos(s_btn_stream_stop, 5, 90);
  lv_obj_add_event_cb(s_btn_stream_stop, UiLedMenu_StreamStopKeyCb, LV_EVENT_CLICKED, NULL);
  lv_obj_add_event_cb(s_btn_stream_stop, UiLedMenu_StreamStopKeyCb, LV_EVENT_KEY, NULL);
  lv_obj_t *stop_label = lv_label_create(s_btn_stream_stop);
  lv_label_set_text(stop_label, "Stop");
  lv_obj_center(stop_label);

  UiLedMenu_ShowPanel(UI_PANEL_MENU);
  lv_timer_create(UiLedMenu_StatusTimerCb, 200, NULL);
}
