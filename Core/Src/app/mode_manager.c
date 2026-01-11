#include "app/mode_manager.h"
#include "led/led_ctrl.h"
#include "stream/lcd_streamer.h"
#include "stream/spectrum_led.h"
#include "stream/usb_stream_rx.h"
#include "ui/lcd_port.h"
#include "lvgl.h"

#define MODE_STREAM 10U

static uint8_t s_stream_active = 0U;
static uint8_t s_stream_pending = 0U;
static uint8_t s_stream_delay = 0U;
static uint8_t s_last_led_mode = 1U;

void ModeManager_Init(void)
{
  s_stream_active = 0U;
  s_stream_pending = 0U;
  s_stream_delay = 0U;
  s_last_led_mode = LedCtrl_GetMode();
  LcdPort_SetStreamingActive(0U);
  LcdStreamer_Init();
  SpectrumLed_Init();
}

void ModeManager_SetLedMode(uint8_t mode_1_9)
{
  if (s_stream_active != 0U)
  {
    ModeManager_ExitStream();
  }
  uint8_t prev = LedCtrl_GetMode();
  LedCtrl_SetMode(mode_1_9);
  s_last_led_mode = LedCtrl_GetMode();
  if (s_last_led_mode != prev)
  {
    LedCtrl_RequestSave();
  }
}

uint8_t ModeManager_GetMode(void)
{
  return (s_stream_active != 0U) ? MODE_STREAM : LedCtrl_GetMode();
}

uint8_t ModeManager_GetLedMode(void)
{
  return (s_stream_active != 0U) ? s_last_led_mode : LedCtrl_GetMode();
}

bool ModeManager_IsStreamActive(void)
{
  return (s_stream_active != 0U);
}

void ModeManager_EnterStream(void)
{
  if (s_stream_active != 0U)
  {
    return;
  }
  s_last_led_mode = LedCtrl_GetMode();
  s_stream_active = 1U;
  s_stream_pending = 1U;
  s_stream_delay = 1U;
  UsbStreamRx_Reset();
  SpectrumLed_SetActive(1U);
  LedCtrl_SetOutputEnabled(0U);
  lv_obj_invalidate(lv_screen_active());
}

void ModeManager_ExitStream(void)
{
  if (s_stream_active == 0U)
  {
    return;
  }
  s_stream_active = 0U;
  s_stream_pending = 0U;
  s_stream_delay = 0U;
  LcdStreamer_Stop();
  LcdPort_SetStreamingActive(0U);
  SpectrumLed_SetActive(0U);
  LedCtrl_SetOutputEnabled(1U);
  lv_obj_invalidate(lv_screen_active());
}

void ModeManager_Poll(void)
{
  if ((s_stream_active != 0U) && (s_stream_pending != 0U))
  {
    if (s_stream_delay != 0U)
    {
      s_stream_delay = 0U;
      return;
    }
    if (LcdPort_IsBusy() == 0U)
    {
      LcdPort_SetStreamingActive(1U);
      LcdStreamer_Start();
      s_stream_pending = 0U;
    }
  }
}
