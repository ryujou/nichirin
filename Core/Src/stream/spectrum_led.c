#include "stream/spectrum_led.h"
#include "led/led_engine.h"
#include <string.h>

static uint8_t s_active = 0U;
static uint8_t s_last_val[SPECTRUM_LED_COUNT];
static uint8_t s_last_peak[SPECTRUM_LED_COUNT];

void SpectrumLed_Init(void)
{
  s_active = 0U;
  memset(s_last_val, 0, sizeof(s_last_val));
  memset(s_last_peak, 0, sizeof(s_last_peak));
}

void SpectrumLed_SetActive(uint8_t active)
{
  s_active = (active != 0U) ? 1U : 0U;
}

bool SpectrumLed_IsActive(void)
{
  return (s_active != 0U);
}

void SpectrumLed_OnBands(const uint8_t *bands_val, const uint8_t *bands_peak)
{
  if (bands_val == NULL)
  {
    return;
  }

  memcpy(s_last_val, bands_val, sizeof(s_last_val));
  if (bands_peak != NULL)
  {
    memcpy(s_last_peak, bands_peak, sizeof(s_last_peak));
  }
  else
  {
    memset(s_last_peak, 0, sizeof(s_last_peak));
  }

  if (s_active != 0U)
  {
    uint8_t cmd[SPECTRUM_LED_COUNT];
    memcpy(cmd, s_last_val, sizeof(cmd));
    LedEngine_UpdateCmd(cmd);
  }
}

void SpectrumLed_GetLastBands(uint8_t out_val[SPECTRUM_LED_COUNT],
                              uint8_t out_peak[SPECTRUM_LED_COUNT])
{
  if (out_val != NULL)
  {
    memcpy(out_val, s_last_val, sizeof(s_last_val));
  }
  if (out_peak != NULL)
  {
    memcpy(out_peak, s_last_peak, sizeof(s_last_peak));
  }
}
