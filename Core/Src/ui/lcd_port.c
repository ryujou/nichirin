#include "ui/lcd_port.h"
#include "st7735.h"

#ifndef LCD_PORT_COLOR_SWAP
#define LCD_PORT_COLOR_SWAP 0
#endif

static lv_display_t *s_disp = NULL;

#define LCD_PORT_BUF_LINES 4U
static lv_color_t s_buf1[ST7735_WIDTH * LCD_PORT_BUF_LINES];
static lv_color_t s_buf2[ST7735_WIDTH * LCD_PORT_BUF_LINES];

static uint8_t *s_flush_ptr = NULL;
static uint32_t s_flush_remaining = 0U;
static volatile uint8_t s_flush_active = 0U;
static uint8_t s_flush_first = 0U;
static volatile uint8_t s_stream_blocked = 0U;

static void lcd_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map);
static void lcd_start_dma_chunk(void);

void LcdPort_Init(void)
{
  s_disp = lv_display_create(ST7735_WIDTH, ST7735_HEIGHT);
  lv_display_set_color_format(s_disp, LV_COLOR_FORMAT_RGB565);
  lv_display_set_buffers(s_disp, s_buf1, s_buf2, sizeof(s_buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_flush_cb(s_disp, lcd_flush_cb);
}

lv_display_t *LcdPort_GetDisplay(void)
{
  return s_disp;
}

static void lcd_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
  if (s_stream_blocked != 0U)
  {
    if (disp != NULL)
    {
      lv_display_flush_ready(disp);
    }
    return;
  }
  uint32_t w = (uint32_t)(area->x2 - area->x1 + 1);
  uint32_t h = (uint32_t)(area->y2 - area->y1 + 1);
  uint32_t len = w * h * sizeof(lv_color_t);

  if (s_flush_active != 0U)
  {
    return;
  }

  ST7735_SetAddrWindow((uint16_t)area->x1, (uint16_t)area->y1,
                       (uint16_t)area->x2, (uint16_t)area->y2);

  s_flush_ptr = px_map;
  s_flush_remaining = len;
  s_flush_active = 1U;
  s_flush_first = 1U;
  (void)disp;
  lcd_start_dma_chunk();
}

static void lcd_swap_bytes(uint8_t *buf, uint32_t len)
{
  for (uint32_t i = 0U; i + 1U < len; i += 2U)
  {
    uint8_t tmp = buf[i];
    buf[i] = buf[i + 1U];
    buf[i + 1U] = tmp;
  }
}

static void lcd_start_dma_chunk(void)
{
  uint32_t chunk = s_flush_remaining;
  if (chunk > 65535U)
  {
    chunk = 65535U;
  }

  if (LCD_PORT_COLOR_SWAP)
  {
    lcd_swap_bytes(s_flush_ptr, chunk);
  }

  if (s_flush_first != 0U)
  {
    s_flush_first = 0U;
    if (!ST7735_WritePixels_DMA(s_flush_ptr, (uint16_t)chunk))
    {
      s_flush_active = 0U;
      if (s_disp != NULL)
      {
        lv_display_flush_ready(s_disp);
      }
      return;
    }
  }
  else if (!ST7735_WritePixels_DMA_Continue(s_flush_ptr, (uint16_t)chunk))
  {
    s_flush_active = 0U;
    if (s_disp != NULL)
    {
      lv_display_flush_ready(s_disp);
    }
    return;
  }

  s_flush_ptr += chunk;
  s_flush_remaining -= chunk;
}

void LcdPort_OnDmaTxComplete(void)
{
  ST7735_OnSpiTxDmaDone();

  if (s_flush_active == 0U)
  {
    return;
  }

  if (s_flush_remaining > 0U)
  {
    lcd_start_dma_chunk();
    return;
  }

  s_flush_active = 0U;
  if (s_disp != NULL)
  {
    lv_display_flush_ready(s_disp);
  }
}

void LcdPort_SetStreamingActive(uint8_t active)
{
  s_stream_blocked = (active != 0U) ? 1U : 0U;
}

uint8_t LcdPort_IsBusy(void)
{
  return s_flush_active;
}
