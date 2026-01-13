#include "stream/lcd_streamer.h"
#include "stream/video_proto.h"
#include "lcd_st7735.h"
#include "st7735.h"
#include "main.h"
#include <stdbool.h>
#include <string.h>

#ifndef LCD_STREAMER_RGB332_BLOCK_LINES
#define LCD_STREAMER_RGB332_BLOCK_LINES 4U
#endif

#ifndef LCD_STREAMER_RGB332_DOUBLE_BUFFER
#define LCD_STREAMER_RGB332_DOUBLE_BUFFER 1U
#endif

#define LCD_STREAMER_RGB332_IN_CHUNK_BYTES  (ST7735_WIDTH * LCD_STREAMER_RGB332_BLOCK_LINES)
#define LCD_STREAMER_RGB332_OUT_CHUNK_BYTES (LCD_STREAMER_RGB332_IN_CHUNK_BYTES * 2U)

#ifndef LCD_STREAMER_RGB4_ENABLE
#define LCD_STREAMER_RGB4_ENABLE 0U
#endif

#ifndef LCD_STREAMER_RGB4_BLOCK_LINES
#define LCD_STREAMER_RGB4_BLOCK_LINES 8U
#endif

#ifndef LCD_STREAMER_RGB4_DOUBLE_BUFFER
#define LCD_STREAMER_RGB4_DOUBLE_BUFFER 1U
#endif

#if (LCD_STREAMER_RGB4_ENABLE != 0)
#define LCD_STREAMER_RGB4_FRAME_BYTES ((ST7735_WIDTH * ST7735_HEIGHT) / 2U)
#define LCD_STREAMER_RGB4_OUT_BLOCK_BYTES (ST7735_WIDTH * 2U * LCD_STREAMER_RGB4_BLOCK_LINES)
#endif

#define LCD_STREAMER_TIMEOUT_MS 800U

typedef struct
{
  uint8_t buf[LCD_STREAMER_FIFO_SIZE];
  uint16_t head;
  uint16_t tail;
} ByteFifo;

static ByteFifo s_fifo;
static volatile uint8_t s_active = 0U;
static volatile uint8_t s_dma_busy = 0U;
static uint8_t s_need_window = 0U;
static uint8_t s_first_chunk = 1U;
static uint8_t s_frame_active = 0U;
static uint8_t s_drop_frame = 0U;
static uint8_t s_no_signal = 0U;
static uint16_t s_current_seq = 0U;
static uint16_t s_last_seq_ok = 0U;
static uint8_t s_frame_width = ST7735_WIDTH;
static uint8_t s_frame_height = ST7735_HEIGHT;
static uint8_t s_pixfmt = VIDEO_PROTO_PIXFMT_RGB565;
static uint8_t s_debug_marked = 0U;
static uint32_t s_frame_remaining = 0U; /* remaining bytes to consume from RX stream (RGB565: bytes, RGB332: pixels/bytes) */
static uint32_t s_frame_total = 0U;
static uint32_t s_received_total = 0U;
static uint32_t s_last_rx_ms = 0U;
static uint16_t s_last_dma_consume_len = 0U;

static uint16_t s_rgb332_to_565[256];
static uint8_t s_rgb332_lut_ready = 0U;

static uint8_t s_rgb332_dma_buf_a[LCD_STREAMER_RGB332_OUT_CHUNK_BYTES];
#if (LCD_STREAMER_RGB332_DOUBLE_BUFFER != 0)
static uint8_t s_rgb332_dma_buf_b[LCD_STREAMER_RGB332_OUT_CHUNK_BYTES];
static uint8_t s_rgb332_dma_buf_sel = 0U;
#endif

static void LcdStreamer_Rgb332InitLut(void)
{
  if (s_rgb332_lut_ready != 0U)
  {
    return;
  }

  for (uint16_t i = 0U; i < 256U; i++)
  {
    uint8_t pix = (uint8_t)i;
    uint8_t r3 = (uint8_t)((pix >> 5) & 0x07U);
    uint8_t g3 = (uint8_t)((pix >> 2) & 0x07U);
    uint8_t b2 = (uint8_t)(pix & 0x03U);

    uint8_t r5 = (uint8_t)((r3 << 2) | (r3 >> 1));
    uint8_t g6 = (uint8_t)((g3 << 3) | g3);
    uint8_t b5 = (uint8_t)((b2 << 3) | (b2 << 1) | (b2 >> 1));

    s_rgb332_to_565[i] = (uint16_t)(((uint16_t)r5 << 11) | ((uint16_t)g6 << 5) | (uint16_t)b5);
  }
  s_rgb332_lut_ready = 1U;
}

static uint8_t *LcdStreamer_GetRgb332DmaBuf(void)
{
#if (LCD_STREAMER_RGB332_DOUBLE_BUFFER != 0)
  if (s_rgb332_dma_buf_sel == 0U)
  {
    return s_rgb332_dma_buf_a;
  }
  return s_rgb332_dma_buf_b;
#else
  return s_rgb332_dma_buf_a;
#endif
}

static void LcdStreamer_SwapRgb332DmaBuf(void)
{
#if (LCD_STREAMER_RGB332_DOUBLE_BUFFER != 0)
  s_rgb332_dma_buf_sel ^= 1U;
#endif
}

#if (LCD_STREAMER_RGB4_ENABLE != 0)
typedef enum
{
  LCD4_IDLE = 0U,
  LCD4_RX = 1U,
  LCD4_TX = 2U
} Lcd4State;

static Lcd4State s_lcd4_state = LCD4_IDLE;
static uint32_t s_lcd4_rx_off = 0U;
static uint16_t s_lcd4_line = 0U; /* next line to schedule for TX */
static uint8_t s_lcd4_frame[LCD_STREAMER_RGB4_FRAME_BYTES];
static uint8_t s_lcd4_tx_buf_a[LCD_STREAMER_RGB4_OUT_BLOCK_BYTES];
#if (LCD_STREAMER_RGB4_DOUBLE_BUFFER != 0)
static uint8_t s_lcd4_tx_buf_b[LCD_STREAMER_RGB4_OUT_BLOCK_BYTES];
#endif
static uint8_t s_lcd4_tx_inflight_sel = 0U;
static uint16_t s_lcd4_tx_inflight_lines = 0U;
static uint8_t s_lcd4_tx_prepared = 0U;
static uint8_t s_lcd4_tx_prepared_sel = 0U;
static uint16_t s_lcd4_tx_prepared_start = 0U;
static uint16_t s_lcd4_tx_prepared_lines = 0U;

static const uint16_t s_rgb4_palette_565[16] = {
  0x0000, /* 0: black */
  0x001F, /* 1: blue */
  0x07E0, /* 2: green */
  0x07FF, /* 3: cyan */
  0xF800, /* 4: red */
  0xF81F, /* 5: magenta */
  0xFFE0, /* 6: yellow */
  0xFFFF, /* 7: white */
  0x8410, /* 8: gray */
  0x041F, /* 9: bright blue-ish */
  0x87E0, /* A: bright green-ish */
  0x87FF, /* B: bright cyan-ish */
  0xFC10, /* C: bright red-ish */
  0xFC1F, /* D: bright magenta-ish */
  0xFFE8, /* E: bright yellow-ish */
  0xFFFF  /* F: bright white */
};

static uint8_t *LcdStreamer_Rgb4GetTxBuf(uint8_t sel)
{
#if (LCD_STREAMER_RGB4_DOUBLE_BUFFER != 0)
  if (sel == 0U)
  {
    return s_lcd4_tx_buf_a;
  }
  return s_lcd4_tx_buf_b;
#else
  (void)sel;
  return s_lcd4_tx_buf_a;
#endif
}

static void LcdStreamer_Rgb4RenderBlock(uint16_t y_start, uint16_t lines, uint8_t *dst)
{
  uint32_t out = 0U;
  for (uint16_t y = 0U; y < lines; y++)
  {
    const uint32_t row_start = (uint32_t)(y_start + y) * ST7735_WIDTH;
    for (uint16_t x = 0U; x < ST7735_WIDTH; x++)
    {
      uint32_t pix_index = row_start + x;
      uint8_t packed = s_lcd4_frame[pix_index >> 1U];
      uint8_t idx4 = (uint8_t)((pix_index & 1U) ? (packed & 0x0FU) : (packed >> 4));
      uint16_t c = s_rgb4_palette_565[idx4 & 0x0FU];
      dst[out++] = (uint8_t)(c >> 8);
      dst[out++] = (uint8_t)c;
    }
  }
}
#endif

static uint8_t fifo_is_empty(const ByteFifo *fifo)
{
  return (fifo->head == fifo->tail);
}

static uint16_t fifo_count(const ByteFifo *fifo)
{
  if (fifo->head >= fifo->tail)
  {
    return (uint16_t)(fifo->head - fifo->tail);
  }
  return (uint16_t)(LCD_STREAMER_FIFO_SIZE - fifo->tail + fifo->head);
}

static uint16_t fifo_free(const ByteFifo *fifo)
{
  uint16_t used = fifo_count(fifo);
  if (used >= (LCD_STREAMER_FIFO_SIZE - 1U))
  {
    return 0U;
  }
  return (uint16_t)((LCD_STREAMER_FIFO_SIZE - 1U) - used);
}

static void fifo_clear(ByteFifo *fifo)
{
  fifo->head = 0U;
  fifo->tail = 0U;
}

static uint16_t fifo_write(ByteFifo *fifo, const uint8_t *data, uint16_t len)
{
  uint16_t written = 0U;
  while (written < len)
  {
    uint16_t next = fifo->head + 1U;
    if (next >= LCD_STREAMER_FIFO_SIZE)
    {
      next = 0U;
    }
    if (next == fifo->tail)
    {
      break;
    }
    fifo->buf[fifo->head] = data[written];
    fifo->head = next;
    written++;
  }
  return written;
}

static uint16_t fifo_peek_contig(const ByteFifo *fifo, const uint8_t **out)
{
  if (fifo->head == fifo->tail)
  {
    *out = NULL;
    return 0U;
  }
  if (fifo->head > fifo->tail)
  {
    *out = &fifo->buf[fifo->tail];
    return (uint16_t)(fifo->head - fifo->tail);
  }
  *out = &fifo->buf[fifo->tail];
  return (uint16_t)(LCD_STREAMER_FIFO_SIZE - fifo->tail);
}

static void fifo_skip(ByteFifo *fifo, uint16_t len)
{
  uint16_t count = fifo_count(fifo);
  if (len > count)
  {
    len = count;
  }
  uint16_t idx = fifo->tail;
  idx = (uint16_t)(idx + len);
  if (idx >= LCD_STREAMER_FIFO_SIZE)
  {
    idx = (uint16_t)(idx % LCD_STREAMER_FIFO_SIZE);
  }
  fifo->tail = idx;
}

static void LcdStreamer_ResetFrameState(void)
{
  fifo_clear(&s_fifo);
  s_frame_active = 0U;
  s_drop_frame = 0U;
  s_need_window = 0U;
  s_first_chunk = 1U;
  s_frame_remaining = 0U;
  s_frame_total = 0U;
  s_received_total = 0U;
  s_last_dma_consume_len = 0U;

#if (LCD_STREAMER_RGB4_ENABLE != 0)
  s_lcd4_state = LCD4_IDLE;
  s_lcd4_rx_off = 0U;
  s_lcd4_line = 0U;
#endif
}

static void LcdStreamer_DropFrame(void)
{
  fifo_clear(&s_fifo);
  s_frame_active = 0U;
  s_drop_frame = 1U;
  s_need_window = 0U;
  s_first_chunk = 1U;
  s_frame_remaining = 0U;
  s_frame_total = 0U;
  s_received_total = 0U;
  s_last_dma_consume_len = 0U;

#if (LCD_STREAMER_RGB4_ENABLE != 0)
  s_lcd4_state = LCD4_IDLE;
  s_lcd4_rx_off = 0U;
  s_lcd4_line = 0U;
#endif
}

void LcdStreamer_Init(void)
{
  fifo_clear(&s_fifo);
  s_active = 0U;
  s_dma_busy = 0U;
  s_need_window = 0U;
  s_first_chunk = 1U;
  s_frame_active = 0U;
  s_drop_frame = 0U;
  s_no_signal = 0U;
  s_current_seq = 0U;
  s_last_seq_ok = 0U;
  s_frame_width = ST7735_WIDTH;
  s_frame_height = ST7735_HEIGHT;
  s_pixfmt = VIDEO_PROTO_PIXFMT_RGB565;
  s_debug_marked = 0U;
  s_frame_remaining = 0U;
  s_frame_total = 0U;
  s_received_total = 0U;
  s_last_rx_ms = 0U;
  s_last_dma_consume_len = 0U;
  s_rgb332_lut_ready = 0U;
#if (LCD_STREAMER_RGB332_DOUBLE_BUFFER != 0)
  s_rgb332_dma_buf_sel = 0U;
#endif

#if (LCD_STREAMER_RGB4_ENABLE != 0)
  s_lcd4_state = LCD4_IDLE;
  s_lcd4_rx_off = 0U;
  s_lcd4_line = 0U;
  s_lcd4_tx_inflight_sel = 0U;
  s_lcd4_tx_inflight_lines = 0U;
  s_lcd4_tx_prepared = 0U;
  s_lcd4_tx_prepared_sel = 0U;
  s_lcd4_tx_prepared_start = 0U;
  s_lcd4_tx_prepared_lines = 0U;
  memset(s_lcd4_frame, 0, sizeof(s_lcd4_frame));
#endif
}

void LcdStreamer_Start(void)
{
  fifo_clear(&s_fifo);
  s_active = 1U;
  s_need_window = 0U;
  s_first_chunk = 1U;
  s_frame_active = 0U;
  s_drop_frame = 0U;
  s_no_signal = 0U;
  s_frame_width = ST7735_WIDTH;
  s_frame_height = ST7735_HEIGHT;
  s_pixfmt = VIDEO_PROTO_PIXFMT_RGB565;
  s_debug_marked = 0U;
  s_frame_remaining = 0U;
  s_frame_total = 0U;
  s_received_total = 0U;
  s_last_rx_ms = HAL_GetTick();
  s_last_seq_ok = 0U;
  s_dma_busy = 0U;
  s_last_dma_consume_len = 0U;
#if (LCD_STREAMER_RGB332_DOUBLE_BUFFER != 0)
  s_rgb332_dma_buf_sel = 0U;
#endif

#if (LCD_STREAMER_RGB4_ENABLE != 0)
  s_lcd4_state = LCD4_IDLE;
  s_lcd4_rx_off = 0U;
  s_lcd4_line = 0U;
  s_lcd4_tx_inflight_sel = 0U;
  s_lcd4_tx_inflight_lines = 0U;
  s_lcd4_tx_prepared = 0U;
  s_lcd4_tx_prepared_sel = 0U;
  s_lcd4_tx_prepared_start = 0U;
  s_lcd4_tx_prepared_lines = 0U;
#endif
}

void LcdStreamer_Stop(void)
{
  s_active = 0U;
  s_no_signal = 0U;
  LcdStreamer_ResetFrameState();
}

bool LcdStreamer_IsActive(void)
{
  return (s_active != 0U);
}

void LcdStreamer_OnPacketOk(uint16_t seq)
{
  (void)seq;
  s_last_rx_ms = HAL_GetTick();
}

void LcdStreamer_OnFrameStart(uint16_t seq,
                              uint8_t width,
                              uint8_t height,
                              uint8_t pixfmt,
                              uint16_t frame_bytes,
                              uint8_t flags)
{
  if (s_active == 0U)
  {
    return;
  }

  /* Avoid tearing: do not start a new frame until the previous one is fully flushed.
     With no TE pin, the safest approach is single-frame-at-a-time. */
  if ((s_frame_active != 0U) && (s_drop_frame == 0U))
  {
    if ((s_dma_busy != 0U) || ST7735_DMA_Busy() || (s_frame_remaining != 0U) || (fifo_is_empty(&s_fifo) == 0U))
    {
      return;
    }
  }

#if (LCD_STREAMER_RGB4_ENABLE != 0)
  /* In RGB4 mode we flush line-by-line after a complete frame is received.
     To guarantee a complete on-screen frame, do not interrupt an in-progress flush. */
  if ((s_pixfmt == VIDEO_PROTO_PIXFMT_RGB4) && (s_lcd4_state == LCD4_TX) && (s_lcd4_line < ST7735_HEIGHT))
  {
    return;
  }
#endif
  if (((pixfmt != VIDEO_PROTO_PIXFMT_RGB565) && (pixfmt != VIDEO_PROTO_PIXFMT_RGB332) && (pixfmt != VIDEO_PROTO_PIXFMT_RGB4)) || (width == 0U) || (height == 0U))
  {
    LcdStreamer_DropFrame();
    return;
  }

  if (pixfmt == VIDEO_PROTO_PIXFMT_RGB565)
  {
    if ((uint32_t)frame_bytes != ((uint32_t)width * (uint32_t)height * 2U))
    {
      LcdStreamer_DropFrame();
      return;
    }
  }
  else if (pixfmt == VIDEO_PROTO_PIXFMT_RGB332)
  {
    if ((uint32_t)frame_bytes != ((uint32_t)width * (uint32_t)height))
    {
      LcdStreamer_DropFrame();
      return;
    }
    LcdStreamer_Rgb332InitLut();
  }
  else
  {
#if (LCD_STREAMER_RGB4_ENABLE != 0)
    if ((width != ST7735_WIDTH) || (height != ST7735_HEIGHT) || (frame_bytes != LCD_STREAMER_RGB4_FRAME_BYTES))
    {
      LcdStreamer_DropFrame();
      return;
    }
#else
    LcdStreamer_DropFrame();
    return;
#endif
  }

  if ((flags & VIDEO_PROTO_FLAG_FORCE_CLEAR) != 0U)
  {
    ST7735_FillColor(0x0000U);
  }

  fifo_clear(&s_fifo);
  s_current_seq = seq;
  s_last_seq_ok = seq;
  s_frame_width = width;
  s_frame_height = height;
  s_pixfmt = pixfmt;
  s_frame_total = frame_bytes;
  s_frame_remaining = frame_bytes;
  s_received_total = 0U;
  s_drop_frame = 0U;
  s_frame_active = 1U;
  s_need_window = 1U;
  s_first_chunk = 1U;
  s_no_signal = 0U;
  s_dma_busy = 0U;
  s_last_dma_consume_len = 0U;
#if (LCD_STREAMER_RGB332_DOUBLE_BUFFER != 0)
  s_rgb332_dma_buf_sel = 0U;
#endif

#if (LCD_STREAMER_RGB4_ENABLE != 0)
  if (pixfmt == VIDEO_PROTO_PIXFMT_RGB4)
  {
    s_lcd4_state = LCD4_RX;
    s_lcd4_rx_off = 0U;
    s_lcd4_line = 0U;
    s_lcd4_tx_inflight_sel = 0U;
    s_lcd4_tx_inflight_lines = 0U;
    s_lcd4_tx_prepared = 0U;
    s_lcd4_tx_prepared_sel = 0U;
    s_lcd4_tx_prepared_start = 0U;
    s_lcd4_tx_prepared_lines = 0U;
  }
  else
  {
    s_lcd4_state = LCD4_IDLE;
  }
#endif
  if (s_debug_marked == 0U)
  {
    s_debug_marked = 1U;
    draw_point(0U, 0U, 0xF800U);
  }
}

void LcdStreamer_OnFrameData(uint16_t seq, const uint8_t *data, uint16_t len)
{
  if ((s_active == 0U) || (data == NULL) || (len == 0U))
  {
    return;
  }
  if ((s_frame_active == 0U) || (s_drop_frame != 0U))
  {
    return;
  }
  if (seq != s_current_seq)
  {
    return;
  }

  s_last_seq_ok = seq;

  uint32_t remaining_rx = (s_frame_total > s_received_total) ? (s_frame_total - s_received_total) : 0U;
  if ((uint32_t)len > remaining_rx)
  {
    LcdStreamer_DropFrame();
    return;
  }

#if (LCD_STREAMER_RGB4_ENABLE != 0)
  if (s_pixfmt == VIDEO_PROTO_PIXFMT_RGB4)
  {
    if (s_lcd4_state != LCD4_RX)
    {
      return;
    }
    memcpy(&s_lcd4_frame[s_lcd4_rx_off], data, len);
    s_lcd4_rx_off += len;
    s_received_total += len;
    if (s_received_total >= s_frame_total)
    {
      s_frame_remaining = 0U;
      s_need_window = 1U;
      s_first_chunk = 1U;
      s_lcd4_state = LCD4_TX;
      s_lcd4_line = 0U;
      s_lcd4_tx_inflight_lines = 0U;
      s_lcd4_tx_prepared = 0U;
      s_lcd4_tx_prepared_start = 0U;
      s_lcd4_tx_prepared_lines = 0U;
    }
    return;
  }
#endif

  uint16_t free = fifo_free(&s_fifo);
  if (len > free)
  {
    LcdStreamer_DropFrame();
    return;
  }

  uint16_t written = fifo_write(&s_fifo, data, len);
  if (written != len)
  {
    LcdStreamer_DropFrame();
    return;
  }
  s_received_total += written;
}

void LcdStreamer_OnCrcError(void)
{
  LcdStreamer_DropFrame();
}

void LcdStreamer_OnDmaTxComplete(void)
{
  ST7735_OnSpiTxDmaDone();
  s_dma_busy = 0U;
}

void LcdStreamer_Poll(void)
{
  if (s_active == 0U)
  {
    return;
  }

  if ((s_no_signal == 0U) && (HAL_GetTick() - s_last_rx_ms > LCD_STREAMER_TIMEOUT_MS))
  {
    s_no_signal = 1U;
    LcdStreamer_DropFrame();
  }

  if ((s_dma_busy != 0U) && (!ST7735_DMA_Busy()))
  {
    if (s_last_dma_consume_len > 0U)
    {
      fifo_skip(&s_fifo, s_last_dma_consume_len);
      if (s_frame_remaining > s_last_dma_consume_len)
      {
        s_frame_remaining -= s_last_dma_consume_len;
      }
      else
      {
        s_frame_remaining = 0U;
      }
      s_last_dma_consume_len = 0U;

      if ((s_frame_remaining == 0U) && (fifo_is_empty(&s_fifo) == 0U))
      {
        LcdStreamer_DropFrame();
      }
    }
    s_dma_busy = 0U;
  }

  if ((s_frame_active == 0U) || (s_drop_frame != 0U))
  {
    return;
  }

#if (LCD_STREAMER_RGB4_ENABLE != 0)
  if (s_pixfmt == VIDEO_PROTO_PIXFMT_RGB4)
  {
    if (s_lcd4_state != LCD4_TX)
    {
      return;
    }
    /* Prepare next block while DMA is running (overlap conversion with SPI). */
    if ((s_lcd4_tx_prepared == 0U) && (s_lcd4_line < ST7735_HEIGHT))
    {
      uint16_t remaining_lines = (uint16_t)(ST7735_HEIGHT - s_lcd4_line);
      uint16_t lines = remaining_lines;
      if (lines > LCD_STREAMER_RGB4_BLOCK_LINES)
      {
        lines = LCD_STREAMER_RGB4_BLOCK_LINES;
      }

#if (LCD_STREAMER_RGB4_DOUBLE_BUFFER != 0)
      uint8_t sel = (uint8_t)(s_lcd4_tx_inflight_sel ^ 1U);
#else
      uint8_t sel = 0U;
#endif
      uint8_t *buf = LcdStreamer_Rgb4GetTxBuf(sel);
      LcdStreamer_Rgb4RenderBlock(s_lcd4_line, lines, buf);
      s_lcd4_tx_prepared = 1U;
      s_lcd4_tx_prepared_sel = sel;
      s_lcd4_tx_prepared_start = s_lcd4_line;
      s_lcd4_tx_prepared_lines = lines;
    }

    if ((s_dma_busy != 0U) || ST7735_DMA_Busy())
    {
      return;
    }

    if ((s_lcd4_line >= ST7735_HEIGHT) && (s_lcd4_tx_prepared == 0U))
    {
      s_frame_active = 0U;
      s_lcd4_state = LCD4_IDLE;
      return;
    }

    if (s_need_window != 0U)
    {
      ST7735_SetAddrWindow(0U, 0U, (uint16_t)(ST7735_WIDTH - 1U), (uint16_t)(ST7735_HEIGHT - 1U));
      s_need_window = 0U;
      s_first_chunk = 1U;
    }

    if (s_lcd4_tx_prepared == 0U)
    {
      return;
    }

    uint8_t *tx = LcdStreamer_Rgb4GetTxBuf(s_lcd4_tx_prepared_sel);
    uint16_t tx_bytes = (uint16_t)(ST7735_WIDTH * 2U * s_lcd4_tx_prepared_lines);
    bool ok = false;
    if (s_first_chunk != 0U)
    {
      s_first_chunk = 0U;
      ok = ST7735_WritePixels_DMA(tx, tx_bytes);
    }
    else
    {
      ok = ST7735_WritePixels_DMA_Continue(tx, tx_bytes);
    }
    if (ok)
    {
      s_dma_busy = 1U;
      s_last_dma_consume_len = 0U;
      s_lcd4_tx_inflight_sel = s_lcd4_tx_prepared_sel;
      s_lcd4_tx_inflight_lines = s_lcd4_tx_prepared_lines;
      s_lcd4_line = (uint16_t)(s_lcd4_line + s_lcd4_tx_prepared_lines);
      s_lcd4_tx_prepared = 0U;
      s_lcd4_tx_prepared_lines = 0U;
    }
    return;
  }
#endif
  if (s_dma_busy != 0U)
  {
    return;
  }
  if (ST7735_DMA_Busy())
  {
    return;
  }

  if (s_frame_remaining == 0U)
  {
    if (fifo_is_empty(&s_fifo) != 0U)
    {
      s_frame_active = 0U;
    }
    return;
  }

  uint16_t available = fifo_count(&s_fifo);
  if (available == 0U)
  {
    return;
  }

  uint16_t required_in = 0U;
  if (s_pixfmt == VIDEO_PROTO_PIXFMT_RGB565)
  {
    required_in = (s_frame_remaining < LCD_STREAMER_DMA_CHUNK_SIZE)
                    ? (uint16_t)s_frame_remaining
                    : (uint16_t)LCD_STREAMER_DMA_CHUNK_SIZE;
  }
  else
  {
    uint32_t max_in = LCD_STREAMER_RGB332_IN_CHUNK_BYTES;
    required_in = (s_frame_remaining < max_in) ? (uint16_t)s_frame_remaining : (uint16_t)max_in;
  }

  if (available < required_in)
  {
    return;
  }

  if (s_need_window != 0U)
  {
    ST7735_SetAddrWindow(0U, 0U, (uint16_t)(s_frame_width - 1U), (uint16_t)(s_frame_height - 1U));
    s_need_window = 0U;
  }

  const uint8_t *chunk_ptr = NULL;
  uint16_t contig = fifo_peek_contig(&s_fifo, &chunk_ptr);
  if ((chunk_ptr == NULL) || (contig == 0U))
  {
    return;
  }

  uint16_t send_in = required_in;
  if (send_in > contig)
  {
    send_in = contig;
  }

  const uint8_t *send_ptr = chunk_ptr;
  uint16_t send_out = send_in;
  uint8_t *rgb332_out = NULL;
  if (s_pixfmt == VIDEO_PROTO_PIXFMT_RGB332)
  {
    rgb332_out = LcdStreamer_GetRgb332DmaBuf();
    for (uint16_t i = 0U; i < send_in; i++)
    {
      uint16_t c = s_rgb332_to_565[send_ptr[i]];
      rgb332_out[i * 2U] = (uint8_t)(c >> 8);
      rgb332_out[i * 2U + 1U] = (uint8_t)c;
    }
    send_ptr = rgb332_out;
    send_out = (uint16_t)(send_in * 2U);
  }

  bool ok = false;
  if (s_first_chunk != 0U)
  {
    s_first_chunk = 0U;
    ok = ST7735_WritePixels_DMA(send_ptr, send_out);
  }
  else
  {
    ok = ST7735_WritePixels_DMA_Continue(send_ptr, send_out);
  }

  if (ok)
  {
    s_dma_busy = 1U;
    s_last_dma_consume_len = send_in;
    if (s_pixfmt == VIDEO_PROTO_PIXFMT_RGB332)
    {
      LcdStreamer_SwapRgb332DmaBuf();
    }
  }
}

uint8_t LcdStreamer_IsDmaBusy(void)
{
  return s_dma_busy;
}

uint8_t LcdStreamer_IsNoSignal(void)
{
  return s_no_signal;
}

uint16_t LcdStreamer_GetLastSeqOk(void)
{
  return s_last_seq_ok;
}

uint16_t LcdStreamer_GetFifoFill(void)
{
  return fifo_count(&s_fifo);
}

uint32_t LcdStreamer_GetFrameRemaining(void)
{
  return s_frame_remaining;
}
