#include "stream/lcd_streamer.h"
#include "stream/video_proto.h"
#include "st7735.h"
#include "main.h"
#include <stdbool.h>
#include <string.h>

#define LCD_STREAMER_TIMEOUT_MS 800U

typedef struct
{
  uint8_t buf[LCD_STREAMER_FIFO_SIZE];
  uint16_t head;
  uint16_t tail;
} ByteFifo;

static ByteFifo s_fifo;
static uint8_t s_tx_buf[LCD_STREAMER_DMA_CHUNK_SIZE];

static volatile uint8_t s_active = 0U;
static volatile uint8_t s_dma_busy = 0U;
static volatile uint8_t s_dma_done = 0U;
static uint8_t s_need_window = 0U;
static uint8_t s_first_chunk = 1U;
static uint8_t s_frame_active = 0U;
static uint8_t s_drop_frame = 0U;
static uint8_t s_no_signal = 0U;
static uint16_t s_current_seq = 0U;
static uint16_t s_last_seq_ok = 0U;
static uint32_t s_frame_remaining = 0U;
static uint32_t s_frame_total = 0U;
static uint32_t s_received_total = 0U;
static uint32_t s_last_rx_ms = 0U;
static uint16_t s_last_dma_len = 0U;

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

static uint16_t fifo_peek(const ByteFifo *fifo, uint8_t *out, uint16_t len)
{
  uint16_t count = fifo_count(fifo);
  if (len > count)
  {
    len = count;
  }
  uint16_t idx = fifo->tail;
  for (uint16_t i = 0U; i < len; i++)
  {
    out[i] = fifo->buf[idx];
    idx++;
    if (idx >= LCD_STREAMER_FIFO_SIZE)
    {
      idx = 0U;
    }
  }
  return len;
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
  s_last_dma_len = 0U;
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
  s_last_dma_len = 0U;
}

void LcdStreamer_Init(void)
{
  fifo_clear(&s_fifo);
  s_active = 0U;
  s_dma_busy = 0U;
  s_dma_done = 0U;
  s_need_window = 0U;
  s_first_chunk = 1U;
  s_frame_active = 0U;
  s_drop_frame = 0U;
  s_no_signal = 0U;
  s_current_seq = 0U;
  s_last_seq_ok = 0U;
  s_frame_remaining = 0U;
  s_frame_total = 0U;
  s_received_total = 0U;
  s_last_rx_ms = 0U;
  s_last_dma_len = 0U;
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
  s_frame_remaining = 0U;
  s_frame_total = 0U;
  s_received_total = 0U;
  s_last_rx_ms = HAL_GetTick();
  s_last_seq_ok = 0U;
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
  if ((width != ST7735_WIDTH) || (height != ST7735_HEIGHT) || (pixfmt != VIDEO_PROTO_PIXFMT_RGB565))
  {
    LcdStreamer_DropFrame();
    return;
  }
  if ((uint32_t)frame_bytes != ((uint32_t)width * (uint32_t)height * 2U))
  {
    LcdStreamer_DropFrame();
    return;
  }

  if ((flags & VIDEO_PROTO_FLAG_FORCE_CLEAR) != 0U)
  {
    ST7735_FillColor(0x0000U);
  }

  fifo_clear(&s_fifo);
  s_current_seq = seq;
  s_last_seq_ok = seq;
  s_frame_total = frame_bytes;
  s_frame_remaining = frame_bytes;
  s_received_total = 0U;
  s_drop_frame = 0U;
  s_frame_active = 1U;
  s_need_window = 1U;
  s_first_chunk = 1U;
  s_no_signal = 0U;
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
  s_dma_done = 1U;
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

  if (s_dma_done != 0U)
  {
    if (s_last_dma_len > 0U)
    {
      if (s_frame_remaining > s_last_dma_len)
      {
        s_frame_remaining -= s_last_dma_len;
      }
      else
      {
        s_frame_remaining = 0U;
      }
    }
    s_last_dma_len = 0U;
    s_dma_done = 0U;

    if ((s_frame_remaining == 0U) && (fifo_is_empty(&s_fifo) == 0U))
    {
      LcdStreamer_DropFrame();
    }
  }

  if ((s_frame_active == 0U) || (s_drop_frame != 0U))
  {
    return;
  }
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

  uint16_t required = (s_frame_remaining < LCD_STREAMER_DMA_CHUNK_SIZE)
                        ? (uint16_t)s_frame_remaining
                        : (uint16_t)LCD_STREAMER_DMA_CHUNK_SIZE;
  uint16_t available = fifo_count(&s_fifo);
  if (available < required)
  {
    return;
  }

  if (s_need_window != 0U)
  {
    ST7735_SetAddrWindow(0U, 0U, ST7735_WIDTH - 1U, ST7735_HEIGHT - 1U);
    s_need_window = 0U;
  }

  uint16_t copied = fifo_peek(&s_fifo, s_tx_buf, required);
  if (copied != required)
  {
    return;
  }

  bool ok = false;
  if (s_first_chunk != 0U)
  {
    s_first_chunk = 0U;
    ok = ST7735_WritePixels_DMA(s_tx_buf, copied);
  }
  else
  {
    ok = ST7735_WritePixels_DMA_Continue(s_tx_buf, copied);
  }

  if (ok)
  {
    fifo_skip(&s_fifo, copied);
    s_dma_busy = 1U;
    s_last_dma_len = copied;
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
