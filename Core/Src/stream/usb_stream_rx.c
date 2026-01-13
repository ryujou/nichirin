#include "stream/usb_stream_rx.h"
#include "stream/video_proto.h"
#include "stream/lcd_streamer.h"
#include "stream/spectrum_led.h"
#include "led/led_engine.h"
#include "st7735.h"
#include "usbd_cdc_if.h"
#include <stddef.h>
#include <string.h>

#define USB_STREAM_RX_BUFFER_SIZE 8192U
#define USB_STREAM_RX_BUFFER_MASK (USB_STREAM_RX_BUFFER_SIZE - 1U)
#define USB_STREAM_RX_MAX_LEN 4096U
#define SMALL_W 20U
#define SMALL_H 40U
#define SMALL_PIX_BYTES (SMALL_W * SMALL_H * 2U)
#define SMALL_PAYLOAD_SIZE (3U + LED_COUNT + SMALL_PIX_BYTES)
_Static_assert(SMALL_PAYLOAD_SIZE <= USB_STREAM_RX_MAX_LEN, "SMALL payload exceeds RX max length");

#define FRAME_START_V2_PAYLOAD_SIZE VIDEO_PROTO_START_PAYLOAD_SIZE_V2

#ifndef USB_STREAM_RX_FRAME_DATA_PUSH_CHUNK
#define USB_STREAM_RX_FRAME_DATA_PUSH_CHUNK 256U
#endif

typedef enum
{
  RX_WAIT_MAGIC0 = 0,
  RX_WAIT_MAGIC1,
  RX_READ_TYPE,
  RX_READ_SEQ0,
  RX_READ_SEQ1,
  RX_READ_LEN0,
  RX_READ_LEN1,
  RX_READ_PAYLOAD
} RxState;

static uint8_t s_rx_buf[USB_STREAM_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;
static volatile uint8_t s_rx_overflow = 0U;
static volatile uint32_t s_rx_overflow_count = 0U;

static RxState s_state = RX_WAIT_MAGIC0;
static uint8_t s_type = 0U;
static uint16_t s_seq = 0U;
static uint16_t s_len = 0U;
static uint16_t s_payload_remaining = 0U;
static uint8_t s_small_payload[SMALL_PAYLOAD_SIZE];
static uint16_t s_small_index = 0U;
static volatile uint32_t s_rx_total = 0U;

static uint8_t s_start_payload[FRAME_START_V2_PAYLOAD_SIZE];
static uint16_t s_start_index = 0U;

static uint8_t s_data_chunk[USB_STREAM_RX_FRAME_DATA_PUSH_CHUNK];
static uint16_t s_data_chunk_len = 0U;

static uint8_t s_stream_seq_valid = 0U;
static uint16_t s_stream_seq = 0U;

static uint16_t s_last_pong_seq = 0U;
static uint8_t s_last_pong_valid = 0U;

static void UsbStreamRx_SendPong(uint16_t seq, uint8_t status)
{
  /* Packet: MAGIC(2) + type(1) + seq(2) + len(2) + payload(6)
     payload[0]=status: 1=frame_done, 0=busy/unknown (reserved)
     payload[1..]=reserved */
  uint8_t pkt[2U + 1U + 2U + 2U + VIDEO_PROTO_PONG_PAYLOAD_SIZE];
  uint16_t off = 0U;
  pkt[off++] = VIDEO_PROTO_MAGIC0;
  pkt[off++] = VIDEO_PROTO_MAGIC1;
  pkt[off++] = VIDEO_PROTO_TYPE_PONG;
  pkt[off++] = (uint8_t)(seq & 0xFFU);
  pkt[off++] = (uint8_t)(seq >> 8);
  pkt[off++] = (uint8_t)(VIDEO_PROTO_PONG_PAYLOAD_SIZE & 0xFFU);
  pkt[off++] = (uint8_t)(VIDEO_PROTO_PONG_PAYLOAD_SIZE >> 8);
  pkt[off++] = status;
  pkt[off++] = 0U;
  pkt[off++] = 0U;
  pkt[off++] = 0U;
  pkt[off++] = 0U;
  pkt[off++] = 0U;
  (void)CDC_Transmit_FS(pkt, (uint16_t)sizeof(pkt));
}

static void UsbStreamRx_PushByte(uint8_t byte)
{
  uint16_t next = (uint16_t)((s_rx_head + 1U) & USB_STREAM_RX_BUFFER_MASK);
  if (next == s_rx_tail)
  {
    s_rx_overflow = 1U;
    s_rx_overflow_count++;
    return;
  }
  s_rx_buf[s_rx_head] = byte;
  s_rx_head = next;
}

static uint8_t UsbStreamRx_PopByte(uint8_t *out)
{
  if (s_rx_head == s_rx_tail)
  {
    return 0U;
  }
  *out = s_rx_buf[s_rx_tail];
  s_rx_tail = (uint16_t)((s_rx_tail + 1U) & USB_STREAM_RX_BUFFER_MASK);
  return 1U;
}

static void UsbStreamRx_ResetParser(void)
{
  s_state = RX_WAIT_MAGIC0;
  s_type = 0U;
  s_seq = 0U;
  s_len = 0U;
  s_payload_remaining = 0U;
  s_small_index = 0U;
  s_start_index = 0U;
  s_data_chunk_len = 0U;
}

void UsbStreamRx_Init(void)
{
  s_rx_head = 0U;
  s_rx_tail = 0U;
  s_rx_total = 0U;
  s_rx_overflow = 0U;
  s_rx_overflow_count = 0U;
  s_stream_seq_valid = 0U;
  s_stream_seq = 0U;
  LcdStreamer_Init();
  LcdStreamer_Start();
  SpectrumLed_Init();
  SpectrumLed_SetActive(1U);
  UsbStreamRx_ResetParser();
}

void UsbStreamRx_Reset(void)
{
  s_rx_head = 0U;
  s_rx_tail = 0U;
  s_rx_total = 0U;
  s_rx_overflow = 0U;
  s_stream_seq_valid = 0U;
  s_stream_seq = 0U;
  LcdStreamer_Start();
  SpectrumLed_SetActive(1U);
  UsbStreamRx_ResetParser();
}

void UsbStreamRx_OnRxData(const uint8_t *data, uint32_t len)
{
  if ((data == NULL) || (len == 0U))
  {
    return;
  }
  s_rx_total += len;
  for (uint32_t i = 0U; i < len; i++)
  {
    UsbStreamRx_PushByte(data[i]);
  }
}

uint32_t UsbStreamRx_GetRxTotal(void)
{
  return s_rx_total;
}

static void UsbStreamRx_RenderSmallFrame(const uint8_t *frame_bytes)
{
  uint8_t line[80U * 2U];
  uint8_t first_line = 1U;

  ST7735_SetAddrWindow(0U, 0U, 79U, 159U);
  for (uint16_t y = 0U; y < SMALL_H; y++)
  {
    const uint8_t *row = &frame_bytes[(uint32_t)y * SMALL_W * 2U];
    for (uint16_t ry = 0U; ry < 4U; ry++)
    {
      uint32_t dst = 0U;
      for (uint16_t x = 0U; x < SMALL_W; x++)
      {
        uint8_t hi = row[x * 2U];
        uint8_t lo = row[x * 2U + 1U];
        for (uint16_t rx = 0U; rx < 4U; rx++)
        {
          line[dst++] = hi;
          line[dst++] = lo;
        }
      }
      if (first_line != 0U)
      {
        while (!ST7735_WritePixels_DMA(line, (uint16_t)sizeof(line)))
        {
        }
        first_line = 0U;
      }
      else
      {
        while (!ST7735_WritePixels_DMA_Continue(line, (uint16_t)sizeof(line)))
        {
        }
      }
      while (ST7735_DMA_Busy())
      {
      }
    }
  }
}

static void UsbStreamRx_HandlePacket(void)
{
  if (s_type == VIDEO_PROTO_TYPE_FRAME_SMALL)
  {
    if (s_len != SMALL_PAYLOAD_SIZE)
    {
      return;
    }

    {
      uint8_t width = s_small_payload[0];
      uint8_t height = s_small_payload[1];
      uint8_t pixfmt = s_small_payload[2];
      if ((width != SMALL_W) || (height != SMALL_H) || (pixfmt != VIDEO_PROTO_PIXFMT_RGB565))
      {
        return;
      }
    }

    {
      const uint8_t *leds = &s_small_payload[3];
      const uint8_t *frame = &s_small_payload[3 + LED_COUNT];
      LedEngine_UpdateCmd(leds);
      UsbStreamRx_RenderSmallFrame(frame);
    }
    return;
  }

  if (s_type == VIDEO_PROTO_TYPE_FRAME_START)
  {
    if ((s_len != VIDEO_PROTO_START_PAYLOAD_SIZE_V1) && (s_len != VIDEO_PROTO_START_PAYLOAD_SIZE_V2))
    {
      s_stream_seq_valid = 0U;
      return;
    }

    uint8_t width = s_start_payload[0];
    uint8_t height = s_start_payload[1];
    uint8_t pixfmt = s_start_payload[2];
    uint16_t frame_bytes = (uint16_t)(s_start_payload[3] | ((uint16_t)s_start_payload[4] << 8U));
    uint8_t flags = 0U;

    if (s_len == VIDEO_PROTO_START_PAYLOAD_SIZE_V2)
    {
      const uint8_t *bands_val = &s_start_payload[5];
      const uint8_t *bands_peak = &s_start_payload[17];
      flags = s_start_payload[29];
      SpectrumLed_OnBands(bands_val, bands_peak);
    }

    /* If we're still flushing a previous frame, reject this START.
       Host should use PONG pacing to avoid ever hitting this path. */
    if ((LcdStreamer_IsDmaBusy() != 0U) || (ST7735_DMA_Busy() != 0U) ||
        (LcdStreamer_GetFrameRemaining() != 0U) || (LcdStreamer_GetFifoFill() != 0U))
    {
      s_stream_seq_valid = 0U;
      UsbStreamRx_SendPong(s_seq, 0U);
      return;
    }

    s_stream_seq_valid = 1U;
    s_stream_seq = s_seq;

    LcdStreamer_OnPacketOk(s_seq);
    LcdStreamer_OnFrameStart(s_seq, width, height, pixfmt, frame_bytes, flags);
    return;
  }

  if (s_type == VIDEO_PROTO_TYPE_FRAME_DATA)
  {
    if ((s_stream_seq_valid == 0U) || (s_seq != s_stream_seq))
    {
      return;
    }
    if (s_data_chunk_len > 0U)
    {
      LcdStreamer_OnFrameData(s_seq, s_data_chunk, s_data_chunk_len);
      s_data_chunk_len = 0U;
    }
    LcdStreamer_OnPacketOk(s_seq);
    return;
  }

  if (s_type == VIDEO_PROTO_TYPE_STOP)
  {
    s_stream_seq_valid = 0U;
    LcdStreamer_Stop();
    SpectrumLed_SetActive(0U);
    return;
  }
}

void UsbStreamRx_Poll(void)
{
  if (s_rx_overflow != 0U)
  {
    s_rx_overflow = 0U;
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_stream_seq_valid = 0U;
    LcdStreamer_OnCrcError();
    UsbStreamRx_ResetParser();
  }

  LcdStreamer_Poll();

  /* Frame-done PONG (once per seq): host can throttle on this to avoid drops/tearing. */
  if ((s_stream_seq_valid != 0U) &&
      (LcdStreamer_IsDmaBusy() == 0U) && (ST7735_DMA_Busy() == 0U) &&
      (LcdStreamer_GetFrameRemaining() == 0U) && (LcdStreamer_GetFifoFill() == 0U))
  {
    if ((s_last_pong_valid == 0U) || (s_last_pong_seq != s_stream_seq))
    {
      s_last_pong_valid = 1U;
      s_last_pong_seq = s_stream_seq;
      UsbStreamRx_SendPong(s_stream_seq, 1U);
    }
  }

  uint8_t byte = 0U;
  while (UsbStreamRx_PopByte(&byte) != 0U)
  {
    switch (s_state)
    {
      case RX_WAIT_MAGIC0:
        if (byte == VIDEO_PROTO_MAGIC0)
        {
          s_state = RX_WAIT_MAGIC1;
        }
        break;
      case RX_WAIT_MAGIC1:
        if (byte == VIDEO_PROTO_MAGIC1)
        {
          s_state = RX_READ_TYPE;
        }
        else if (byte == VIDEO_PROTO_MAGIC0)
        {
          s_state = RX_WAIT_MAGIC1;
        }
        else
        {
          s_state = RX_WAIT_MAGIC0;
        }
        break;
      case RX_READ_TYPE:
        s_type = byte;
        s_state = RX_READ_SEQ0;
        break;
      case RX_READ_SEQ0:
        s_seq = byte;
        s_state = RX_READ_SEQ1;
        break;
      case RX_READ_SEQ1:
        s_seq |= (uint16_t)((uint16_t)byte << 8U);
        s_state = RX_READ_LEN0;
        break;
      case RX_READ_LEN0:
        s_len = byte;
        s_state = RX_READ_LEN1;
        break;
      case RX_READ_LEN1:
        s_len |= (uint16_t)((uint16_t)byte << 8U);
        if (s_len > USB_STREAM_RX_MAX_LEN)
        {
          UsbStreamRx_ResetParser();
        }
        else if (s_len == 0U)
        {
          UsbStreamRx_HandlePacket();
          UsbStreamRx_ResetParser();
        }
        else
        {
          s_payload_remaining = s_len;
          s_small_index = 0U;
          s_start_index = 0U;
          s_data_chunk_len = 0U;
          s_state = RX_READ_PAYLOAD;
        }
        break;
      case RX_READ_PAYLOAD:
        if (s_type == VIDEO_PROTO_TYPE_FRAME_SMALL)
        {
          if (s_small_index < SMALL_PAYLOAD_SIZE)
          {
            s_small_payload[s_small_index++] = byte;
          }
        }
        else if (s_type == VIDEO_PROTO_TYPE_FRAME_START)
        {
          if (s_start_index < FRAME_START_V2_PAYLOAD_SIZE)
          {
            s_start_payload[s_start_index++] = byte;
          }
        }
        else if (s_type == VIDEO_PROTO_TYPE_FRAME_DATA)
        {
          if (s_data_chunk_len < USB_STREAM_RX_FRAME_DATA_PUSH_CHUNK)
          {
            s_data_chunk[s_data_chunk_len++] = byte;
          }
          else
          {
            if ((s_stream_seq_valid != 0U) && (s_seq == s_stream_seq))
            {
              LcdStreamer_OnFrameData(s_seq, s_data_chunk, s_data_chunk_len);
            }
            s_data_chunk_len = 0U;
            s_data_chunk[s_data_chunk_len++] = byte;
          }
        }
        if (--s_payload_remaining == 0U)
        {
          if ((s_type == VIDEO_PROTO_TYPE_FRAME_DATA) && (s_data_chunk_len > 0U))
          {
            if ((s_stream_seq_valid != 0U) && (s_seq == s_stream_seq))
            {
              LcdStreamer_OnFrameData(s_seq, s_data_chunk, s_data_chunk_len);
            }
            s_data_chunk_len = 0U;
          }
          UsbStreamRx_HandlePacket();
          UsbStreamRx_ResetParser();
        }
        break;
      default:
        UsbStreamRx_ResetParser();
        break;
    }
  }

  LcdStreamer_Poll();

  if ((s_stream_seq_valid != 0U) &&
      (LcdStreamer_IsDmaBusy() == 0U) && (ST7735_DMA_Busy() == 0U) &&
      (LcdStreamer_GetFrameRemaining() == 0U) && (LcdStreamer_GetFifoFill() == 0U))
  {
    if ((s_last_pong_valid == 0U) || (s_last_pong_seq != s_stream_seq))
    {
      s_last_pong_valid = 1U;
      s_last_pong_seq = s_stream_seq;
      UsbStreamRx_SendPong(s_stream_seq, 1U);
    }
  }
}
