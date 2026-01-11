#include "stream/usb_stream_rx.h"
#include "stream/video_proto.h"
#include "stream/lcd_streamer.h"
#include "stream/spectrum_led.h"
#include "app/mode_manager.h"
#include "usbd_cdc_if.h"
#include "main.h"
#include <stddef.h>
#include <string.h>

#define USB_STREAM_RX_BUFFER_SIZE 8192U
#define USB_STREAM_RX_BUFFER_MASK (USB_STREAM_RX_BUFFER_SIZE - 1U)
#define USB_STREAM_RX_DATA_CHUNK 1024U
#define USB_STREAM_RX_MAX_LEN 4096U
#define USB_STREAM_STOP_EXITS_MODE 1U

typedef enum
{
  RX_WAIT_MAGIC0 = 0,
  RX_WAIT_MAGIC1,
  RX_READ_TYPE,
  RX_READ_SEQ0,
  RX_READ_SEQ1,
  RX_READ_LEN0,
  RX_READ_LEN1,
  RX_READ_PAYLOAD,
  RX_READ_CRC0,
  RX_READ_CRC1
} RxState;

static uint8_t s_rx_buf[USB_STREAM_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;

static RxState s_state = RX_WAIT_MAGIC0;
static uint8_t s_type = 0U;
static uint16_t s_seq = 0U;
static uint16_t s_len = 0U;
static uint16_t s_crc_calc = 0U;
static uint16_t s_crc_recv = 0U;
static uint16_t s_payload_remaining = 0U;
static uint8_t s_start_payload[VIDEO_PROTO_START_PAYLOAD_SIZE_V2];
static uint8_t s_start_index = 0U;
static uint8_t s_data_chunk[USB_STREAM_RX_DATA_CHUNK];
static uint16_t s_chunk_len = 0U;

static uint8_t s_pong_buf[2U + 1U + 2U + 2U + VIDEO_PROTO_PONG_PAYLOAD_SIZE + 2U];

static void UsbStreamRx_PushByte(uint8_t byte)
{
  uint16_t next = (uint16_t)((s_rx_head + 1U) & USB_STREAM_RX_BUFFER_MASK);
  if (next == s_rx_tail)
  {
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) & USB_STREAM_RX_BUFFER_MASK);
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
  s_crc_calc = 0U;
  s_crc_recv = 0U;
  s_payload_remaining = 0U;
  s_start_index = 0U;
  s_chunk_len = 0U;
}

void UsbStreamRx_Init(void)
{
  s_rx_head = 0U;
  s_rx_tail = 0U;
  UsbStreamRx_ResetParser();
}

void UsbStreamRx_Reset(void)
{
  s_rx_head = 0U;
  s_rx_tail = 0U;
  UsbStreamRx_ResetParser();
}

void UsbStreamRx_OnRxData(const uint8_t *data, uint32_t len)
{
  if ((data == NULL) || (len == 0U))
  {
    return;
  }
  for (uint32_t i = 0U; i < len; i++)
  {
    UsbStreamRx_PushByte(data[i]);
  }
}

static void UsbStreamRx_FlushChunk(void)
{
  if (s_chunk_len > 0U)
  {
    LcdStreamer_OnFrameData(s_seq, s_data_chunk, s_chunk_len);
    s_chunk_len = 0U;
  }
}

static void UsbStreamRx_SendPong(uint16_t seq)
{
  uint32_t uptime = HAL_GetTick();
  uint16_t last_seq = LcdStreamer_GetLastSeqOk();

  uint8_t *p = s_pong_buf;
  p[0] = VIDEO_PROTO_MAGIC0;
  p[1] = VIDEO_PROTO_MAGIC1;
  p[2] = VIDEO_PROTO_TYPE_PONG;
  p[3] = (uint8_t)(seq & 0xFFU);
  p[4] = (uint8_t)(seq >> 8U);
  p[5] = (uint8_t)(VIDEO_PROTO_PONG_PAYLOAD_SIZE & 0xFFU);
  p[6] = (uint8_t)(VIDEO_PROTO_PONG_PAYLOAD_SIZE >> 8U);
  p[7] = (uint8_t)(uptime & 0xFFU);
  p[8] = (uint8_t)((uptime >> 8U) & 0xFFU);
  p[9] = (uint8_t)((uptime >> 16U) & 0xFFU);
  p[10] = (uint8_t)((uptime >> 24U) & 0xFFU);
  p[11] = (uint8_t)(last_seq & 0xFFU);
  p[12] = (uint8_t)(last_seq >> 8U);

  uint16_t crc = VideoProto_Crc16Ccitt(&p[2], (uint16_t)(5U + VIDEO_PROTO_PONG_PAYLOAD_SIZE));
  p[13] = (uint8_t)(crc & 0xFFU);
  p[14] = (uint8_t)(crc >> 8U);

  (void)CDC_Transmit_FS(p, (uint16_t)sizeof(s_pong_buf));
}

static void UsbStreamRx_HandlePacket(void)
{
  if (s_crc_calc != s_crc_recv)
  {
    LcdStreamer_OnCrcError();
    return;
  }

  LcdStreamer_OnPacketOk(s_seq);

  if (s_type == VIDEO_PROTO_TYPE_FRAME_START)
  {
    uint8_t width = 0U;
    uint8_t height = 0U;
    uint8_t pixfmt = 0U;
    uint16_t frame_bytes = 0U;
    uint8_t bands_val[VIDEO_PROTO_BANDS_COUNT];
    uint8_t bands_peak[VIDEO_PROTO_BANDS_COUNT];
    uint8_t flags = 0U;
    memset(bands_val, 0, sizeof(bands_val));
    memset(bands_peak, 0, sizeof(bands_peak));

    if (s_len == VIDEO_PROTO_START_PAYLOAD_SIZE_V1)
    {
      width = s_start_payload[0];
      height = s_start_payload[1];
      pixfmt = s_start_payload[2];
      frame_bytes = (uint16_t)((uint16_t)s_start_payload[3] | ((uint16_t)s_start_payload[4] << 8U));
      flags = 0U;
    }
    else if (s_len == VIDEO_PROTO_START_PAYLOAD_SIZE_V2)
    {
      width = s_start_payload[0];
      height = s_start_payload[1];
      pixfmt = s_start_payload[2];
      frame_bytes = (uint16_t)((uint16_t)s_start_payload[3] | ((uint16_t)s_start_payload[4] << 8U));
      memcpy(bands_val, &s_start_payload[5], VIDEO_PROTO_BANDS_COUNT);
      memcpy(bands_peak, &s_start_payload[5 + VIDEO_PROTO_BANDS_COUNT], VIDEO_PROTO_BANDS_COUNT);
      flags = s_start_payload[5 + VIDEO_PROTO_BANDS_COUNT + VIDEO_PROTO_BANDS_COUNT];
    }
    else
    {
      return;
    }

    SpectrumLed_OnBands(bands_val, bands_peak);
    LcdStreamer_OnFrameStart(s_seq, width, height, pixfmt, frame_bytes, flags);
  }
  else if (s_type == VIDEO_PROTO_TYPE_STOP)
  {
#if USB_STREAM_STOP_EXITS_MODE
    ModeManager_ExitStream();
#else
    LcdStreamer_Stop();
#endif
  }
  else if (s_type == VIDEO_PROTO_TYPE_PING)
  {
    UsbStreamRx_SendPong(s_seq);
  }
}

void UsbStreamRx_Poll(void)
{
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
        s_crc_calc = VideoProto_Crc16CcittUpdate(0xFFFFU, byte);
        s_state = RX_READ_SEQ0;
        break;
      case RX_READ_SEQ0:
        s_seq = byte;
        s_crc_calc = VideoProto_Crc16CcittUpdate(s_crc_calc, byte);
        s_state = RX_READ_SEQ1;
        break;
      case RX_READ_SEQ1:
        s_seq |= (uint16_t)((uint16_t)byte << 8U);
        s_crc_calc = VideoProto_Crc16CcittUpdate(s_crc_calc, byte);
        s_state = RX_READ_LEN0;
        break;
      case RX_READ_LEN0:
        s_len = byte;
        s_crc_calc = VideoProto_Crc16CcittUpdate(s_crc_calc, byte);
        s_state = RX_READ_LEN1;
        break;
      case RX_READ_LEN1:
        s_len |= (uint16_t)((uint16_t)byte << 8U);
        s_crc_calc = VideoProto_Crc16CcittUpdate(s_crc_calc, byte);
        if (s_len > USB_STREAM_RX_MAX_LEN)
        {
          UsbStreamRx_ResetParser();
        }
        else if (s_len == 0U)
        {
          s_state = RX_READ_CRC0;
        }
        else
        {
          s_payload_remaining = s_len;
          s_start_index = 0U;
          s_chunk_len = 0U;
          s_state = RX_READ_PAYLOAD;
        }
        break;
      case RX_READ_PAYLOAD:
        s_crc_calc = VideoProto_Crc16CcittUpdate(s_crc_calc, byte);
        if (s_type == VIDEO_PROTO_TYPE_FRAME_START)
        {
          if (s_start_index < VIDEO_PROTO_START_PAYLOAD_SIZE_V2)
          {
            s_start_payload[s_start_index++] = byte;
          }
        }
        else if (s_type == VIDEO_PROTO_TYPE_FRAME_DATA)
        {
          s_data_chunk[s_chunk_len++] = byte;
          if (s_chunk_len >= USB_STREAM_RX_DATA_CHUNK)
          {
            UsbStreamRx_FlushChunk();
          }
        }
        if (--s_payload_remaining == 0U)
        {
          UsbStreamRx_FlushChunk();
          s_state = RX_READ_CRC0;
        }
        break;
      case RX_READ_CRC0:
        s_crc_recv = byte;
        s_state = RX_READ_CRC1;
        break;
      case RX_READ_CRC1:
        s_crc_recv |= (uint16_t)((uint16_t)byte << 8U);
        UsbStreamRx_HandlePacket();
        UsbStreamRx_ResetParser();
        break;
      default:
        UsbStreamRx_ResetParser();
        break;
    }
  }
}
