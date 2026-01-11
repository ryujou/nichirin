#include "stream/video_proto.h"
#include <stddef.h>

#define CRC16_INIT 0xFFFFU
#define CRC16_POLY 0x1021U

uint16_t VideoProto_Crc16CcittUpdate(uint16_t crc, uint8_t data)
{
  crc ^= (uint16_t)data << 8U;
  for (uint8_t i = 0U; i < 8U; i++)
  {
    if (crc & 0x8000U)
    {
      crc = (uint16_t)((crc << 1U) ^ CRC16_POLY);
    }
    else
    {
      crc <<= 1U;
    }
  }
  return crc;
}

uint16_t VideoProto_Crc16Ccitt(const uint8_t *data, uint16_t len)
{
  uint16_t crc = CRC16_INIT;
  if (data == NULL)
  {
    return crc;
  }
  for (uint16_t i = 0U; i < len; i++)
  {
    crc = VideoProto_Crc16CcittUpdate(crc, data[i]);
  }
  return crc;
}
