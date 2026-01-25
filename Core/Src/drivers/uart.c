/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : uart.c
  * @brief          : UART Modbus RX (USART1/USART2) using DMA + IDLE
  ******************************************************************************
  * CubeMX notes:
  * - Enable USART1/USART2 with DMA RX.
  * - RX DMA should be Circular.
  * - Enable USART1/USART2 global interrupts (for IDLE detection).
  * - Enable DMA channel IRQs used by USART1/USART2 RX.
  ******************************************************************************
  */
/* USER CODE END Header */
#include "drivers/uart.h"
#include "app.h"

#include <string.h>

#define UART_SPECTRUM_RX_SIZE 256U
#define MODBUS_BAND_COUNT 12U
#define MODBUS_BAND_START 0x0100U
#define MODBUS_CFG_COUNT 5U
#define MODBUS_ADDR 0x01U
#define MODBUS_FUNC_READ 0x03U
#define MODBUS_FUNC_WRITE_SINGLE 0x06U
#define MODBUS_FUNC_WRITE_MULTI 0x10U
#define MODBUS_MAX_TX 64U

typedef struct
{
  UART_HandleTypeDef *uart;
  uint8_t rx_buf[UART_SPECTRUM_RX_SIZE];
  uint8_t port_id;
  uint8_t tx_buf[MODBUS_MAX_TX];
  uint16_t tx_len;
  uint8_t tx_pending;
} UartSpectrumRx;

static UartSpectrumRx s_rx1;
static UartSpectrumRx s_rx2;
static uint8_t s_initialized = 0U;

static volatile uint8_t s_bands_raw[MODBUS_BAND_COUNT];
static volatile uint32_t s_bands_last_ms = 0U;
static volatile uint8_t s_bands_src = 0U;
static volatile uint32_t s_rx_count = 0U;

static uint16_t UartSpectrum_Crc16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFU;
  uint16_t i;

  for (i = 0U; i < len; i++)
  {
    uint8_t bit;
    crc ^= data[i];
    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
      }
      else
      {
        crc = (uint16_t)(crc >> 1U);
      }
    }
  }
  return crc;
}

static void Modbus_QueueResponse(UartSpectrumRx *rx, const uint8_t *data, uint16_t len)
{
  if ((rx == NULL) || (data == NULL) || (len == 0U) || (len > MODBUS_MAX_TX))
  {
    return;
  }
  if (rx->tx_pending != 0U)
  {
    return;
  }
  memcpy(rx->tx_buf, data, len);
  rx->tx_len = len;
  rx->tx_pending = 1U;
}

static void Modbus_SendException(UartSpectrumRx *rx, uint8_t addr, uint8_t func, uint8_t code)
{
  uint8_t resp[5];
  uint16_t crc;

  resp[0] = addr;
  resp[1] = (uint8_t)(func | 0x80U);
  resp[2] = code;
  crc = UartSpectrum_Crc16(resp, 3U);
  resp[3] = (uint8_t)(crc & 0xFFU);
  resp[4] = (uint8_t)(crc >> 8U);
  Modbus_QueueResponse(rx, resp, 5U);
}

static uint8_t Modbus_IsConfigRange(uint16_t start, uint16_t count)
{
  if (count == 0U)
  {
    return 0U;
  }
  return (uint8_t)(start < MODBUS_CFG_COUNT && ((uint32_t)start + (uint32_t)count) <= MODBUS_CFG_COUNT);
}

static uint8_t Modbus_IsBandRange(uint16_t start, uint16_t count)
{
  uint32_t end = (uint32_t)start + (uint32_t)count;
  if (count == 0U)
  {
    return 0U;
  }
  return (uint8_t)(start >= MODBUS_BAND_START &&
                   end <= ((uint32_t)MODBUS_BAND_START + (uint32_t)MODBUS_BAND_COUNT));
}

static void Modbus_StoreBandRange(uint16_t start, const uint8_t *data, uint16_t count, uint8_t port_id)
{
  uint32_t primask;
  uint16_t i;
  uint32_t now_ms = HAL_GetTick();
  uint16_t base = (uint16_t)(start - MODBUS_BAND_START);

  primask = __get_PRIMASK();
  __disable_irq();
  for (i = 0U; i < count; i++)
  {
    uint16_t val = (uint16_t)((uint16_t)data[2U * i] << 8U) | data[2U * i + 1U];
    s_bands_raw[base + i] = (uint8_t)val;
  }
  s_bands_last_ms = now_ms;
  s_bands_src = port_id;
  __set_PRIMASK(primask);
}

static void Modbus_HandleRead(UartSpectrumRx *rx, uint8_t addr, uint16_t start, uint16_t count)
{
  uint8_t resp[MODBUS_MAX_TX];
  uint16_t byte_count;
  uint16_t resp_len;
  uint16_t i;
  uint16_t crc;

  if (count == 0U)
  {
    Modbus_SendException(rx, addr, MODBUS_FUNC_READ, 0x03U);
    return;
  }
  if (!(Modbus_IsConfigRange(start, count) || Modbus_IsBandRange(start, count)))
  {
    Modbus_SendException(rx, addr, MODBUS_FUNC_READ, 0x02U);
    return;
  }

  byte_count = (uint16_t)(count * 2U);
  resp_len = (uint16_t)(3U + byte_count + 2U);
  if (resp_len > MODBUS_MAX_TX)
  {
    Modbus_SendException(rx, addr, MODBUS_FUNC_READ, 0x03U);
    return;
  }

  resp[0] = addr;
  resp[1] = MODBUS_FUNC_READ;
  resp[2] = (uint8_t)byte_count;
  if (Modbus_IsConfigRange(start, count))
  {
    for (i = 0U; i < count; i++)
    {
      uint16_t val = 0U;
      if (app_modbus_read_reg((uint16_t)(start + i), &val) != APP_MODBUS_OK)
      {
        Modbus_SendException(rx, addr, MODBUS_FUNC_READ, 0x02U);
        return;
      }
      resp[3U + (2U * i)] = (uint8_t)(val >> 8U);
      resp[4U + (2U * i)] = (uint8_t)(val & 0xFFU);
    }
  }
  else
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (i = 0U; i < count; i++)
    {
      uint16_t idx = (uint16_t)((start - MODBUS_BAND_START) + i);
      uint16_t val = s_bands_raw[idx];
      resp[3U + (2U * i)] = (uint8_t)(val >> 8U);
      resp[4U + (2U * i)] = (uint8_t)(val & 0xFFU);
    }
    __set_PRIMASK(primask);
  }
  crc = UartSpectrum_Crc16(resp, (uint16_t)(3U + byte_count));
  resp[3U + byte_count] = (uint8_t)(crc & 0xFFU);
  resp[4U + byte_count] = (uint8_t)(crc >> 8U);
  Modbus_QueueResponse(rx, resp, resp_len);
}

static void Modbus_HandleWriteSingle(UartSpectrumRx *rx, const uint8_t *frame)
{
  uint16_t reg = (uint16_t)((uint16_t)frame[2] << 8U) | frame[3];
  uint16_t val = (uint16_t)((uint16_t)frame[4] << 8U) | frame[5];

  if (Modbus_IsConfigRange(reg, 1U))
  {
    (void)app_modbus_write_reg(reg, val, HAL_GetTick());
    Modbus_QueueResponse(rx, frame, 8U);
    return;
  }
  if (Modbus_IsBandRange(reg, 1U))
  {
    uint8_t buf[2];
    buf[0] = (uint8_t)(val >> 8U);
    buf[1] = (uint8_t)(val & 0xFFU);
    Modbus_StoreBandRange(reg, buf, 1U, rx->port_id);
    Modbus_QueueResponse(rx, frame, 8U);
    return;
  }

  Modbus_SendException(rx, frame[0], MODBUS_FUNC_WRITE_SINGLE, 0x02U);
}

static void Modbus_HandleWriteMulti(UartSpectrumRx *rx, const uint8_t *frame, uint16_t frame_len)
{
  uint16_t start = (uint16_t)((uint16_t)frame[2] << 8U) | frame[3];
  uint16_t count = (uint16_t)((uint16_t)frame[4] << 8U) | frame[5];
  uint8_t byte_count = frame[6];
  uint16_t expected = (uint16_t)(count * 2U);
  uint8_t resp[8];
  uint16_t crc;
  uint16_t i;
  uint32_t now_ms = HAL_GetTick();

  (void)frame_len;

  if ((count == 0U) || (byte_count != expected))
  {
    Modbus_SendException(rx, frame[0], MODBUS_FUNC_WRITE_MULTI, 0x03U);
    return;
  }
  if (Modbus_IsConfigRange(start, count))
  {
    for (i = 0U; i < count; i++)
    {
      uint16_t val = (uint16_t)((uint16_t)frame[7U + (2U * i)] << 8U)
                     | frame[8U + (2U * i)];
      (void)app_modbus_write_reg((uint16_t)(start + i), val, now_ms);
    }
  }
  else if (Modbus_IsBandRange(start, count))
  {
    Modbus_StoreBandRange(start, &frame[7U], count, rx->port_id);
  }
  else
  {
    Modbus_SendException(rx, frame[0], MODBUS_FUNC_WRITE_MULTI, 0x02U);
    return;
  }

  resp[0] = frame[0];
  resp[1] = MODBUS_FUNC_WRITE_MULTI;
  resp[2] = frame[2];
  resp[3] = frame[3];
  resp[4] = frame[4];
  resp[5] = frame[5];
  crc = UartSpectrum_Crc16(resp, 6U);
  resp[6] = (uint8_t)(crc & 0xFFU);
  resp[7] = (uint8_t)(crc >> 8U);
  Modbus_QueueResponse(rx, resp, 8U);
}

static void Modbus_ParseBuffer(UartSpectrumRx *rx, const uint8_t *data, uint16_t len)
{
  uint16_t idx = 0U;

  if ((rx == NULL) || (data == NULL) || (len == 0U))
  {
    return;
  }
  if (rx->tx_pending != 0U)
  {
    return;
  }

  while (idx < len)
  {
    uint8_t addr = data[idx];
    if (addr != MODBUS_ADDR)
    {
      idx++;
      continue;
    }
    if ((idx + 2U) > len)
    {
      break;
    }

    {
      uint8_t func = data[idx + 1U];
      if ((func == MODBUS_FUNC_READ) || (func == MODBUS_FUNC_WRITE_SINGLE))
      {
        uint16_t frame_len = 8U;
        uint16_t crc;
        uint16_t rx_crc;
        if ((idx + frame_len) > len)
        {
          break;
        }
        crc = UartSpectrum_Crc16(&data[idx], (uint16_t)(frame_len - 2U));
        rx_crc = (uint16_t)data[idx + frame_len - 2U]
                 | ((uint16_t)data[idx + frame_len - 1U] << 8U);
        if (crc == rx_crc)
        {
          if (func == MODBUS_FUNC_READ)
          {
            uint16_t start = (uint16_t)((uint16_t)data[idx + 2U] << 8U) | data[idx + 3U];
            uint16_t count = (uint16_t)((uint16_t)data[idx + 4U] << 8U) | data[idx + 5U];
            Modbus_HandleRead(rx, addr, start, count);
          }
          else
          {
            Modbus_HandleWriteSingle(rx, &data[idx]);
          }
          idx = (uint16_t)(idx + frame_len);
          continue;
        }
      }
      else if (func == MODBUS_FUNC_WRITE_MULTI)
      {
        if ((idx + 9U) > len)
        {
          break;
        }
        {
          uint16_t count = (uint16_t)((uint16_t)data[idx + 4U] << 8U) | data[idx + 5U];
          uint8_t byte_count = data[idx + 6U];
          uint16_t expected = (uint16_t)(count * 2U);
          uint16_t frame_len = (uint16_t)(9U + byte_count);
          uint16_t crc;
          uint16_t rx_crc;

          if ((byte_count != expected) || ((idx + frame_len) > len))
          {
            idx++;
            continue;
          }
          crc = UartSpectrum_Crc16(&data[idx], (uint16_t)(frame_len - 2U));
          rx_crc = (uint16_t)data[idx + frame_len - 2U]
                   | ((uint16_t)data[idx + frame_len - 1U] << 8U);
          if (crc == rx_crc)
          {
            Modbus_HandleWriteMulti(rx, &data[idx], frame_len);
            idx = (uint16_t)(idx + frame_len);
            continue;
          }
        }
      }
    }
    idx++;
  }
}

static void UartSpectrum_StartRx(UartSpectrumRx *rx)
{
  if (rx->uart == NULL)
  {
    return;
  }

  if (HAL_UARTEx_ReceiveToIdle_DMA(rx->uart, rx->rx_buf, UART_SPECTRUM_RX_SIZE) == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(rx->uart->hdmarx, DMA_IT_HT);
  }
}

void uart_init(UART_HandleTypeDef *uart1, UART_HandleTypeDef *uart2)
{
  uint8_t i;

  s_rx1.uart = uart1;
  s_rx1.port_id = 1U;
  s_rx1.tx_len = 0U;
  s_rx1.tx_pending = 0U;

  s_rx2.uart = uart2;
  s_rx2.port_id = 2U;
  s_rx2.tx_len = 0U;
  s_rx2.tx_pending = 0U;

  for (i = 0U; i < MODBUS_BAND_COUNT; i++)
  {
    s_bands_raw[i] = 0U;
  }
  s_bands_last_ms = 0U;
  s_bands_src = 0U;
  s_rx_count = 0U;

  UartSpectrum_StartRx(&s_rx1);
  UartSpectrum_StartRx(&s_rx2);
  s_initialized = 1U;
}

void uart_tick(void)
{
  if (s_initialized == 0U)
  {
    return;
  }

  if ((s_rx1.tx_pending != 0U) && (s_rx1.uart != NULL))
  {
    if (HAL_UART_Transmit(s_rx1.uart, s_rx1.tx_buf, s_rx1.tx_len, 20U) == HAL_OK)
    {
      s_rx1.tx_pending = 0U;
    }
  }
  if ((s_rx2.tx_pending != 0U) && (s_rx2.uart != NULL))
  {
    if (HAL_UART_Transmit(s_rx2.uart, s_rx2.tx_buf, s_rx2.tx_len, 20U) == HAL_OK)
    {
      s_rx2.tx_pending = 0U;
    }
  }
}

void uart_get_bands(uint8_t bands[12], uint32_t *last_ms, uint8_t *src_port)
{
  uint32_t primask;
  uint8_t i;

  if (bands == NULL)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  for (i = 0U; i < MODBUS_BAND_COUNT; i++)
  {
    bands[i] = s_bands_raw[i];
  }
  if (last_ms != NULL)
  {
    *last_ms = s_bands_last_ms;
  }
  if (src_port != NULL)
  {
    *src_port = s_bands_src;
  }
  __set_PRIMASK(primask);
}

uint32_t uart_get_rx_count(void)
{
  uint32_t primask;
  uint32_t count;

  primask = __get_PRIMASK();
  __disable_irq();
  count = s_rx_count;
  __set_PRIMASK(primask);

  return count;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint16_t use_len;

  if (s_initialized == 0U)
  {
    return;
  }

  if (huart == s_rx1.uart)
  {
    use_len = Size;
    if (use_len > UART_SPECTRUM_RX_SIZE)
    {
      use_len = UART_SPECTRUM_RX_SIZE;
    }
    if (use_len > 0U)
    {
      s_rx_count++;
      Modbus_ParseBuffer(&s_rx1, s_rx1.rx_buf, use_len);
    }
    UartSpectrum_StartRx(&s_rx1);
  }
  else if (huart == s_rx2.uart)
  {
    use_len = Size;
    if (use_len > UART_SPECTRUM_RX_SIZE)
    {
      use_len = UART_SPECTRUM_RX_SIZE;
    }
    if (use_len > 0U)
    {
      s_rx_count++;
      Modbus_ParseBuffer(&s_rx2, s_rx2.rx_buf, use_len);
    }
    UartSpectrum_StartRx(&s_rx2);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (s_initialized == 0U)
  {
    return;
  }

  if (huart == s_rx1.uart)
  {
    UartSpectrum_StartRx(&s_rx1);
  }
  else if (huart == s_rx2.uart)
  {
    UartSpectrum_StartRx(&s_rx2);
  }
}
