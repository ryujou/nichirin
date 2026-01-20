/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : uart.c
  * @brief          : UART spectrum RX (USART1/USART2) using DMA + IDLE
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

#include <string.h>

#define UART_SPECTRUM_RX_SIZE 256U
#define SPECTRUM_FRAME_LEN 16U
#define SPECTRUM_ADDR 0x01U
#define SPECTRUM_FUNC 0x20U
#define SPECTRUM_BANDS 12U

typedef struct
{
  uint8_t buf[SPECTRUM_FRAME_LEN];
  uint8_t count;
} SpectrumParser;

typedef struct
{
  UART_HandleTypeDef *uart;
  uint8_t rx_buf[UART_SPECTRUM_RX_SIZE];
  SpectrumParser parser;
  uint8_t port_id;
} UartSpectrumRx;

static UartSpectrumRx s_rx1;
static UartSpectrumRx s_rx2;
static uint8_t s_initialized = 0U;

static volatile uint8_t s_bands_raw[SPECTRUM_BANDS];
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

static void UartSpectrum_StoreBands(const uint8_t *data, uint8_t port_id)
{
  uint32_t primask = __get_PRIMASK();
  uint8_t i;

  __disable_irq();
  for (i = 0U; i < SPECTRUM_BANDS; i++)
  {
    s_bands_raw[i] = data[i];
  }
  s_bands_last_ms = HAL_GetTick();
  s_bands_src = port_id;
  __set_PRIMASK(primask);
}

static void UartSpectrum_ParseByte(SpectrumParser *parser, uint8_t byte, uint8_t port_id)
{
  parser->buf[parser->count++] = byte;
  if (parser->count < SPECTRUM_FRAME_LEN)
  {
    return;
  }

  if ((parser->buf[0] == SPECTRUM_ADDR) && (parser->buf[1] == SPECTRUM_FUNC))
  {
    uint16_t crc = UartSpectrum_Crc16(parser->buf, (uint16_t)(SPECTRUM_FRAME_LEN - 2U));
    uint16_t rx_crc = (uint16_t)parser->buf[SPECTRUM_FRAME_LEN - 2U]
                      | ((uint16_t)parser->buf[SPECTRUM_FRAME_LEN - 1U] << 8U);
    if (crc == rx_crc)
    {
      UartSpectrum_StoreBands(&parser->buf[2], port_id);
      parser->count = 0U;
      return;
    }
  }

  (void)memmove(&parser->buf[0], &parser->buf[1], (size_t)(SPECTRUM_FRAME_LEN - 1U));
  parser->count = (uint8_t)(SPECTRUM_FRAME_LEN - 1U);
}

static void UartSpectrum_Feed(UartSpectrumRx *rx, const uint8_t *data, uint16_t len)
{
  uint16_t i;

  for (i = 0U; i < len; i++)
  {
    UartSpectrum_ParseByte(&rx->parser, data[i], rx->port_id);
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
  s_rx1.parser.count = 0U;

  s_rx2.uart = uart2;
  s_rx2.port_id = 2U;
  s_rx2.parser.count = 0U;

  for (i = 0U; i < SPECTRUM_BANDS; i++)
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
  (void)s_initialized;
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
  for (i = 0U; i < SPECTRUM_BANDS; i++)
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
      UartSpectrum_Feed(&s_rx1, s_rx1.rx_buf, use_len);
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
      UartSpectrum_Feed(&s_rx2, s_rx2.rx_buf, use_len);
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
