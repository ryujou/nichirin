/* ST7735 80x160 SPI driver with TX DMA support */
#include "st7735.h"
#include "main.h"

#define ST7735_SWRESET 0x01
#define ST7735_SLPOUT  0x11
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36
#define ST7735_CASET   0x2A
#define ST7735_PASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_DISPON  0x29

#define ST7735_MADCTL_RGB 0x00
#define ST7735_MADCTL_BGR 0x08
#define ST7735_MADCTL_MX  0x40
#define ST7735_MADCTL_MY  0x80
#define ST7735_MADCTL_MV  0x20

#define LCD_SPI_HANDLE hspi1
extern SPI_HandleTypeDef hspi1;

static volatile uint8_t s_lcd_dma_busy = 0U;

#define CS_L()  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET)
#define CS_H()  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET)
#define DC_L()  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET)
#define DC_H()  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET)
#define RST_L() HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_RESET)
#define RST_H() HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_SET)
#define BLK_L() HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, GPIO_PIN_RESET)
#define BLK_H() HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, GPIO_PIN_SET)

static void ST7735_WriteCommand(uint8_t cmd)
{
  while (s_lcd_dma_busy != 0U)
  {
  }
  CS_L();
  DC_L();
  (void)HAL_SPI_Transmit(&LCD_SPI_HANDLE, &cmd, 1U, HAL_MAX_DELAY);
  CS_H();
}

static void ST7735_WriteData(const uint8_t *data, uint16_t len)
{
  if ((data == NULL) || (len == 0U))
  {
    return;
  }
  while (s_lcd_dma_busy != 0U)
  {
  }
  CS_L();
  DC_H();
  (void)HAL_SPI_Transmit(&LCD_SPI_HANDLE, (uint8_t *)data, len, HAL_MAX_DELAY);
  CS_H();
}

static void ST7735_Reset(void)
{
  RST_L();
  HAL_Delay(5);
  RST_H();
  HAL_Delay(120);
}

void ST7735_Init(void)
{
  CS_H();
  DC_H();
  BLK_L();
  ST7735_Reset();

  ST7735_WriteCommand(ST7735_SWRESET);
  HAL_Delay(150);

  ST7735_WriteCommand(ST7735_SLPOUT);
  HAL_Delay(120);

  {
    uint8_t colmod = 0x55;
    ST7735_WriteCommand(ST7735_COLMOD);
    ST7735_WriteData(&colmod, 1U);
  }

  {
    uint8_t madctl = (uint8_t)(ST7735_MADCTL_RGB | ST7735_MADCTL_MX | ST7735_MADCTL_MY);
    ST7735_WriteCommand(ST7735_MADCTL);
    ST7735_WriteData(&madctl, 1U);
  }

  ST7735_WriteCommand(ST7735_DISPON);
  HAL_Delay(20);
  BLK_H();
}

void ST7735_SetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  uint16_t xs = (uint16_t)(x0 + ST7735_X_OFFSET);
  uint16_t xe = (uint16_t)(x1 + ST7735_X_OFFSET);
  uint16_t ys = (uint16_t)(y0 + ST7735_Y_OFFSET);
  uint16_t ye = (uint16_t)(y1 + ST7735_Y_OFFSET);
  uint8_t data[4];

  ST7735_WriteCommand(ST7735_CASET);
  data[0] = (uint8_t)(xs >> 8);
  data[1] = (uint8_t)xs;
  data[2] = (uint8_t)(xe >> 8);
  data[3] = (uint8_t)xe;
  ST7735_WriteData(data, 4U);

  ST7735_WriteCommand(ST7735_PASET);
  data[0] = (uint8_t)(ys >> 8);
  data[1] = (uint8_t)ys;
  data[2] = (uint8_t)(ye >> 8);
  data[3] = (uint8_t)ye;
  ST7735_WriteData(data, 4U);
}

bool ST7735_DMA_Busy(void)
{
  return (s_lcd_dma_busy != 0U);
}

static bool ST7735_WritePixels_DMA_Internal(const uint8_t *data, uint16_t len, uint8_t send_cmd)
{
  if ((data == NULL) || (len == 0U))
  {
    return false;
  }
  if (s_lcd_dma_busy != 0U)
  {
    return false;
  }

  s_lcd_dma_busy = 1U;
  CS_L();
  if (send_cmd != 0U)
  {
    DC_L();
    {
      uint8_t cmd = ST7735_RAMWR;
      if (HAL_SPI_Transmit(&LCD_SPI_HANDLE, &cmd, 1U, HAL_MAX_DELAY) != HAL_OK)
      {
        CS_H();
        s_lcd_dma_busy = 0U;
        return false;
      }
    }
  }
  DC_H();
  if (HAL_SPI_Transmit_DMA(&LCD_SPI_HANDLE, (uint8_t *)data, len) != HAL_OK)
  {
    CS_H();
    s_lcd_dma_busy = 0U;
    return false;
  }
  return true;
}

bool ST7735_WritePixels_DMA(const uint8_t *data, uint16_t len)
{
  return ST7735_WritePixels_DMA_Internal(data, len, 1U);
}

bool ST7735_WritePixels_DMA_Continue(const uint8_t *data, uint16_t len)
{
  return ST7735_WritePixels_DMA_Internal(data, len, 0U);
}

void ST7735_OnSpiTxDmaDone(void)
{
  CS_H();
  s_lcd_dma_busy = 0U;
}

void ST7735_FillColor(uint16_t color)
{
  uint8_t line[ST7735_WIDTH * 2U];
  uint8_t hi = (uint8_t)(color >> 8);
  uint8_t lo = (uint8_t)color;
  for (uint16_t i = 0; i < ST7735_WIDTH; i++)
  {
    line[i * 2U] = hi;
    line[i * 2U + 1U] = lo;
  }

  ST7735_SetAddrWindow(0U, 0U, ST7735_WIDTH - 1U, ST7735_HEIGHT - 1U);
  for (uint16_t y = 0U; y < ST7735_HEIGHT; y++)
  {
    while (!ST7735_WritePixels_DMA(line, (uint16_t)sizeof(line)))
    {
    }
    while (ST7735_DMA_Busy())
    {
    }
  }
}
