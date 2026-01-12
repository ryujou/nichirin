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

  /* Frame rate control */
  {
    uint8_t data[3];
    ST7735_WriteCommand(0xB1);
    data[0] = 0x01;
    data[1] = 0x2C;
    data[2] = 0x2D;
    ST7735_WriteData(data, 3U);
    ST7735_WriteCommand(0xB2);
    ST7735_WriteData(data, 3U);
    ST7735_WriteCommand(0xB3);
    ST7735_WriteData(data, 3U);
    ST7735_WriteData(data, 3U);
  }

  /* Display inversion control */
  {
    uint8_t data = 0x07;
    ST7735_WriteCommand(0xB4);
    ST7735_WriteData(&data, 1U);
  }

  /* Power control */
  {
    uint8_t data[3];
    ST7735_WriteCommand(0xC0);
    data[0] = 0xA2;
    data[1] = 0x02;
    data[2] = 0x84;
    ST7735_WriteData(data, 3U);

    ST7735_WriteCommand(0xC1);
    data[0] = 0xC5;
    ST7735_WriteData(data, 1U);

    ST7735_WriteCommand(0xC2);
    data[0] = 0x0A;
    data[1] = 0x00;
    ST7735_WriteData(data, 2U);

    ST7735_WriteCommand(0xC3);
    data[0] = 0x8A;
    data[1] = 0x2A;
    ST7735_WriteData(data, 2U);

    ST7735_WriteCommand(0xC4);
    data[0] = 0x8A;
    data[1] = 0xEE;
    ST7735_WriteData(data, 2U);

    ST7735_WriteCommand(0xC5);
    data[0] = 0x0E;
    ST7735_WriteData(data, 1U);
  }

  {
    uint8_t madctl = (uint8_t)(ST7735_MADCTL_BGR | ST7735_MADCTL_MX | ST7735_MADCTL_MY);
    ST7735_WriteCommand(ST7735_MADCTL);
    ST7735_WriteData(&madctl, 1U);
  }

  /* Gamma correction */
  {
    uint8_t data[16] = {
      0x0F, 0x1A, 0x0F, 0x18, 0x2F, 0x28, 0x20, 0x22,
      0x1F, 0x1B, 0x23, 0x37, 0x00, 0x07, 0x02, 0x10
    };
    ST7735_WriteCommand(0xE0);
    ST7735_WriteData(data, 16U);
    data[0] = 0x0F;
    data[1] = 0x1B;
    data[2] = 0x0F;
    data[3] = 0x17;
    data[4] = 0x33;
    data[5] = 0x2C;
    data[6] = 0x29;
    data[7] = 0x2E;
    data[8] = 0x28;
    data[9] = 0x30;
    data[10] = 0x30;
    data[11] = 0x39;
    data[12] = 0x3F;
    data[13] = 0x00;
    data[14] = 0x07;
    data[15] = 0x03;
    ST7735_WriteCommand(0xE1);
    ST7735_WriteData(data, 16U);
    {
      uint8_t extra = 0x10;
      ST7735_WriteData(&extra, 1U);
    }
  }

  {
    uint8_t data[4];
    ST7735_WriteCommand(ST7735_CASET);
    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x7F;
    ST7735_WriteData(data, 4U);
    ST7735_WriteCommand(ST7735_PASET);
    data[3] = 0x9F;
    ST7735_WriteData(data, 4U);
  }

  {
    uint8_t data = 0x01;
    ST7735_WriteCommand(0xF0);
    ST7735_WriteData(&data, 1U);
    data = 0x00;
    ST7735_WriteCommand(0xF6);
    ST7735_WriteData(&data, 1U);
  }

  {
    uint8_t colmod = 0x55;
    ST7735_WriteCommand(ST7735_COLMOD);
    ST7735_WriteData(&colmod, 1U);
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
