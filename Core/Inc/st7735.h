/* ST7735 80x160 SPI driver (RGB565) */
#ifndef ST7735_H
#define ST7735_H

#include <stdbool.h>
#include <stdint.h>

/* Physical resolution */
#define ST7735_WIDTH  80U
#define ST7735_HEIGHT 160U

/* Panel offset: adjust if image is shifted (common values: X=26, Y=1). */
#define ST7735_X_OFFSET 0U
#define ST7735_Y_OFFSET 0U

void ST7735_Init(void);
void ST7735_SetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void ST7735_FillColor(uint16_t color);

bool ST7735_DMA_Busy(void);
bool ST7735_WritePixels_DMA(const uint8_t *data, uint16_t len);
bool ST7735_WritePixels_DMA_Continue(const uint8_t *data, uint16_t len);
void ST7735_OnSpiTxDmaDone(void);

#endif /* ST7735_H */
