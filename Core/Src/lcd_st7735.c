#include "lcd_st7735.h"
#include "font.h"
#include "st7735.h"
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;

_lcd_dev  lcddev;
static volatile uint8_t s_lcd_dma_busy = 0U;

void TFT_WRITE_DATA(uint8_t data)
{
	while (s_lcd_dma_busy != 0U)
	{
	}
	CS_LOW;
	DATA;
	(void)HAL_SPI_Transmit(&hspi1, &data, 1U, HAL_MAX_DELAY);
	CS_HIGH;
}

void TFT_WRITE_COMMAND(uint8_t command)
{
	while (s_lcd_dma_busy != 0U)
	{
	}
	CS_LOW;
	COMMAND;
	(void)HAL_SPI_Transmit(&hspi1, &command, 1U, HAL_MAX_DELAY);
	CS_HIGH;
}

void TFT_WRITE_REG(uint8_t reg,uint16_t reg_value)
{
	TFT_WRITE_COMMAND(reg);
	TFT_WRITE_DATA(reg_value);
}

void TFT_WRITE_u16_DATA(uint16_t temp)
{

		uint8_t tempBuf[2];
		tempBuf[0] = temp>>8;
		tempBuf[1] = temp;
		while (s_lcd_dma_busy != 0U)
		{
		}
		CS_LOW ;
		DATA ;
		(void)HAL_SPI_Transmit(&hspi1, tempBuf, 2U, HAL_MAX_DELAY);
		CS_HIGH ;
}


//Set of coordinate for start point
void set_windows(uint16_t xStar,uint16_t yStar,uint16_t xEnd,uint16_t yEnd)
{
	TFT_WRITE_COMMAND(lcddev.setxcmd);
	TFT_WRITE_DATA(0x00);
	TFT_WRITE_DATA(xStar+lcddev.xoffset);
	TFT_WRITE_DATA(0x00);
	TFT_WRITE_DATA(xEnd+lcddev.xoffset);

	TFT_WRITE_COMMAND(lcddev.setycmd);
	TFT_WRITE_DATA(0x00);
	TFT_WRITE_DATA(yStar+lcddev.yoffset);
	TFT_WRITE_DATA(0x00);
	TFT_WRITE_DATA(yEnd+lcddev.yoffset);

	TFT_WRITE_COMMAND(0X2C);
}

//Clear screen to single color
void lcd_clear(uint16_t color)
{
	uint16_t i,m;
	set_windows(0,0,lcddev.width-1,lcddev.height-1);
	DATA;
	CS_LOW;

	for(i=0;i<lcddev.height;i++)
		{
	    for(m=0;m<lcddev.width;m++)
	    {
	    	TFT_WRITE_u16_DATA(color);
			}
		}
	CS_HIGH;
}

//设置方向 默认为横屏幕
void  lcd_set_direction()
{
	lcddev.setxcmd=LCD_X;
	lcddev.setycmd=LCD_Y;
	lcddev.wramcmd=LCD_CMD;
	lcddev.width=LCD_Width;
	lcddev.height=LCD_Height;
	lcddev.xoffset=LCD_XOffset;
	lcddev.yoffset=LCD_YOffset;
	lcddev.dir=LCD_DIR;
}

void tft_init()
{

	RSTL;
	HAL_Delay(100);
	RSTH;
	HAL_Delay(100);

	TFT_WRITE_COMMAND(0x11);
	HAL_Delay(120);

	TFT_WRITE_COMMAND(0xB1);
	TFT_WRITE_DATA(0x01);
	TFT_WRITE_DATA(0x2C);
	TFT_WRITE_DATA(0x2D);

	TFT_WRITE_COMMAND(0xB2);
	TFT_WRITE_DATA(0x01);
	TFT_WRITE_DATA(0x2C);
	TFT_WRITE_DATA(0x2D);


	TFT_WRITE_COMMAND(0xB3);
	TFT_WRITE_DATA(0x01);
	TFT_WRITE_DATA(0x2C);
	TFT_WRITE_DATA(0x2D);
	TFT_WRITE_DATA(0x01);
	TFT_WRITE_DATA(0x2C);
	TFT_WRITE_DATA(0x2D);

	TFT_WRITE_COMMAND(0xB4);
	TFT_WRITE_DATA(0x07);

	TFT_WRITE_COMMAND(0xC0);
	TFT_WRITE_DATA(0xA2);
	TFT_WRITE_DATA(0x02);
	TFT_WRITE_DATA(0x84);
	TFT_WRITE_COMMAND(0xC1);
	TFT_WRITE_DATA(0xC5);


	TFT_WRITE_COMMAND(0xC2);
	TFT_WRITE_DATA(0x0A);
	TFT_WRITE_DATA(0x00);

	TFT_WRITE_COMMAND(0xC3);
	TFT_WRITE_DATA(0x8A);
	TFT_WRITE_DATA(0x2A);
	TFT_WRITE_COMMAND(0xC4);
	TFT_WRITE_DATA(0x8A);
	TFT_WRITE_DATA(0xEE);

	TFT_WRITE_COMMAND(0xC5);
	TFT_WRITE_DATA(0x0E);

	TFT_WRITE_COMMAND(0x36);
	TFT_WRITE_DATA(0xC8);

	TFT_WRITE_COMMAND(0xe0);
	TFT_WRITE_DATA(0x0F);
	TFT_WRITE_DATA(0x1A);
	TFT_WRITE_DATA(0x0F);
	TFT_WRITE_DATA(0x18);
	TFT_WRITE_DATA(0x2F);
	TFT_WRITE_DATA(0x28);

	TFT_WRITE_DATA(0x20);
	TFT_WRITE_DATA(0x22);
	TFT_WRITE_DATA(0x1F);

	TFT_WRITE_DATA(0x1B);
	TFT_WRITE_DATA(0x23);
	TFT_WRITE_DATA(0x37);

	TFT_WRITE_DATA(0x00);

	TFT_WRITE_DATA(0x07);
	TFT_WRITE_DATA(0x02);
	TFT_WRITE_DATA(0x10);

	TFT_WRITE_COMMAND(0xe1);
	TFT_WRITE_DATA(0x0F);
	TFT_WRITE_DATA(0x1B);
	TFT_WRITE_DATA(0x0F);

	TFT_WRITE_DATA(0x17);

	TFT_WRITE_DATA(0x33);
	TFT_WRITE_DATA(0x2C);

	TFT_WRITE_DATA(0x29);

	TFT_WRITE_DATA(0x2E);
	TFT_WRITE_DATA(0x28);
	TFT_WRITE_DATA(0x30);

	TFT_WRITE_DATA(0x30);

	TFT_WRITE_DATA(0x39);
	TFT_WRITE_DATA(0x3F);

	TFT_WRITE_DATA(0x00);
	TFT_WRITE_DATA(0x07);
	TFT_WRITE_DATA(0x03);
	TFT_WRITE_DATA(0x10);

	TFT_WRITE_COMMAND(0x2A);
	TFT_WRITE_DATA(0x00);
	TFT_WRITE_DATA(0x00);
	TFT_WRITE_DATA(0x00);
	TFT_WRITE_DATA(0x7F);

	TFT_WRITE_COMMAND(0x2B);
	TFT_WRITE_DATA(0x00);
	TFT_WRITE_DATA(0x00);
	TFT_WRITE_DATA(0x00);
	TFT_WRITE_DATA(0x9F);

	TFT_WRITE_COMMAND(0xF0);
	TFT_WRITE_DATA(0x01);
	TFT_WRITE_COMMAND(0xF6);
	TFT_WRITE_DATA(0x00);

	TFT_WRITE_COMMAND(0x3A);
	TFT_WRITE_DATA(0x05);
	TFT_WRITE_COMMAND(0x29);
	lcd_set_direction();

	TFT_WRITE_REG(0x36,(1<<3)|(1<<6)|(1<<7));

	TFT_WRITE_COMMAND(0xc0);
	TFT_WRITE_DATA(0x0A);

	LEDH;
}

//Display point in (x, y) with color
void draw_point(uint16_t x,uint16_t y,uint16_t color)
{
	set_windows(x,y,x,y);
	TFT_WRITE_u16_DATA(color);
}

//Display ch (char type) in (x, y) with back_color, front_color and front_size. font.h is essential.
void TFT_ShowChar(uint16_t x, uint16_t y, char ch, uint16_t back_color, uint16_t font_color, uint8_t font_size)
{
	int i = 0, j = 0;
	uint8_t temp = 0;
	uint8_t size = 0;
	uint8_t t = 0;

	 if((x > (LCD_Width - font_size / 2)) || (y > (LCD_Height - font_size)))
		 return;

	 set_windows(x, y, x + font_size/2 - 1, y + font_size - 1);

	 uint8_t ch_index = (uint8_t)(ch - ' ');


	 if((font_size == 16) || (font_size == 32) )
	 {
 			size = (font_size / 8 + ((font_size % 8) ? 1 : 0)) * (font_size / 2);

			for(i = 0; i < size; i++)
			{
					if(font_size == 16)
						temp = asc2_1608[ch_index][i];
					else if(font_size == 32)
							temp = asc2_3216[ch_index][i];
					else
							return;

					for(j = 0; j < 8; j++)
					{
							if(temp & 0x80)
								{

									TFT_WRITE_u16_DATA(font_color);
								}
							else
							{

								TFT_WRITE_u16_DATA(back_color);
							}
							temp <<= 1;
					}
			}
	 }
	 else if(font_size == 12)
	 {
 			size = (font_size / 8 + ((font_size % 8) ? 1 : 0)) * (font_size / 2);

			for(i = 0; i < size; i++)
			{
				  temp = asc2_1206[ch_index][i];

					for(j = 0; j < 6; j++)
					{
							if(temp & 0x80)
							{
								TFT_WRITE_u16_DATA(font_color);
							}
							else
							{
								TFT_WRITE_u16_DATA(back_color);
							}

							temp <<= 1;
					}
			}
	 }
	 else if(font_size == 24)
	 {
 			size = (font_size * 16) / 8;

			for(i = 0; i < size; i++)
			{
				  temp = asc2_2412[ch_index][i];
					if(i % 2 == 0)
							t = 8;
					else
							t = 4;
					for(j = 0; j < t; j++)
					{
							if(temp & 0x80)
							{

								TFT_WRITE_u16_DATA(font_color);
							}
							else
							{

								TFT_WRITE_u16_DATA(back_color);
							}

							temp <<= 1;
					}
			}
	 }
	 else
		 return;
}

//Display str (string type) in (x, y) with back_color, front_color and front_size under max_width. font.h is essential.
void LCD_ShowCharStr(uint16_t x, uint16_t y, uint32_t max_width, char* str, uint16_t back_color, uint16_t font_color, uint8_t font_size)
{

	max_width += x;

	while((*str <= '~') && (*str >= ' '))
	{
			if(x >= max_width)
			{
					break;
			}

			TFT_ShowChar(x,y,*str,back_color, font_color,font_size);
			x += font_size / 2;
			str++;
	}
}


//Display number (byte type) in (x, y) with back_color, front_color and front_size under max_width. font.h is essential.
void LCD_ShowCharNumber(uint16_t x, uint16_t y, uint32_t max_width, uint8_t  number, uint16_t back_color, uint16_t font_color, uint8_t font_size)
{
	char number_ascii[10];

	sprintf(number_ascii,"%d",number);

	LCD_ShowCharStr(x,y,max_width,number_ascii,back_color,font_color,font_size);
}

static void lcd_write_command_blocking(uint8_t cmd)
{
	while (s_lcd_dma_busy != 0U)
	{
	}
	CS_LOW;
	COMMAND;
	(void)HAL_SPI_Transmit(&hspi1, &cmd, 1U, HAL_MAX_DELAY);
	CS_HIGH;
}

static void lcd_write_data_blocking(const uint8_t *data, uint16_t len)
{
	if ((data == NULL) || (len == 0U))
	{
		return;
	}
	while (s_lcd_dma_busy != 0U)
	{
	}
	CS_LOW;
	DATA;
	(void)HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, HAL_MAX_DELAY);
	CS_HIGH;
}

void ST7735_Init(void)
{
	s_lcd_dma_busy = 0U;
	tft_init();
}

void ST7735_SetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	uint16_t xs = (uint16_t)(x0 + LCD_XOffset);
	uint16_t xe = (uint16_t)(x1 + LCD_XOffset);
	uint16_t ys = (uint16_t)(y0 + LCD_YOffset);
	uint16_t ye = (uint16_t)(y1 + LCD_YOffset);
	uint8_t data[4];

	lcd_write_command_blocking(0x2A);
	data[0] = (uint8_t)(xs >> 8);
	data[1] = (uint8_t)xs;
	data[2] = (uint8_t)(xe >> 8);
	data[3] = (uint8_t)xe;
	lcd_write_data_blocking(data, 4U);

	lcd_write_command_blocking(0x2B);
	data[0] = (uint8_t)(ys >> 8);
	data[1] = (uint8_t)ys;
	data[2] = (uint8_t)(ye >> 8);
	data[3] = (uint8_t)ye;
	lcd_write_data_blocking(data, 4U);
}

bool ST7735_DMA_Busy(void)
{
	return (s_lcd_dma_busy != 0U);
}

static bool lcd_write_pixels_dma_internal(const uint8_t *data, uint16_t len, uint8_t send_cmd)
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
	CS_LOW;
	if (send_cmd != 0U)
	{
		uint8_t cmd = 0x2C;
		COMMAND;
		if (HAL_SPI_Transmit(&hspi1, &cmd, 1U, HAL_MAX_DELAY) != HAL_OK)
		{
			CS_HIGH;
			s_lcd_dma_busy = 0U;
			return false;
		}
	}
	DATA;
	if (HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)data, len) != HAL_OK)
	{
		CS_HIGH;
		s_lcd_dma_busy = 0U;
		return false;
	}
	return true;
}

bool ST7735_WritePixels_DMA(const uint8_t *data, uint16_t len)
{
	return lcd_write_pixels_dma_internal(data, len, 1U);
}

bool ST7735_WritePixels_DMA_Continue(const uint8_t *data, uint16_t len)
{
	return lcd_write_pixels_dma_internal(data, len, 0U);
}

void ST7735_OnSpiTxDmaDone(void)
{
	CS_HIGH;
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

