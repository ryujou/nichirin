/*
 *
 * For 0.96 inch TFT LCD driven by ST7735S
 *
 */
#ifndef _lcd_st7735_h
#define _lcd_st7735_h

#include "main.h"
/* TFT LCD Module Connection:
 MODULE          STM32G431CBU6
 CS                 PA4
 DC(DATA/COMMAND)   PB0
 BACK_LED           PA8
 RESET              PB1
 SCK                PA5 (SPI1 CLK)
 SDA                PA7 (SPI1 MOSI)
 VCC                3.3V/5V
 GND                0V
 */

#define CS_GPIO LCD_CS_Pin
#define CS_GPIO_PORT LCD_CS_GPIO_Port

#define CMD_DATA_GPIO LCD_DC_Pin
#define CMD_DATA_GPIO_PORT LCD_DC_GPIO_Port

#define BACK_LED_GPIO LCD_BLK_Pin
#define BACK_LED_GPIO_PORT LCD_BLK_GPIO_Port

#define RESET_GPIO LCD_RES_Pin
#define RESET_GPIO_PORT LCD_RES_GPIO_Port


#define CS_HIGH   HAL_GPIO_WritePin(CS_GPIO_PORT, CS_GPIO, GPIO_PIN_SET)
#define CS_LOW    HAL_GPIO_WritePin(CS_GPIO_PORT, CS_GPIO, GPIO_PIN_RESET)

#define DATA      HAL_GPIO_WritePin(CMD_DATA_GPIO_PORT, CMD_DATA_GPIO, GPIO_PIN_SET)
#define COMMAND   HAL_GPIO_WritePin(CMD_DATA_GPIO_PORT, CMD_DATA_GPIO, GPIO_PIN_RESET)

#define RSTH      HAL_GPIO_WritePin(RESET_GPIO_PORT, RESET_GPIO, GPIO_PIN_SET)
#define RSTL      HAL_GPIO_WritePin(RESET_GPIO_PORT, RESET_GPIO, GPIO_PIN_RESET)

#define LEDH      HAL_GPIO_WritePin(BACK_LED_GPIO_PORT, BACK_LED_GPIO, GPIO_PIN_SET)
#define LEDL      HAL_GPIO_WritePin(BACK_LED_GPIO_PORT, BACK_LED_GPIO, GPIO_PIN_RESET)

//LCD colors used as inversion of RGB565
#define WHITE     ((uint16_t)~0xFFFFu)
#define BLACK     ((uint16_t)~0x0000u)
#define RED       ((uint16_t)~0xF800u)
#define GREEN     ((uint16_t)~0x07E0u)
#define BLUE      ((uint16_t)~0x001Fu)
#define YELLOW    ((uint16_t)~0xFFE0u)
#define PURPLE    ((uint16_t)~0xF81Fu)
#define MAGENTA   ((uint16_t)~0xF81Fu)
#define NAVY      ((uint16_t)~0x000Fu)
#define DGREEN    ((uint16_t)~0x03E0u)
#define DCYAN     ((uint16_t)~0x03EFu)
#define MAROON    ((uint16_t)~0x7800u)
#define OLIVE     ((uint16_t)~0x7BE0u)
#define LGRAY     ((uint16_t)~0xC618u)
#define DGRAY     ((uint16_t)~0x7BEFu)
#define CYAN      ((uint16_t)~0x07FFu)

 //LCD parameters
#define LCD_Width 80
#define LCD_Height 160
#define LCD_X 0x2a
#define LCD_Y 0x2b
#define LCD_XOffset 0x18  //Adjust to correct display location of left-top point
#define LCD_YOffset 0x00  //Adjust to correct display location of left-top point
#define LCD_CMD 0x2c
#define LCD_DIR 0

 typedef struct
 {
 	uint16_t  width;			//LCD width
 	uint16_t  height;	        //LCD height
 	uint16_t  id;		        //LCD ID
 	uint8_t   dir;			    //LCD direction  0: vertical; 1: horizontal;
 	uint16_t  wramcmd;		    //cmd to write gram
 	uint16_t  setxcmd;		    //x coordinate
 	uint16_t  setycmd;		    //y coordinate
 	uint8_t   xoffset;          //x offset
 	uint8_t	  yoffset;          //y offset
 } _lcd_dev;

 void lcd_clear(uint16_t color);
 void tft_init();
 void draw_point(uint16_t x,uint16_t y,uint16_t color);
 void TFT_ShowChar(uint16_t x, uint16_t y, char ch, uint16_t back_color, uint16_t font_color, uint8_t font_size);
 void LCD_ShowCharNumber(uint16_t x, uint16_t y, uint32_t max_width, uint8_t  number, uint16_t back_color, uint16_t font_color, uint8_t font_size);
 void LCD_ShowCharStr(uint16_t x, uint16_t y, uint32_t max_width, char* str, uint16_t back_color, uint16_t font_color, uint8_t font_size);

#endif

