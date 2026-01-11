#ifndef LCD_PORT_H
#define LCD_PORT_H

#include "lvgl.h"
#include <stdint.h>

void LcdPort_Init(void);
void LcdPort_OnDmaTxComplete(void);
lv_display_t *LcdPort_GetDisplay(void);
void LcdPort_SetStreamingActive(uint8_t active);
uint8_t LcdPort_IsBusy(void);

#endif /* LCD_PORT_H */
