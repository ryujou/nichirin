#ifndef LED_CTRL_H
#define LED_CTRL_H

#include <stdbool.h>
#include <stdint.h>

void LedCtrl_Init(void);
void LedCtrl_Start(void);
void LedCtrl_10msTick(void);
void LedCtrl_SetMode(uint8_t mode_1_9);
uint8_t LedCtrl_GetMode(void);
void LedCtrl_SetLevel(uint8_t mode_1_9, uint8_t level_1_7);
uint8_t LedCtrl_GetLevel(uint8_t mode_1_9);
void LedCtrl_RequestSave(void);
void LedCtrl_SaveNow(void);
bool LedCtrl_IsDirty(void);
void LedCtrl_SetOutputEnabled(uint8_t enabled);

#endif /* LED_CTRL_H */
