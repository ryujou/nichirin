#ifndef LED_EFFECTS_H
#define LED_EFFECTS_H

#include <stdint.h>
#include "led/led_types.h"

void LedEffects_Init(void);
void LedEffects_SetMode(uint8_t mode);
void LedEffects_SetLevel(uint8_t mode, uint8_t level_1_to_7);
void LedEffects_Tick10ms(uint8_t out_cmd[LED_COUNT]);

#endif /* LED_EFFECTS_H */
