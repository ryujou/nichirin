#ifndef LED_ENGINE_H
#define LED_ENGINE_H

#include <stdint.h>
#include "led/led_types.h"

void LedEngine_Init(void);
void LedEngine_UpdateCmd(const uint8_t cmd_0_255[LED_COUNT]);
void LedEngine_Tim6Isr(void);

#endif /* LED_ENGINE_H */
