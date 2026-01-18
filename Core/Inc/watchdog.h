#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "stm32g0xx_hal.h"
#include <stdint.h>

typedef enum
{
  WDG_TASK_ENCODER_1MS = 0,
  WDG_TASK_EFFECT_10MS,
  WDG_TASK_TLC_UPDATE,
  WDG_TASK_FLASH_SAVE,
  WDG_TASK_COUNT
} WDG_TaskId;

typedef enum
{
  WDG_RESET_UNKNOWN = 0,
  WDG_RESET_IWDG = 1,
  WDG_RESET_OTHER = 2
} WDG_ResetReason;

void WDG_Init(IWDG_HandleTypeDef *hiwdg);
void WDG_TaskKick_Notify(uint32_t task_id);
void WDG_Tick10ms(void);
void WDG_ForceReset_Test(void);

extern volatile WDG_ResetReason wdg_last_reset_reason;

WDG_ResetReason WDG_GetLastResetReason(void);
uint8_t WDG_WasIwdgReset(void);

#endif /* WATCHDOG_H */
