#ifndef BOOT_SELFTEST_H
#define BOOT_SELFTEST_H

#include "stm32g0xx_hal.h"
#include <stdint.h>

typedef enum
{
  BOOT_ERR_NONE = 0,
  BOOT_ERR_CFG_INVALID = 1U << 0,
  BOOT_ERR_I2C_LINES = 1U << 1,
  BOOT_ERR_TLC_ACK = 1U << 2,
  BOOT_ERR_ENCODER_WARN = 1U << 3
} BootErrorFlags;

typedef struct
{
  uint32_t error_flags;
  uint8_t last_error;
  uint8_t encoder_warn;
} BootDiag;

void BootSelfTest_Init(I2C_HandleTypeDef *hi2c);
void BootSelfTest_Tick10ms(void);
uint8_t BootSelfTest_IsDone(void);
uint8_t BootSelfTest_IsSafeMode(void);
void BootSelfTest_RequestRetry(void);
const BootDiag *BootSelfTest_GetDiag(void);

#endif /* BOOT_SELFTEST_H */
