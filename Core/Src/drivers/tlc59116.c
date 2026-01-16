/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : tlc59116.c
  * @brief          : TLC59116 16-channel LED driver (I2C)
  ******************************************************************************
  */
/* USER CODE END Header */
#include "drivers/tlc59116.h"

#define TLC59116_REG_MODE1   0x00U
#define TLC59116_REG_MODE2   0x01U
#define TLC59116_REG_PWM0    0x02U
#define TLC59116_REG_LEDOUT0 0x14U
#define TLC59116_REG_LEDOUT1 0x15U
#define TLC59116_REG_LEDOUT2 0x16U
#define TLC59116_REG_LEDOUT3 0x17U

static I2C_HandleTypeDef *s_hi2c;

/* Function: TLC59116_WriteReg
 * Purpose: Write a single TLC59116 register via I2C.
 * Inputs: reg - register address, val - data byte.
 * Outputs: true on success, false on I2C error.
 */
static bool TLC59116_WriteReg(uint8_t reg, uint8_t val)
{
  if (s_hi2c == NULL)
  {
    return false;
  }
  return (HAL_I2C_Mem_Write(s_hi2c, TLC59116_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            &val, 1U, TLC59116_I2C_TIMEOUT_MS) == HAL_OK);
}

/* Function: TLC59116_WritePWMBlock
 * Purpose: Write a contiguous PWM block starting at PWM0.
 * Inputs: pwm - data buffer, len - number of bytes.
 * Outputs: true on success, false on I2C error.
 */
static bool TLC59116_WritePWMBlock(const uint8_t *pwm, uint16_t len)
{
  if ((s_hi2c == NULL) || (pwm == NULL) || (len == 0U))
  {
    return false;
  }
  return (HAL_I2C_Mem_Write(s_hi2c, TLC59116_I2C_ADDR, TLC59116_REG_PWM0,
                            I2C_MEMADD_SIZE_8BIT, (uint8_t *)pwm, len,
                            TLC59116_I2C_TIMEOUT_MS) == HAL_OK);
}

/* Function: TLC59116_Init
 * Purpose: Initialize TLC59116 registers for PWM control and clear outputs.
 * Inputs: hi2c - I2C handle to use.
 * Outputs: true on success, false on failure.
 */
bool TLC59116_Init(I2C_HandleTypeDef *hi2c)
{
  uint8_t pwm[16] = {0};
  s_hi2c = hi2c;
  if (s_hi2c == NULL)
  {
    return false;
  }

  /* MODE1: enable auto-increment. MODE2: default polarity (non-inverted). */
  if (!TLC59116_WriteReg(TLC59116_REG_MODE1, 0x20U))
  {
    return false;
  }
  if (!TLC59116_WriteReg(TLC59116_REG_MODE2, 0x00U))
  {
    return false;
  }

  if (!TLC59116_WriteReg(TLC59116_REG_LEDOUT0, 0xAAU))
  {
    return false;
  }
  if (!TLC59116_WriteReg(TLC59116_REG_LEDOUT1, 0xAAU))
  {
    return false;
  }
  if (!TLC59116_WriteReg(TLC59116_REG_LEDOUT2, 0xAAU))
  {
    return false;
  }
  if (!TLC59116_WriteReg(TLC59116_REG_LEDOUT3, 0x00U))
  {
    return false;
  }

  return TLC59116_WritePWMBlock(pwm, (uint16_t)sizeof(pwm));
}

/* Function: TLC59116_SetPWM
 * Purpose: Set PWM for one channel.
 * Inputs: ch - channel index 0..15, val - 0..255 duty.
 * Outputs: true on success, false on failure.
 */
bool TLC59116_SetPWM(uint8_t ch, uint8_t val)
{
  if (ch >= 16U)
  {
    return false;
  }
  return TLC59116_WriteReg((uint8_t)(TLC59116_REG_PWM0 + ch), val);
}

/* Function: TLC59116_SetPWM12
 * Purpose: Set PWM for channels 0..11 in one I2C transaction.
 * Inputs: pwm12 - array of 12 duty values.
 * Outputs: true on success, false on failure.
 */
bool TLC59116_SetPWM12(const uint8_t pwm12[12])
{
  uint8_t pwm[16] = {0};
  if (pwm12 == NULL)
  {
    return false;
  }
  for (uint8_t i = 0; i < 12U; i++)
  {
    pwm[i] = pwm12[i];
  }
  return TLC59116_WritePWMBlock(pwm, (uint16_t)sizeof(pwm));
}

/* Function: TLC59116_AllOff
 * Purpose: Clear all PWM outputs to off.
 * Inputs: None.
 * Outputs: None (I2C write side effects).
 */
void TLC59116_AllOff(void)
{
  uint8_t pwm[16] = {0};
  (void)TLC59116_WritePWMBlock(pwm, (uint16_t)sizeof(pwm));
}
