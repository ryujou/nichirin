/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : tlc59116.c
   * @brief          : TLC59116 16-channel LED driver (soft I2C)
  ******************************************************************************
  */
/* USER CODE END Header */
#include "drivers/tlc59116.h"
#include "drivers/soft_i2c.h"
#include <stddef.h>

#define TLC59116_REG_MODE1   0x00U
#define TLC59116_REG_MODE2   0x01U
#define TLC59116_REG_PWM0    0x02U
#define TLC59116_REG_GRPPWM  0x12U
#define TLC59116_REG_GRPFREQ 0x13U
#define TLC59116_REG_LEDOUT0 0x14U
#define TLC59116_REG_LEDOUT1 0x15U
#define TLC59116_REG_LEDOUT2 0x16U
#define TLC59116_REG_LEDOUT3 0x17U

static bool TLC59116_WriteReg(uint8_t reg, uint8_t val)
{
  return soft_i2c_write_u8(TLC_ADDR_7B, reg, val);
}

static bool TLC59116_WriteBlock(uint8_t reg, const uint8_t *data, uint16_t len)
{
  return soft_i2c_write_reg(TLC_ADDR_7B, reg, data, len);
}

bool TLC59116_InitSoft(void)
{
  /* Basic Arduino-like init sequence. */
  if (!TLC59116_WriteReg(TLC59116_REG_MODE1, 0x00U))
  {
    return false;
  }
  if (!TLC59116_WriteReg(TLC59116_REG_MODE2, 0x00U))
  {
    return false;
  }

  /* Enable auto-increment for burst writes. */
  if (!TLC59116_WriteReg(TLC59116_REG_MODE1, 0x20U))
  {
    return false;
  }

  if (!TLC59116_SetAllPWM(0x00U))
  {
    return false;
  }

  (void)TLC59116_WriteReg(TLC59116_REG_GRPPWM, 0xFFU);
  (void)TLC59116_WriteReg(TLC59116_REG_GRPFREQ, 0x00U);

  if (!TLC59116_WriteReg(TLC59116_REG_LEDOUT0, 0x55U))
  {
    return false;
  }
  if (!TLC59116_WriteReg(TLC59116_REG_LEDOUT1, 0x55U))
  {
    return false;
  }
  if (!TLC59116_WriteReg(TLC59116_REG_LEDOUT2, 0x55U))
  {
    return false;
  }
  if (!TLC59116_WriteReg(TLC59116_REG_LEDOUT3, 0x00U))
  {
    return false;
  }

  return true;
}

bool TLC59116_SetAllPWM(uint8_t pwm)
{
  uint8_t data[16];
  for (uint8_t i = 0; i < 16U; i++)
  {
    data[i] = pwm;
  }
  return TLC59116_WriteBlock(TLC59116_REG_PWM0, data, (uint16_t)sizeof(data));
}

bool TLC59116_SetAllPWMMode(void)
{
  /* LEDOUT0..2 = 0xAA => OUT0..OUT11 PWM; LEDOUT3 unused. */
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
  return true;
}

bool TLC59116_SetPWM(uint8_t ch, uint8_t val)
{
  if (ch >= 16U)
  {
    return false;
  }
  return TLC59116_WriteReg((uint8_t)(TLC59116_REG_PWM0 + ch), val);
}

bool TLC59116_SetPWM12(const uint8_t pwm12[12])
{
  if (pwm12 == NULL)
  {
    return false;
  }

  uint8_t data[16] = {0};
  for (uint8_t i = 0; i < 12U; i++)
  {
    data[i] = pwm12[i];
  }
  return TLC59116_WriteBlock(TLC59116_REG_PWM0, data, (uint16_t)sizeof(data));
}

bool TLC59116_SetPWM12_Single(const uint8_t pwm12[12])
{
  if (pwm12 == NULL)
  {
    return false;
  }

  for (uint8_t i = 0; i < 12U; i++)
  {
    if (!TLC59116_SetPWM(i, pwm12[i]))
    {
      return false;
    }
  }
  return true;
}

bool TLC59116_SetAllOn(void)
{
  /* LEDOUT0..2 = 0x55 => OUT0..OUT11 ON; LEDOUT3 unused. */
  if (!TLC59116_WriteReg(TLC59116_REG_LEDOUT0, 0x55U))
  {
    return false;
  }
  if (!TLC59116_WriteReg(TLC59116_REG_LEDOUT1, 0x55U))
  {
    return false;
  }
  if (!TLC59116_WriteReg(TLC59116_REG_LEDOUT2, 0x55U))
  {
    return false;
  }
  if (!TLC59116_WriteReg(TLC59116_REG_LEDOUT3, 0x00U))
  {
    return false;
  }
  return true;
}

bool TLC59116_SetAllOff(void)
{
  if (!TLC59116_WriteReg(TLC59116_REG_LEDOUT0, 0x00U))
  {
    return false;
  }
  if (!TLC59116_WriteReg(TLC59116_REG_LEDOUT1, 0x00U))
  {
    return false;
  }
  if (!TLC59116_WriteReg(TLC59116_REG_LEDOUT2, 0x00U))
  {
    return false;
  }
  if (!TLC59116_WriteReg(TLC59116_REG_LEDOUT3, 0x00U))
  {
    return false;
  }
  return true;
}
