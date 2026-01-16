/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : tlc59116.c
  * @brief          : TLC59116 16 路 LED 驱动 (I2C)
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

/* 函数: TLC59116_WriteReg
 * 功能: 通过 I2C 写入 TLC59116 单个寄存器。
 * 输入: reg - 寄存器地址, val - 数据字节。
 * 输出: 成功返回 true, I2C 错误返回 false。
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

/* 函数: TLC59116_WritePWMBlock
 * 功能: 从 PWM0 开始写入连续 PWM 数据块。
 * 输入: pwm - 数据缓冲, len - 字节长度。
 * 输出: 成功返回 true, I2C 错误返回 false。
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

/* 函数: TLC59116_Init
 * 功能: 初始化 TLC59116 寄存器为 PWM 模式并清零输出。
 * 输入: hi2c - I2C 句柄。
 * 输出: 成功返回 true, 失败返回 false。
 */
bool TLC59116_Init(I2C_HandleTypeDef *hi2c)
{
  uint8_t pwm[16] = {0};
  s_hi2c = hi2c;
  if (s_hi2c == NULL)
  {
    return false;
  }

  /* MODE1: 使能自动地址递增。MODE2: 默认极性 (不反相)。 */
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

/* 函数: TLC59116_SetPWM
 * 功能: 设置单路 PWM。
 * 输入: ch - 通道 0..15, val - 0..255 占空比。
 * 输出: 成功返回 true, 失败返回 false。
 */
bool TLC59116_SetPWM(uint8_t ch, uint8_t val)
{
  if (ch >= 16U)
  {
    return false;
  }
  return TLC59116_WriteReg((uint8_t)(TLC59116_REG_PWM0 + ch), val);
}

/* 函数: TLC59116_SetPWM12
 * 功能: 一次 I2C 写入通道 0..11 的 PWM。
 * 输入: pwm12 - 12 路占空比数组。
 * 输出: 成功返回 true, 失败返回 false。
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

/* 函数: TLC59116_AllOff
 * 功能: 关闭所有 PWM 输出。
 * 输入: 无。
 * 输出: 无 (I2C 写入副作用)。
 */
void TLC59116_AllOff(void)
{
  uint8_t pwm[16] = {0};
  (void)TLC59116_WritePWMBlock(pwm, (uint16_t)sizeof(pwm));
}
