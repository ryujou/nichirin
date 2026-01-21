/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : soft_i2c.c
  * @brief          : Software I2C (bit-bang) driver
  ******************************************************************************
  */
/* USER CODE END Header */
#include "drivers/soft_i2c.h"
#include "main.h"
#include "stm32g0xx_hal.h"

/* Half-period delay (tune for ~100 kHz). */
static inline void i2c_delay_half(void)
{
  for (volatile uint32_t i = 0; i < 40U; i++)
  {
    __NOP();
  }
}

static void sda_out(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SDA_GPIO_Port, &GPIO_InitStruct);
}

static void sda_in(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDA_GPIO_Port, &GPIO_InitStruct);
}

static void scl_out(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SCL_GPIO_Port, &GPIO_InitStruct);
}

static void sda_high(void)
{
  HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, GPIO_PIN_SET);
}

static void sda_low(void)
{
  HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, GPIO_PIN_RESET);
}

static void scl_high(void)
{
  HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_SET);
}

static void scl_low(void)
{
  HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_RESET);
}

void soft_i2c_init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();

  scl_out();
  sda_out();

  /* Release bus lines high. */
  scl_high();
  sda_high();
  i2c_delay_half();
}

bool soft_i2c_start(void)
{
  sda_out();
  sda_high();
  scl_high();
  i2c_delay_half();

  /* Start: SDA high -> low while SCL high. */
  sda_low();
  i2c_delay_half();
  scl_low();
  i2c_delay_half();
  return true;
}

void soft_i2c_stop(void)
{
  sda_out();
  sda_low();
  i2c_delay_half();
  scl_high();
  i2c_delay_half();
  sda_high();
  i2c_delay_half();
}

bool soft_i2c_write_byte(uint8_t byte)
{
  sda_out();
  for (uint8_t i = 0; i < 8U; i++)
  {
    if ((byte & 0x80U) != 0U)
    {
      sda_high();
    }
    else
    {
      sda_low();
    }
    i2c_delay_half();
    scl_high();
    i2c_delay_half();
    scl_low();
    i2c_delay_half();
    byte <<= 1U;
  }

  /* ACK bit: release SDA and sample on 9th clock. */
  sda_in();
  i2c_delay_half();
  scl_high();
  i2c_delay_half();
  GPIO_PinState ack = HAL_GPIO_ReadPin(SDA_GPIO_Port, SDA_Pin);
  scl_low();
  i2c_delay_half();
  sda_out();

  return (ack == GPIO_PIN_RESET);
}

bool soft_i2c_write_reg(uint8_t addr7, uint8_t reg, const uint8_t *data, uint16_t len)
{
  if ((data == NULL) || (len == 0U))
  {
    return false;
  }

  if (!soft_i2c_start())
  {
    return false;
  }

  if (!soft_i2c_write_byte((uint8_t)(addr7 << 1)))
  {
    soft_i2c_stop();
    return false;
  }

  if (!soft_i2c_write_byte(reg))
  {
    soft_i2c_stop();
    return false;
  }

  for (uint16_t i = 0; i < len; i++)
  {
    if (!soft_i2c_write_byte(data[i]))
    {
      soft_i2c_stop();
      return false;
    }
  }

  soft_i2c_stop();
  return true;
}

bool soft_i2c_write_u8(uint8_t addr7, uint8_t reg, uint8_t val)
{
  return soft_i2c_write_reg(addr7, reg, &val, 1U);
}
