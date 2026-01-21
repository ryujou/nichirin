/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : soft_i2c.h
  * @brief          : Software I2C (bit-bang) driver
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef DRIVERS_SOFT_I2C_H
#define DRIVERS_SOFT_I2C_H

#include <stdbool.h>
#include <stdint.h>

void soft_i2c_init(void);
bool soft_i2c_start(void);
void soft_i2c_stop(void);
bool soft_i2c_write_byte(uint8_t byte);
bool soft_i2c_write_reg(uint8_t addr7, uint8_t reg, const uint8_t *data, uint16_t len);
bool soft_i2c_write_u8(uint8_t addr7, uint8_t reg, uint8_t val);

#endif /* DRIVERS_SOFT_I2C_H */
