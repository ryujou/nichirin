/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : app.h
  * @brief          : WS2812 framebuffer demo
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef APP_H
#define APP_H

#include <stdint.h>

typedef enum
{
  APP_MODBUS_OK = 0,
  APP_MODBUS_IGNORED = 1,
  APP_MODBUS_ILLEGAL_ADDR = 2
} AppModbusStatus;

void app_init(void);
void app_loop(void);

void led_group_set_rgb(uint8_t group, uint8_t r, uint8_t g, uint8_t b);
void led_group_set_hsv(uint8_t group, uint16_t h, uint8_t s, uint8_t v);
void led_group_set_brightness(uint8_t group, uint8_t br);
void led_group_fill(uint8_t group, uint8_t r, uint8_t g, uint8_t b);

AppModbusStatus app_modbus_read_reg(uint16_t reg, uint16_t *out);
AppModbusStatus app_modbus_write_reg(uint16_t reg, uint16_t value, uint32_t now_ms);

#endif /* APP_H */
