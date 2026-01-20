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

void app_init(void);
void app_loop(void);

void led_group_set_rgb(uint8_t group, uint8_t r, uint8_t g, uint8_t b);
void led_group_set_hsv(uint8_t group, uint16_t h, uint8_t s, uint8_t v);
void led_group_set_brightness(uint8_t group, uint8_t br);
void led_group_fill(uint8_t group, uint8_t r, uint8_t g, uint8_t b);

#endif /* APP_H */
