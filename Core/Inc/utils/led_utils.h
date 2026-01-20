#ifndef LED_UTILS_H
#define LED_UTILS_H

#include <stdint.h>

void led_hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v,
                    uint8_t *r, uint8_t *g, uint8_t *b);

void led_rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b,
                    uint16_t *h, uint8_t *s, uint8_t *v);

uint8_t led_breath_from_phase(uint16_t phase);

/* WS2812 encode: GRB order */
void led_ws2812_encode_rgb(uint8_t r, uint8_t g, uint8_t b,
                           uint16_t *pwm, uint32_t *idx, uint32_t max_len);

#endif /* LED_UTILS_H */
