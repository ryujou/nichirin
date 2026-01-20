#include "utils/led_utils.h"
#include "drivers/ws2812_dma.h"

uint8_t led_breath_from_phase(uint16_t phase)
{
  uint8_t t = (uint8_t)(phase & 0xFFU);
  if (phase & 0x100U)
  {
    t = (uint8_t)(255U - t);
  }
  return t;
}

void led_hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v,
                    uint8_t *r, uint8_t *g, uint8_t *b)
{
  uint16_t region;
  uint16_t rem;
  uint16_t p;
  uint16_t q;
  uint16_t t;

  if (s == 0U)
  {
    *r = v;
    *g = v;
    *b = v;
    return;
  }

  region = (uint16_t)(h / 60U);
  rem = (uint16_t)(h % 60U);

  p = (uint16_t)v * (255U - s) / 255U;
  q = (uint16_t)v * (255U - (s * rem) / 60U) / 255U;
  t = (uint16_t)v * (255U - (s * (60U - rem)) / 60U) / 255U;

  switch (region)
  {
    case 0:
      *r = v;
      *g = (uint8_t)t;
      *b = (uint8_t)p;
      break;
    case 1:
      *r = (uint8_t)q;
      *g = v;
      *b = (uint8_t)p;
      break;
    case 2:
      *r = (uint8_t)p;
      *g = v;
      *b = (uint8_t)t;
      break;
    case 3:
      *r = (uint8_t)p;
      *g = (uint8_t)q;
      *b = v;
      break;
    case 4:
      *r = (uint8_t)t;
      *g = (uint8_t)p;
      *b = v;
      break;
    default:
      *r = v;
      *g = (uint8_t)p;
      *b = (uint8_t)q;
      break;
  }
}

void led_rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b,
                    uint16_t *h, uint8_t *s, uint8_t *v)
{
  uint8_t max = r;
  uint8_t min = r;
  uint16_t delta;
  int16_t hue;

  if (g > max) { max = g; }
  if (b > max) { max = b; }
  if (g < min) { min = g; }
  if (b < min) { min = b; }

  *v = max;
  delta = (uint16_t)(max - min);
  if (max == 0U)
  {
    *s = 0U;
    *h = 0U;
    return;
  }

  *s = (uint8_t)((delta * 255U) / max);
  if (delta == 0U)
  {
    *h = 0U;
    return;
  }

  if (max == r)
  {
    hue = (int16_t)(60 * (int16_t)(g - b) / (int16_t)delta);
  }
  else if (max == g)
  {
    hue = (int16_t)(120 + 60 * (int16_t)(b - r) / (int16_t)delta);
  }
  else
  {
    hue = (int16_t)(240 + 60 * (int16_t)(r - g) / (int16_t)delta);
  }

  if (hue < 0)
  {
    hue += 360;
  }
  *h = (uint16_t)hue;
}

static void led_ws2812_encode_byte(uint8_t value, uint16_t *pwm,
                                   uint32_t *idx, uint32_t max_len)
{
  uint8_t bit;
  for (bit = 0U; bit < 8U; bit++)
  {
    if ((*idx) >= max_len)
    {
      return;
    }
    pwm[(*idx)++] = (value & 0x80U) ? WS2812_CCR_1 : WS2812_CCR_0;
    value <<= 1U;
  }
}

void led_ws2812_encode_rgb(uint8_t r, uint8_t g, uint8_t b,
                           uint16_t *pwm, uint32_t *idx, uint32_t max_len)
{
  if ((*idx) >= max_len)
  {
    return;
  }
  led_ws2812_encode_byte(g, pwm, idx, max_len);
  led_ws2812_encode_byte(r, pwm, idx, max_len);
  led_ws2812_encode_byte(b, pwm, idx, max_len);
}
