#include "led/led_engine.h"
#include "main.h"
#include <math.h>

typedef struct
{
  uint16_t a;
  uint16_t b;
  uint16_t c;
} LedPortMask;

typedef enum
{
  LED_PORT_A = 0U,
  LED_PORT_B = 1U,
  LED_PORT_C = 2U
} LedPortIndex_t;

#define PWM_BITS 6U
#define PWM_STEPS (1U << PWM_BITS)
#define PWM_MAX (PWM_STEPS - 1U)
#define DITHER_BITS (8U - PWM_BITS)
#define DITHER_CYCLE (1U << DITHER_BITS)

#define LEDA_MASK (LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin)
#define LEDB_MASK (LED5_Pin | LED6_Pin | LED7_Pin | LED8_Pin | LED9_Pin | LED10_Pin | LED11_Pin)
#define LEDC_MASK (LED12_Pin)

static const LedPortIndex_t s_led_port_index[LED_COUNT] = {
  LED_PORT_A, LED_PORT_A, LED_PORT_A, LED_PORT_A,
  LED_PORT_B, LED_PORT_B, LED_PORT_B, LED_PORT_B, LED_PORT_B, LED_PORT_B, LED_PORT_B,
  LED_PORT_C
};

static const uint16_t s_led_pin_bits[LED_COUNT] = {
  LED1_Pin, LED2_Pin, LED3_Pin, LED4_Pin,
  LED5_Pin, LED6_Pin, LED7_Pin, LED8_Pin, LED9_Pin, LED10_Pin, LED11_Pin,
  LED12_Pin
};

static volatile uint8_t s_bam_bit = 0U;
static volatile uint8_t s_dither_phase = 0U;
static volatile uint8_t s_bam_bit_ticks_left = 0U;
static volatile uint8_t s_bam_table_ready = 0U;

static LedPortMask s_bam_buf_a[DITHER_CYCLE][PWM_BITS];
static LedPortMask s_bam_buf_b[DITHER_CYCLE][PWM_BITS];
static LedPortMask (*s_bam_cur)[PWM_BITS] = s_bam_buf_a;
static LedPortMask (*s_bam_next)[PWM_BITS] = s_bam_buf_b;

static uint8_t s_gamma_lut[256];

static inline void LedEngine_WriteMasks(const LedPortMask *on)
{
  uint16_t on_a = on->a;
  uint16_t off_a = (uint16_t)(LEDA_MASK & ~on_a);
  GPIOA->BSRR = (uint32_t)on_a | ((uint32_t)off_a << 16);

  uint16_t on_b = on->b;
  uint16_t off_b = (uint16_t)(LEDB_MASK & ~on_b);
  GPIOB->BSRR = (uint32_t)on_b | ((uint32_t)off_b << 16);

  uint16_t on_c = on->c;
  uint16_t off_c = (uint16_t)(LEDC_MASK & ~on_c);
  GPIOC->BSRR = (uint32_t)on_c | ((uint32_t)off_c << 16);
}

static void LedEngine_GammaInit(void)
{
  const float gamma = 2.2f;
  for (uint16_t i = 0U; i < 256U; i++)
  {
    float x = (float)i / 255.0f;
    float y = powf(x, gamma);
    uint32_t duty = (uint32_t)(y * 255.0f + 0.5f);
    if (duty > 255U)
    {
      duty = 255U;
    }
    s_gamma_lut[i] = (uint8_t)duty;
  }
}

static void LedEngine_ClearTable(LedPortMask table[DITHER_CYCLE][PWM_BITS])
{
  for (uint8_t d = 0U; d < DITHER_CYCLE; d++)
  {
    for (uint8_t b = 0U; b < PWM_BITS; b++)
    {
      table[d][b].a = 0U;
      table[d][b].b = 0U;
      table[d][b].c = 0U;
    }
  }
}

static void LedEngine_BuildTable(const uint8_t cmd[LED_COUNT])
{
  uint8_t duty_next[LED_COUNT];
  for (uint8_t i = 0U; i < LED_COUNT; i++)
  {
    duty_next[i] = s_gamma_lut[cmd[i]];
  }

  LedEngine_ClearTable(s_bam_next);

  for (uint8_t i = 0U; i < LED_COUNT; i++)
  {
    uint32_t scaled = ((uint32_t)duty_next[i] * (PWM_MAX * DITHER_CYCLE) + 127U) / 255U;
    uint8_t base = (uint8_t)(scaled >> DITHER_BITS);
    uint8_t frac = (uint8_t)(scaled & (DITHER_CYCLE - 1U));
    for (uint8_t d = 0U; d < DITHER_CYCLE; d++)
    {
      uint8_t value = base;
      if ((base < PWM_MAX) && (d < frac))
      {
        value = (uint8_t)(base + 1U);
      }
      if (value > PWM_MAX)
      {
        value = PWM_MAX;
      }
      for (uint8_t b = 0U; b < PWM_BITS; b++)
      {
        if ((value >> b) & 0x1U)
        {
          switch (s_led_port_index[i])
          {
            case LED_PORT_A:
              s_bam_next[d][b].a |= s_led_pin_bits[i];
              break;
            case LED_PORT_B:
              s_bam_next[d][b].b |= s_led_pin_bits[i];
              break;
            case LED_PORT_C:
              s_bam_next[d][b].c |= s_led_pin_bits[i];
              break;
            default:
              break;
          }
        }
      }
    }
  }

  __disable_irq();
  s_bam_table_ready = 1U;
  __enable_irq();
}

void LedEngine_Init(void)
{
  LedEngine_GammaInit();
  LedEngine_ClearTable(s_bam_buf_a);
  LedEngine_ClearTable(s_bam_buf_b);
  s_bam_cur = s_bam_buf_a;
  s_bam_next = s_bam_buf_b;
  s_bam_bit = 0U;
  s_dither_phase = 0U;
  s_bam_bit_ticks_left = 0U;
  s_bam_table_ready = 0U;
}

void LedEngine_UpdateCmd(const uint8_t cmd_0_255[LED_COUNT])
{
  if (cmd_0_255 == NULL)
  {
    return;
  }
  __disable_irq();
  if (s_bam_table_ready != 0U)
  {
    __enable_irq();
    return;
  }
  __enable_irq();

  LedEngine_BuildTable(cmd_0_255);
}

void LedEngine_Tim6Isr(void)
{
  if (s_bam_bit_ticks_left == 0U)
  {
    s_bam_bit_ticks_left = (uint8_t)(1U << s_bam_bit);
  }

  LedEngine_WriteMasks(&s_bam_cur[s_dither_phase][s_bam_bit]);

  s_bam_bit_ticks_left--;
  if (s_bam_bit_ticks_left == 0U)
  {
    s_bam_bit++;
    if (s_bam_bit >= PWM_BITS)
    {
      s_bam_bit = 0U;
      s_dither_phase++;
      if (s_dither_phase >= DITHER_CYCLE)
      {
        s_dither_phase = 0U;
      }
      if (s_bam_table_ready != 0U)
      {
        LedPortMask (*tmp)[PWM_BITS] = s_bam_cur;
        s_bam_cur = s_bam_next;
        s_bam_next = tmp;
        s_bam_table_ready = 0U;
        s_dither_phase = 0U;
      }
    }
  }
}
