/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : encoder.c
  * @brief          : 旋转编码器采样与按键事件
  ******************************************************************************
  */
/* USER CODE END Header */
#include "drivers/encoder.h"

#define ENCODER_DEBOUNCE_MAX 8U
#define ENCODER_LONGPRESS_MS 600U
/* 全步进编码器取 4，半步进编码器取 2。 */
#define ENCODER_EDGES_PER_STEP 4

static GPIO_TypeDef *s_a_port;
static GPIO_TypeDef *s_b_port;
static GPIO_TypeDef *s_k_port;
static uint16_t s_a_pin;
static uint16_t s_b_pin;
static uint16_t s_k_pin;
static uint8_t s_prev_state;
static int8_t s_edge_accum;
static volatile int8_t s_detent_delta;
static uint8_t s_key_integrator;
static uint8_t s_key_stable;
static uint8_t s_key_last;
static uint16_t s_key_press_ms;
static uint8_t s_key_long_fired;
static volatile EncoderEvent s_key_event;
static uint8_t s_initialized;

/* 函数: Encoder_ReadAB
 * 功能: 读取 A/B 电平组成 2-bit 状态。
 * 输入: 无 (使用已配置 GPIO)。
 * 输出: 2-bit 状态: bit1=A, bit0=B。
 */
static uint8_t Encoder_ReadAB(void)
{
  uint8_t a = (HAL_GPIO_ReadPin(s_a_port, s_a_pin) == GPIO_PIN_SET) ? 1U : 0U;
  uint8_t b = (HAL_GPIO_ReadPin(s_b_port, s_b_pin) == GPIO_PIN_SET) ? 1U : 0U;
  return (uint8_t)((a << 1U) | b);
}

/* 函数: Encoder_ReadKeyPressed
 * 功能: 读取编码器按键电平并转换为按下/松开。
 * 输入: 无 (使用已配置 GPIO)。
 * 输出: 按下返回 1，松开返回 0。
 */
static uint8_t Encoder_ReadKeyPressed(void)
{
  return (HAL_GPIO_ReadPin(s_k_port, s_k_pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

/* 函数: Encoder_Init
 * 功能: 初始化编码器 GPIO 绑定与内部状态。
 * 输入: a_port/a_pin, b_port/b_pin, k_port/k_pin - GPIO 映射。
 * 输出: 无 (初始化模块状态)。
 */
void Encoder_Init(GPIO_TypeDef *a_port, uint16_t a_pin,
                  GPIO_TypeDef *b_port, uint16_t b_pin,
                  GPIO_TypeDef *k_port, uint16_t k_pin)
{
  s_a_port = a_port;
  s_b_port = b_port;
  s_k_port = k_port;
  s_a_pin = a_pin;
  s_b_pin = b_pin;
  s_k_pin = k_pin;
  s_prev_state = Encoder_ReadAB();
  s_edge_accum = 0;
  s_detent_delta = 0;
  s_key_integrator = 0;
  s_key_stable = Encoder_ReadKeyPressed();
  s_key_last = s_key_stable;
  s_key_press_ms = 0;
  s_key_long_fired = 0U;
  s_key_event = ENC_EVENT_NONE;
  s_initialized = 1U;
}

/* 函数: Encoder_1msTick
 * 功能: 每 1 ms 采样 A/B 和按键，更新增量与事件。
 * 输入: 无 (读取 GPIO)。
 * 输出: 无 (更新内部累加器)。
 */
void Encoder_1msTick(void)
{
  if (s_initialized == 0U)
  {
    return;
  }

  {
    static const int8_t table[16] = {
      0, -1,  1,  0,
      1,  0,  0, -1,
     -1,  0,  0,  1,
      0,  1, -1,  0
    };
    uint8_t cur = Encoder_ReadAB();
    uint8_t idx = (uint8_t)((s_prev_state << 2U) | cur);
    int8_t delta = table[idx];
    s_prev_state = cur;
    if (delta != 0)
    {
      s_edge_accum = (int8_t)(s_edge_accum + delta);
      if (s_edge_accum >= ENCODER_EDGES_PER_STEP)
      {
        s_edge_accum = (int8_t)(s_edge_accum - ENCODER_EDGES_PER_STEP);
        s_detent_delta++;
      }
      else if (s_edge_accum <= -ENCODER_EDGES_PER_STEP)
      {
        s_edge_accum = (int8_t)(s_edge_accum + ENCODER_EDGES_PER_STEP);
        s_detent_delta--;
      }
    }
  }

  {
    uint8_t pressed = Encoder_ReadKeyPressed();
    if (pressed != 0U)
    {
      if (s_key_integrator < ENCODER_DEBOUNCE_MAX)
      {
        s_key_integrator++;
      }
    }
    else
    {
      if (s_key_integrator > 0U)
      {
        s_key_integrator--;
      }
    }

    if (s_key_integrator == 0U)
    {
      s_key_stable = 0U;
    }
    else if (s_key_integrator >= ENCODER_DEBOUNCE_MAX)
    {
      s_key_stable = 1U;
    }

    if (s_key_stable != 0U)
    {
      if (s_key_press_ms < ENCODER_LONGPRESS_MS)
      {
        s_key_press_ms++;
      }
      if ((s_key_press_ms >= ENCODER_LONGPRESS_MS) && (s_key_long_fired == 0U))
      {
        s_key_long_fired = 1U;
        if (s_key_event == ENC_EVENT_NONE)
        {
          s_key_event = ENC_EVENT_LONGPRESS;
        }
      }
    }
    else
    {
      if ((s_key_last != 0U) && (s_key_long_fired == 0U) && (s_key_press_ms > 0U))
      {
        if (s_key_event == ENC_EVENT_NONE)
        {
          s_key_event = ENC_EVENT_CLICK;
        }
      }
      s_key_press_ms = 0U;
      s_key_long_fired = 0U;
    }
    s_key_last = s_key_stable;
  }
}

/* 函数: Encoder_GetDelta
 * 功能: 读取并清零累计的步进增量。
 * 输入: 无。
 * 输出: 自上次调用以来的步进增量。
 */
int8_t Encoder_GetDelta(void)
{
  int8_t delta;
  __disable_irq();
  delta = s_detent_delta;
  s_detent_delta = 0;
  __enable_irq();
  return delta;
}

/* 函数: Encoder_GetKeyEvent
 * 功能: 读取并清零按键事件。
 * 输入: 无。
 * 输出: EncoderEvent (无/单击/长按)。
 */
EncoderEvent Encoder_GetKeyEvent(void)
{
  EncoderEvent ev;
  __disable_irq();
  ev = s_key_event;
  s_key_event = ENC_EVENT_NONE;
  __enable_irq();
  return ev;
}
