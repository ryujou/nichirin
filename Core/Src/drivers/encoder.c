/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : encoder.c
  * @brief          : Rotary encoder sampling and key events
  ******************************************************************************
  */
/* USER CODE END Header */
#include "drivers/encoder.h"

#define ENCODER_DEBOUNCE_MAX 8U
#define ENCODER_LONGPRESS_MS 600U
/* Use 4 for full-step encoders, or 2 for half-step encoders. */
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

/* Function: Encoder_ReadAB
 * Purpose: Read current A/B GPIO levels as a 2-bit state.
 * Inputs: None (uses configured GPIO).
 * Outputs: 2-bit state: bit1=A, bit0=B.
 */
static uint8_t Encoder_ReadAB(void)
{
  uint8_t a = (HAL_GPIO_ReadPin(s_a_port, s_a_pin) == GPIO_PIN_SET) ? 1U : 0U;
  uint8_t b = (HAL_GPIO_ReadPin(s_b_port, s_b_pin) == GPIO_PIN_SET) ? 1U : 0U;
  return (uint8_t)((a << 1U) | b);
}

/* Function: Encoder_ReadKeyPressed
 * Purpose: Read the encoder key level and convert to pressed/released.
 * Inputs: None (uses configured GPIO).
 * Outputs: 1 if pressed, 0 if released.
 */
static uint8_t Encoder_ReadKeyPressed(void)
{
  return (HAL_GPIO_ReadPin(s_k_port, s_k_pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

/* Function: Encoder_Init
 * Purpose: Initialize encoder GPIO bindings and internal state.
 * Inputs: a_port/a_pin, b_port/b_pin, k_port/k_pin - GPIO mapping.
 * Outputs: None (initializes module state).
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

/* Function: Encoder_1msTick
 * Purpose: Sample A/B and key every 1 ms to update delta/events.
 * Inputs: None (reads GPIO).
 * Outputs: None (updates internal accumulators).
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

/* Function: Encoder_GetDelta
 * Purpose: Fetch and clear the accumulated detent delta.
 * Inputs: None.
 * Outputs: Signed delta (steps) since last call.
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

/* Function: Encoder_GetKeyEvent
 * Purpose: Fetch and clear the last key event.
 * Inputs: None.
 * Outputs: EncoderEvent (none/click/longpress).
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
