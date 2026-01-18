#include "ui_damping.h"

static uint16_t s_accum[DAMP_PARAM_COUNT];

int16_t Damping_Apply(DampingParam id, int16_t delta, uint16_t value,
                      uint16_t min, uint16_t max, uint16_t zone, uint16_t gain)
{
  if ((id >= DAMP_PARAM_COUNT) || (delta == 0) || (gain == 0U))
  {
    return delta;
  }

  if ((value <= (uint16_t)(min + zone)) && (delta < 0))
  {
    s_accum[id] = (uint16_t)(s_accum[id] + (uint16_t)(-delta));
  }
  else if ((value >= (uint16_t)(max - zone)) && (delta > 0))
  {
    s_accum[id] = (uint16_t)(s_accum[id] + (uint16_t)delta);
  }
  else
  {
    s_accum[id] = 0U;
    return delta;
  }

  if (s_accum[id] < gain)
  {
    return 0;
  }

  {
    uint16_t steps = (uint16_t)(s_accum[id] / gain);
    s_accum[id] = (uint16_t)(s_accum[id] % gain);
    return (delta > 0) ? (int16_t)steps : (int16_t)-(int16_t)steps;
  }
}

void Damping_Reset(DampingParam id)
{
  if (id < DAMP_PARAM_COUNT)
  {
    s_accum[id] = 0U;
  }
}

void Damping_ResetAll(void)
{
  for (uint8_t i = 0; i < (uint8_t)DAMP_PARAM_COUNT; i++)
  {
    s_accum[i] = 0U;
  }
}
