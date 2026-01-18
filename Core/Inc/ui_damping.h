#ifndef UI_DAMPING_H
#define UI_DAMPING_H

#include <stdint.h>

typedef enum
{
  DAMP_PARAM_BRIGHTNESS = 0,
  DAMP_PARAM_SPEED,
  DAMP_PARAM_LOWBOOST,
  DAMP_PARAM_BREATH_SHAPE,
  DAMP_PARAM_FLASH_PERIOD,
  DAMP_PARAM_PHASE_OFFSET,
  DAMP_PARAM_COUNT
} DampingParam;

int16_t Damping_Apply(DampingParam id, int16_t delta, uint16_t value,
                      uint16_t min, uint16_t max, uint16_t zone, uint16_t gain);
void Damping_Reset(DampingParam id);
void Damping_ResetAll(void);

#endif /* UI_DAMPING_H */
