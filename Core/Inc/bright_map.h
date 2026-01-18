#ifndef BRIGHT_MAP_H
#define BRIGHT_MAP_H

#include <stdint.h>

typedef enum
{
  BRIGHT_PROFILE_GAMMA22 = 0,
  BRIGHT_PROFILE_GAMMA26 = 1,
  BRIGHT_PROFILE_LOWBOOST = 2
} BrightProfile;

typedef struct
{
  uint8_t profile;
  uint8_t lowboost_strength;
} BrightCalib;

uint8_t Bright_Map(uint8_t level, const BrightCalib *c);

#endif /* BRIGHT_MAP_H */
