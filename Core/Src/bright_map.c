#include "bright_map.h"
#include <stddef.h>

static const uint8_t LUT_gamma22[256] = {
    0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   1U,
    1U,   1U,   1U,   1U,   1U,   1U,   1U,   1U,   1U,   2U,   2U,   2U,   2U,   2U,   2U,   2U,
    3U,   3U,   3U,   3U,   3U,   4U,   4U,   4U,   4U,   5U,   5U,   5U,   5U,   6U,   6U,   6U,
    6U,   7U,   7U,   7U,   8U,   8U,   8U,   9U,   9U,   9U,  10U,  10U,  11U,  11U,  11U,  12U,
   12U,  13U,  13U,  13U,  14U,  14U,  15U,  15U,  16U,  16U,  17U,  17U,  18U,  18U,  19U,  19U,
   20U,  20U,  21U,  22U,  22U,  23U,  23U,  24U,  25U,  25U,  26U,  26U,  27U,  28U,  28U,  29U,
   30U,  30U,  31U,  32U,  33U,  33U,  34U,  35U,  35U,  36U,  37U,  38U,  39U,  39U,  40U,  41U,
   42U,  43U,  43U,  44U,  45U,  46U,  47U,  48U,  49U,  49U,  50U,  51U,  52U,  53U,  54U,  55U,
   56U,  57U,  58U,  59U,  60U,  61U,  62U,  63U,  64U,  65U,  66U,  67U,  68U,  69U,  70U,  71U,
   73U,  74U,  75U,  76U,  77U,  78U,  79U,  81U,  82U,  83U,  84U,  85U,  87U,  88U,  89U,  90U,
   91U,  93U,  94U,  95U,  97U,  98U,  99U, 100U, 102U, 103U, 105U, 106U, 107U, 109U, 110U, 111U,
  113U, 114U, 116U, 117U, 119U, 120U, 121U, 123U, 124U, 126U, 127U, 129U, 130U, 132U, 133U, 135U,
  137U, 138U, 140U, 141U, 143U, 145U, 146U, 148U, 149U, 151U, 153U, 154U, 156U, 158U, 159U, 161U,
  163U, 165U, 166U, 168U, 170U, 172U, 173U, 175U, 177U, 179U, 181U, 182U, 184U, 186U, 188U, 190U,
  192U, 194U, 196U, 197U, 199U, 201U, 203U, 205U, 207U, 209U, 211U, 213U, 215U, 217U, 219U, 221U,
  223U, 225U, 227U, 229U, 231U, 234U, 236U, 238U, 240U, 242U, 244U, 246U, 248U, 251U, 253U, 255U,
};

static const uint8_t LUT_gamma26[256] = {
    0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,
    0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   1U,   1U,   1U,   1U,   1U,   1U,   1U,   1U,
    1U,   1U,   1U,   1U,   2U,   2U,   2U,   2U,   2U,   2U,   2U,   2U,   3U,   3U,   3U,   3U,
    3U,   3U,   4U,   4U,   4U,   4U,   5U,   5U,   5U,   5U,   5U,   6U,   6U,   6U,   6U,   7U,
    7U,   7U,   8U,   8U,   8U,   9U,   9U,   9U,  10U,  10U,  10U,  11U,  11U,  11U,  12U,  12U,
   13U,  13U,  13U,  14U,  14U,  15U,  15U,  16U,  16U,  17U,  17U,  18U,  18U,  19U,  19U,  20U,
   20U,  21U,  21U,  22U,  22U,  23U,  24U,  24U,  25U,  25U,  26U,  27U,  27U,  28U,  29U,  29U,
   30U,  31U,  31U,  32U,  33U,  34U,  34U,  35U,  36U,  37U,  38U,  38U,  39U,  40U,  41U,  42U,
   42U,  43U,  44U,  45U,  46U,  47U,  48U,  49U,  50U,  51U,  52U,  53U,  54U,  55U,  56U,  57U,
   58U,  59U,  60U,  61U,  62U,  63U,  64U,  65U,  66U,  68U,  69U,  70U,  71U,  72U,  73U,  75U,
   76U,  77U,  78U,  80U,  81U,  82U,  84U,  85U,  86U,  88U,  89U,  90U,  92U,  93U,  94U,  96U,
   97U,  99U, 100U, 102U, 103U, 105U, 106U, 108U, 109U, 111U, 112U, 114U, 115U, 117U, 119U, 120U,
  122U, 124U, 125U, 127U, 129U, 130U, 132U, 134U, 136U, 137U, 139U, 141U, 143U, 145U, 146U, 148U,
  150U, 152U, 154U, 156U, 158U, 160U, 162U, 164U, 166U, 168U, 170U, 172U, 174U, 176U, 178U, 180U,
  182U, 184U, 186U, 188U, 191U, 193U, 195U, 197U, 199U, 202U, 204U, 206U, 209U, 211U, 213U, 215U,
  218U, 220U, 223U, 225U, 227U, 230U, 232U, 235U, 237U, 240U, 242U, 245U, 247U, 250U, 252U, 255U,
};

static uint8_t LowBoost_Map(uint8_t level, uint8_t strength)
{
  if (level == 0U)
  {
    return 0U;
  }

  {
    uint16_t inv = (uint16_t)(255U - level);
    uint16_t ease = (uint16_t)(255U - (uint16_t)(((uint32_t)inv * (uint32_t)inv + 127U) / 255U));
    uint16_t out = (uint16_t)(((uint32_t)(255U - strength) * level + (uint32_t)strength * ease + 127U) / 255U);
    return (uint8_t)out;
  }
}

uint8_t Bright_Map(uint8_t level, const BrightCalib *c)
{
  if (c == NULL)
  {
    return level;
  }

  switch (c->profile)
  {
    case BRIGHT_PROFILE_GAMMA26:
      return LUT_gamma26[level];
    case BRIGHT_PROFILE_LOWBOOST:
      return LowBoost_Map(level, c->lowboost_strength);
    case BRIGHT_PROFILE_GAMMA22:
    default:
      return LUT_gamma22[level];
  }
}
