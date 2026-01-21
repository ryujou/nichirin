#include "storage/cfg_store.h"
#include "storage/flash_cfg.h"
#include "stm32g0xx_hal.h"
#include <stddef.h>

__weak void CfgStore_OnCommit(void) {}

bool Cfg_Load(Config *out)
{
  return FlashCfg_Load(out);
}

bool Cfg_SaveAtomic(const Config *in)
{
  return FlashCfg_Append(in);
}

void Cfg_ResetToDefault(Config *out)
{
  FlashCfg_InitDefaults(out);
}

bool CfgStore_Load(Config *out)
{
  if (out == NULL)
  {
    return false;
  }

  if (FlashCfg_Load(out))
  {
    return true;
  }

  FlashCfg_InitDefaults(out);
  return FlashCfg_Append(out);
}

void CfgStore_MarkDirty(const Config *cur, uint32_t now_ms)
{
  (void)cur;
  (void)now_ms;
}

void CfgStore_PollCommit(const Config *cur, uint32_t now_ms)
{
  (void)cur;
  (void)now_ms;
}
