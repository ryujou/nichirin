#ifndef CFG_STORE_H
#define CFG_STORE_H

#include "storage/flash_cfg.h"
#include <stdbool.h>
#include <stdint.h>

#define CFG_STORE_SAVE_DELAY_MS 5000U
#define CFG_STORE_RETRY_DELAY_MS 200U
#define CFG_STORE_DEBUG 0U

bool Cfg_Load(Config *out);
bool Cfg_SaveAtomic(const Config *in);
void Cfg_ResetToDefault(Config *out);
bool CfgStore_Load(Config *out);
void CfgStore_MarkDirty(const Config *cur, uint32_t now_ms);
void CfgStore_PollCommit(const Config *cur, uint32_t now_ms);
void CfgStore_OnCommit(void);

#endif /* CFG_STORE_H */
