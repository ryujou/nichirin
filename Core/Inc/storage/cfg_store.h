#ifndef CFG_STORE_H
#define CFG_STORE_H

#include "storage/flash_cfg.h"
#include <stdbool.h>

bool Cfg_Load(Config *out);
bool Cfg_SaveAtomic(const Config *in);
void Cfg_ResetToDefault(Config *out);

#endif /* CFG_STORE_H */
