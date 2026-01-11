#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include <stdbool.h>
#include <stdint.h>

#define CONFIG_VERSION 1U

typedef struct
{
  uint8_t mode;
  uint8_t run_state;
  uint8_t param_level[10];
  uint8_t reserved[21];
} ConfigV1;

typedef ConfigV1 Config;

void ConfigStore_InitDefaults(Config *out);
bool ConfigStore_Load(Config *out);
bool ConfigStore_Save(const Config *in);

#endif /* CONFIG_STORE_H */
