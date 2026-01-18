#include "storage/cfg_store.h"
#include "stm32g0xx_hal.h"
#include "fw_version.h"
#include "bright_map.h"
#include <string.h>

#ifndef CFG_PAGE_SIZE
#define CFG_PAGE_SIZE 0x800U
#endif

#ifndef CFG_SLOT0_ADDR
/* STM32G030F6Px 32 KB Flash: last two 2 KB pages */
#define CFG_SLOT0_ADDR 0x08007000UL
#endif

#ifndef CFG_SLOT1_ADDR
#define CFG_SLOT1_ADDR 0x08007800UL
#endif

#define CFG_MAGIC 0x32474643UL
#define CFG_VERSION 2U

#define LEGACY_MAGIC 0x31474643UL
#define LEGACY_VERSION 1U
#define LEGACY_RECORD_SIZE 64U
#define LEGACY_CRC_OFFSET 60U
#define LEGACY_HEADER_SIZE 12U

#define CFG_STATE_ERASED 0xFFFFFFFFUL
#define CFG_STATE_WRITING 0xFFFFFFFEUL
#define CFG_STATE_VALID 0xFFFFFFFCUL

#define CFG_HDR_MAGIC_OFF 0U
#define CFG_HDR_VERSION_OFF 4U
#define CFG_HDR_LEN_OFF 6U
#define CFG_HDR_SEQ_OFF 8U
#define CFG_HDR_CRC_OFF 12U
#define CFG_HDR_STATE_OFF 16U
#define CFG_HDR_RSVD_OFF 20U
#define CFG_HDR_PAYLOAD_OFF 24U

_Static_assert((CFG_HDR_PAYLOAD_OFF + sizeof(Config)) <= CFG_PAGE_SIZE, "Config too large for slot");

static uint32_t g_active_slot = CFG_SLOT0_ADDR;
static uint32_t g_next_seq = 1U;
static uint8_t g_has_active = 0U;
static Config g_cached_cfg;
static uint8_t g_has_cached = 0U;

/* Function: Crc32_Calc
 * Purpose: Compute CRC32 over a byte buffer.
 * Inputs: data - byte buffer, len - number of bytes.
 * Outputs: CRC32 value.
 */
static uint32_t Crc32_Calc(const uint8_t *data, uint32_t len)
{
  uint32_t crc = 0xFFFFFFFFUL;
  for (uint32_t i = 0; i < len; i++)
  {
    crc ^= (uint32_t)data[i] << 24U;
    for (uint8_t b = 0U; b < 8U; b++)
    {
      if (crc & 0x80000000UL)
      {
        crc = (crc << 1U) ^ 0x04C11DB7UL;
      }
      else
      {
        crc <<= 1U;
      }
    }
  }
  return crc;
}

static uint32_t Read_U32(uint32_t addr)
{
  uint32_t val;
  memcpy(&val, (const void *)addr, sizeof(val));
  return val;
}

static uint16_t Read_U16(uint32_t addr)
{
  uint16_t val;
  memcpy(&val, (const void *)addr, sizeof(val));
  return val;
}

static bool Slot_ReadConfig(uint32_t slot_addr, Config *out, uint32_t *seq_out)
{
  uint32_t magic = Read_U32(slot_addr + CFG_HDR_MAGIC_OFF);
  uint16_t version = Read_U16(slot_addr + CFG_HDR_VERSION_OFF);
  uint16_t length = Read_U16(slot_addr + CFG_HDR_LEN_OFF);
  uint32_t seq = Read_U32(slot_addr + CFG_HDR_SEQ_OFF);
  uint32_t crc = Read_U32(slot_addr + CFG_HDR_CRC_OFF);
  uint32_t state = Read_U32(slot_addr + CFG_HDR_STATE_OFF);

  if ((magic != CFG_MAGIC) || (version != CFG_VERSION))
  {
    return false;
  }
  if ((length == 0U) || (length > sizeof(Config)))
  {
    return false;
  }
  if (state != CFG_STATE_VALID)
  {
    return false;
  }

  uint8_t payload[sizeof(Config)];
  memcpy(payload, (const void *)(slot_addr + CFG_HDR_PAYLOAD_OFF), length);

  uint32_t crc_calc = Crc32_Calc(payload, length);
  if (crc_calc != crc)
  {
    return false;
  }

  memset(out, 0, sizeof(Config));
  memcpy(out, payload, length);
  *seq_out = seq;
  return true;
}

static bool Erase_Slot(uint32_t slot_addr)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t page_error = 0U;
  uint32_t page_index = (slot_addr - FLASH_BASE) / CFG_PAGE_SIZE;

  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Page = page_index;
  erase.NbPages = 1U;

  if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)
  {
    return false;
  }
  return true;
}

static bool Program_DoubleWord(uint32_t addr, const uint8_t data[8])
{
  uint64_t word;
  memcpy(&word, data, sizeof(word));
  return (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, word) == HAL_OK);
}

static bool Program_Buffer(uint32_t addr, const uint8_t *data, uint32_t len)
{
  uint8_t buf[8];
  uint32_t offset = 0U;

  while (offset < len)
  {
    uint32_t chunk = len - offset;
    if (chunk > sizeof(buf))
    {
      chunk = sizeof(buf);
    }

    memset(buf, 0xFF, sizeof(buf));
    memcpy(buf, &data[offset], chunk);

    if (!Program_DoubleWord(addr + offset, buf))
    {
      return false;
    }

    offset += sizeof(buf);
  }

  return true;
}

static void Build_DW0(uint8_t dw0[8])
{
  uint32_t magic = CFG_MAGIC;
  uint16_t version = (uint16_t)CFG_VERSION;
  uint16_t length = (uint16_t)sizeof(Config);
  memcpy(&dw0[0], &magic, sizeof(magic));
  memcpy(&dw0[4], &version, sizeof(version));
  memcpy(&dw0[6], &length, sizeof(length));
}

static void Build_DW1(uint8_t dw1[8], uint32_t seq, uint32_t crc)
{
  memcpy(&dw1[0], &seq, sizeof(seq));
  memcpy(&dw1[4], &crc, sizeof(crc));
}

static void Build_DW2(uint8_t dw2[8], uint32_t state)
{
  uint32_t rsvd = 0xFFFFFFFFUL;
  memcpy(&dw2[0], &state, sizeof(state));
  memcpy(&dw2[4], &rsvd, sizeof(rsvd));
}

static bool Legacy_ReadRecord(uint32_t addr, Config *out, uint32_t *seq_out)
{
  uint8_t buf[LEGACY_RECORD_SIZE];
  memcpy(buf, (const void *)addr, LEGACY_RECORD_SIZE);

  uint32_t magic = 0U;
  uint16_t version = 0U;
  uint16_t length = 0U;
  uint32_t seq = 0U;
  uint32_t crc_stored = 0U;

  memcpy(&magic, &buf[0], sizeof(magic));
  memcpy(&version, &buf[4], sizeof(version));
  memcpy(&length, &buf[6], sizeof(length));
  memcpy(&seq, &buf[8], sizeof(seq));
  memcpy(&crc_stored, &buf[LEGACY_CRC_OFFSET], sizeof(crc_stored));

  if ((magic != LEGACY_MAGIC) || (version != LEGACY_VERSION))
  {
    return false;
  }
  if ((length == 0U) || (length > sizeof(Config)))
  {
    return false;
  }

  uint32_t crc_calc = Crc32_Calc(buf, LEGACY_CRC_OFFSET);
  if (crc_calc != crc_stored)
  {
    return false;
  }

  memset(out, 0, sizeof(Config));
  memcpy(out, &buf[LEGACY_HEADER_SIZE], length);
  *seq_out = seq;
  return true;
}

static bool Legacy_Load(Config *out)
{
  bool found = false;
  uint32_t best_seq = 0U;

  const uint32_t pages[2] = {CFG_SLOT0_ADDR, CFG_SLOT1_ADDR};
  for (uint32_t p = 0U; p < 2U; p++)
  {
    uint32_t page_addr = pages[p];
    uint32_t slots = (uint32_t)(CFG_PAGE_SIZE / LEGACY_RECORD_SIZE);
    for (uint32_t slot = 0U; slot < slots; slot++)
    {
      uint32_t addr = page_addr + slot * LEGACY_RECORD_SIZE;
      Config tmp;
      uint32_t seq = 0U;
      if (Legacy_ReadRecord(addr, &tmp, &seq))
      {
        if (!found || (seq > best_seq))
        {
          *out = tmp;
          best_seq = seq;
          found = true;
        }
      }
    }
  }

  return found;
}

static void Clamp_Config(Config *cfg)
{
  if (cfg == NULL)
  {
    return;
  }

  if (cfg->mode < 1U || cfg->mode > 9U)
  {
    cfg->mode = 1U;
  }

  if (cfg->param_layer > 1U)
  {
    cfg->param_layer = 0U;
  }

  if (cfg->lock_enabled > 1U)
  {
    cfg->lock_enabled = 0U;
  }

  if (cfg->global_brightness == 0U)
  {
    cfg->global_brightness = 255U;
  }

  if (cfg->gamma_profile > BRIGHT_PROFILE_LOWBOOST)
  {
    cfg->gamma_profile = BRIGHT_PROFILE_GAMMA22;
  }

  cfg->format_tag = FLASH_CFG_FORMAT_TAG;
}

static bool Cfg_MigrateIfNeeded(Config *cfg, uint8_t *need_save)
{
  if ((cfg == NULL) || (need_save == NULL))
  {
    return false;
  }

  *need_save = 0U;

  uint32_t ver = cfg->cfg_version;
  if (ver == 0U)
  {
    ver = 1U;
    cfg->cfg_version = 1U;
    *need_save = 1U;
  }

  if (ver > CFG_STRUCT_VERSION)
  {
    return false;
  }

  if (ver < 2U)
  {
    cfg->param_layer = 0U;
    cfg->lock_enabled = 0U;
    if (cfg->global_brightness == 0U)
    {
      cfg->global_brightness = 255U;
    }
    *need_save = 1U;
    ver = 2U;
    cfg->cfg_version = 2U;
  }

  if (ver < 3U)
  {
    if (cfg->gamma_profile > BRIGHT_PROFILE_LOWBOOST)
    {
      cfg->gamma_profile = BRIGHT_PROFILE_GAMMA22;
    }
    if (cfg->adv_gamma >= 200U)
    {
      cfg->gamma_profile = BRIGHT_PROFILE_LOWBOOST;
    }
    *need_save = 1U;
    if (cfg->created_by_fw == 0U)
    {
      cfg->created_by_fw = FW_VERSION_U32;
      *need_save = 1U;
    }
    cfg->last_migrated_by_fw = FW_VERSION_U32;
    cfg->cfg_version = CFG_STRUCT_VERSION;
    ver = CFG_STRUCT_VERSION;
    *need_save = 1U;
  }

  if (cfg->created_by_fw == 0U)
  {
    cfg->created_by_fw = FW_VERSION_U32;
    *need_save = 1U;
  }

  if ((cfg->last_migrated_by_fw == 0U) && (cfg->cfg_version == CFG_STRUCT_VERSION))
  {
    cfg->last_migrated_by_fw = FW_VERSION_U32;
    *need_save = 1U;
  }

  Clamp_Config(cfg);
  return true;
}

bool Cfg_Load(Config *out)
{
  if (out == NULL)
  {
    return false;
  }

  bool found = false;
  uint8_t legacy_used = 0U;
  uint32_t best_seq = 0U;
  uint32_t best_slot = CFG_SLOT0_ADDR;

  const uint32_t slots[2] = {CFG_SLOT0_ADDR, CFG_SLOT1_ADDR};
  for (uint32_t i = 0U; i < 2U; i++)
  {
    Config tmp;
    uint32_t seq = 0U;
    if (Slot_ReadConfig(slots[i], &tmp, &seq))
    {
      if (!found || (seq > best_seq))
      {
        *out = tmp;
        best_seq = seq;
        best_slot = slots[i];
        found = true;
      }
    }
  }

  if (!found)
  {
    found = Legacy_Load(out);
    if (found)
    {
      legacy_used = 1U;
    }
  }

  if (found)
  {
    uint8_t need_save = 0U;
    if (!Cfg_MigrateIfNeeded(out, &need_save))
    {
      g_active_slot = CFG_SLOT0_ADDR;
      g_next_seq = 1U;
      g_has_active = 0U;
      g_has_cached = 0U;
      return false;
    }

    g_active_slot = best_slot;
    g_next_seq = best_seq + 1U;
    g_has_active = (legacy_used == 0U) ? 1U : 0U;
    g_cached_cfg = *out;
    g_has_cached = 1U;

    if (need_save != 0U)
    {
      (void)Cfg_SaveAtomic(out);
    }
  }
  else
  {
    g_active_slot = CFG_SLOT0_ADDR;
    g_next_seq = 1U;
    g_has_active = 0U;
    g_has_cached = 0U;
  }

  return found;
}

bool Cfg_SaveAtomic(const Config *in)
{
  if (in == NULL)
  {
    return false;
  }

  if (g_has_cached != 0U)
  {
    if (memcmp(&g_cached_cfg, in, sizeof(Config)) == 0)
    {
      return true;
    }
  }

  uint32_t target_slot = CFG_SLOT0_ADDR;
  if (g_has_active != 0U)
  {
    target_slot = (g_active_slot == CFG_SLOT0_ADDR) ? CFG_SLOT1_ADDR : CFG_SLOT0_ADDR;
  }

  uint32_t crc = Crc32_Calc((const uint8_t *)in, sizeof(Config));

  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_PROGERR |
                         FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_SIZERR |
                         FLASH_FLAG_PGSERR | FLASH_FLAG_MISERR | FLASH_FLAG_FASTERR);

  if (!Erase_Slot(target_slot))
  {
    HAL_FLASH_Lock();
    return false;
  }

  {
    uint8_t dw0[8];
    uint8_t dw1[8];
    uint8_t dw2[8];
    Build_DW0(dw0);
    Build_DW1(dw1, g_next_seq, 0xFFFFFFFFUL);
    Build_DW2(dw2, CFG_STATE_WRITING);

    if (!Program_DoubleWord(target_slot + CFG_HDR_MAGIC_OFF, dw0))
    {
      HAL_FLASH_Lock();
      return false;
    }
    if (!Program_DoubleWord(target_slot + CFG_HDR_SEQ_OFF, dw1))
    {
      HAL_FLASH_Lock();
      return false;
    }
    if (!Program_DoubleWord(target_slot + CFG_HDR_STATE_OFF, dw2))
    {
      HAL_FLASH_Lock();
      return false;
    }
  }

  if (!Program_Buffer(target_slot + CFG_HDR_PAYLOAD_OFF, (const uint8_t *)in, sizeof(Config)))
  {
    HAL_FLASH_Lock();
    return false;
  }

  {
    uint8_t dw1[8];
    Build_DW1(dw1, g_next_seq, crc);
    if (!Program_DoubleWord(target_slot + CFG_HDR_SEQ_OFF, dw1))
    {
      HAL_FLASH_Lock();
      return false;
    }
  }

  {
    uint8_t dw2[8];
    Build_DW2(dw2, CFG_STATE_VALID);
    if (!Program_DoubleWord(target_slot + CFG_HDR_STATE_OFF, dw2))
    {
      HAL_FLASH_Lock();
      return false;
    }
  }

  HAL_FLASH_Lock();

  g_active_slot = target_slot;
  g_next_seq++;
  g_has_active = 1U;
  g_cached_cfg = *in;
  g_has_cached = 1U;
  return true;
}

void Cfg_ResetToDefault(Config *out)
{
  if (out == NULL)
  {
    return;
  }

  memset(out, 0, sizeof(Config));
  out->mode = 1U;
  out->param_layer = 0U;
  out->lock_enabled = 0U;
  out->global_brightness = 255U;
  out->param_speed_idx = 0U;
  out->flash_period = 0U;
  out->adv_gamma = 128U;
  out->adv_breath_shape = 0U;
  out->adv_phase_offset = 0U;
  out->gamma_profile = BRIGHT_PROFILE_GAMMA22;
  out->format_tag = FLASH_CFG_FORMAT_TAG;
  out->cfg_version = CFG_STRUCT_VERSION;
  out->created_by_fw = FW_VERSION_U32;
  out->last_migrated_by_fw = FW_VERSION_U32;
  for (uint8_t i = 1U; i <= 9U; i++)
  {
    out->param_level[i] = 128U;
  }
  out->param_level[0] = 0U;
}
