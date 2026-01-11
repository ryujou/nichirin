#include "storage/config_store.h"
#include "stm32g4xx_hal.h"
#include <string.h>

#ifndef CFG_PAGE_SIZE
#ifdef FLASH_PAGE_SIZE
#define CFG_PAGE_SIZE FLASH_PAGE_SIZE
#else
#define CFG_PAGE_SIZE 0x800U
#endif
#endif

#ifndef CFG_PAGE0_ADDR
#define CFG_PAGE0_ADDR 0x0801F000UL
#endif

#ifndef CFG_PAGE1_ADDR
#define CFG_PAGE1_ADDR 0x0801F800UL
#endif

#define CFG_MAGIC 0x31474643UL
#define CFG_RECORD_SIZE 64U
#define CFG_CRC_OFFSET 60U
#define CFG_HEADER_SIZE 12U
#define CFG_SLOTS_PER_PAGE (CFG_PAGE_SIZE / CFG_RECORD_SIZE)

_Static_assert(sizeof(Config) <= (CFG_CRC_OFFSET - CFG_HEADER_SIZE), "Config too large for record");

static uint32_t g_active_page = CFG_PAGE0_ADDR;
static uint32_t g_next_seq = 1U;

static uint32_t Crc32_Calc(const uint8_t *data, uint32_t len)
{
  uint32_t crc = 0xFFFFFFFFUL;
  for (uint32_t i = 0U; i < len; i++)
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

static bool Slot_IsEmpty(uint32_t addr)
{
  const uint8_t *p = (const uint8_t *)addr;
  for (uint32_t i = 0U; i < CFG_RECORD_SIZE; i++)
  {
    if (p[i] != 0xFFU)
    {
      return false;
    }
  }
  return true;
}

static int32_t Find_EmptySlot(uint32_t page_addr)
{
  for (uint32_t slot = 0U; slot < CFG_SLOTS_PER_PAGE; slot++)
  {
    uint32_t addr = page_addr + slot * CFG_RECORD_SIZE;
    if (Slot_IsEmpty(addr))
    {
      return (int32_t)slot;
    }
  }
  return -1;
}

static bool Read_Record(uint32_t addr, Config *out, uint32_t *seq_out)
{
  uint8_t buf[CFG_RECORD_SIZE];
  memcpy(buf, (const void *)addr, CFG_RECORD_SIZE);

  uint32_t magic = 0U;
  uint16_t version = 0U;
  uint16_t length = 0U;
  uint32_t seq = 0U;
  uint32_t crc_stored = 0U;

  memcpy(&magic, &buf[0], sizeof(magic));
  memcpy(&version, &buf[4], sizeof(version));
  memcpy(&length, &buf[6], sizeof(length));
  memcpy(&seq, &buf[8], sizeof(seq));
  memcpy(&crc_stored, &buf[CFG_CRC_OFFSET], sizeof(crc_stored));

  if ((magic != CFG_MAGIC) || (version != CONFIG_VERSION))
  {
    return false;
  }
  if ((length == 0U) || (length > sizeof(Config)))
  {
    return false;
  }

  uint32_t crc_calc = Crc32_Calc(buf, CFG_CRC_OFFSET);
  if (crc_calc != crc_stored)
  {
    return false;
  }

  memset(out, 0, sizeof(Config));
  memcpy(out, &buf[CFG_HEADER_SIZE], length);
  *seq_out = seq;
  return true;
}

static bool Erase_Page(uint32_t page_addr)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t page_error = 0U;
  uint32_t page_index = (page_addr - FLASH_BASE) / CFG_PAGE_SIZE;

  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Banks = FLASH_BANK_1;
  erase.Page = page_index;
  erase.NbPages = 1U;

  if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)
  {
    return false;
  }
  return true;
}

static bool Program_Record(uint32_t addr, const uint8_t *record)
{
  for (uint32_t offset = 0U; offset < (CFG_RECORD_SIZE - 8U); offset += 8U)
  {
    uint64_t word;
    memcpy(&word, &record[offset], sizeof(word));
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + offset, word) != HAL_OK)
    {
      return false;
    }
  }

  {
    uint32_t offset = CFG_RECORD_SIZE - 8U;
    uint64_t word;
    memcpy(&word, &record[offset], sizeof(word));
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + offset, word) != HAL_OK)
    {
      return false;
    }
  }

  return true;
}

void ConfigStore_InitDefaults(Config *out)
{
  memset(out, 0, sizeof(Config));
  out->mode = 1U;
  out->run_state = 0U;
  for (uint8_t i = 1U; i <= 9U; i++)
  {
    out->param_level[i] = 4U;
  }
}

bool ConfigStore_Load(Config *out)
{
  bool found = false;
  uint32_t best_seq = 0U;
  uint32_t best_page = CFG_PAGE0_ADDR;

  const uint32_t pages[2] = {CFG_PAGE0_ADDR, CFG_PAGE1_ADDR};
  for (uint32_t p = 0U; p < 2U; p++)
  {
    uint32_t page_addr = pages[p];
    for (uint32_t slot = 0U; slot < CFG_SLOTS_PER_PAGE; slot++)
    {
      uint32_t addr = page_addr + slot * CFG_RECORD_SIZE;
      if (Slot_IsEmpty(addr))
      {
        continue;
      }

      Config tmp;
      uint32_t seq = 0U;
      if (Read_Record(addr, &tmp, &seq))
      {
        if (!found || (seq > best_seq))
        {
          *out = tmp;
          best_seq = seq;
          best_page = page_addr;
          found = true;
        }
      }
    }
  }

  if (found)
  {
    g_active_page = best_page;
    g_next_seq = best_seq + 1U;
  }
  else
  {
    g_active_page = CFG_PAGE0_ADDR;
    g_next_seq = 1U;
  }

  return found;
}

bool ConfigStore_Save(const Config *in)
{
  uint8_t record[CFG_RECORD_SIZE];
  memset(record, 0xFF, sizeof(record));

  {
    uint32_t magic = CFG_MAGIC;
    uint16_t version = (uint16_t)CONFIG_VERSION;
    uint16_t length = (uint16_t)sizeof(Config);
    uint32_t seq = g_next_seq;
    memcpy(&record[0], &magic, sizeof(magic));
    memcpy(&record[4], &version, sizeof(version));
    memcpy(&record[6], &length, sizeof(length));
    memcpy(&record[8], &seq, sizeof(seq));
  }
  memcpy(&record[CFG_HEADER_SIZE], in, sizeof(Config));

  uint32_t crc = Crc32_Calc(record, CFG_CRC_OFFSET);
  memcpy(&record[CFG_CRC_OFFSET], &crc, sizeof(crc));

  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_ALL_ERRORS);

  int32_t slot = Find_EmptySlot(g_active_page);
  if (slot < 0)
  {
    uint32_t other = (g_active_page == CFG_PAGE0_ADDR) ? CFG_PAGE1_ADDR : CFG_PAGE0_ADDR;
    if (!Erase_Page(other))
    {
      HAL_FLASH_Lock();
      return false;
    }
    g_active_page = other;
    slot = 0;
  }

  uint32_t addr = g_active_page + (uint32_t)slot * CFG_RECORD_SIZE;
  bool ok = Program_Record(addr, record);

  HAL_FLASH_Lock();
  if (ok)
  {
    g_next_seq++;
  }
  return ok;
}
