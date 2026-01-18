#include "boot_selftest.h"
#include "storage/cfg_store.h"
#include "drivers/tlc59116.h"
#include "main.h"

#define BOOT_I2C_SCL_PORT GPIOB
#define BOOT_I2C_SCL_PIN GPIO_PIN_3
#define BOOT_I2C_SDA_PORT GPIOB
#define BOOT_I2C_SDA_PIN GPIO_PIN_7

#define BOOT_I2C_TIMEOUT_MS 5U
#define BOOT_ENCODER_SAMPLE_COUNT 3U

typedef enum
{
  BOOT_STATE_IDLE = 0,
  BOOT_STATE_CFG,
  BOOT_STATE_I2C_LINES,
  BOOT_STATE_TLC_ACK,
  BOOT_STATE_ENCODER,
  BOOT_STATE_DONE
} BootState;

static BootState s_state = BOOT_STATE_IDLE;
static uint8_t s_done = 0U;
static uint8_t s_safe_mode = 0U;
static uint8_t s_enc_samples = 0U;
static I2C_HandleTypeDef *s_hi2c = NULL;
static BootDiag s_diag = {0};

static void Boot_SetError(BootErrorFlags flag, uint8_t code)
{
  s_diag.error_flags |= (uint32_t)flag;
  s_diag.last_error = code;
}

void BootSelfTest_Init(I2C_HandleTypeDef *hi2c)
{
  s_hi2c = hi2c;
  s_state = BOOT_STATE_CFG;
  s_done = 0U;
  s_safe_mode = 0U;
  s_enc_samples = 0U;
  s_diag.error_flags = 0U;
  s_diag.last_error = 0U;
  s_diag.encoder_warn = 0U;
}

void BootSelfTest_RequestRetry(void)
{
  s_state = BOOT_STATE_CFG;
  s_done = 0U;
  s_safe_mode = 0U;
  s_enc_samples = 0U;
  s_diag.error_flags = 0U;
  s_diag.last_error = 0U;
  s_diag.encoder_warn = 0U;
}

void BootSelfTest_Tick10ms(void)
{
  if (s_done != 0U)
  {
    return;
  }

  switch (s_state)
  {
    case BOOT_STATE_CFG:
      {
        Config cfg;
        if (!Cfg_Load(&cfg))
        {
          Boot_SetError(BOOT_ERR_CFG_INVALID, 1U);
        }
      }
      s_state = BOOT_STATE_I2C_LINES;
      break;

    case BOOT_STATE_I2C_LINES:
      {
        GPIO_PinState sda = HAL_GPIO_ReadPin(BOOT_I2C_SDA_PORT, BOOT_I2C_SDA_PIN);
        GPIO_PinState scl = HAL_GPIO_ReadPin(BOOT_I2C_SCL_PORT, BOOT_I2C_SCL_PIN);
        if ((sda == GPIO_PIN_RESET) || (scl == GPIO_PIN_RESET))
        {
          Boot_SetError(BOOT_ERR_I2C_LINES, 2U);
        }
      }
      s_state = BOOT_STATE_TLC_ACK;
      break;

    case BOOT_STATE_TLC_ACK:
      if (s_hi2c != NULL)
      {
        uint8_t reg = 0U;
        if (HAL_I2C_Mem_Read(s_hi2c, TLC59116_I2C_ADDR, 0x00U, I2C_MEMADD_SIZE_8BIT,
                             &reg, 1U, BOOT_I2C_TIMEOUT_MS) != HAL_OK)
        {
          Boot_SetError(BOOT_ERR_TLC_ACK, 3U);
        }
      }
      else
      {
        Boot_SetError(BOOT_ERR_TLC_ACK, 3U);
      }
      s_state = BOOT_STATE_ENCODER;
      break;

    case BOOT_STATE_ENCODER:
      {
        GPIO_PinState a = HAL_GPIO_ReadPin(ENC_A_GPIO_Port, ENC_A_Pin);
        GPIO_PinState b = HAL_GPIO_ReadPin(ENC_B_GPIO_Port, ENC_B_Pin);
        GPIO_PinState k = HAL_GPIO_ReadPin(ENC_K_GPIO_Port, ENC_K_Pin);
        if ((a == GPIO_PIN_RESET) && (b == GPIO_PIN_RESET) && (k == GPIO_PIN_RESET))
        {
          s_enc_samples++;
          if (s_enc_samples >= BOOT_ENCODER_SAMPLE_COUNT)
          {
            s_diag.encoder_warn = 1U;
            Boot_SetError(BOOT_ERR_ENCODER_WARN, 4U);
          }
        }
      }
      s_state = BOOT_STATE_DONE;
      break;

    case BOOT_STATE_DONE:
    default:
      s_done = 1U;
      break;
  }

  if ((s_diag.error_flags & (BOOT_ERR_CFG_INVALID | BOOT_ERR_I2C_LINES | BOOT_ERR_TLC_ACK)) != 0U)
  {
    s_safe_mode = 1U;
  }

  if (s_state == BOOT_STATE_DONE)
  {
    s_done = 1U;
  }
}

uint8_t BootSelfTest_IsDone(void)
{
  return s_done;
}

uint8_t BootSelfTest_IsSafeMode(void)
{
  return s_safe_mode;
}

const BootDiag *BootSelfTest_GetDiag(void)
{
  return &s_diag;
}
