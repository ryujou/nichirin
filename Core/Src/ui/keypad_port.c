#include "ui/keypad_port.h"
#include "main.h"

static lv_indev_t *s_indev = NULL;
static lv_group_t *s_group = NULL;

static lv_key_t s_last_key = 0;
static uint8_t s_last_state = 0U;
static lv_key_t s_prev_raw = 0;
static uint8_t s_stable_cnt = 0U;
static uint32_t s_last_sample = 0U;

static lv_key_t keypad_read_raw(void)
{
  if (HAL_GPIO_ReadPin(RIGHT_GPIO_Port, RIGHT_Pin) == GPIO_PIN_RESET)
  {
    return LV_KEY_RIGHT;
  }
  if (HAL_GPIO_ReadPin(LEFT_GPIO_Port, LEFT_Pin) == GPIO_PIN_RESET)
  {
    return LV_KEY_LEFT;
  }
  if (HAL_GPIO_ReadPin(DOWN_GPIO_Port, DOWN_Pin) == GPIO_PIN_RESET)
  {
    return LV_KEY_DOWN;
  }
  if (HAL_GPIO_ReadPin(UP_GPIO_Port, UP_Pin) == GPIO_PIN_RESET)
  {
    return LV_KEY_UP;
  }
  if (HAL_GPIO_ReadPin(CENTER_GPIO_Port, CENTER_Pin) == GPIO_PIN_RESET)
  {
    return LV_KEY_ENTER;
  }
  return 0;
}

static void keypad_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
  uint32_t now = HAL_GetTick();
  if ((now - s_last_sample) >= 10U)
  {
    s_last_sample = now;
    lv_key_t raw = keypad_read_raw();
    if (raw == s_prev_raw)
    {
      if (s_stable_cnt < 2U)
      {
        s_stable_cnt++;
      }
    }
    else
    {
      s_prev_raw = raw;
      s_stable_cnt = 0U;
    }
    if (s_stable_cnt >= 1U)
    {
      s_last_key = raw;
      s_last_state = (raw != 0U) ? 1U : 0U;
    }
  }

  data->key = (s_last_key != 0) ? s_last_key : LV_KEY_UP;
  data->state = s_last_state ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
  (void)indev;
}

void KeypadPort_Init(void)
{
  s_group = lv_group_create();
  lv_group_set_default(s_group);

  s_indev = lv_indev_create();
  lv_indev_set_type(s_indev, LV_INDEV_TYPE_KEYPAD);
  lv_indev_set_read_cb(s_indev, keypad_read_cb);
  lv_indev_set_group(s_indev, s_group);
}

lv_group_t *KeypadPort_GetGroup(void)
{
  return s_group;
}
