#include <string.h>
#include "main.h"
#include "pcanpro_timestamp.h"
#include "pcanpro_led.h"

static struct
{
  uint32_t arg;
  uint32_t delay;
  uint32_t timestamp;
  uint16_t mode;
  uint8_t  state;
} led_mode_array[LED_TOTAL];

static void led_write_hardware(void)
{
  /* STM32H745I-DISCO onboard LED1 (Green, PJ2) and LED2 (Red, PI13) are active-LOW. */
  const uint8_t stat_on = (led_mode_array[LED_STAT].state != 0U) ? 1U : 0U;
  const uint8_t bus_on = (led_mode_array[LED_CH0_RX].state |
                          led_mode_array[LED_CH0_TX].state |
                          led_mode_array[LED_CH1_RX].state |
                          led_mode_array[LED_CH1_TX].state) != 0U;

  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, stat_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, bus_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void pcan_led_init(void)
{
  memset(led_mode_array, 0, sizeof(led_mode_array));
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
}

void pcan_led_set_mode(int led, int mode, uint32_t arg)
{
  if ((led < 0) || (led >= LED_TOTAL))
  {
    return;
  }

  uint32_t now_ms = pcan_timestamp_millis();

  led_mode_array[led].mode = (uint16_t)mode;
  if (led_mode_array[led].timestamp == 0U)
  {
    led_mode_array[led].timestamp = (now_ms != 0U) ? now_ms : 1U;
  }
  led_mode_array[led].delay = 0U;

  if ((mode == LED_MODE_BLINK_FAST) || (mode == LED_MODE_BLINK_SLOW))
  {
    led_mode_array[led].delay = (mode == LED_MODE_BLINK_FAST) ? 50U : 200U;
    if (arg != 0xFFFFFFFFU)
    {
      arg = now_ms + arg;
    }
  }

  led_mode_array[led].arg = arg;
}

void pcan_led_poll(void)
{
  uint32_t ts_ms = pcan_timestamp_millis();
  uint8_t updated = 0U;

  for (int i = 0; i < LED_TOTAL; i++)
  {
    if (led_mode_array[i].timestamp == 0U)
    {
      continue;
    }

    if ((uint32_t)(ts_ms - led_mode_array[i].timestamp) < led_mode_array[i].delay)
    {
      continue;
    }

    switch (led_mode_array[i].mode)
    {
      default:
      case LED_MODE_DEVICE:
        led_mode_array[i].timestamp = 0U;
        break;

      case LED_MODE_OFF:
      case LED_MODE_ON:
        led_mode_array[i].state = (led_mode_array[i].mode == LED_MODE_ON) ? 1U : 0U;
        led_mode_array[i].timestamp = 0U;
        updated = 1U;
        break;

      case LED_MODE_BLINK_FAST:
      case LED_MODE_BLINK_SLOW:
        led_mode_array[i].state ^= 1U;
        led_mode_array[i].timestamp += led_mode_array[i].delay;
        if ((led_mode_array[i].arg != 0xFFFFFFFFU) &&
            ((int32_t)(ts_ms - led_mode_array[i].arg) >= 0))
        {
          pcan_led_set_mode(i, LED_MODE_OFF, 0U);
        }
        updated = 1U;
        break;
    }
  }

  if (updated != 0U)
  {
    led_write_hardware();
  }
}
