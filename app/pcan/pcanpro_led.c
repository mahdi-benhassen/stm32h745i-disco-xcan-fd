#include <stddef.h>
#include "board.h"
#include "pcanpro_timestamp.h"
#include "pcanpro_led.h"

struct t_led_ctx
{
  int      mode;
  uint32_t arg;
  uint32_t delay;
  uint32_t timestamp;
  uint8_t  state;
};

static struct t_led_ctx led_ctx_array[LED_TOTAL];
static uint8_t last_hw_state = 0xFFU;

static void led_drive_hw(uint8_t on)
{
  if (on == last_hw_state)
  {
    return;
  }
  last_hw_state = on;
  /* HPM5300EVK has a single active-LOW LED on PA23 (0 = ON, 1 = OFF) */
  board_led_write((on != 0U) ? BOARD_LED_ON_LEVEL : BOARD_LED_OFF_LEVEL);
}

void pcan_led_init(void)
{
  board_init_led_pins();
  last_hw_state = 0xFFU;
  led_drive_hw(0U);

  pcan_led_set_mode(LED_STAT, LED_MODE_BLINK_SLOW, 0xFFFFFFFFU);
  pcan_led_set_mode(LED_CH0_RX, LED_MODE_OFF, 0U);
  pcan_led_set_mode(LED_CH0_TX, LED_MODE_OFF, 0U);
}

void pcan_led_set_mode(int led, int mode, uint32_t arg)
{
  uint32_t ts_now = pcan_timestamp_millis();

  if (led >= LED_TOTAL)
  {
    return;
  }

  struct t_led_ctx *ctx = &led_ctx_array[led];
  ctx->mode = mode;
  ctx->timestamp = ts_now;
  ctx->delay = 0U;

  if (arg == 0U)
  {
    ctx->arg = 0U;
  }
  else if (arg == 0xFFFFFFFFU)
  {
    ctx->arg = 0xFFFFFFFFU;
  }
  else
  {
    ctx->arg = ts_now + arg;
  }

  if (ctx->mode == LED_MODE_BLINK_FAST)
  {
    ctx->delay = 50U;
  }
  else if (ctx->mode == LED_MODE_BLINK_SLOW)
  {
    ctx->delay = 200U;
  }
  else if (ctx->mode == LED_MODE_ON)
  {
    ctx->state = 1U;
  }
  else
  {
    ctx->state = 0U;
  }
}

void pcan_led_poll(void)
{
  uint32_t ts_now = pcan_timestamp_millis();

  for (int i = 0; i < LED_TOTAL; i++)
  {
    struct t_led_ctx *ctx = &led_ctx_array[i];
    if (ctx->timestamp == 0U)
    {
      continue;
    }

    if ((ctx->arg != 0xFFFFFFFFU) && (ctx->arg != 0U) && ((int32_t)(ts_now - ctx->arg) >= 0))
    {
      pcan_led_set_mode(i, LED_MODE_OFF, 0U);
      continue;
    }

    switch (ctx->mode)
    {
      default:
      case LED_MODE_DEVICE:
        ctx->state = 0U;
        break;

      case LED_MODE_BLINK_FAST:
      case LED_MODE_BLINK_SLOW:
        if ((int32_t)(ts_now - ctx->timestamp) >= 0)
        {
          ctx->timestamp = ts_now + ctx->delay;
          ctx->state ^= 1U;
        }
        break;

      case LED_MODE_OFF:
        ctx->state = 0U;
        break;

      case LED_MODE_ON:
        ctx->state = 1U;
        break;
    }
  }

  /* Mux single PA23 LED: brief TX/RX pulse overrides heartbeat LED_STAT */
  uint8_t combined = led_ctx_array[LED_STAT].state;
  if ((led_ctx_array[LED_CH0_RX].mode == LED_MODE_BLINK_FAST) ||
      (led_ctx_array[LED_CH0_TX].mode == LED_MODE_BLINK_FAST))
  {
    combined = (uint8_t)(led_ctx_array[LED_CH0_RX].state | led_ctx_array[LED_CH0_TX].state);
  }
  led_drive_hw(combined);
}
