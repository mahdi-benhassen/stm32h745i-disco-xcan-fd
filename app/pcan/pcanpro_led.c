#include <stddef.h>
#include "at32f45x_board.h"
#include "pcanpro_timestamp.h"
#include "pcanpro_led.h"

struct t_led_ctx
{
  int      mode;
  uint32_t arg;
  uint32_t delay;
  uint32_t timestamp;
  uint8_t  state;
  uint8_t  has_hw_led;
  led_type hw_led;
};

static struct t_led_ctx led_ctx_array[LED_TOTAL] =
{
  [LED_CH0_RX] = { .has_hw_led = 1U, .hw_led = LED3 }, /* PD14 */
  [LED_CH1_RX] = { .has_hw_led = 0U, .hw_led = LED3 }, /* Unused on single-CAN AT-START-F457 */
  [LED_CH0_TX] = { .has_hw_led = 1U, .hw_led = LED4 }, /* PD15 */
  [LED_CH1_TX] = { .has_hw_led = 0U, .hw_led = LED4 }, /* Unused on single-CAN AT-START-F457 */
  [LED_STAT]   = { .has_hw_led = 1U, .hw_led = LED2 }, /* PD13 */
};

static void led_set_hw(const struct t_led_ctx *ctx, uint8_t on)
{
  if (ctx->has_hw_led == 0U)
  {
    return;
  }

  if (on != 0U)
  {
    at32_led_on(ctx->hw_led);
  }
  else
  {
    at32_led_off(ctx->hw_led);
  }
}

void pcan_led_init(void)
{
  pcan_led_set_mode(LED_STAT, LED_MODE_BLINK_SLOW, 0xFFFFFFFFU);
  pcan_led_set_mode(LED_CH0_RX, LED_MODE_OFF, 0xFFFFFFFFU);
  pcan_led_set_mode(LED_CH0_TX, LED_MODE_OFF, 0xFFFFFFFFU);
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
    led_set_hw(ctx, 1U);
  }
  else
  {
    ctx->state = 0U;
    led_set_hw(ctx, 0U);
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

    if ((ctx->arg != 0xFFFFFFFFU) && ((int32_t)(ts_now - ctx->arg) >= 0))
    {
      pcan_led_set_mode(i, LED_MODE_OFF, 0U);
      continue;
    }

    switch (ctx->mode)
    {
      default:
      case LED_MODE_DEVICE:
        ctx->state = 0U;
        led_set_hw(ctx, 0U);
        break;

      case LED_MODE_BLINK_FAST:
      case LED_MODE_BLINK_SLOW:
        if ((int32_t)(ts_now - ctx->timestamp) >= 0)
        {
          ctx->timestamp = ts_now + ctx->delay;
          ctx->state ^= 1U;
          led_set_hw(ctx, ctx->state);
        }
        break;

      case LED_MODE_OFF:
      case LED_MODE_ON:
        if ((ctx->arg != 0U) && (ctx->arg != 0xFFFFFFFFU))
        {
          if (i != LED_STAT)
          {
            led_set_hw(ctx, (uint8_t)(ctx->mode == LED_MODE_ON));
          }
        }
        break;
    }
  }
}
