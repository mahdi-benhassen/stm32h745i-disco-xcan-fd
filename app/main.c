#include "at32f45x_board.h"
#include "at32f45x_clock.h"
#include "at32f45x_int.h"
#include "usb_conf.h"
#include "pcanpro_timestamp.h"
#include "pcanpro_led.h"
#include "pcanpro_protocol.h"
#include "pcan_usb_device.h"

void usb_delay_us(uint32_t us)
{
  uint32_t start = pcan_timestamp_us();
  while ((uint32_t)(pcan_timestamp_us() - start) < us)
  {
  }
}

void usb_delay_ms(uint32_t ms)
{
  while (ms > 0U)
  {
    usb_delay_us(1000U);
    ms--;
  }
}

int main(void)
{
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  system_clock_config();

  at32_led_init(LED2);
  at32_led_init(LED3);
  at32_led_init(LED4);
  at32_led_off(LED2);
  at32_led_off(LED3);
  at32_led_off(LED4);

  /* 1 ms SysTick timebase (non-preempting USB/CAN polling loop) */
  (void)SysTick_Config(system_core_clock / 1000U);

  pcan_timestamp_init();
  pcan_led_init();
  pcan_protocol_init();
  pcan_usb_device_init();

  for (;;)
  {
    pcan_usb_device_poll();
    pcan_protocol_poll();
    pcan_led_poll();
  }
}
