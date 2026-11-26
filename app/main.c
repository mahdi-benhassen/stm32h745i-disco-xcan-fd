#include "board.h"
#include "hpm_l1c_drv.h"
#include "pcanpro_timestamp.h"
#include "pcanpro_led.h"
#include "pcanpro_protocol.h"
#include "pcan_usb_device.h"
#include "plin_protocol.h"
#include "plin_lin.h"

int main(void)
{
  board_init();

  /* Disable D-cache so non-cacheable CherryUSB QHD/QTD & TX/RX DMA stay coherent */
  l1c_dc_disable();
  l1c_ic_enable();

  pcan_timestamp_init();
  pcan_led_init();
  pcan_protocol_init();
  plin_protocol_init();
  plin_lin_init();
  pcan_usb_device_init();

  for (;;)
  {
    pcan_usb_device_poll();
    pcan_protocol_poll();
    plin_protocol_poll();
    plin_lin_poll();
    pcan_led_poll();
  }
}
