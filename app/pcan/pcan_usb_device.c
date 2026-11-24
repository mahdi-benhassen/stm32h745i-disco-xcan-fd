#include <string.h>
#include "at32f45x.h"
#include "usb_conf.h"
#include "usbd_int.h"
#include "pcan_usbd_desc.h"
#include "pcanpro_usbd.h"
#include "pcan_usb_device.h"

otg_core_type otg_core_struct;

static void usb_gpio_config(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(OTG_PIN_GPIO_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);

  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_pins = OTG_PIN_DM | OTG_PIN_DP;
  gpio_init(OTG_PIN_GPIO, &gpio_init_struct);

  gpio_pin_mux_config(OTG_PIN_GPIO, OTG_PIN_DM_SOURCE, OTG_PIN_MUX);
  gpio_pin_mux_config(OTG_PIN_GPIO, OTG_PIN_DP_SOURCE, OTG_PIN_MUX);
}

void pcan_usb_device_init(void)
{
  usb_gpio_config();

  crm_periph_clock_enable(OTG_CLOCK, TRUE);
  crm_usb_clock_source_select(CRM_USB_CLOCK_SOURCE_PLLU);

  (void)usbd_init(&otg_core_struct,
                  USB_FULL_SPEED_CORE_ID,
                  USB_ID,
                  &usbd_pcanpro,
                  &pcan_at32_usbd_desc);

  /* Clean soft detach/attach pulse so host always re-enumerates after reset */
  usbd_disconnect(&otg_core_struct.dev);
  usb_delay_ms(50U);
  usbd_connect(&otg_core_struct.dev);
}

void pcan_usb_device_poll(void)
{
  usbd_irq_handler(&otg_core_struct);
}

uint16_t pcan_usb_frame_number(void)
{
  if (otg_core_struct.usb_reg == NULL)
  {
    return 0U;
  }

  return (uint16_t)(OTG_DEVICE(otg_core_struct.usb_reg)->dsts_bit.soffn & 0x3FFFU);
}

int pcan_flush_ep(uint8_t ep)
{
  struct t_class_data *p_data = (struct t_class_data *)usbd_pcanpro.pdata;
  uint8_t ep_idx = (uint8_t)(ep & 0x0FU);

  if ((p_data == NULL) || (ep_idx >= 8U))
  {
    return 0;
  }

  p_data->ep_tx_in_use[ep_idx] = 0U;
  p_data->ep_tx_need_zlp[ep_idx] = 0U;
  usbd_flush_tx_fifo(&otg_core_struct.dev, ep_idx);
  return 1;
}

int pcan_flush_data(struct t_m2h_fsm *pfsm, void *src, int size)
{
  struct t_class_data *p_data = (struct t_class_data *)usbd_pcanpro.pdata;
  usbd_core_type *pdev = &otg_core_struct.dev;

  if ((p_data == NULL) || (pfsm == NULL) || (pdev->conn_state != USB_CONN_STATE_CONFIGURED))
  {
    return 0;
  }

  uint8_t ep_idx = (uint8_t)(pfsm->ep_addr & 0x0FU);
  if (ep_idx >= 8U)
  {
    return 0;
  }

  switch (pfsm->state)
  {
    case 1:
      if (p_data->ep_tx_in_use[ep_idx] != 0U)
      {
        return 0;
      }
      pfsm->state = 0U;
      /* fall through */

    case 0:
      if ((p_data->ep_tx_in_use[ep_idx] != 0U) || (size < 0) || ((uint32_t)size > pfsm->dbsize))
      {
        return 0;
      }

      if ((size > 0) && (src != NULL))
      {
        memcpy(pfsm->pdbuf, src, (size_t)size);
      }

      p_data->ep_tx_in_use[ep_idx] = 1U;
      p_data->ep_tx_need_zlp[ep_idx] = ((size > 0) && (((uint32_t)size % PCAN_FS_MAX_BULK_PACKET_SIZE) == 0U)) ? 1U : 0U;
      usbd_ept_send(pdev, pfsm->ep_addr, pfsm->pdbuf, (uint16_t)size);

      pfsm->total_tx += (uint32_t)size;
      pfsm->state = 1U;
      return 1;

    default:
      pfsm->state = 0U;
      return 0;
  }
}
