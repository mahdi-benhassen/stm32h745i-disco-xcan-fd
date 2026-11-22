#include <string.h>
#include "main.h"
#include "usbd_core.h"
#include "usbd_conf.h"
#include "pcan_usbd_desc.h"
#include "pcanpro_usbd.h"
#include "pcan_usb_device.h"

#ifndef USB_MODULE_ID
#define USB_MODULE_ID DEVICE_FS
#endif

USBD_HandleTypeDef h_pcan_usb_device;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

void pcan_usb_device_init(void)
{
  if (USBD_Init(&h_pcan_usb_device, &pcan_usbd_desc, USB_MODULE_ID) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_RegisterClass(&h_pcan_usb_device, &usbd_pcanpro) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_Stop(&h_pcan_usb_device) != USBD_OK)
  {
    Error_Handler();
  }

  HAL_Delay(50U);

  if (USBD_Start(&h_pcan_usb_device) != USBD_OK)
  {
    Error_Handler();
  }
}

void pcan_usb_device_poll(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

uint16_t pcan_usb_frame_number(void)
{
  if (h_pcan_usb_device.pData == NULL)
  {
    return 0U;
  }

  uint32_t USBx_BASE = (uint32_t)(((PCD_HandleTypeDef *)h_pcan_usb_device.pData)->Instance);
  return (uint16_t)((USBx_DEVICE->DSTS >> 8U) & 0x3FFFU);
}

int pcan_flush_ep(uint8_t ep)
{
  USBD_HandleTypeDef *pdev = &h_pcan_usb_device;
  struct t_class_data *p_data = (struct t_class_data *)pdev->pClassData;

  if (p_data == NULL)
  {
    return 0;
  }

  p_data->ep_tx_in_use[ep & EP_ADDR_MSK] = 0U;
  return (USBD_LL_FlushEP(pdev, ep) == USBD_OK) ? 1 : 0;
}

int pcan_flush_data(struct t_m2h_fsm *pfsm, void *src, int size)
{
  USBD_HandleTypeDef *pdev = &h_pcan_usb_device;
  struct t_class_data *p_data = (struct t_class_data *)pdev->pClassData;

  if ((p_data == NULL) || (pfsm == NULL) || (pdev->dev_state != USBD_STATE_CONFIGURED))
  {
    return 0;
  }

  uint8_t ep_idx = pfsm->ep_addr & EP_ADDR_MSK;

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
      pdev->ep_in[ep_idx].total_length = (uint32_t)size;
      (void)USBD_LL_Transmit(pdev, pfsm->ep_addr, pfsm->pdbuf, (uint32_t)size);

      pfsm->total_tx += (uint32_t)size;
      pfsm->state = 1U;
      return 1;

    default:
      pfsm->state = 0U;
      return 0;
  }
}
