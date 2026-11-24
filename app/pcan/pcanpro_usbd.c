#include <string.h>
#include "pcan_usb_ids.h"
#include "pcan_usbd_desc.h"
#include "pcanpro_usbd.h"
#include "pcanpro_protocol.h"

#define PCAN_USB_CONFIG_DESC_LEN  (111U)

static struct t_class_data pcan_class_data;
volatile uint32_t pcan_bulk_out_rx_len = 0U;

#if defined(__ICCARM__)
  #pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t pcan_lin_str_desc[32] ALIGNED_TAIL;

#if defined(__ICCARM__)
  #pragma data_alignment=4
#endif
ALIGNED_HEAD uint8_t pcan_fs_cfg_desc[PCAN_USB_CONFIG_DESC_LEN] ALIGNED_TAIL =
{
  /* Configuration Descriptor */
  0x09U,                              /* bLength */
  USB_DESCIPTOR_TYPE_CONFIGURATION,   /* bDescriptorType */
  (uint8_t)(PCAN_USB_CONFIG_DESC_LEN & 0xFFU),
  (uint8_t)(PCAN_USB_CONFIG_DESC_LEN >> 8U),
  0x02U,                              /* bNumInterfaces: CAN-FD + LIN */
  0x01U,                              /* bConfigurationValue */
  USB_CONFIG_STRING,                  /* iConfiguration */
  0x80U,                              /* bmAttributes: Bus-powered */
  0x78U,                              /* MaxPower: 240 mA (120 * 2 mA) */

  /* Interface 0: PCAN-USB Pro FD (CAN-FD) */
  0x09U,                              /* bLength */
  USB_DESCIPTOR_TYPE_INTERFACE,       /* bDescriptorType */
  0x00U,                              /* bInterfaceNumber */
  0x00U,                              /* bAlternateSetting */
  0x06U,                              /* bNumEndpoints */
  0x00U,                              /* bInterfaceClass (matches genuine PCAN-USB Pro FD) */
  0x00U,                              /* bInterfaceSubClass */
  0x00U,                              /* bInterfaceProtocol */
  USB_INTERFACE_STRING,               /* iInterface */

  /* EP1 IN (CMD) */
  0x07U, USB_DESCIPTOR_TYPE_ENDPOINT, PCAN_USB_EP_CMDIN, 0x02U,
  PCAN_FS_MAX_BULK_PACKET_SIZE, 0x00U, 0x00U,
  /* EP1 OUT (CMD) */
  0x07U, USB_DESCIPTOR_TYPE_ENDPOINT, PCAN_USB_EP_CMDOUT, 0x02U,
  PCAN_FS_MAX_BULK_PACKET_SIZE, 0x00U, 0x00U,
  /* EP2 IN (CH1 DATA) */
  0x07U, USB_DESCIPTOR_TYPE_ENDPOINT, PCAN_USB_EP_MSGIN_CH1, 0x02U,
  PCAN_FS_MAX_BULK_PACKET_SIZE, 0x00U, 0x00U,
  /* EP2 OUT (CH1 DATA) */
  0x07U, USB_DESCIPTOR_TYPE_ENDPOINT, PCAN_USB_EP_MSGOUT_CH1, 0x02U,
  PCAN_FS_MAX_BULK_PACKET_SIZE, 0x00U, 0x00U,
  /* EP3 IN (CH2 DATA) */
  0x07U, USB_DESCIPTOR_TYPE_ENDPOINT, PCAN_USB_EP_MSGIN_CH2, 0x02U,
  PCAN_FS_MAX_BULK_PACKET_SIZE, 0x00U, 0x00U,
  /* EP3 OUT (CH2 DATA) */
  0x07U, USB_DESCIPTOR_TYPE_ENDPOINT, PCAN_USB_EP_MSGOUT_CH2, 0x02U,
  PCAN_FS_MAX_BULK_PACKET_SIZE, 0x00U, 0x00U,

  /* Interface 1: PCAN-USB Pro FD (LIN placeholder) */
  0x09U,                              /* bLength */
  USB_DESCIPTOR_TYPE_INTERFACE,       /* bDescriptorType */
  0x01U,                              /* bInterfaceNumber */
  0x00U,                              /* bAlternateSetting */
  0x06U,                              /* bNumEndpoints */
  0x00U,                              /* bInterfaceClass */
  0x00U,                              /* bInterfaceSubClass */
  0x00U,                              /* bInterfaceProtocol */
  PCAN_LIN_INTERFACE_STR_INDEX,       /* iInterface */

  /* LIN placeholder Bulk endpoints (0x84..0x06, distinct from Interface 0) */
  0x07U, USB_DESCIPTOR_TYPE_ENDPOINT, 0x84U, 0x02U,
  PCAN_FS_MAX_BULK_PACKET_SIZE, 0x00U, 0x00U,
  0x07U, USB_DESCIPTOR_TYPE_ENDPOINT, 0x04U, 0x02U,
  PCAN_FS_MAX_BULK_PACKET_SIZE, 0x00U, 0x00U,
  0x07U, USB_DESCIPTOR_TYPE_ENDPOINT, 0x85U, 0x02U,
  PCAN_FS_MAX_BULK_PACKET_SIZE, 0x00U, 0x00U,
  0x07U, USB_DESCIPTOR_TYPE_ENDPOINT, 0x05U, 0x02U,
  PCAN_FS_MAX_BULK_PACKET_SIZE, 0x00U, 0x00U,
  0x07U, USB_DESCIPTOR_TYPE_ENDPOINT, 0x86U, 0x02U,
  PCAN_FS_MAX_BULK_PACKET_SIZE, 0x00U, 0x00U,
  0x07U, USB_DESCIPTOR_TYPE_ENDPOINT, 0x06U, 0x02U,
  PCAN_FS_MAX_BULK_PACKET_SIZE, 0x00U, 0x00U,
};

_Static_assert(sizeof(pcan_fs_cfg_desc) == PCAN_USB_CONFIG_DESC_LEN,
               "Invalid PCAN_PRO_FD USB config descriptor size");

const uint16_t pcan_fs_cfg_desc_len = PCAN_USB_CONFIG_DESC_LEN;

static void pcan_reset_ep_tx_flags(void)
{
  memset(pcan_class_data.ep_tx_in_use, 0, sizeof(pcan_class_data.ep_tx_in_use));
  memset(pcan_class_data.ep_tx_need_zlp, 0, sizeof(pcan_class_data.ep_tx_need_zlp));
}

static usb_sts_type pcan_usbd_init_handler(void *udev)
{
  usbd_core_type *pdev = (usbd_core_type *)udev;

  pcan_reset_ep_tx_flags();

  usbd_ept_open(pdev, PCAN_USB_EP_CMDOUT, EPT_BULK_TYPE, PCAN_FS_MAX_BULK_PACKET_SIZE);
  usbd_ept_open(pdev, PCAN_USB_EP_CMDIN, EPT_BULK_TYPE, PCAN_FS_MAX_BULK_PACKET_SIZE);
  usbd_ept_open(pdev, PCAN_USB_EP_MSGOUT_CH1, EPT_BULK_TYPE, PCAN_FS_MAX_BULK_PACKET_SIZE);
  usbd_ept_open(pdev, PCAN_USB_EP_MSGIN_CH1, EPT_BULK_TYPE, PCAN_FS_MAX_BULK_PACKET_SIZE);
  usbd_ept_open(pdev, PCAN_USB_EP_MSGOUT_CH2, EPT_BULK_TYPE, PCAN_FS_MAX_BULK_PACKET_SIZE);
  usbd_ept_open(pdev, PCAN_USB_EP_MSGIN_CH2, EPT_BULK_TYPE, PCAN_FS_MAX_BULK_PACKET_SIZE);

  usbd_ept_recv(pdev, PCAN_USB_EP_CMDOUT, pcan_class_data.buffer_cmd, PCAN_FS_MAX_BULK_PACKET_SIZE);
  usbd_ept_recv(pdev, PCAN_USB_EP_MSGOUT_CH1, pcan_class_data.buffer_data, PCAN_FS_MAX_BULK_PACKET_SIZE);
  usbd_ept_recv(pdev, PCAN_USB_EP_MSGOUT_CH2, pcan_class_data.buffer_ch2, PCAN_FS_MAX_BULK_PACKET_SIZE);

  return USB_OK;
}

static usb_sts_type pcan_usbd_clear_handler(void *udev)
{
  usbd_core_type *pdev = (usbd_core_type *)udev;

  usbd_ept_close(pdev, PCAN_USB_EP_CMDOUT);
  usbd_ept_close(pdev, PCAN_USB_EP_CMDIN);
  usbd_ept_close(pdev, PCAN_USB_EP_MSGOUT_CH1);
  usbd_ept_close(pdev, PCAN_USB_EP_MSGIN_CH1);
  usbd_ept_close(pdev, PCAN_USB_EP_MSGOUT_CH2);
  usbd_ept_close(pdev, PCAN_USB_EP_MSGIN_CH2);

  pcan_reset_ep_tx_flags();
  return USB_OK;
}

static usb_sts_type pcan_usbd_setup_handler(void *udev, usb_setup_type *setup)
{
  usbd_core_type *pdev = (usbd_core_type *)udev;

  /* String index 6 ("LIN" interface name) falls through from usbd_sdr.c default branch */
  if (((setup->bmRequestType & USB_REQ_TYPE_RESERVED) == USB_REQ_TYPE_STANDARD) &&
      (setup->bRequest == USB_STD_REQ_GET_DESCRIPTOR) &&
      ((setup->wValue >> 8U) == USB_DESCIPTOR_TYPE_STRING))
  {
    if ((setup->wValue & 0xFFU) == PCAN_LIN_INTERFACE_STR_INDEX)
    {
      uint16_t str_len = 0U;
      pcan_usbd_encode_utf16_str(PCAN_USB_IF_LIN_STR, pcan_lin_str_desc,
                                 (uint16_t)sizeof(pcan_lin_str_desc), &str_len);
      uint16_t send_len = (setup->wLength < str_len) ? setup->wLength : str_len;
      if (send_len > 0U)
      {
        usbd_ctrl_send(pdev, pcan_lin_str_desc, send_len);
      }
      return USB_OK;
    }

    usbd_ctrl_unsupport(pdev);
    return USB_FAIL;
  }

  if ((setup->bmRequestType & USB_REQ_TYPE_RESERVED) == USB_REQ_TYPE_VENDOR)
  {
    uint8_t status = pcan_protocol_device_setup(pdev, setup);
    return (status == USBD_OK) ? USB_OK : USB_FAIL;
  }

  /* CLEAR_FEATURE / SET_FEATURE callbacks on endpoints are informational */
  if ((setup->bmRequestType & USB_REQ_TYPE_RESERVED) == USB_REQ_TYPE_STANDARD)
  {
    if (setup->bRequest == USB_STD_REQ_CLEAR_FEATURE)
    {
      uint8_t ep_idx = (uint8_t)(setup->wIndex & 0x0FU);
      if (ep_idx < 8U)
      {
        pcan_class_data.ep_tx_in_use[ep_idx] = 0U;
        pcan_class_data.ep_tx_need_zlp[ep_idx] = 0U;
      }
    }
    return USB_OK;
  }

  usbd_ctrl_unsupport(pdev);
  return USB_FAIL;
}

static usb_sts_type pcan_usbd_ept0_rx_handler(void *udev)
{
  UNUSED(udev);
  pcan_ep0_receive();
  return USB_OK;
}

static usb_sts_type pcan_usbd_in_handler(void *udev, uint8_t ept_num)
{
  usbd_core_type *pdev = (usbd_core_type *)udev;
  uint8_t ep_idx = (uint8_t)(ept_num & 0x0FU);

  if (ep_idx < 8U)
  {
    if (pcan_class_data.ep_tx_need_zlp[ep_idx] != 0U)
    {
      pcan_class_data.ep_tx_need_zlp[ep_idx] = 0U;
      usbd_ept_send(pdev, (uint8_t)(ep_idx | 0x80U), NULL, 0U);
      return USB_OK;
    }

    pcan_class_data.ep_tx_in_use[ep_idx] = 0U;
  }

  return USB_OK;
}

static usb_sts_type pcan_usbd_out_handler(void *udev, uint8_t ept_num)
{
  usbd_core_type *pdev = (usbd_core_type *)udev;
  uint8_t ep_addr = (uint8_t)(ept_num & 0x0FU);
  uint16_t rx_len = (uint16_t)usbd_get_recv_len(pdev, ep_addr);
  pcan_bulk_out_rx_len = rx_len;

  if (ep_addr == (PCAN_USB_EP_CMDOUT & 0x0FU))
  {
    pcan_protocol_process_data(ep_addr, pcan_class_data.buffer_cmd, rx_len);
    usbd_ept_recv(pdev, PCAN_USB_EP_CMDOUT, pcan_class_data.buffer_cmd, PCAN_FS_MAX_BULK_PACKET_SIZE);
  }
  else if (ep_addr == (PCAN_USB_EP_MSGOUT_CH1 & 0x0FU))
  {
    pcan_protocol_process_data(ep_addr, pcan_class_data.buffer_data, rx_len);
    usbd_ept_recv(pdev, PCAN_USB_EP_MSGOUT_CH1, pcan_class_data.buffer_data, PCAN_FS_MAX_BULK_PACKET_SIZE);
  }
  else if (ep_addr == (PCAN_USB_EP_MSGOUT_CH2 & 0x0FU))
  {
    pcan_protocol_process_data(ep_addr, pcan_class_data.buffer_ch2, rx_len);
    usbd_ept_recv(pdev, PCAN_USB_EP_MSGOUT_CH2, pcan_class_data.buffer_ch2, PCAN_FS_MAX_BULK_PACKET_SIZE);
  }

  return USB_OK;
}

static usb_sts_type pcan_usbd_event_handler(void *udev, usbd_event_type event)
{
  usbd_core_type *pdev = (usbd_core_type *)udev;

  switch (event)
  {
    case USBD_RESET_EVENT:
      pcan_reset_ep_tx_flags();
      break;

    case USBD_SUSPEND_EVENT:
    case USBD_WAKEUP_EVENT:
      /* Keep OTGFS PHY clock running in polling mode (usbd_enter_suspend stops it) */
      if (pdev->usb_reg != NULL)
      {
        usb_open_phy_clk(pdev->usb_reg);
      }
      break;

    default:
      break;
  }

  return USB_OK;
}

usbd_class_handler usbd_pcanpro =
{
  .init_handler = pcan_usbd_init_handler,
  .clear_handler = pcan_usbd_clear_handler,
  .setup_handler = pcan_usbd_setup_handler,
  .ept0_tx_handler = NULL,
  .ept0_rx_handler = pcan_usbd_ept0_rx_handler,
  .in_handler = pcan_usbd_in_handler,
  .out_handler = pcan_usbd_out_handler,
  .sof_handler = NULL,
  .event_handler = pcan_usbd_event_handler,
  .pdata = &pcan_class_data,
};
