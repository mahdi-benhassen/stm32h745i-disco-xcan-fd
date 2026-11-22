#include <string.h>
#include "usbd_ctlreq.h"
#include "usbd_ioreq.h"
#include "usbd_conf.h"
#include "usbd_helper.h"
#include "pcan_usb_ids.h"
#include "pcanpro_protocol.h"
#include "pcanpro_usbd.h"

static struct t_class_data pcanpro_data;

struct __attribute__((packed)) t_pcanpro_description
{
  USB_CONFIGURATION_DESCRIPTOR con0;
  USB_INTERFACE_DESCRIPTOR     if0;
  USB_ENDPOINT_DESCRIPTOR      ep1_i0;
  USB_ENDPOINT_DESCRIPTOR      ep2_i0;
  USB_ENDPOINT_DESCRIPTOR      ep3_i0;
  USB_ENDPOINT_DESCRIPTOR      ep4_i0;
#if defined(PCAN_PRO) || defined(PCAN_PRO_FD) || defined(PCAN_X6)
  USB_ENDPOINT_DESCRIPTOR      ep5_i0;
  USB_ENDPOINT_DESCRIPTOR      ep6_i0;
#endif
#if INCLUDE_LIN_INTERFACE
  USB_INTERFACE_DESCRIPTOR     if1;
  USB_ENDPOINT_DESCRIPTOR      ep1_i1;
  USB_ENDPOINT_DESCRIPTOR      ep2_i1;
  USB_ENDPOINT_DESCRIPTOR      ep3_i1;
  USB_ENDPOINT_DESCRIPTOR      ep4_i1;
  USB_ENDPOINT_DESCRIPTOR      ep5_i1;
  USB_ENDPOINT_DESCRIPTOR      ep6_i1;
#endif
};

#if INCLUDE_LIN_INTERFACE
_Static_assert(sizeof(struct t_pcanpro_description) == 111U,
               "Invalid PCAN_PRO_FD USB config descriptor size");
#else
_Static_assert(sizeof(struct t_pcanpro_description) == 60U,
               "Invalid PCAN_PRO_FD USB config descriptor size");
#endif

#if defined(__ICCARM__)
  #pragma data_alignment=4
#endif
__ALIGN_BEGIN static const USB_DEVICE_QUALIFIER_DESCRIPTOR dev_qua __ALIGN_END =
{
  .bLength            = (uint8_t)sizeof(USB_DEVICE_QUALIFIER_DESCRIPTOR),
  .bDescriptorType    = USB_QUALIFIER_DESCRIPTOR_TYPE,
  .bcdUSB             = 0x0200U,
  .bDeviceClass       = 0U,
  .bDeviceSubClass    = 0U,
  .bDeviceProtocol    = 0U,
  .bMaxPacketSize0    = USB_MAX_EP0_SIZE,
  .bNumConfigurations = 1U,
  .bReserved          = 0U,
};

#if defined(__ICCARM__)
  #pragma data_alignment=4
#endif
__ALIGN_BEGIN static struct t_pcanpro_description pcanpro_dev __ALIGN_END =
{
  .con0 =
  {
    .bLength              = (uint8_t)sizeof(USB_CONFIGURATION_DESCRIPTOR),
    .bDescriptorType      = USB_CONFIGURATION_DESCRIPTOR_TYPE,
    .wTotalLength         = (uint16_t)sizeof(struct t_pcanpro_description),
#if INCLUDE_LIN_INTERFACE
    .bNumInterfaces       = 2U,
#else
    .bNumInterfaces       = 1U,
#endif
    .bConfigurationValue  = 1U,
    .iConfiguration       = USBD_IDX_CONFIG_STR,
    .bmAttributes         = USB_CONFIG_BUS_POWERED,
    .MaxPower             = 120U, /* 240 mA */
  },
  /* CAN INTERFACE */
  .if0 =
  {
    .bLength              = (uint8_t)sizeof(USB_INTERFACE_DESCRIPTOR),
    .bDescriptorType      = USB_INTERFACE_DESCRIPTOR_TYPE,
    .bInterfaceNumber     = 0U,
    .bAlternateSetting    = 0U,
#if defined(PCAN_PRO) || defined(PCAN_PRO_FD) || defined(PCAN_X6)
    .bNumEndpoints        = 6U,
#else
    .bNumEndpoints        = 4U,
#endif
    .bInterfaceClass      = 0U,
    .bInterfaceSubClass   = 0U,
    .bInterfaceProtocol   = 0U,
    .iInterface           = USBD_IDX_INTERFACE_STR,
  },
  .ep1_i0 =
  {
    .bLength              = (uint8_t)sizeof(USB_ENDPOINT_DESCRIPTOR),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = PCAN_USB_EP_CMDIN,
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = PCAN_FS_MAX_BULK_PACKET_SIZE,
    .bInterval            = 0U,
  },
  .ep2_i0 =
  {
    .bLength              = (uint8_t)sizeof(USB_ENDPOINT_DESCRIPTOR),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = PCAN_USB_EP_CMDOUT,
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = PCAN_FS_MAX_BULK_PACKET_SIZE,
    .bInterval            = 0U,
  },
  .ep3_i0 =
  {
    .bLength              = (uint8_t)sizeof(USB_ENDPOINT_DESCRIPTOR),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = PCAN_USB_EP_MSGIN_CH1,
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = PCAN_FS_MAX_BULK_PACKET_SIZE,
    .bInterval            = 0U,
  },
  .ep4_i0 =
  {
    .bLength              = (uint8_t)sizeof(USB_ENDPOINT_DESCRIPTOR),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = PCAN_USB_EP_MSGOUT_CH1,
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = PCAN_FS_MAX_BULK_PACKET_SIZE,
    .bInterval            = 0U,
  },
#if defined(PCAN_PRO) || defined(PCAN_PRO_FD) || defined(PCAN_X6)
  .ep5_i0 =
  {
    .bLength              = (uint8_t)sizeof(USB_ENDPOINT_DESCRIPTOR),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = PCAN_USB_EP_MSGIN_CH2,
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = PCAN_FS_MAX_BULK_PACKET_SIZE,
    .bInterval            = 0U,
  },
  .ep6_i0 =
  {
    .bLength              = (uint8_t)sizeof(USB_ENDPOINT_DESCRIPTOR),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = PCAN_USB_EP_MSGOUT_CH2,
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = PCAN_FS_MAX_BULK_PACKET_SIZE,
    .bInterval            = 0U,
  },
#endif
#if INCLUDE_LIN_INTERFACE
  /* LIN INTERFACE */
  .if1 =
  {
    .bLength              = (uint8_t)sizeof(USB_INTERFACE_DESCRIPTOR),
    .bDescriptorType      = USB_INTERFACE_DESCRIPTOR_TYPE,
    .bInterfaceNumber     = 1U,
    .bAlternateSetting    = 0U,
    .bNumEndpoints        = 6U,
    .bInterfaceClass      = 0U,
    .bInterfaceSubClass   = 0U,
    .bInterfaceProtocol   = 0U,
    .iInterface           = 6U,
  },
  .ep1_i1 =
  {
    .bLength              = (uint8_t)sizeof(USB_ENDPOINT_DESCRIPTOR),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = 0x84U,
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = PCAN_FS_MAX_BULK_PACKET_SIZE,
    .bInterval            = 0U,
  },
  .ep2_i1 =
  {
    .bLength              = (uint8_t)sizeof(USB_ENDPOINT_DESCRIPTOR),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = 0x04U,
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = PCAN_FS_MAX_BULK_PACKET_SIZE,
    .bInterval            = 0U,
  },
  .ep3_i1 =
  {
    .bLength              = (uint8_t)sizeof(USB_ENDPOINT_DESCRIPTOR),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = 0x85U,
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = PCAN_FS_MAX_BULK_PACKET_SIZE,
    .bInterval            = 0U,
  },
  .ep4_i1 =
  {
    .bLength              = (uint8_t)sizeof(USB_ENDPOINT_DESCRIPTOR),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = 0x05U,
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = PCAN_FS_MAX_BULK_PACKET_SIZE,
    .bInterval            = 0U,
  },
  .ep5_i1 =
  {
    .bLength              = (uint8_t)sizeof(USB_ENDPOINT_DESCRIPTOR),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = 0x86U,
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = PCAN_FS_MAX_BULK_PACKET_SIZE,
    .bInterval            = 0U,
  },
  .ep6_i1 =
  {
    .bLength              = (uint8_t)sizeof(USB_ENDPOINT_DESCRIPTOR),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = 0x06U,
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = PCAN_FS_MAX_BULK_PACKET_SIZE,
    .bInterval            = 0U,
  },
#endif
};

static uint32_t pcan_bulk_out_rx_len(const USBD_HandleTypeDef *pdev, uint32_t buf_size)
{
  uint32_t mps = (pdev->dev_speed == USBD_SPEED_HIGH)
                     ? PCAN_HS_MAX_BULK_PACKET_SIZE
                     : PCAN_FS_MAX_BULK_PACKET_SIZE;
  return (mps < buf_size) ? mps : buf_size;
}

static uint8_t device_init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  USB_ENDPOINT_DESCRIPTOR *p_ep = &pcanpro_dev.ep1_i0;
  UNUSED(cfgidx);

  memset(&pcanpro_data, 0, sizeof(pcanpro_data));

  for (uint8_t i = 0U; i < pcanpro_dev.if0.bNumEndpoints; i++)
  {
    uint8_t ep_addr = p_ep[i].bEndpointAddress;

    if (p_ep[i].bmAttributes == USB_ENDPOINT_TYPE_BULK)
    {
      p_ep[i].wMaxPacketSize = (pdev->dev_speed == USBD_SPEED_HIGH)
                                   ? PCAN_HS_MAX_BULK_PACKET_SIZE
                                   : PCAN_FS_MAX_BULK_PACKET_SIZE;
    }

    (void)USBD_LL_OpenEP(pdev, ep_addr, p_ep[i].bmAttributes, p_ep[i].wMaxPacketSize);

    if ((ep_addr & 0x80U) != 0U)
    {
      pdev->ep_in[ep_addr & EP_ADDR_MSK].is_used = 1U;
    }
    else
    {
      pdev->ep_out[ep_addr & EP_ADDR_MSK].is_used = 1U;
    }
  }

  pdev->pClassData = (void *)&pcanpro_data;
  pdev->pClassDataCmsit[pdev->classId] = (void *)&pcanpro_data;

  (void)USBD_LL_PrepareReceive(
      pdev, PCAN_USB_EP_CMDOUT, pcanpro_data.cmd_ep_buffer,
      pcan_bulk_out_rx_len(pdev, (uint32_t)sizeof(pcanpro_data.cmd_ep_buffer)));
  (void)USBD_LL_PrepareReceive(
      pdev, PCAN_USB_EP_MSGOUT_CH1, pcanpro_data.data1_ep_buffer,
      pcan_bulk_out_rx_len(pdev, (uint32_t)sizeof(pcanpro_data.data1_ep_buffer)));
#if defined(PCAN_PRO) || defined(PCAN_PRO_FD) || defined(PCAN_X6)
  (void)USBD_LL_PrepareReceive(
      pdev, PCAN_USB_EP_MSGOUT_CH2, pcanpro_data.data2_ep_buffer,
      pcan_bulk_out_rx_len(pdev, (uint32_t)sizeof(pcanpro_data.data2_ep_buffer)));
#endif

  return (uint8_t)USBD_OK;
}

static uint8_t device_deinit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  const USB_ENDPOINT_DESCRIPTOR *p_ep = &pcanpro_dev.ep1_i0;
  UNUSED(cfgidx);

  for (uint8_t i = 0U; i < pcanpro_dev.if0.bNumEndpoints; i++)
  {
    uint8_t ep_addr = p_ep[i].bEndpointAddress;
    (void)USBD_LL_FlushEP(pdev, ep_addr);
    (void)USBD_LL_CloseEP(pdev, ep_addr);
    if ((ep_addr & 0x80U) != 0U)
    {
      pdev->ep_in[ep_addr & EP_ADDR_MSK].is_used = 0U;
    }
    else
    {
      pdev->ep_out[ep_addr & EP_ADDR_MSK].is_used = 0U;
    }
  }

  memset(pcanpro_data.ep_tx_in_use, 0, sizeof(pcanpro_data.ep_tx_in_use));
  pdev->pClassData = NULL;
  pdev->pClassDataCmsit[pdev->classId] = NULL;

  return (uint8_t)USBD_OK;
}

static uint8_t device_setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_VENDOR:
      return pcan_protocol_device_setup(pdev, req);
    case USB_REQ_TYPE_CLASS:
    case USB_REQ_TYPE_STANDARD:
      return (uint8_t)USBD_OK;
    default:
      USBD_CtlError(pdev, req);
      return (uint8_t)USBD_FAIL;
  }
}

static uint8_t device_ep0_rx_ready(USBD_HandleTypeDef *pdev)
{
  UNUSED(pdev);
  pcan_ep0_receive();
  return (uint8_t)USBD_OK;
}

static uint8_t device_data_in(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  struct t_class_data *p_data = (struct t_class_data *)pdev->pClassData;
  uint8_t ep_idx = epnum & EP_ADDR_MSK;

  if (p_data == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *)pdev->pData;
  uint32_t len = pdev->ep_in[ep_idx].total_length;

  if ((hpcd != NULL) && (len != 0U) && (hpcd->IN_ep[ep_idx].maxpacket != 0U) &&
      ((len % hpcd->IN_ep[ep_idx].maxpacket) == 0U))
  {
    pdev->ep_in[ep_idx].total_length = 0U;
    (void)USBD_LL_Transmit(pdev, (uint8_t)(ep_idx | 0x80U), NULL, 0U);
  }
  else
  {
    pdev->ep_in[ep_idx].total_length = 0U;
    p_data->ep_tx_in_use[ep_idx] = 0U;
  }

  return (uint8_t)USBD_OK;
}

static uint8_t device_data_out(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  uint16_t size;

  if (pdev->pClassData == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  size = (uint16_t)USBD_LL_GetRxDataSize(pdev, epnum);

  if (epnum == PCAN_USB_EP_CMDOUT)
  {
    pcan_protocol_process_data(epnum, pcanpro_data.cmd_ep_buffer, size);
    (void)USBD_LL_PrepareReceive(
        pdev, epnum, pcanpro_data.cmd_ep_buffer,
        pcan_bulk_out_rx_len(pdev, (uint32_t)sizeof(pcanpro_data.cmd_ep_buffer)));
  }
  else if (epnum == PCAN_USB_EP_MSGOUT_CH1)
  {
    pcan_protocol_process_data(epnum, pcanpro_data.data1_ep_buffer, size);
    (void)USBD_LL_PrepareReceive(
        pdev, epnum, pcanpro_data.data1_ep_buffer,
        pcan_bulk_out_rx_len(pdev, (uint32_t)sizeof(pcanpro_data.data1_ep_buffer)));
  }
  else if (epnum == PCAN_USB_EP_MSGOUT_CH2)
  {
    pcan_protocol_process_data(epnum, pcanpro_data.data2_ep_buffer, size);
    (void)USBD_LL_PrepareReceive(
        pdev, epnum, pcanpro_data.data2_ep_buffer,
        pcan_bulk_out_rx_len(pdev, (uint32_t)sizeof(pcanpro_data.data2_ep_buffer)));
  }
  else
  {
    return (uint8_t)USBD_FAIL;
  }

  return (uint8_t)USBD_OK;
}

static uint8_t *device_get_hs_cfg(uint16_t *length)
{
  *length = (uint16_t)sizeof(struct t_pcanpro_description);
  return (uint8_t *)&pcanpro_dev;
}

static uint8_t *device_get_fs_cfg(uint16_t *length)
{
  *length = (uint16_t)sizeof(struct t_pcanpro_description);
  return (uint8_t *)&pcanpro_dev;
}

static uint8_t *device_get_other_speed_cfg(uint16_t *length)
{
  *length = (uint16_t)sizeof(struct t_pcanpro_description);
  return (uint8_t *)&pcanpro_dev;
}

static uint8_t *device_get_device_qualifier(uint16_t *length)
{
  *length = (uint16_t)sizeof(USB_DEVICE_QUALIFIER_DESCRIPTOR);
  return (uint8_t *)&dev_qua;
}

#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
static uint8_t *device_get_user_string(USBD_HandleTypeDef *pdev, uint8_t index, uint16_t *length)
{
  __ALIGN_BEGIN static uint8_t pcan_usr_str_desc[128] __ALIGN_END;
  UNUSED(pdev);

  if (index == 6U)
  {
    USBD_GetString((uint8_t *)PCAN_USB_IF_LIN_STR, pcan_usr_str_desc, length);
    return pcan_usr_str_desc;
  }

  *length = 0U;
  return NULL;
}
#endif

USBD_ClassTypeDef usbd_pcanpro =
{
  .Init = device_init,
  .DeInit = device_deinit,
  .Setup = device_setup,
  .EP0_TxSent = NULL,
  .EP0_RxReady = device_ep0_rx_ready,
  .DataIn = device_data_in,
  .DataOut = device_data_out,
  .SOF = NULL,
  .IsoINIncomplete = NULL,
  .IsoOUTIncomplete = NULL,
  .GetHSConfigDescriptor = device_get_hs_cfg,
  .GetFSConfigDescriptor = device_get_fs_cfg,
  .GetOtherSpeedConfigDescriptor = device_get_other_speed_cfg,
  .GetDeviceQualifierDescriptor = device_get_device_qualifier,
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
  .GetUsrStrDescriptor = device_get_user_string,
#endif
};
