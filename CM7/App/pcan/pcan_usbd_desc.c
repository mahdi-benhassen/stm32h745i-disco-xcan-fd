#include "usbd_core.h"
#include "usbd_ctlreq.h"
#include "usbd_conf.h"
#include "pcan_usb_ids.h"
#include "pcan_usbd_desc.h"

#define PCAN_USBD_LANGID_STRING 1033U

#if defined(__ICCARM__)
  #pragma data_alignment=4
#endif
__ALIGN_BEGIN static uint8_t pcan_usbd_device_desc[USB_LEN_DEV_DESC] __ALIGN_END =
{
  0x12U,                       /* bLength */
  USB_DESC_TYPE_DEVICE,        /* bDescriptorType */
#if (USBD_LPM_ENABLED == 1U)
  0x01U,                       /* bcdUSB LSB (2.01 for LPM) */
#else
  0x00U,                       /* bcdUSB LSB (2.00) */
#endif
  0x02U,                       /* bcdUSB MSB */
  0x00U,                       /* bDeviceClass */
  0x00U,                       /* bDeviceSubClass */
  0x00U,                       /* bDeviceProtocol */
  USB_MAX_EP0_SIZE,            /* bMaxPacketSize0 */
  LOBYTE(PCAN_USB_VID),        /* idVendor LSB */
  HIBYTE(PCAN_USB_VID),        /* idVendor MSB */
  LOBYTE(PCAN_USB_PID),        /* idProduct LSB */
  HIBYTE(PCAN_USB_PID),        /* idProduct MSB */
  0x00U,                       /* bcdDevice LSB */
  0x00U,                       /* bcdDevice MSB */
  USBD_IDX_MFC_STR,            /* iManufacturer */
  USBD_IDX_PRODUCT_STR,        /* iProduct */
  0x00U,                       /* iSerialNumber */
  USBD_MAX_NUM_CONFIGURATION   /* bNumConfigurations */
};

#if (USBD_LPM_ENABLED == 1U)
#define PCAN_USB_SIZ_BOS_DESC 0x0CU
#if defined(__ICCARM__)
  #pragma data_alignment=4
#endif
__ALIGN_BEGIN static uint8_t pcan_usbd_bos_desc[PCAN_USB_SIZ_BOS_DESC] __ALIGN_END =
{
  0x05U,
  USB_DESC_TYPE_BOS,
  0x0CU,
  0x00U,
  0x01U,
  0x07U,
  USB_DEVICE_CAPABITY_TYPE,
  0x02U,
  0x02U,
  0x00U,
  0x00U,
  0x00U
};
#endif

#if defined(__ICCARM__)
  #pragma data_alignment=4
#endif
__ALIGN_BEGIN static uint8_t pcan_usbd_langid_desc[USB_LEN_LANGID_STR_DESC] __ALIGN_END =
{
  USB_LEN_LANGID_STR_DESC,
  USB_DESC_TYPE_STRING,
  LOBYTE(PCAN_USBD_LANGID_STRING),
  HIBYTE(PCAN_USBD_LANGID_STRING)
};

#if defined(__ICCARM__)
  #pragma data_alignment=4
#endif
__ALIGN_BEGIN static uint8_t pcan_usbd_str_desc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

static uint8_t *pcan_get_device_descriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = (uint16_t)sizeof(pcan_usbd_device_desc);
  return pcan_usbd_device_desc;
}

static uint8_t *pcan_get_langid_str_descriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = (uint16_t)sizeof(pcan_usbd_langid_desc);
  return pcan_usbd_langid_desc;
}

static uint8_t *pcan_get_manufacturer_str_descriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *)PCAN_USB_MANUFACTURER_STR, pcan_usbd_str_desc, length);
  return pcan_usbd_str_desc;
}

static uint8_t *pcan_get_product_str_descriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *)PCAN_USB_PRODUCT_STR, pcan_usbd_str_desc, length);
  return pcan_usbd_str_desc;
}

static uint8_t *pcan_get_config_str_descriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *)PCAN_USB_CONFIG_STR, pcan_usbd_str_desc, length);
  return pcan_usbd_str_desc;
}

static uint8_t *pcan_get_interface_str_descriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *)PCAN_USB_IF_CAN_STR, pcan_usbd_str_desc, length);
  return pcan_usbd_str_desc;
}

#if (USBD_LPM_ENABLED == 1U)
static uint8_t *pcan_get_bos_descriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = (uint16_t)sizeof(pcan_usbd_bos_desc);
  return pcan_usbd_bos_desc;
}
#endif

USBD_DescriptorsTypeDef pcan_usbd_desc =
{
  .GetDeviceDescriptor = pcan_get_device_descriptor,
  .GetLangIDStrDescriptor = pcan_get_langid_str_descriptor,
  .GetManufacturerStrDescriptor = pcan_get_manufacturer_str_descriptor,
  .GetProductStrDescriptor = pcan_get_product_str_descriptor,
  .GetSerialStrDescriptor = NULL,
  .GetConfigurationStrDescriptor = pcan_get_config_str_descriptor,
  .GetInterfaceStrDescriptor = pcan_get_interface_str_descriptor,
#if (USBD_LPM_ENABLED == 1U)
  .GetBOSDescriptor = pcan_get_bos_descriptor,
#endif
};
