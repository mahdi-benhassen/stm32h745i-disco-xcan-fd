#include <string.h>
#include "pcan_usb_ids.h"
#include "pcan_usbd_desc.h"

#define PCAN_USBD_LANGID_STRING   1033U
#define PCAN_USBD_MAX_STR_BYTES   128U

#if defined(__ICCARM__)
  #pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t pcan_usbd_device_desc[USB_LEN_DEV_DESC] ALIGNED_TAIL =
{
  0x12U,                              /* bLength */
  USB_DESCIPTOR_TYPE_DEVICE,          /* bDescriptorType */
  0x00U,                              /* bcdUSB LSB (2.00) */
  0x02U,                              /* bcdUSB MSB */
  0x00U,                              /* bDeviceClass */
  0x00U,                              /* bDeviceSubClass */
  0x00U,                              /* bDeviceProtocol */
  USB_MAX_EP0_SIZE,                   /* bMaxPacketSize0 */
  (uint8_t)(PCAN_USB_VID & 0xFFU),    /* idVendor LSB */
  (uint8_t)(PCAN_USB_VID >> 8U),      /* idVendor MSB */
  (uint8_t)(PCAN_USB_PID & 0xFFU),    /* idProduct LSB */
  (uint8_t)(PCAN_USB_PID >> 8U),      /* idProduct MSB */
  0x00U,                              /* bcdDevice LSB */
  0x00U,                              /* bcdDevice MSB */
  USB_MFC_STRING,                     /* iManufacturer */
  USB_PRODUCT_STRING,                 /* iProduct */
  0x00U,                              /* iSerialNumber */
  0x01U                               /* bNumConfigurations */
};

#if defined(__ICCARM__)
  #pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t pcan_usbd_langid_desc[USB_LEN_LANGID_STR_DESC] ALIGNED_TAIL =
{
  USB_LEN_LANGID_STR_DESC,
  USB_DESCIPTOR_TYPE_STRING,
  (uint8_t)(PCAN_USBD_LANGID_STRING & 0xFFU),
  (uint8_t)(PCAN_USBD_LANGID_STRING >> 8U)
};

#if defined(__ICCARM__)
  #pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t pcan_usbd_str_buf[PCAN_USBD_MAX_STR_BYTES] ALIGNED_TAIL;

/* PCAN-USB Pro FD FS configuration descriptor (defined in pcanpro_usbd.c) */
extern uint8_t pcan_fs_cfg_desc[];
extern const uint16_t pcan_fs_cfg_desc_len;

void pcan_usbd_encode_utf16_str(const char *ascii, uint8_t *out, uint16_t max_bytes, uint16_t *out_len)
{
  uint16_t idx = 2U;

  while ((*ascii != '\0') && ((idx + 2U) <= max_bytes))
  {
    out[idx++] = (uint8_t)(*ascii++);
    out[idx++] = 0x00U;
  }

  out[0] = (uint8_t)idx;
  out[1] = USB_DESCIPTOR_TYPE_STRING;
  if (out_len != NULL)
  {
    *out_len = idx;
  }
}

static usbd_desc_t pcan_desc_wrapper;

static usbd_desc_t *pcan_get_device_descriptor(void)
{
  pcan_desc_wrapper.length = (uint16_t)sizeof(pcan_usbd_device_desc);
  pcan_desc_wrapper.descriptor = pcan_usbd_device_desc;
  return &pcan_desc_wrapper;
}

static usbd_desc_t *pcan_get_device_configuration(void)
{
  pcan_desc_wrapper.length = pcan_fs_cfg_desc_len;
  pcan_desc_wrapper.descriptor = pcan_fs_cfg_desc;
  return &pcan_desc_wrapper;
}

static usbd_desc_t *pcan_get_device_lang_id(void)
{
  pcan_desc_wrapper.length = (uint16_t)sizeof(pcan_usbd_langid_desc);
  pcan_desc_wrapper.descriptor = pcan_usbd_langid_desc;
  return &pcan_desc_wrapper;
}

static usbd_desc_t *pcan_get_str_desc(const char *str)
{
  uint16_t len = 0U;
  pcan_usbd_encode_utf16_str(str, pcan_usbd_str_buf, (uint16_t)sizeof(pcan_usbd_str_buf), &len);
  pcan_desc_wrapper.length = len;
  pcan_desc_wrapper.descriptor = pcan_usbd_str_buf;
  return &pcan_desc_wrapper;
}

static usbd_desc_t *pcan_get_manufacturer_string(void)
{
  return pcan_get_str_desc(PCAN_USB_MANUFACTURER_STR);
}

static usbd_desc_t *pcan_get_product_string(void)
{
  return pcan_get_str_desc(PCAN_USB_PRODUCT_STR);
}

static usbd_desc_t *pcan_get_serial_string(void)
{
  return NULL;
}

static usbd_desc_t *pcan_get_config_string(void)
{
  return pcan_get_str_desc(PCAN_USB_CONFIG_STR);
}

static usbd_desc_t *pcan_get_interface_string(void)
{
  return pcan_get_str_desc(PCAN_USB_IF_CAN_STR);
}

usbd_desc_handler pcan_at32_usbd_desc =
{
  .get_device_descriptor = pcan_get_device_descriptor,
  .get_device_qualifier = NULL,
  .get_device_configuration = pcan_get_device_configuration,
  .get_device_other_speed = NULL,
  .get_device_lang_id = pcan_get_device_lang_id,
  .get_device_manufacturer_string = pcan_get_manufacturer_string,
  .get_device_product_string = pcan_get_product_string,
  .get_device_serial_string = pcan_get_serial_string,
  .get_device_interface_string = pcan_get_interface_string,
  .get_device_config_string = pcan_get_config_string,
};
