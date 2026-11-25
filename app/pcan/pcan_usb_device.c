#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "board.h"
#include "hpm_soc.h"
#include "hpm_interrupt.h"
#include "usbd_core.h"
#include "hpm_usbd_compat.h"
#include "pcan_usb_ids.h"
#include "pcanpro_usbd.h"
#include "pcanpro_protocol.h"
#include "pcan_usb_device.h"

#define PCAN_USB_CONFIG_DESC_LEN  (111U)
#define PCAN_RX_RING_SLOTS        (16U)
#define PCAN_TX_BOUNCE_SLOTS      (4U)
#define PCAN_TX_BOUNCE_SIZE       (2048U)

/* USB Descriptors (HS 512B + FS 64B, genuine Pro FD bInterfaceClass = 0x00) */
#define PCAN_CFG_DESC_BODY(desc_type, mps_lo, mps_hi)                                     \
  /* Configuration Descriptor */                                                          \
  0x09U, (desc_type), (uint8_t)(PCAN_USB_CONFIG_DESC_LEN & 0xFFU),                        \
  (uint8_t)(PCAN_USB_CONFIG_DESC_LEN >> 8U),                                              \
  0x02U,                                      /* bNumInterfaces: CAN-FD + LIN */          \
  0x01U,                                      /* bConfigurationValue */                   \
  USB_STRING_CONFIG_INDEX,                    /* iConfiguration */                        \
  0x80U,                                      /* bmAttributes: Bus-powered */             \
  0x78U,                                      /* MaxPower: 240 mA */                      \
  /* Interface 0: PCAN-USB Pro FD (CAN-FD) */                                             \
  0x09U, USB_DESCRIPTOR_TYPE_INTERFACE, 0x00U, 0x00U, 0x06U, 0x00U, 0x00U, 0x00U,         \
  USB_STRING_INTERFACE_INDEX,                                                             \
  0x07U, USB_DESCRIPTOR_TYPE_ENDPOINT, PCAN_USB_EP_CMDIN,      0x02U, (mps_lo), (mps_hi), 0x00U, \
  0x07U, USB_DESCRIPTOR_TYPE_ENDPOINT, PCAN_USB_EP_CMDOUT,     0x02U, (mps_lo), (mps_hi), 0x00U, \
  0x07U, USB_DESCRIPTOR_TYPE_ENDPOINT, PCAN_USB_EP_MSGIN_CH1,  0x02U, (mps_lo), (mps_hi), 0x00U, \
  0x07U, USB_DESCRIPTOR_TYPE_ENDPOINT, PCAN_USB_EP_MSGOUT_CH1, 0x02U, (mps_lo), (mps_hi), 0x00U, \
  0x07U, USB_DESCRIPTOR_TYPE_ENDPOINT, PCAN_USB_EP_MSGIN_CH2,  0x02U, (mps_lo), (mps_hi), 0x00U, \
  0x07U, USB_DESCRIPTOR_TYPE_ENDPOINT, PCAN_USB_EP_MSGOUT_CH2, 0x02U, (mps_lo), (mps_hi), 0x00U, \
  /* Interface 1: PCAN-USB Pro FD (LIN placeholder, distinct EPs 0x84..0x06) */           \
  0x09U, USB_DESCRIPTOR_TYPE_INTERFACE, 0x01U, 0x00U, 0x06U, 0x00U, 0x00U, 0x00U,         \
  PCAN_LIN_INTERFACE_STR_INDEX,                                                           \
  0x07U, USB_DESCRIPTOR_TYPE_ENDPOINT, 0x84U, 0x02U, (mps_lo), (mps_hi), 0x00U,           \
  0x07U, USB_DESCRIPTOR_TYPE_ENDPOINT, 0x04U, 0x02U, (mps_lo), (mps_hi), 0x00U,           \
  0x07U, USB_DESCRIPTOR_TYPE_ENDPOINT, 0x85U, 0x02U, (mps_lo), (mps_hi), 0x00U,           \
  0x07U, USB_DESCRIPTOR_TYPE_ENDPOINT, 0x05U, 0x02U, (mps_lo), (mps_hi), 0x00U,           \
  0x07U, USB_DESCRIPTOR_TYPE_ENDPOINT, 0x86U, 0x02U, (mps_lo), (mps_hi), 0x00U,           \
  0x07U, USB_DESCRIPTOR_TYPE_ENDPOINT, 0x06U, 0x02U, (mps_lo), (mps_hi), 0x00U

static const uint8_t pcan_device_descriptor[] =
{
  USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00U, 0x00U, 0x00U, PCAN_USB_VID, PCAN_USB_PID, 0x0000U, 0x01U),
};

static const uint8_t pcan_device_quality_descriptor[] =
{
  USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(USB_2_0, 0x00U, 0x00U, 0x00U, 0x01U),
};

static const uint8_t pcan_config_desc_hs[] =
{
  PCAN_CFG_DESC_BODY(USB_DESCRIPTOR_TYPE_CONFIGURATION, 0x00U, 0x02U)
};

static const uint8_t pcan_config_desc_fs[] =
{
  PCAN_CFG_DESC_BODY(USB_DESCRIPTOR_TYPE_CONFIGURATION, 0x40U, 0x00U)
};

static const uint8_t pcan_other_speed_desc_hs[] =
{
  PCAN_CFG_DESC_BODY(USB_DESCRIPTOR_TYPE_OTHER_SPEED, 0x40U, 0x00U)
};

static const uint8_t pcan_other_speed_desc_fs[] =
{
  PCAN_CFG_DESC_BODY(USB_DESCRIPTOR_TYPE_OTHER_SPEED, 0x00U, 0x02U)
};

_Static_assert(sizeof(pcan_config_desc_hs) == PCAN_USB_CONFIG_DESC_LEN,
               "Invalid PCAN_PRO_FD HS config descriptor size");
_Static_assert(sizeof(pcan_config_desc_fs) == PCAN_USB_CONFIG_DESC_LEN,
               "Invalid PCAN_PRO_FD FS config descriptor size");

static const uint8_t *device_descriptor_cb(uint8_t speed)
{
  UNUSED(speed);
  return pcan_device_descriptor;
}

static const uint8_t *config_descriptor_cb(uint8_t speed)
{
  return (speed == USB_SPEED_HIGH) ? pcan_config_desc_hs : pcan_config_desc_fs;
}

static const uint8_t *device_quality_cb(uint8_t speed)
{
  UNUSED(speed);
  return pcan_device_quality_descriptor;
}

static const uint8_t *other_speed_cb(uint8_t speed)
{
  return (speed == USB_SPEED_HIGH) ? pcan_other_speed_desc_hs : pcan_other_speed_desc_fs;
}

static const char *string_descriptor_cb(uint8_t speed, uint8_t index)
{
  static const char langid[2] = { 0x09, 0x04 };
  UNUSED(speed);

  switch (index)
  {
    case USB_STRING_LANGID_INDEX:
      return langid;
    case USB_STRING_MFC_INDEX:
      return PCAN_USB_MANUFACTURER_STR;
    case USB_STRING_PRODUCT_INDEX:
      return PCAN_USB_PRODUCT_STR;
    case USB_STRING_SERIAL_INDEX:
      return NULL;
    case USB_STRING_CONFIG_INDEX:
      return PCAN_USB_CONFIG_STR;
    case USB_STRING_INTERFACE_INDEX:
      return PCAN_USB_IF_CAN_STR;
    case PCAN_LIN_INTERFACE_STR_INDEX:
      return PCAN_USB_IF_LIN_STR;
    default:
      return NULL;
  }
}

static const struct usb_descriptor pcan_cherry_desc =
{
  .device_descriptor_callback = device_descriptor_cb,
  .config_descriptor_callback = config_descriptor_cb,
  .device_quality_descriptor_callback = device_quality_cb,
  .other_speed_descriptor_callback = other_speed_cb,
  .string_descriptor_callback = string_descriptor_cb,
};

/* Control staging & ISR-to-main rings (non-cacheable DMA buffers) */
struct t_rx_slot
{
  uint8_t  ep;
  uint16_t len;
  uint8_t  data[PCAN_HS_MAX_BULK_PACKET_SIZE];
};

static USBD_HandleTypeDef g_compat_pdev;
static uint8_t g_ctl_in_shadow[128];
static uint16_t g_ctl_in_len;
static uint8_t *g_ctl_out_dst;
static uint16_t g_ctl_out_len;
static volatile bool g_ctl_failed;
static volatile uint8_t g_ep0_rx_pending;

static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t g_ep1_rx_buf[PCAN_HS_MAX_BULK_PACKET_SIZE];
static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t g_ep2_rx_buf[PCAN_HS_MAX_BULK_PACKET_SIZE];
static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t g_ep3_rx_buf[PCAN_HS_MAX_BULK_PACKET_SIZE];
static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t g_tx_bounce[PCAN_TX_BOUNCE_SLOTS][PCAN_TX_BOUNCE_SIZE];

static struct t_rx_slot g_rx_ring[PCAN_RX_RING_SLOTS];
static volatile uint32_t g_rx_head;
static volatile uint32_t g_rx_tail;

static volatile uint8_t g_ep_tx_in_use[8];
static volatile uint8_t g_ep_tx_need_zlp[8];
static volatile uint16_t g_negotiated_mps = PCAN_FS_MAX_BULK_PACKET_SIZE;

uint8_t USBD_CtlSendData(USBD_HandleTypeDef *pdev, uint8_t *pbuf, uint16_t len)
{
  UNUSED(pdev);
  if (len > (uint16_t)sizeof(g_ctl_in_shadow))
  {
    len = (uint16_t)sizeof(g_ctl_in_shadow);
  }
  if ((pbuf != NULL) && (len > 0U))
  {
    memcpy(g_ctl_in_shadow, pbuf, len);
  }
  g_ctl_in_len = len;
  return USBD_OK;
}

uint8_t USBD_CtlPrepareRx(USBD_HandleTypeDef *pdev, uint8_t *pbuf, uint16_t len)
{
  UNUSED(pdev);
  g_ctl_out_dst = pbuf;
  g_ctl_out_len = len;
  return USBD_OK;
}

uint8_t USBD_CtlSendStatus(USBD_HandleTypeDef *pdev)
{
  UNUSED(pdev);
  return USBD_OK;
}

void USBD_CtlError(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  UNUSED(pdev);
  UNUSED(req);
  g_ctl_failed = true;
}

static int pcan_vendor_handler(uint8_t busid, struct usb_setup_packet *setup,
                               uint8_t **data, uint32_t *len)
{
  UNUSED(busid);

  bool out_with_data = (setup->wLength != 0U) && ((setup->bmRequestType & 0x80U) == 0U);
  g_ctl_in_len = 0U;
  g_ctl_out_dst = NULL;
  g_ctl_out_len = 0U;
  g_ctl_failed = false;

  uint8_t st = pcan_protocol_device_setup(&g_compat_pdev, (USBD_SetupReqTypedef *)setup);
  if (g_ctl_failed || (st != USBD_OK))
  {
    return -1;
  }

  if (g_ctl_in_len > 0U)
  {
    *data = g_ctl_in_shadow;
    *len = g_ctl_in_len;
    return 0;
  }

  if (out_with_data)
  {
    if ((g_ctl_out_dst != NULL) && (*data != NULL))
    {
      uint32_t n = MIN(*len, (uint32_t)g_ctl_out_len);
      memcpy(g_ctl_out_dst, *data, n);
    }
    g_ep0_rx_pending = 1U;
  }

  return 0;
}

static uint8_t *ep_rx_buf_for_addr(uint8_t ep_addr)
{
  switch (ep_addr)
  {
    case PCAN_USB_EP_CMDOUT:
      return g_ep1_rx_buf;
    case PCAN_USB_EP_MSGOUT_CH1:
      return g_ep2_rx_buf;
    case PCAN_USB_EP_MSGOUT_CH2:
      return g_ep3_rx_buf;
    default:
      return NULL;
  }
}

static void pcan_ep_in_cb(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
  UNUSED(nbytes);
  uint8_t idx = (uint8_t)(ep & 0x0FU);
  if (idx >= 8U)
  {
    return;
  }

  if (g_ep_tx_need_zlp[idx] != 0U)
  {
    g_ep_tx_need_zlp[idx] = 0U;
    (void)usbd_ep_start_write(busid, ep, g_tx_bounce[idx], 0U);
    return;
  }

  g_ep_tx_in_use[idx] = 0U;
}

static void pcan_ep_out_cb(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
  uint8_t *rx_buf = ep_rx_buf_for_addr(ep);
  if (rx_buf == NULL)
  {
    return;
  }

  uint32_t next_head = (g_rx_head + 1U) % PCAN_RX_RING_SLOTS;
  if (next_head != g_rx_tail)
  {
    uint16_t copy_len = (nbytes <= PCAN_HS_MAX_BULK_PACKET_SIZE)
                            ? (uint16_t)nbytes
                            : PCAN_HS_MAX_BULK_PACKET_SIZE;
    g_rx_ring[g_rx_head].ep = ep;
    g_rx_ring[g_rx_head].len = copy_len;
    if (copy_len > 0U)
    {
      memcpy(g_rx_ring[g_rx_head].data, rx_buf, copy_len);
    }
    g_rx_head = next_head;
  }

  uint16_t mps = g_negotiated_mps;
  if ((mps != PCAN_HS_MAX_BULK_PACKET_SIZE) && (mps != PCAN_FS_MAX_BULK_PACKET_SIZE))
  {
    mps = PCAN_HS_MAX_BULK_PACKET_SIZE;
  }
  (void)usbd_ep_start_read(busid, ep, rx_buf, mps);
}

static void pcan_usbd_event_cb(uint8_t busid, uint8_t event)
{
  switch (event)
  {
    case USBD_EVENT_RESET:
      memset((void *)g_ep_tx_in_use, 0, sizeof(g_ep_tx_in_use));
      memset((void *)g_ep_tx_need_zlp, 0, sizeof(g_ep_tx_need_zlp));
      g_rx_head = 0U;
      g_rx_tail = 0U;
      g_compat_pdev.dev_state = 0U;
      break;

    case USBD_EVENT_CONFIGURED:
    {
      uint16_t mps = usbd_get_ep_mps(busid, PCAN_USB_EP_MSGOUT_CH1);
      if (mps == 0U)
      {
        mps = PCAN_FS_MAX_BULK_PACKET_SIZE;
      }
      g_negotiated_mps = mps;
      g_compat_pdev.dev_state = 1U;
      memset((void *)g_ep_tx_in_use, 0, sizeof(g_ep_tx_in_use));
      memset((void *)g_ep_tx_need_zlp, 0, sizeof(g_ep_tx_need_zlp));

      (void)usbd_ep_start_read(busid, PCAN_USB_EP_CMDOUT, g_ep1_rx_buf, mps);
      (void)usbd_ep_start_read(busid, PCAN_USB_EP_MSGOUT_CH1, g_ep2_rx_buf, mps);
      (void)usbd_ep_start_read(busid, PCAN_USB_EP_MSGOUT_CH2, g_ep3_rx_buf, mps);
      break;
    }

    default:
      break;
  }
}

static struct usbd_interface g_intf_can =
{
  .vendor_handler = pcan_vendor_handler,
};

static struct usbd_interface g_intf_lin =
{
  .vendor_handler = pcan_vendor_handler,
};

static struct usbd_endpoint g_ep_cmd_in  = { .ep_addr = PCAN_USB_EP_CMDIN,      .ep_cb = pcan_ep_in_cb };
static struct usbd_endpoint g_ep_cmd_out = { .ep_addr = PCAN_USB_EP_CMDOUT,     .ep_cb = pcan_ep_out_cb };
static struct usbd_endpoint g_ep_ch1_in  = { .ep_addr = PCAN_USB_EP_MSGIN_CH1,  .ep_cb = pcan_ep_in_cb };
static struct usbd_endpoint g_ep_ch1_out = { .ep_addr = PCAN_USB_EP_MSGOUT_CH1, .ep_cb = pcan_ep_out_cb };
static struct usbd_endpoint g_ep_ch2_in  = { .ep_addr = PCAN_USB_EP_MSGIN_CH2,  .ep_cb = pcan_ep_in_cb };
static struct usbd_endpoint g_ep_ch2_out = { .ep_addr = PCAN_USB_EP_MSGOUT_CH2, .ep_cb = pcan_ep_out_cb };

void pcan_usb_device_init(void)
{
  board_init_usb(BOARD_USB_BASE);
  intc_set_irq_priority(BOARD_USB_IRQn, 2U);

  memset(&g_compat_pdev, 0, sizeof(g_compat_pdev));

  usbd_desc_register(0U, &pcan_cherry_desc);
  usbd_add_interface(0U, &g_intf_can);
  usbd_add_interface(0U, &g_intf_lin);
  usbd_add_endpoint(0U, &g_ep_cmd_in);
  usbd_add_endpoint(0U, &g_ep_cmd_out);
  usbd_add_endpoint(0U, &g_ep_ch1_in);
  usbd_add_endpoint(0U, &g_ep_ch1_out);
  usbd_add_endpoint(0U, &g_ep_ch2_in);
  usbd_add_endpoint(0U, &g_ep_ch2_out);

  (void)usbd_initialize(0U, (uintptr_t)BOARD_USB_BASE, pcan_usbd_event_cb);
}

void pcan_usb_device_poll(void)
{
  if (g_ep0_rx_pending != 0U)
  {
    g_ep0_rx_pending = 0U;
    pcan_ep0_receive();
  }

  while (g_rx_tail != g_rx_head)
  {
    struct t_rx_slot slot;
    slot.ep = g_rx_ring[g_rx_tail].ep;
    slot.len = g_rx_ring[g_rx_tail].len;
    if (slot.len > 0U)
    {
      memcpy(slot.data, g_rx_ring[g_rx_tail].data, slot.len);
    }
    g_rx_tail = (g_rx_tail + 1U) % PCAN_RX_RING_SLOTS;

    pcan_protocol_process_data(slot.ep, slot.data, slot.len);
  }
}

uint16_t pcan_usb_frame_number(void)
{
  return (uint16_t)(BOARD_USB_BASE->FRINDEX & 0x3FFFU);
}

uint16_t pcan_usb_get_mps(void)
{
  return g_negotiated_mps;
}

int pcan_flush_ep(uint8_t ep)
{
  uint8_t idx = (uint8_t)(ep & 0x0FU);
  if (idx >= 8U)
  {
    return 0;
  }

  g_ep_tx_in_use[idx] = 0U;
  g_ep_tx_need_zlp[idx] = 0U;
  return 1;
}

int pcan_flush_data(struct t_m2h_fsm *pfsm, void *src, int size)
{
  if ((pfsm == NULL) || !usb_device_is_configured(0U))
  {
    return 0;
  }

  uint8_t ep_idx = (uint8_t)(pfsm->ep_addr & 0x0FU);
  if ((ep_idx >= PCAN_TX_BOUNCE_SLOTS) || (ep_idx == 0U))
  {
    return 0;
  }

  switch (pfsm->state)
  {
    case 1:
      if (g_ep_tx_in_use[ep_idx] != 0U)
      {
        return 0;
      }
      pfsm->state = 0U;
      /* fall through */

    case 0:
      if ((g_ep_tx_in_use[ep_idx] != 0U) || (size < 0) ||
          ((uint32_t)size > pfsm->dbsize) || ((uint32_t)size > PCAN_TX_BOUNCE_SIZE))
      {
        return 0;
      }

      if ((size > 0) && (src != NULL))
      {
        memcpy(g_tx_bounce[ep_idx], src, (size_t)size);
      }

      g_ep_tx_in_use[ep_idx] = 1U;
      g_ep_tx_need_zlp[ep_idx] =
          ((size > 0) && (g_negotiated_mps > 0U) && (((uint32_t)size % g_negotiated_mps) == 0U)) ? 1U : 0U;

      if (usbd_ep_start_write(0U, pfsm->ep_addr, g_tx_bounce[ep_idx], (uint32_t)size) != 0)
      {
        g_ep_tx_in_use[ep_idx] = 0U;
        g_ep_tx_need_zlp[ep_idx] = 0U;
        return 0;
      }

      pfsm->total_tx += (uint32_t)size;
      pfsm->state = 1U;
      return 1;

    default:
      pfsm->state = 0U;
      return 0;
  }
}
