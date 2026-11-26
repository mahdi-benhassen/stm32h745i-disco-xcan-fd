#include <stdint.h>
#include <string.h>
#include "hpm_usbd_compat.h"
#include "pcanpro_can.h"
#include "pcan_usbpro_fw.h"
#include "pcanfd_usb_fw.h"
#include "pcanpro_timestamp.h"
#include "pcanpro_protocol.h"
#include "pcanpro_led.h"
#include "pcanpro_usbd.h"
#include "pcan_usb_device.h"
#include "plin_protocol.h"

#define CAN_CHANNEL_MAX            (2U)
#define PCAN_USB_DATA_BUFFER_SIZE  (2048U)
#define PCAN_TX_REASM_BUFFER_SIZE  (1024U)

struct pcan_usbfd_fw_info
{
  uint16_t  size_of;        /* sizeof this */
  uint16_t  type;           /* type of this structure */
  uint8_t   hw_type;        /* Type of hardware (HW_TYPE_xxx) */
  uint8_t   bl_version[3];  /* Bootloader version */
  uint8_t   hw_version;     /* Hardware version (PCB) */
  uint8_t   fw_version[3];  /* Firmware version */
  uint32_t  dev_id[2];      /* "device id" per CAN */
  uint32_t  ser_no;         /* S/N */
  uint32_t  flags;          /* special functions */
  uint8_t   unk[8];
} __attribute__((packed));

static struct
{
  uint32_t device_nr;
  uint32_t last_bus_load_update;
  uint32_t last_time_sync;
  uint32_t last_time_flush;
  uint8_t  can_drv_loaded;
  uint8_t  lin_drv_loaded;

  struct
  {
    uint8_t   silient;
    uint8_t   bus_active;
    uint8_t   loopback;
    uint8_t   err_mask;
    uint32_t  channel_nr;
    uint16_t  opt_mask;
    uint8_t   led_is_busy;

    struct ucan_timing_slow slow_br;
    struct ucan_timing_fast fast_br;

    uint32_t can_clock;
    uint32_t nominal_qt;
    uint32_t data_qt;
    uint32_t tx_time_ns;
    uint32_t rx_time_ns;
    uint32_t bus_load;
  } can[CAN_CHANNEL_MAX];
} pcan_device =
{
  .device_nr = 0xFFFFFFFFU,
  .can[0] =
  {
    .channel_nr = 0xFFFFFFFFU,
    .can_clock = PCAN_FDCAN_CLOCK_HZ,
    .nominal_qt = 2000U, /* 500 kbps */
    .data_qt = 500U,     /* 2 Mbps */
  },
  .can[1] =
  {
    .channel_nr = 0xFFFFFFFFU,
    .can_clock = PCAN_FDCAN_CLOCK_HZ,
    .nominal_qt = 2000U,
    .data_qt = 500U,
  },
};

__ALIGN_BEGIN static uint8_t resp_buffer[2][PCAN_USB_DATA_BUFFER_SIZE] __ALIGN_END;
__ALIGN_BEGIN static uint8_t data_buffer[PCAN_USB_DATA_BUFFER_SIZE] __ALIGN_END;
static uint8_t drv_load_packet[16];
static uint16_t data_pos = 0U;
__ALIGN_BEGIN static uint8_t tx_reasm_buf[CAN_CHANNEL_MAX][PCAN_TX_REASM_BUFFER_SIZE] __ALIGN_END;
static uint16_t tx_reasm_len[CAN_CHANNEL_MAX];

static struct t_m2h_fsm resp_fsm[2] =
{
  [0] =
  {
    .state = 0U,
    .ep_addr = PCAN_USB_EP_CMDIN,
    .pdbuf = resp_buffer[0],
    .dbsize = PCAN_USB_DATA_BUFFER_SIZE,
  },
  [1] =
  {
    .state = 0U,
    .ep_addr = PCAN_USB_EP_MSGIN_CH1,
    .pdbuf = resp_buffer[1],
    .dbsize = PCAN_USB_DATA_BUFFER_SIZE,
  },
};

static void *pcan_data_alloc_buffer(uint16_t type, uint16_t size)
{
  uint16_t aligned_size = (uint16_t)((size + 3U) & ~3U);
  if ((uint32_t)sizeof(data_buffer) < ((uint32_t)aligned_size + (uint32_t)data_pos + 4U))
  {
    return NULL;
  }

  struct ucan_msg *pmsg = (struct ucan_msg *)&data_buffer[data_pos];
  uint64_t ts = pcan_timestamp_us64();

  pmsg->size = aligned_size;
  pmsg->type = type;
  pmsg->ts_low = (uint32_t)ts;
  pmsg->ts_high = (uint32_t)(ts >> 32);

  data_pos = (uint16_t)(data_pos + aligned_size);
  return pmsg;
}

static int pcan_protocol_try_flush_data(void)
{
  if (data_pos == 0U)
  {
    return 0;
  }

  uint16_t flush_size = (uint16_t)((data_pos + 4U + 63U) & ~63U);
  if (flush_size > (uint16_t)sizeof(data_buffer))
  {
    flush_size = (uint16_t)sizeof(data_buffer);
  }
  if (flush_size > data_pos)
  {
    memset(&data_buffer[data_pos], 0, (size_t)(flush_size - data_pos));
  }

  if (pcan_flush_data(&resp_fsm[1], data_buffer, (int)flush_size) != 0)
  {
    data_pos = 0U;
    pcan_device.last_time_flush = pcan_timestamp_us();
    return 1;
  }

  return 0;
}

int pcan_protocol_can_rx_has_space(void)
{
  if (((uint32_t)sizeof(data_buffer) - (uint32_t)data_pos) >= 96U)
  {
    return 1;
  }
  return pcan_protocol_try_flush_data();
}

uint8_t pcan_protocol_device_setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  switch (req->bRequest)
  {
    case USB_VENDOR_REQUEST_INFO:
      switch (req->wValue)
      {
        case USB_VENDOR_REQUEST_wVALUE_INFO_BOOTLOADER:
        {
          static struct pcan_usbpro_bootloader_info bli =
          {
            .ctrl_type = BOOTLOADER_INFO_STRUCT_TYPE,
            .version = { 2U, 1U, 0U, 0U },
            .day = 1U,
            .month = 1U,
            .year = 26U,
            .dummy = 0U,
            .serial_num_high = 0U,
            .serial_num_low = 1U,
            .hw_type = 0U,
            .hw_rev = 0U,
          };

          if (req->wLength == 0U)
          {
            USBD_CtlError(pdev, req);
            return (uint8_t)USBD_FAIL;
          }

          bli.serial_num_low = pcan_device.device_nr;
          uint16_t send_len = MIN(req->wLength, (uint16_t)sizeof(bli));
          (void)USBD_CtlSendData(pdev, (uint8_t *)&bli, send_len);
          return (uint8_t)USBD_OK;
        }

        case USB_VENDOR_REQUEST_wVALUE_INFO_FIRMWARE:
        {
          static struct pcan_usbfd_fw_info fwi =
          {
            .size_of = (uint16_t)sizeof(struct pcan_usbfd_fw_info),
            .type = 2U,
            .hw_type = 1U,
#if defined(PCAN_X6)
            .bl_version = { 1U, 1U, 0U },
            .hw_version = 3U,
#else
            .bl_version = { 2U, 1U, 0U },
            .hw_version = 2U,
#endif
            .fw_version = { 3U, 2U, 0U },
            .dev_id = { 0xFFFFFFFFU, 0xFFFFFFFFU },
            .ser_no = 0xFFFFFFFFU,
            .flags = 0x00000000U,
            .unk =
            {
              0x01U, /* cmd_out */
              0x81U, /* cmd_in */
              0x02U, /* write ch1 */
              0x03U, /* write ch2 */
              0x82U, /* read */
              0x00U,
              0x00U,
              0x00U
            }
          };

          if (req->wLength == 0U)
          {
            USBD_CtlError(pdev, req);
            return (uint8_t)USBD_FAIL;
          }

          uint16_t send_len = MIN(req->wLength, (uint16_t)sizeof(fwi));
          fwi.size_of = send_len;
          fwi.dev_id[0] = pcan_device.can[0].channel_nr;
          fwi.dev_id[1] = pcan_device.can[1].channel_nr;
          fwi.ser_no = pcan_device.device_nr;
          (void)USBD_CtlSendData(pdev, (uint8_t *)&fwi, send_len);
          return (uint8_t)USBD_OK;
        }
        default:
          USBD_CtlError(pdev, req);
          return (uint8_t)USBD_FAIL;
      }

    case USB_VENDOR_REQUEST_FKT:
      switch (req->wValue)
      {
        case USB_VENDOR_REQUEST_wVALUE_SETFKT_BOOT:
          USBD_CtlError(pdev, req);
          return (uint8_t)USBD_FAIL;

        case USB_VENDOR_REQUEST_wVALUE_SETFKT_INTERFACE_DRIVER_LOADED:
        {
          uint16_t rx_len = MIN(req->wLength, (uint16_t)sizeof(drv_load_packet));
          memset(drv_load_packet, 0, sizeof(drv_load_packet));
          if (rx_len > 0U)
          {
            (void)USBD_CtlPrepareRx(pdev, drv_load_packet, rx_len);
          }
          else
          {
            (void)USBD_CtlSendStatus(pdev);
          }
          return (uint8_t)USBD_OK;
        }

        default:
          USBD_CtlError(pdev, req);
          return (uint8_t)USBD_FAIL;
      }

    case USB_VENDOR_REQUEST_ZERO:
      if (req->wLength == 0U)
      {
        (void)USBD_CtlSendStatus(pdev);
        return (uint8_t)USBD_OK;
      }
      USBD_CtlError(pdev, req);
      return (uint8_t)USBD_FAIL;

    default:
      USBD_CtlError(pdev, req);
      return (uint8_t)USBD_FAIL;
  }
}

void pcan_ep0_receive_data(const uint8_t *buf, uint16_t len)
{
  if ((buf == NULL) || (len < 2U))
  {
    return;
  }

  if (buf[0] == 0U)
  {
    (void)pcan_flush_ep(PCAN_USB_EP_MSGIN_CH1);
    (void)pcan_flush_ep(PCAN_USB_EP_CMDIN);
    resp_fsm[0].state = 0U;
    resp_fsm[1].state = 0U;
    data_pos = 0U;
    memset(tx_reasm_len, 0, sizeof(tx_reasm_len));
    pcan_device.can_drv_loaded = buf[1];
    pcan_led_set_mode(LED_STAT,
                      (pcan_device.can_drv_loaded != 0U) ? LED_MODE_BLINK_SLOW : LED_MODE_BLINK_FAST,
                      0xFFFFFFFFU);
  }
  else
  {
    pcan_device.lin_drv_loaded = buf[1];
    pcan_usb_flush_lin_eps();
    plin_protocol_on_driver_state(pcan_device.lin_drv_loaded);
  }
}

static int pcan_protocol_rx_frame(uint8_t channel, struct t_can_msg *pmsg)
{
  static const uint8_t pcan_fd_len2dlc[65] =
  {
     0,  1,  2,  3,  4,  5,  6,  7,  8,
     9,  9,  9,  9,
    10, 10, 10, 10,
    11, 11, 11, 11,
    12, 12, 12, 12,
    13, 13, 13, 13, 13, 13, 13, 13,
    14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
    15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15
  };

  if ((channel >= CAN_CHANNEL_MAX) || (pmsg == NULL))
  {
    return -1;
  }

  uint8_t payload_len = (pmsg->size > CAN_PAYLOAD_MAX_SIZE) ? (uint8_t)CAN_PAYLOAD_MAX_SIZE : pmsg->size;
  uint16_t rec_size = (uint16_t)(sizeof(struct ucan_rx_msg) + payload_len);
  struct ucan_rx_msg *pcan_msg = pcan_data_alloc_buffer(UCAN_MSG_CAN_RX, rec_size);
  if (pcan_msg == NULL)
  {
    if (pcan_protocol_try_flush_data() != 0)
    {
      pcan_msg = pcan_data_alloc_buffer(UCAN_MSG_CAN_RX, rec_size);
    }
    if (pcan_msg == NULL)
    {
      return -1;
    }
  }

  if ((pmsg->flags & MSG_FLAG_ECHO) == 0U)
  {
    uint32_t nqt = pcan_device.can[channel].nominal_qt;
    uint32_t dqt = pcan_device.can[channel].data_qt;
    pcan_device.can[channel].rx_time_ns += pcan_can_msg_time(pmsg, nqt, dqt);
    if (pcan_device.can[channel].led_is_busy == 0U)
    {
      pcan_led_set_mode((channel != 0U) ? LED_CH1_RX : LED_CH0_RX, LED_MODE_BLINK_FAST, 237U);
    }
  }

  pcan_msg->channel_dlc = (uint8_t)UCAN_MSG_CHANNEL_DLC(channel, pcan_fd_len2dlc[payload_len]);
  pcan_msg->client = pmsg->dummy;
  pcan_msg->flags = 0U;
  pcan_msg->tag_low = 0U;
  pcan_msg->tag_high = 0U;

  if ((pmsg->flags & MSG_FLAG_RTR) != 0U)
  {
    pcan_msg->flags |= UCAN_MSG_RTR;
  }
  if ((pmsg->flags & MSG_FLAG_EXT) != 0U)
  {
    pcan_msg->flags |= UCAN_MSG_EXT_ID;
  }
  if ((pmsg->flags & MSG_FLAG_FD) != 0U)
  {
    pcan_msg->flags |= UCAN_MSG_EXT_DATA_LEN;
    if ((pmsg->flags & MSG_FLAG_BRS) != 0U)
    {
      pcan_msg->flags |= UCAN_MSG_BITRATE_SWITCH;
    }
    if ((pmsg->flags & MSG_FLAG_ESI) != 0U)
    {
      pcan_msg->flags |= UCAN_MSG_ERROR_STATE_IND;
    }
  }
  if ((pmsg->flags & MSG_FLAG_ECHO) != 0U)
  {
    pcan_msg->flags |= UCAN_MSG_API_SRR | UCAN_MSG_HW_SRR;
  }
  else
  {
    pcan_msg->ts_low = pmsg->timestamp;
  }

  pcan_msg->can_id = pmsg->id;
  if (payload_len > 0U)
  {
    memcpy(pcan_msg->d, pmsg->data, payload_len);
  }
  return 0;
}

static int pcan_protocol_tx_frame_cb(uint8_t channel, struct t_can_msg *pmsg)
{
  if ((channel >= CAN_CHANNEL_MAX) || (pmsg == NULL))
  {
    return -1;
  }

  if ((pmsg->flags & MSG_FLAG_ECHO) != 0U)
  {
    (void)pcan_protocol_rx_frame(channel, pmsg);
  }

  if (pcan_device.can[channel].led_is_busy == 0U)
  {
    pcan_led_set_mode((channel != 0U) ? LED_CH1_TX : LED_CH0_TX, LED_MODE_BLINK_FAST, 237U);
  }

  uint32_t nqt = pcan_device.can[channel].nominal_qt;
  uint32_t dqt = pcan_device.can[channel].data_qt;
  pcan_device.can[channel].tx_time_ns += pcan_can_msg_time(pmsg, nqt, dqt);
  return 0;
}

static int pcan_protocol_tx_frame(const struct ucan_tx_msg *pmsg, uint16_t msg_bytes)
{
  static const uint8_t pcan_fd_dlc2len[16] =
  {
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
    8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U
  };

  uint8_t channel = UCAN_MSG_CHANNEL(pmsg);
  if (channel >= CAN_CHANNEL_MAX)
  {
    return -1;
  }

  uint8_t dlc = UCAN_MSG_DLC(pmsg);
  if (dlc >= 16U)
  {
    return -1;
  }

  struct t_can_msg msg = { 0 };
  msg.id = pmsg->can_id;

  if ((pmsg->flags & UCAN_MSG_EXT_DATA_LEN) != 0U)
  {
    msg.flags |= MSG_FLAG_FD;
    if ((pmsg->flags & UCAN_MSG_BITRATE_SWITCH) != 0U)
    {
      msg.flags |= MSG_FLAG_BRS;
    }
    if ((pmsg->flags & UCAN_MSG_ERROR_STATE_IND) != 0U)
    {
      msg.flags |= MSG_FLAG_ESI;
    }
    msg.size = pcan_fd_dlc2len[dlc];
  }
  else
  {
    msg.size = (dlc > 8U) ? 8U : dlc;
  }

  if ((msg.size > (uint8_t)sizeof(msg.data)) ||
      ((uint32_t)msg_bytes < (uint32_t)(sizeof(struct ucan_tx_msg) + msg.size)))
  {
    return -1;
  }

  if ((pmsg->flags & UCAN_MSG_RTR) != 0U)
  {
    msg.flags |= MSG_FLAG_RTR;
  }
  if ((pmsg->flags & UCAN_MSG_EXT_ID) != 0U)
  {
    msg.flags |= MSG_FLAG_EXT;
  }
  if ((pmsg->flags & UCAN_MSG_HW_SRR) != 0U)
  {
    msg.flags |= MSG_FLAG_ECHO;
    msg.dummy = pmsg->client;
  }

  if (msg.size > 0U)
  {
    memcpy(msg.data, pmsg->d, msg.size);
  }
  msg.timestamp = pcan_timestamp_us();

  (void)pcan_can_write(channel, &msg);
  return 0;
}

static void pcan_protocol_send_status(uint8_t channel, uint8_t status)
{
  struct ucan_status_msg *ps = pcan_data_alloc_buffer(UCAN_MSG_STATUS, (uint16_t)sizeof(struct ucan_status_msg));
  if (ps == NULL)
  {
    return;
  }

  ps->channel_p_w_b = (uint8_t)((channel & 0x0FU) | ((status & 0x0FU) << 4));
  ps->unused[0] = 0U;
  ps->unused[1] = 0U;
  ps->unused[2] = 0U;
}

static int pcan_protocol_set_baudrate(uint8_t channel, const struct t_can_bitrate *pbitrate,
                                      const struct t_can_bitrate *pdata_bitrate)
{
  const struct t_can_bitrate *pcur = (pbitrate != NULL) ? pbitrate : pdata_bitrate;
  if ((channel >= CAN_CHANNEL_MAX) || (pcur == NULL))
  {
    return -1;
  }

  uint32_t clk = pcan_device.can[channel].can_clock;
  uint32_t total_tq = 1U + (uint32_t)pcur->tseg1 + (uint32_t)pcur->tseg2;
  if ((pcur->brp == 0U) || (total_tq <= 1U) || (clk == 0U))
  {
    return -1;
  }

  uint32_t bitrate = (clk / pcur->brp) / total_tq;
  if (bitrate == 0U)
  {
    return -1;
  }

  int is_data = (pdata_bitrate != NULL) ? 1 : 0;

  if ((PCAN_FDCAN_CLOCK_HZ % clk) == 0U)
  {
    uint32_t scale = PCAN_FDCAN_CLOCK_HZ / clk;
    uint32_t hw_brp = (uint32_t)pcur->brp * scale;
    uint32_t max_brp = (is_data != 0) ? 32U : 512U;

    if (hw_brp <= max_brp)
    {
      int rc = (is_data != 0)
                   ? pcan_can_set_data_timing(channel, hw_brp, pcur->tseg1, pcur->tseg2, pcur->sjw)
                   : pcan_can_set_nominal_timing(channel, hw_brp, pcur->tseg1, pcur->tseg2, pcur->sjw);
      if (rc < 0)
      {
        return rc;
      }
    }
    else
    {
      pcan_can_set_bitrate(channel, bitrate, is_data);
    }
  }
  else
  {
    pcan_can_set_bitrate(channel, bitrate, is_data);
  }

  if (is_data != 0)
  {
    pcan_device.can[channel].data_qt = 1000000000U / bitrate;
  }
  else
  {
    pcan_device.can[channel].nominal_qt = 1000000000U / bitrate;
  }

  return 0;
}

static void pcan_protocol_process_cmd(const uint8_t *ptr, uint16_t size)
{
  while (size >= sizeof(struct ucan_command))
  {
    struct ucan_command cmd;
    memcpy(&cmd, ptr, sizeof(cmd));

    uint16_t opcode = UCAN_CMD_OPCODE(&cmd);
    uint8_t channel = (uint8_t)UCAN_CMD_CHANNEL(&cmd);

    switch (opcode)
    {
      case UCAN_CMD_NOP:
        break;

      case UCAN_CMD_RESET_MODE:
        if (channel < CAN_CHANNEL_MAX)
        {
          uint8_t bus_iso_mode = (pcan_device.can[channel].opt_mask & UCAN_OPTION_ISO_MODE) != 0U;
          pcan_can_set_iso_mode(channel, bus_iso_mode);
          pcan_can_set_20ab_mode(channel, (pcan_device.can[channel].opt_mask & UCAN_OPTION_20AB_MODE) != 0U);
          pcan_can_set_bus_active(channel, 0U);
          pcan_device.can[channel].bus_active = 0U;
        }
        break;

      case UCAN_CMD_NORMAL_MODE:
        if (channel < CAN_CHANNEL_MAX)
        {
          if (pcan_device.can[channel].silient != 0U)
          {
            pcan_can_set_silent(channel, 0U);
            pcan_device.can[channel].silient = 0U;
          }
          pcan_device.can[channel].bus_active = 1U;
          pcan_can_set_bus_active(channel, 1U);
          pcan_protocol_send_status(channel, 0U);
        }
        break;

      case UCAN_CMD_LISTEN_ONLY_MODE:
        if (channel < CAN_CHANNEL_MAX)
        {
          pcan_can_set_silent(channel, 1U);
          pcan_can_set_bus_active(channel, 1U);
          pcan_device.can[channel].silient = 1U;
          pcan_device.can[channel].bus_active = 1U;
          pcan_protocol_send_status(channel, 0U);
        }
        break;

      case UCAN_CMD_TIMING_SLOW:
        if (channel < CAN_CHANNEL_MAX)
        {
          struct ucan_timing_slow timing;
          struct t_can_bitrate br;
          memcpy(&timing, ptr, sizeof(timing));
          pcan_device.can[channel].slow_br = timing;

          br.brp = (uint16_t)(((uint32_t)timing.brp & (uint32_t)UCAN_TSLOW_BRP_MASK) + 1U);
          br.tseg1 = (uint16_t)(((uint32_t)timing.tseg1 & (uint32_t)UCAN_TSLOW_TSEG1_MASK) + 1U);
          br.tseg2 = (uint8_t)(((uint32_t)timing.tseg2 & (uint32_t)UCAN_TSLOW_TSEG2_MASK) + 1U);
          br.sjw = (uint8_t)(((uint32_t)timing.sjw_t & (uint32_t)UCAN_TSLOW_SJW_MASK) + 1U);

          (void)pcan_protocol_set_baudrate(channel, &br, NULL);
        }
        break;

      case UCAN_CMD_TIMING_FAST:
        if (channel < CAN_CHANNEL_MAX)
        {
          struct ucan_timing_fast timing;
          struct t_can_bitrate br;
          memcpy(&timing, ptr, sizeof(timing));
          pcan_device.can[channel].fast_br = timing;

          br.brp = (uint16_t)(((uint32_t)timing.brp & (uint32_t)UCAN_TFAST_BRP_MASK) + 1U);
          br.tseg1 = (uint16_t)(((uint32_t)timing.tseg1 & (uint32_t)UCAN_TFAST_TSEG1_MASK) + 1U);
          br.tseg2 = (uint8_t)(((uint32_t)timing.tseg2 & (uint32_t)UCAN_TFAST_TSEG2_MASK) + 1U);
          br.sjw = (uint8_t)(((uint32_t)timing.sjw & (uint32_t)UCAN_TFAST_SJW_MASK) + 1U);

          (void)pcan_protocol_set_baudrate(channel, NULL, &br);
        }
        break;

      case UCAN_CMD_SET_STD_FILTER:
      case UCAN_CMD_FILTER_STD:
        if (channel < CAN_CHANNEL_MAX)
        {
          (void)pcan_can_set_filter_mask(channel, 0, 0, 0U, 0U);
        }
        break;

      case UCAN_CMD_TX_ABORT:
        if (channel < CAN_CHANNEL_MAX)
        {
          pcan_can_tx_abort(channel);
        }
        break;

      case UCAN_CMD_SET_EN_OPTION:
        if (channel < CAN_CHANNEL_MAX)
        {
          struct ucan_option opt;
          memcpy(&opt, ptr, sizeof(opt));
          pcan_device.can[channel].opt_mask |= opt.mask;
          if ((opt.mask & UCAN_OPTION_ISO_MODE) != 0U)
          {
            pcan_can_set_iso_mode(channel, 1U);
          }
          if ((opt.mask & UCAN_OPTION_20AB_MODE) != 0U)
          {
            pcan_can_set_20ab_mode(channel, 1U);
          }
        }
        break;

      case UCAN_CMD_CLR_DIS_OPTION:
        if (channel < CAN_CHANNEL_MAX)
        {
          struct ucan_option opt;
          memcpy(&opt, ptr, sizeof(opt));
          pcan_device.can[channel].opt_mask &= (uint16_t)(~opt.mask);
          if ((opt.mask & UCAN_OPTION_ISO_MODE) != 0U)
          {
            pcan_can_set_iso_mode(channel, 0U);
          }
          if ((opt.mask & UCAN_OPTION_20AB_MODE) != 0U)
          {
            pcan_can_set_20ab_mode(channel, 0U);
          }
        }
        break;

      case UCAN_USB_CMD_CLK_SET:
        if (channel < CAN_CHANNEL_MAX)
        {
          struct ucan_usb_clock clk;
          memcpy(&clk, ptr, sizeof(clk));
          switch (clk.mode)
          {
            default:
            case UCAN_USB_CLK_80MHZ:
              pcan_device.can[channel].can_clock = 80000000U;
              break;
            case UCAN_USB_CLK_60MHZ:
              pcan_device.can[channel].can_clock = 60000000U;
              break;
            case UCAN_USB_CLK_40MHZ:
              pcan_device.can[channel].can_clock = 40000000U;
              break;
            case UCAN_USB_CLK_30MHZ:
              pcan_device.can[channel].can_clock = 30000000U;
              break;
            case UCAN_USB_CLK_24MHZ:
              pcan_device.can[channel].can_clock = 24000000U;
              break;
            case UCAN_USB_CLK_20MHZ:
              pcan_device.can[channel].can_clock = 20000000U;
              break;
          }
        }
        break;

      case UCAN_USB_CMD_LED_SET:
        if (channel < CAN_CHANNEL_MAX)
        {
          struct ucan_usb_led led;
          memcpy(&led, ptr, sizeof(led));
          pcan_device.can[channel].led_is_busy = led.mode;
          pcan_led_set_mode(channel, led.mode, 0xFFFFFFFFU);
        }
        break;

      case UCAN_USB_CMD_DEVID_SET:
        if (channel < CAN_CHANNEL_MAX)
        {
          struct ucan_usb_device_id devid;
          memcpy(&devid, ptr, sizeof(devid));
          pcan_device.can[channel].channel_nr = devid.device_id;
        }
        break;

      case 0x87: /* Vendor ISO mode command: 0xFF enable, 0x55 disable */
        if (channel < CAN_CHANNEL_MAX)
        {
          uint8_t mode_val = (uint8_t)(cmd.args[0] & 0xFFU);
          if (mode_val == 0xFFU)
          {
            pcan_device.can[channel].opt_mask |= UCAN_OPTION_ISO_MODE;
            pcan_can_set_iso_mode(channel, 1U);
          }
          else if (mode_val == 0x55U)
          {
            pcan_device.can[channel].opt_mask &= (uint16_t)(~UCAN_OPTION_ISO_MODE);
            pcan_can_set_iso_mode(channel, 0U);
          }
        }
        break;

      case UCAN_CMD_END_OF_COLLECTION:
        return;

      case UCAN_CMD_RESERVED2:
      case UCAN_CMD_WR_ERR_CNT:
      case UCAN_CMD_SET_ERR_GEN1:
      case UCAN_CMD_SET_ERR_GEN2:
      case UCAN_CMD_DIS_ERR_GEN:
      case UCAN_CMD_RX_BARRIER:
      case UCAN_CMD_SET_ERR_GEN_S:
      default:
        break;
    }

    size = (uint16_t)(size - sizeof(struct ucan_command));
    ptr += sizeof(struct ucan_command);
  }
}

void pcan_protocol_process_data(uint8_t ep, uint8_t *ptr, uint16_t size)
{
  if (ptr == NULL)
  {
    return;
  }

  if (ep == PCAN_USB_EP_CMDOUT)
  {
    pcan_protocol_process_cmd(ptr, size);
    return;
  }

  uint8_t ch;
  if (ep == PCAN_USB_EP_MSGOUT_CH1)
  {
    ch = CAN_BUS_1;
  }
  else if (ep == PCAN_USB_EP_MSGOUT_CH2)
  {
    ch = CAN_BUS_2;
  }
  else
  {
    return;
  }

  if (size > PCAN_TX_REASM_BUFFER_SIZE)
  {
    tx_reasm_len[ch] = 0U;
    return;
  }

  uint8_t *buf = tx_reasm_buf[ch];
  uint16_t len = tx_reasm_len[ch];

  if (((uint32_t)len + (uint32_t)size) > PCAN_TX_REASM_BUFFER_SIZE)
  {
    len = 0U;
  }

  if (size > 0U)
  {
    memcpy(&buf[len], ptr, size);
    len = (uint16_t)(len + size);
  }

  uint16_t pos = 0U;
  uint8_t saw_endmark = 0U;

  while ((uint16_t)(len - pos) >= 4U)
  {
    uint16_t msg_size = 0U;
    uint16_t msg_type = 0U;
    memcpy(&msg_size, &buf[pos], sizeof(msg_size));
    memcpy(&msg_type, &buf[pos + 2U], sizeof(msg_type));

    if ((msg_size == 0U) || (msg_type == 0U) ||
        (msg_type == UCAN_CMD_END_OF_COLLECTION) || (msg_type == 0xFFFFU))
    {
      saw_endmark = 1U;
      break;
    }

    if ((msg_size < 4U) || (msg_size > PCAN_TX_REASM_BUFFER_SIZE))
    {
      saw_endmark = 1U;
      break;
    }

    if ((uint16_t)(len - pos) < msg_size)
    {
      break;
    }

    switch (msg_type)
    {
      case UCAN_MSG_CAN_TX:
        if (msg_size >= sizeof(struct ucan_tx_msg))
        {
          (void)pcan_protocol_tx_frame((const struct ucan_tx_msg *)&buf[pos], msg_size);
        }
        break;

      case UCAN_MSG_CAN_TX_PAUSE:
        break;

      default:
        break;
    }

    pos = (uint16_t)(pos + msg_size);
  }

  if ((saw_endmark != 0U) || (size < pcan_usb_get_mps()))
  {
    tx_reasm_len[ch] = 0U;
  }
  else
  {
    uint16_t rem = (uint16_t)(len - pos);
    if ((rem > 0U) && (pos > 0U))
    {
      memmove(buf, &buf[pos], rem);
    }
    tx_reasm_len[ch] = rem;
  }
}

void pcan_protocol_init(void)
{
  uint32_t uid0 = hpm_get_uid_w0();
  pcan_device.device_nr = ((uid0 == 0U) || (uid0 == 0xFFFFFFFFU)) ? 1U : uid0;
  memset(tx_reasm_len, 0, sizeof(tx_reasm_len));

  (void)pcan_can_init_ex(CAN_BUS_1, 500000U);
  (void)pcan_can_set_filter_mask(CAN_BUS_1, 0, 0, 0U, 0U);
  (void)pcan_can_init_ex(CAN_BUS_2, 500000U);
  (void)pcan_can_set_filter_mask(CAN_BUS_2, 0, 0, 0U, 0U);

  pcan_can_set_iso_mode(CAN_BUS_1, 0U);
  pcan_can_set_iso_mode(CAN_BUS_2, 0U);

  pcan_can_install_rx_callback(CAN_BUS_1, pcan_protocol_rx_frame);
  pcan_can_install_rx_callback(CAN_BUS_2, pcan_protocol_rx_frame);

  pcan_can_install_tx_callback(CAN_BUS_1, pcan_protocol_tx_frame_cb);
  pcan_can_install_tx_callback(CAN_BUS_2, pcan_protocol_tx_frame_cb);

  pcan_can_install_status_callback(CAN_BUS_1, pcan_protocol_send_status);
  pcan_can_install_status_callback(CAN_BUS_2, pcan_protocol_send_status);
}

void pcan_protocol_poll(void)
{
  uint32_t ts_ms = pcan_timestamp_millis();

  pcan_can_poll();
  (void)pcan_protocol_try_flush_data();

  if (pcan_device.can_drv_loaded == 0U)
  {
    return;
  }

  if ((uint32_t)(ts_ms - pcan_device.last_bus_load_update) >= 250U)
  {
    pcan_device.last_bus_load_update = ts_ms;
    for (uint8_t i = 0U; i < CAN_CHANNEL_MAX; i++)
    {
      uint32_t total_ns = pcan_device.can[i].tx_time_ns + pcan_device.can[i].rx_time_ns;
      uint32_t pct = total_ns / (250000000U / 100U);
      pcan_device.can[i].bus_load = (pct > 100U) ? 100U : pct;
      pcan_device.can[i].tx_time_ns = 0U;
      pcan_device.can[i].rx_time_ns = 0U;
    }
  }

  if ((uint32_t)(ts_ms - pcan_device.last_time_sync) < 1000U)
  {
    return;
  }

  struct ucan_usb_ts_msg *pts = pcan_data_alloc_buffer(
      UCAN_USB_MSG_CALIBRATION, (uint16_t)sizeof(struct ucan_usb_ts_msg));
  if (pts == NULL)
  {
    return;
  }

  pts->usb_frame_index = pcan_usb_frame_number();
  pts->unused = 0U;
  pcan_device.last_time_sync = ts_ms;

  for (uint8_t i = 0U; i < CAN_CHANNEL_MAX; i++)
  {
    if (((pcan_device.can[i].opt_mask & UCAN_OPTION_BUSLOAD) == 0U) ||
        (pcan_device.can[i].bus_active == 0U))
    {
      continue;
    }

    struct ucan_bus_load_msg *pbs = pcan_data_alloc_buffer(
        UCAN_MSG_BUSLOAD, (uint16_t)sizeof(struct ucan_bus_load_msg));
    if (pbs == NULL)
    {
      return;
    }

    pbs->channel = i;
    pbs->unused = 0U;
    pbs->bus_load = (uint16_t)((pcan_device.can[i].bus_load * 4095U) / 100U);
  }
}
