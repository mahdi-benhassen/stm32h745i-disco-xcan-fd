#include <string.h>
#include "hpm_usbd_compat.h"
#include "pcanpro_timestamp.h"
#include "pcanpro_led.h"
#include "pcan_usb_device.h"
#include "plin_protocol.h"
#include "plin_lin.h"

enum
{
  PLIN_CMD_INITIALIZE_HW     = 0x00U,
  PLIN_CMD_RST_HW_CONFIG     = 0x01U,
  PLIN_CMD_SET_FRAME_ENTRY   = 0x02U,
  PLIN_CMD_GET_FRAME_ENTRY   = 0x03U,
  PLIN_CMD_START_AUTO_BAUD   = 0x04U,
  PLIN_CMD_GET_BAUDRATE      = 0x05U,
  PLIN_CMD_SET_ID_FILTER     = 0x06U,
  PLIN_CMD_GET_ID_FILTER     = 0x07U,
  PLIN_CMD_GET_MASTER_MODE   = 0x08U,
  PLIN_CMD_HW_INFO           = 0x09U,
  PLIN_CMD_GET_HW_NAME       = 0x0AU,
  PLIN_CMD_IDENTIFY_HW       = 0x0BU,
  PLIN_CMD_GET_FW_VERSION    = 0x0CU,
  PLIN_CMD_START_KEEPALIVE   = 0x0DU,
  PLIN_CMD_RESUME_KEEPALIVE  = 0x0EU,
  PLIN_CMD_SUSPEND_KEEPALIVE = 0x0FU,
  PLIN_CMD_ADD_SLOT          = 0x10U,
  PLIN_CMD_DELETE_SCHEDULE   = 0x11U,
  PLIN_CMD_GET_SLOT_COUNT    = 0x12U,
  PLIN_CMD_GET_SLOT          = 0x13U,
  PLIN_CMD_SET_BREAKPOINT    = 0x14U,
  PLIN_CMD_START_SCHEDULE    = 0x15U,
  PLIN_CMD_RESUME_SCHEDULE   = 0x16U,
  PLIN_CMD_SUSPEND_SCHEDULE  = 0x17U,
  PLIN_CMD_GET_STATUS        = 0x18U,
  PLIN_CMD_RST_HW            = 0x1EU,
  PLIN_CMD_UPDATE_BYTE_ARRAY = 0x1FU,
  PLIN_CMD_XMT_WAKE_UP       = 0x21U,
  PLIN_CMD_GET_SERIAL_NUMBER = 0x23U,
  PLIN_CMD_GET_HARDWARE_TYPE = 0x24U,
  PLIN_CMD_GET_VERSION       = 0x25U,
  PLIN_CMD_GET_TARGET_TIME   = 0x26U,
  PLIN_CMD_RESPONSE_REMAP    = 0x27U,
  PLIN_CMD_HW_CONFIG         = 0x28U,
  PLIN_CMD_BREAK_LEN         = 0x29U,
  PLIN_CMD_TERMINATION       = 0x2AU,

  /* Custom Flash schedule persistence opcodes (stubbed cleanly when Flash store is absent) */
  PLIN_CMD_STORE_SCHEDULE    = 0x30U,
  PLIN_CMD_LOAD_SCHEDULE     = 0x31U,
  PLIN_CMD_GET_SCHEDULE_INFO = 0x32U,
  PLIN_CMD_SELECT_SCHEDULE   = 0x33U,
};

enum
{
  PLIN_MODE_NONE   = 0U,
  PLIN_MODE_SLAVE  = 1U,
  PLIN_MODE_MASTER = 2U,
};

struct plin_frame_entry
{
  uint8_t raw[PLIN_FRAME_ENTRY_SIZE];
};

struct plin_channel
{
  uint8_t  mode;
  uint16_t baudrate;
  uint8_t  filter[8];

  uint8_t  frame_valid[PLIN_MAX_FRAMES];
  struct plin_frame_entry frames[PLIN_MAX_FRAMES];

  uint8_t  slot_count[PLIN_MAX_SCHEDULES];
  uint8_t  active_schedule;
  uint8_t  build_schedule;
  uint8_t  schedule_active;
  uint8_t  slots[PLIN_MAX_SCHEDULES][PLIN_MAX_SLOTS][PLIN_SLOT_SIZE];

  uint8_t  keepalive_enable;
  uint8_t  keepalive_id;
  uint16_t keepalive_interval;
  uint32_t keepalive_next_ms;

  uint8_t  break_length;
  uint8_t  termination;
  uint8_t  user_data[8];
  uint8_t  remap[64];
  uint32_t bp_handle[2];

  uint16_t rx_overrun;
  uint8_t  rx_head;
  uint8_t  rx_tail;
  uint8_t  rx_count;
  uint8_t  rx_queue[PLIN_RX_QUEUE_LEN][PLIN_RX_MSG_SIZE];
};

static struct plin_channel plin_channels[PLIN_CHANNELS];
static uint8_t plin_cmd_pending[PLIN_EP_SIZE];
static uint16_t plin_cmd_pending_len;
static uint8_t plin_device_no = 1U;
static uint8_t plin_client_no;

static void plin_send_raw(const uint8_t *data, uint16_t size)
{
  if ((data == NULL) || (size == 0U))
  {
    return;
  }
  if (size > PLIN_EP_SIZE)
  {
    size = PLIN_EP_SIZE;
  }

  if (pcan_usb_send_lin_ep(PLIN_EP_CMDIN, data, size) == 0)
  {
    memcpy(plin_cmd_pending, data, size);
    plin_cmd_pending_len = size;
  }
}

static void plin_reply(const uint8_t *req, const uint8_t *payload, uint16_t size)
{
  uint8_t rsp[PLIN_EP_SIZE];

  if (size > (uint16_t)(sizeof(rsp) - 4U))
  {
    size = (uint16_t)(sizeof(rsp) - 4U);
  }

  rsp[0] = req[0];
  rsp[1] = 0x01U;
  rsp[2] = req[2];
  rsp[3] = req[3];
  if ((size > 0U) && (payload != NULL))
  {
    memcpy(&rsp[4], payload, size);
  }

  plin_send_raw(rsp, (uint16_t)(4U + size));
}

static void plin_reset_config(struct plin_channel *chan)
{
  chan->mode = PLIN_MODE_NONE;
  chan->baudrate = 0U;
  memset(chan->slot_count, 0, sizeof(chan->slot_count));
  chan->active_schedule = 0U;
  chan->build_schedule = 0U;
  chan->schedule_active = 0U;
  chan->keepalive_enable = 0U;
  chan->keepalive_id = 0U;
  chan->keepalive_interval = 0U;
  chan->break_length = 13U;
  chan->termination = 0U;
  memset(chan->user_data, 0, sizeof(chan->user_data));
  memset(chan->bp_handle, 0, sizeof(chan->bp_handle));
  for (uint8_t i = 0U; i < 64U; i++)
  {
    chan->remap[i] = i;
  }
  memset(chan->filter, 0, sizeof(chan->filter));
  memset(chan->frame_valid, 0, sizeof(chan->frame_valid));
  memset(chan->frames, 0, sizeof(chan->frames));
  memset(chan->slots, 0, sizeof(chan->slots));
}

static void plin_reset_queues(struct plin_channel *chan)
{
  chan->rx_head = 0U;
  chan->rx_tail = 0U;
  chan->rx_count = 0U;
  chan->rx_overrun = 0U;
}

static void plin_kick_rx(uint8_t channel)
{
  if (channel >= PLIN_CHANNELS)
  {
    return;
  }

  struct plin_channel *chan = &plin_channels[channel];
  uint8_t ep = (channel == 0U) ? PLIN_EP_DATA0IN : PLIN_EP_DATA1IN;

  while (chan->rx_count > 0U)
  {
    if (pcan_usb_send_lin_ep(ep, chan->rx_queue[chan->rx_tail], PLIN_RX_MSG_SIZE) == 0)
    {
      break;
    }
    chan->rx_tail = (uint8_t)((chan->rx_tail + 1U) % PLIN_RX_QUEUE_LEN);
    chan->rx_count--;
  }
}

void plin_protocol_init(void)
{
  memset(plin_channels, 0, sizeof(plin_channels));
  plin_cmd_pending_len = 0U;
  plin_device_no = 1U;
  plin_client_no = 0U;

  for (uint8_t ch = 0U; ch < PLIN_CHANNELS; ch++)
  {
    plin_channels[ch].break_length = 13U;
    for (uint8_t i = 0U; i < 64U; i++)
    {
      plin_channels[ch].remap[i] = i;
    }
  }
}

void plin_protocol_poll(void)
{
  uint32_t ts_ms = pcan_timestamp_millis();

  if (plin_cmd_pending_len > 0U)
  {
    if (pcan_usb_send_lin_ep(PLIN_EP_CMDIN, plin_cmd_pending, plin_cmd_pending_len) != 0)
    {
      plin_cmd_pending_len = 0U;
    }
  }

  for (uint8_t ch = 0U; ch < PLIN_CHANNELS; ch++)
  {
    struct plin_channel *chan = &plin_channels[ch];

    if ((chan->keepalive_enable != 0U) && (chan->mode != PLIN_MODE_NONE) &&
        ((int32_t)(ts_ms - chan->keepalive_next_ms) >= 0))
    {
      chan->keepalive_next_ms = ts_ms + chan->keepalive_interval;
      plin_lin_send_wakeup(ch);
    }
  }

  plin_kick_rx(0U);
  plin_kick_rx(1U);
}

void plin_protocol_on_tx_done(uint8_t channel)
{
  if (channel >= PLIN_CHANNELS)
  {
    return;
  }
  plin_kick_rx(channel);
}

void plin_protocol_on_driver_state(uint8_t loaded)
{
  if (loaded == 0U)
  {
    plin_cmd_pending_len = 0U;
    for (uint8_t ch = 0U; ch < PLIN_CHANNELS; ch++)
    {
      plin_channels[ch].schedule_active = 0U;
      plin_channels[ch].keepalive_enable = 0U;
      plin_lin_set_active(ch, 0U);
      plin_reset_queues(&plin_channels[ch]);
    }
  }
}

void plin_protocol_report_msg(uint8_t channel, const uint8_t *msg, uint16_t size)
{
  if ((channel >= PLIN_CHANNELS) || (msg == NULL) || (size == 0U))
  {
    return;
  }

  struct plin_channel *chan = &plin_channels[channel];
  if (size > PLIN_RX_MSG_SIZE)
  {
    size = PLIN_RX_MSG_SIZE;
  }

  if (chan->rx_count >= PLIN_RX_QUEUE_LEN)
  {
    chan->rx_overrun++;
    return;
  }

  memset(chan->rx_queue[chan->rx_head], 0, PLIN_RX_MSG_SIZE);
  memcpy(chan->rx_queue[chan->rx_head], msg, size);
  chan->rx_head = (uint8_t)((chan->rx_head + 1U) % PLIN_RX_QUEUE_LEN);
  chan->rx_count++;
}

void plin_protocol_process_command(const uint8_t *buf, uint16_t size)
{
  if ((buf == NULL) || (size < 4U))
  {
    return;
  }

  uint8_t command = buf[0];
  uint8_t channel = (uint8_t)(buf[2] & 0x01U);
  struct plin_channel *chan = &plin_channels[channel];

  plin_device_no = buf[1];
  plin_client_no = buf[3];

  switch (command)
  {
    case PLIN_CMD_INITIALIZE_HW:
      if (size >= 8U)
      {
        chan->baudrate = (uint16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8U));
        chan->mode = (buf[6] == PLIN_MODE_MASTER) ? PLIN_MODE_MASTER :
                     (buf[6] == PLIN_MODE_SLAVE)  ? PLIN_MODE_SLAVE  :
                                                    PLIN_MODE_NONE;
        plin_lin_set_config(channel, chan->baudrate, chan->mode);
      }
      break;

    case PLIN_CMD_RST_HW_CONFIG:
      plin_reset_config(chan);
      plin_lin_set_config(channel, 19200U, PLIN_MODE_NONE);
      break;

    case PLIN_CMD_SET_FRAME_ENTRY:
      if (size >= (4U + PLIN_FRAME_ENTRY_SIZE))
      {
        uint8_t id = (uint8_t)(buf[4] & 0x3FU);
        memcpy(chan->frames[id].raw, &buf[4], PLIN_FRAME_ENTRY_SIZE);
        chan->frame_valid[id] = 1U;
      }
      break;

    case PLIN_CMD_GET_FRAME_ENTRY:
    {
      uint8_t entry[PLIN_FRAME_ENTRY_SIZE];
      uint8_t id = (size >= 5U) ? (uint8_t)(buf[4] & 0x3FU) : 0U;

      memset(entry, 0, sizeof(entry));
      if (chan->frame_valid[id] != 0U)
      {
        memcpy(entry, chan->frames[id].raw, PLIN_FRAME_ENTRY_SIZE);
      }
      plin_reply(buf, entry, sizeof(entry));
      break;
    }

    case PLIN_CMD_SET_ID_FILTER:
      if (size >= 12U)
      {
        memcpy(chan->filter, &buf[4], sizeof(chan->filter));
      }
      break;

    case PLIN_CMD_GET_ID_FILTER:
      plin_reply(buf, chan->filter, sizeof(chan->filter));
      break;

    case PLIN_CMD_GET_HW_NAME:
    {
      uint8_t name[48] = {0};
      memcpy(name, "HPM5300EVK-LIN", 14U);
      plin_reply(buf, name, sizeof(name));
      break;
    }

    case PLIN_CMD_GET_MASTER_MODE:
    {
      uint8_t mode[4] = {chan->mode, 0U, 0U, 0U};
      plin_reply(buf, mode, sizeof(mode));
      break;
    }

    case PLIN_CMD_IDENTIFY_HW:
      pcan_led_set_mode(LED_CH0_RX, LED_MODE_BLINK_FAST, 1000U);
      pcan_led_set_mode(LED_CH0_TX, LED_MODE_BLINK_FAST, 1000U);
      break;

    case PLIN_CMD_GET_FW_VERSION:
    {
      static const uint8_t version[4] = {1U, 4U, 1U, 0U};
      plin_reply(buf, version, sizeof(version));
      break;
    }

    case PLIN_CMD_START_KEEPALIVE:
      if (size >= 8U)
      {
        chan->keepalive_id = buf[5];
        chan->keepalive_interval = (uint16_t)((uint16_t)buf[6] | ((uint16_t)buf[7] << 8U));
        if (chan->keepalive_interval < 10U)
        {
          chan->keepalive_interval = 10U;
        }
        chan->keepalive_enable = 1U;
        chan->keepalive_next_ms = pcan_timestamp_millis() + chan->keepalive_interval;
      }
      break;

    case PLIN_CMD_RESUME_KEEPALIVE:
      if (chan->keepalive_interval > 0U)
      {
        chan->keepalive_enable = 1U;
        chan->keepalive_next_ms = pcan_timestamp_millis() + chan->keepalive_interval;
      }
      break;

    case PLIN_CMD_SUSPEND_KEEPALIVE:
      chan->keepalive_enable = 0U;
      break;

    case PLIN_CMD_START_AUTO_BAUD:
    {
      uint8_t rsp[8] = {0};
      memcpy(rsp, buf, 4U);
      plin_send_raw(rsp, sizeof(rsp));
      break;
    }

    case PLIN_CMD_HW_INFO:
      if (size >= 32U)
      {
        memcpy(chan->user_data, &buf[24], sizeof(chan->user_data));
      }
      break;

    case PLIN_CMD_SET_BREAKPOINT:
      if (size >= 12U)
      {
        uint8_t bp = (uint8_t)(buf[4] & 0x01U);
        chan->bp_handle[bp] = (uint32_t)buf[8] | ((uint32_t)buf[9] << 8U) |
                              ((uint32_t)buf[10] << 16U) | ((uint32_t)buf[11] << 24U);
      }
      break;

    case PLIN_CMD_RESPONSE_REMAP:
      if ((size >= 72U) && (buf[4] == 1U))
      {
        memcpy(chan->remap, &buf[8], sizeof(chan->remap));
      }
      break;

    case PLIN_CMD_BREAK_LEN:
      if (size >= 6U)
      {
        uint8_t blen = buf[5];
        if (blen < 13U)
        {
          blen = 13U;
        }
        else if (blen > 20U)
        {
          blen = 20U;
        }
        chan->break_length = blen;
      }
      break;

    case PLIN_CMD_TERMINATION:
      if (size >= 5U)
      {
        chan->termination = buf[4];
      }
      break;

    case PLIN_CMD_ADD_SLOT:
      if (size >= 24U)
      {
        uint8_t rsp[32] = {0};
        uint8_t sched = (uint8_t)(buf[4] % PLIN_MAX_SCHEDULES);

        if (chan->slot_count[sched] < PLIN_MAX_SLOTS)
        {
          memcpy(chan->slots[sched][chan->slot_count[sched]], &buf[4], PLIN_SLOT_SIZE);
          chan->slot_count[sched]++;
        }

        memcpy(rsp, buf, 24U);
        plin_send_raw(rsp, sizeof(rsp));
      }
      break;

    case PLIN_CMD_DELETE_SCHEDULE:
      if (size >= 8U)
      {
        uint8_t rsp[20] = {0};
        uint8_t sched = (uint8_t)(buf[4] % PLIN_MAX_SCHEDULES);

        memset(chan->slots[sched], 0, sizeof(chan->slots[sched]));
        chan->slot_count[sched] = 0U;
        if (chan->active_schedule == sched)
        {
          chan->schedule_active = 0U;
          plin_protocol_set_schedule_active(channel, 0U);
        }

        memcpy(rsp, buf, 8U);
        rsp[8] = (uint8_t)(0x52U + sched);
        rsp[9] = 0x34U;
        plin_send_raw(rsp, sizeof(rsp));
      }
      break;

    case PLIN_CMD_GET_SLOT_COUNT:
    {
      uint8_t count[4] = {chan->slot_count[chan->active_schedule % PLIN_MAX_SCHEDULES], 0U, 0U, 0U};
      plin_reply(buf, count, sizeof(count));
      break;
    }

    case PLIN_CMD_GET_SLOT:
    {
      uint8_t slot[PLIN_SLOT_SIZE] = {0};
      uint8_t index = (size >= 5U) ? buf[4] : 0U;

      if (plin_protocol_get_slot(channel, index, slot) == 0)
      {
        memset(slot, 0, sizeof(slot));
      }
      plin_reply(buf, slot, sizeof(slot));
      break;
    }

    case PLIN_CMD_START_SCHEDULE:
    case PLIN_CMD_RESUME_SCHEDULE:
      if (size >= 8U)
      {
        chan->active_schedule = (uint8_t)(buf[4] % PLIN_MAX_SCHEDULES);
        chan->schedule_active = 1U;
        plin_protocol_set_schedule_active(channel, 1U);
        plin_send_raw(buf, 8U);
      }
      break;

    case PLIN_CMD_SUSPEND_SCHEDULE:
      if (size >= 8U)
      {
        uint8_t rsp[12] = {0};
        uint16_t copy_n = MIN(size, (uint16_t)sizeof(rsp));

        if (chan->active_schedule == (uint8_t)(buf[4] % PLIN_MAX_SCHEDULES))
        {
          chan->schedule_active = 0U;
          plin_protocol_set_schedule_active(channel, 0U);
        }
        memcpy(rsp, buf, copy_n);
        plin_send_raw(rsp, sizeof(rsp));
      }
      break;

    case PLIN_CMD_GET_STATUS:
    {
      uint8_t status[20] = {0};

      status[0] = chan->mode;
      status[1] = 16U;
      status[2] = 0x00U;
      status[3] = 0x01U;
      status[4] = (uint8_t)(chan->baudrate & 0xFFU);
      status[5] = (uint8_t)(chan->baudrate >> 8U);
      status[6] = (uint8_t)(chan->rx_overrun & 0xFFU);
      status[7] = (uint8_t)(chan->rx_overrun >> 8U);
      status[16] = (chan->mode != PLIN_MODE_NONE) ? 2U : 0U;
      plin_reply(buf, status, sizeof(status));
      break;
    }

    case PLIN_CMD_RST_HW:
      plin_reset_queues(chan);
      break;

    case PLIN_CMD_UPDATE_BYTE_ARRAY:
      if (size >= 16U)
      {
        uint8_t id = (uint8_t)(buf[4] & 0x3FU);
        uint8_t length = buf[5];
        uint16_t offset = (uint16_t)((uint16_t)buf[6] | ((uint16_t)buf[7] << 8U));

        if ((chan->frame_valid[id] != 0U) && (length <= 8U) && (offset < 8U))
        {
          uint8_t count = length;
          if ((uint16_t)(offset + count) > 8U)
          {
            count = (uint8_t)(8U - offset);
          }
          memcpy(&chan->frames[id].raw[6U + offset], &buf[8], count);
        }
      }
      break;

    case PLIN_CMD_GET_HARDWARE_TYPE:
    {
      uint8_t info[60] = {0};
      uint32_t uid0 = hpm_get_uid_w0();
      if ((uid0 == 0U) || (uid0 == 0xFFFFFFFFU))
      {
        uid0 = 0x12345678U;
      }

      info[0] = 2U;
      info[4] = 0x9FU;
      info[5] = 0xDFU;
      memcpy(&info[20], "HPM5300EVK-LIN", 14U);
      info[44] = (uint8_t)(uid0 & 0xFFU);
      info[45] = (uint8_t)((uid0 >> 8U) & 0xFFU);
      info[46] = (uint8_t)((uid0 >> 16U) & 0xFFU);
      info[47] = (uint8_t)((uid0 >> 24U) & 0xFFU);
      plin_reply(buf, info, sizeof(info));
      break;
    }

    case PLIN_CMD_STORE_SCHEDULE:
      if (size >= 6U)
      {
        uint8_t rsp[8] = {0};
        memcpy(rsp, buf, 6U);
        rsp[6] = 1U; /* Flash store unsupported */
        plin_send_raw(rsp, sizeof(rsp));
      }
      break;

    case PLIN_CMD_LOAD_SCHEDULE:
      if (size >= 5U)
      {
        uint8_t rsp[8] = {0};
        memcpy(rsp, buf, 5U);
        rsp[6] = 1U;
        plin_send_raw(rsp, sizeof(rsp));
      }
      break;

    case PLIN_CMD_GET_SCHEDULE_INFO:
    {
      uint8_t info[8] = {0};
      plin_reply(buf, info, sizeof(info));
      break;
    }

    case PLIN_CMD_SELECT_SCHEDULE:
      if (size >= 8U)
      {
        uint8_t rsp[8] = {0};
        uint8_t index = (uint8_t)(buf[4] % PLIN_MAX_SCHEDULES);
        uint8_t was_active = chan->schedule_active;

        if (was_active != 0U)
        {
          plin_protocol_set_schedule_active(channel, 0U);
        }
        chan->active_schedule = index;
        chan->build_schedule = index;
        if (was_active != 0U)
        {
          plin_protocol_set_schedule_active(channel, 1U);
        }

        memcpy(rsp, buf, 8U);
        rsp[4] = index;
        plin_send_raw(rsp, sizeof(rsp));
      }
      break;

    case PLIN_CMD_XMT_WAKE_UP:
      plin_lin_send_wakeup(channel);
      break;

    case PLIN_CMD_GET_BAUDRATE:
    case PLIN_CMD_GET_SERIAL_NUMBER:
    case PLIN_CMD_GET_VERSION:
    case PLIN_CMD_GET_TARGET_TIME:
    case PLIN_CMD_HW_CONFIG:
    default:
      break;
  }
}

int plin_protocol_get_frame(uint8_t channel, uint8_t id, uint8_t *entry)
{
  if ((channel >= PLIN_CHANNELS) || (id >= PLIN_MAX_FRAMES) || (entry == NULL))
  {
    return 0;
  }
  if (plin_channels[channel].frame_valid[id] == 0U)
  {
    return 0;
  }

  memcpy(entry, plin_channels[channel].frames[id].raw, PLIN_FRAME_ENTRY_SIZE);
  return 1;
}

int plin_protocol_get_slot(uint8_t channel, uint8_t index, uint8_t *slot)
{
  if ((channel >= PLIN_CHANNELS) || (index >= PLIN_MAX_SLOTS) || (slot == NULL))
  {
    return 0;
  }

  const struct plin_channel *chan = &plin_channels[channel];
  uint8_t sched = (uint8_t)(chan->active_schedule % PLIN_MAX_SCHEDULES);
  if (index >= chan->slot_count[sched])
  {
    return 0;
  }

  memcpy(slot, chan->slots[sched][index], PLIN_SLOT_SIZE);
  return 1;
}

uint8_t plin_protocol_get_slot_count(uint8_t channel)
{
  if (channel >= PLIN_CHANNELS)
  {
    return 0U;
  }

  const struct plin_channel *chan = &plin_channels[channel];
  return chan->slot_count[chan->active_schedule % PLIN_MAX_SCHEDULES];
}

void plin_protocol_set_schedule_active(uint8_t channel, uint8_t active)
{
  if (channel >= PLIN_CHANNELS)
  {
    return;
  }

  plin_channels[channel].schedule_active = (active != 0U) ? 1U : 0U;
  plin_lin_set_active(channel, plin_channels[channel].schedule_active);
}

void plin_protocol_set_frame_data(uint8_t channel, uint8_t id,
                                   const uint8_t *data, uint8_t length)
{
  if ((channel >= PLIN_CHANNELS) || (id >= PLIN_MAX_FRAMES) || (data == NULL))
  {
    return;
  }

  struct plin_channel *chan = &plin_channels[channel];
  if (length > 8U)
  {
    length = 8U;
  }

  chan->frames[id].raw[1] = length;
  if (length > 0U)
  {
    memcpy(&chan->frames[id].raw[6], data, length);
  }
  chan->frame_valid[id] = 1U;
}

void plin_protocol_process_data(uint8_t channel, const uint8_t *buf, uint16_t size)
{
  if ((channel >= PLIN_CHANNELS) || (buf == NULL) || (size < 24U))
  {
    return;
  }

  uint16_t rec_size = (((size % 24U) == 0U) && (size != 36U)) ? 24U : 36U;
  uint8_t data_off = (rec_size == 24U) ? 16U : 28U;
  uint16_t off = 0U;

  while ((uint16_t)(off + rec_size) <= size)
  {
    const uint8_t *msg = &buf[off];
    uint8_t pid = msg[8];
    uint8_t length = msg[9];
    uint8_t direction = msg[10];
    uint8_t checksum_type = msg[11];

    plin_device_no = msg[5];
    plin_client_no = msg[7];

    if (length > 8U)
    {
      length = 8U;
    }

    if (direction == 1U)
    {
      plin_protocol_set_frame_data(channel, (uint8_t)(pid & 0x3FU), &msg[data_off], length);
    }

    plin_lin_write(channel, pid, length, direction, checksum_type, &msg[data_off]);
    off = (uint16_t)(off + rec_size);
  }
}

uint8_t plin_protocol_get_device_no(void)
{
  return plin_device_no;
}

uint8_t plin_protocol_get_client_no(void)
{
  return plin_client_no;
}

uint8_t plin_protocol_get_break_len(uint8_t channel)
{
  if (channel >= PLIN_CHANNELS)
  {
    return 13U;
  }
  uint8_t blen = plin_channels[channel].break_length;
  return (blen < 13U) ? 13U : blen;
}
