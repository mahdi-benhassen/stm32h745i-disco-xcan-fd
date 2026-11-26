#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_interrupt.h"
#include "hpm_uart_drv.h"
#include "pcanpro_led.h"
#include "pcanpro_timestamp.h"
#include "plin_lin.h"
#include "plin_protocol.h"

#define LIN_RX_RING_SIZE  128U
#define LIN_RX_RING_MASK  (LIN_RX_RING_SIZE - 1U)
#define LIN_TXQ_LEN       8U
#define LIN_WAKEUP_US     400U

struct lin_tx_request
{
  uint8_t pid;
  uint8_t length;
  uint8_t direction;
  uint8_t checksum_type;
  uint8_t data[8];
};

enum
{
  LIN_PARSE_WAIT_BREAK = 0U,
  LIN_PARSE_WAIT_SYNC,
  LIN_PARSE_WAIT_PID,
  LIN_PARSE_DATA,
  LIN_PARSE_CHECKSUM,
};

enum
{
  LIN_PHASE_IDLE = 0U,
  LIN_PHASE_BREAK_LOW,
  LIN_PHASE_BREAK_DELIM,
  LIN_PHASE_TX,
  LIN_PHASE_TX_SLAVE,
  LIN_PHASE_RX,
};

struct lin_channel
{
  UART_Type *uart;
  uint8_t  is_physical;
  uint8_t  mode;
  uint16_t baudrate;
  uint8_t  active;

  uint8_t  slot_index;
  uint32_t next_slot_ms;
  uint16_t slot_delay;

  volatile uint8_t  rx_ring[LIN_RX_RING_SIZE];
  volatile uint8_t  rx_head;
  volatile uint8_t  rx_tail;
  volatile uint8_t  break_flag;

  /* Non-blocking break & TX tracking */
  uint32_t break_deadline_us;
  uint32_t break_delim_us;
  uint32_t tx_deadline_ms;
  uint8_t  tx_buf[16];
  uint8_t  tx_len;
  uint8_t  tx_echo_skip;

  struct lin_tx_request txq[LIN_TXQ_LEN];
  uint8_t  txq_head;
  uint8_t  txq_tail;

  /* Active frame state */
  uint8_t  phase;
  uint8_t  frame_id;
  uint8_t  frame_pid;
  uint8_t  frame_len;
  uint8_t  frame_dir;
  uint8_t  frame_cs;
  uint8_t  frame_report;
  uint16_t frame_error;
  uint8_t  rx_expected;
  uint8_t  rx_count;
  uint32_t rx_deadline_ms;
  uint8_t  rx_data[8];

  /* Slave parser state */
  uint8_t  parse_state;
  uint8_t  pid;
  uint8_t  rx_len;
  uint8_t  parse_count;
  uint32_t last_rx_ms;

  uint16_t sequence;
};

static struct lin_channel lin_channels[PLIN_CHANNELS] =
{
  [0] = { .uart = BOARD_UART_LIN, .is_physical = 1U, .baudrate = 19200U },
  [1] = { .uart = NULL,           .is_physical = 0U, .baudrate = 19200U },
};

static uint8_t lin_pid(uint8_t id)
{
  uint8_t p0 = (uint8_t)(((id >> 0U) ^ (id >> 1U) ^ (id >> 2U) ^ (id >> 4U)) & 1U);
  uint8_t p1 = (uint8_t)(~((id >> 1U) ^ (id >> 3U) ^ (id >> 4U) ^ (id >> 5U)) & 1U);
  return (uint8_t)(id | (p0 << 6U) | (p1 << 7U));
}

/* LIN 2.x carry-folded modulo-255 inverted checksum */
static uint8_t lin_checksum(uint8_t id, uint8_t pid, uint8_t type,
                            const uint8_t *data, uint8_t length)
{
  uint16_t sum = 0U;
  bool enhanced = (type == 2U) || ((type == 3U) && (id != 0x3CU) && (id != 0x3DU));

  if (enhanced)
  {
    sum = (uint16_t)(sum + pid);
  }

  for (uint8_t i = 0U; i < length; i++)
  {
    sum = (uint16_t)(sum + data[i]);
    if (sum >= 0x100U)
    {
      sum = (uint16_t)(sum - 0xFFU);
    }
  }

  return (uint8_t)(~sum);
}

static void lin_push_rx(struct lin_channel *chan, uint8_t data)
{
  uint8_t next = (uint8_t)((chan->rx_head + 1U) & LIN_RX_RING_MASK);
  if (next != chan->rx_tail)
  {
    chan->rx_ring[chan->rx_head] = data;
    chan->rx_head = next;
  }
}

static int lin_pop_rx(struct lin_channel *chan, uint8_t *data)
{
  if (chan->rx_tail == chan->rx_head)
  {
    return 0;
  }

  *data = chan->rx_ring[chan->rx_tail];
  chan->rx_tail = (uint8_t)((chan->rx_tail + 1U) & LIN_RX_RING_MASK);
  return 1;
}

static void lin_flush_rx(struct lin_channel *chan)
{
  if (chan->uart != NULL)
  {
    uart_clear_rx_fifo(chan->uart);
    (void)uart_get_status(chan->uart);
  }
  chan->rx_head = 0U;
  chan->rx_tail = 0U;
  chan->break_flag = 0U;
}

static void lin_report(struct lin_channel *chan, uint8_t pid, uint8_t length,
                       uint8_t direction, uint8_t checksum_type,
                       uint16_t error_flags, uint8_t checksum,
                       const uint8_t *data)
{
  uint8_t msg[PLIN_RX_MSG_SIZE] = {0};
  uint8_t ch = (uint8_t)(chan - lin_channels);

  if (length > 8U)
  {
    length = 8U;
  }

  msg[0] = 0x02U;
  msg[1] = (uint8_t)(chan->sequence & 0xFFU);
  msg[2] = (uint8_t)(chan->sequence >> 8U);
  chan->sequence++;

  msg[4] = 0x02U;
  msg[5] = plin_protocol_get_device_no();
  msg[6] = ch;
  msg[7] = plin_protocol_get_client_no();

  msg[8]  = pid;
  msg[9]  = length;
  msg[10] = direction;
  msg[11] = checksum_type;
  msg[12] = (uint8_t)(error_flags & 0xFFU);
  msg[13] = (uint8_t)(error_flags >> 8U);
  msg[14] = checksum;

  if ((data != NULL) && (length > 0U))
  {
    memcpy(&msg[16], data, length);
  }

  plin_protocol_report_msg(ch, msg, sizeof(msg));

  if ((error_flags & 0xFFDFU) == 0U)
  {
    pcan_led_set_mode((direction == 1U) ? LED_CH0_TX : LED_CH0_RX, LED_MODE_BLINK_FAST, 100U);
  }
}

static void lin_frame_done(struct lin_channel *chan)
{
  chan->phase = LIN_PHASE_IDLE;
  if ((chan->mode == PLIN_LIN_MODE_MASTER) && (chan->active != 0U))
  {
    chan->next_slot_ms = pcan_timestamp_millis() + chan->slot_delay;
  }
}

/* Non-blocking TX break start (bit-banged dominant pulse on PA15, never blocks CAN/USB polling) */
static void lin_start_break_pulse(struct lin_channel *chan, uint32_t low_us, uint32_t delim_us)
{
  UART_Type *uart = chan->uart;
  if ((chan->is_physical == 0U) || (uart == NULL))
  {
    lin_frame_done(chan);
    return;
  }

  uart_disable_irq(uart, uart_intr_rx_data_avail_or_timeout | uart_intr_rx_line_stat);
  init_uart_pin_as_gpio(uart);
  gpio_set_pin_output(HPM_GPIO0, BOARD_UART_LIN_TX_PORT, BOARD_UART_LIN_TX_PIN);
  gpio_write_pin(HPM_GPIO0, BOARD_UART_LIN_TX_PORT, BOARD_UART_LIN_TX_PIN, 0U);

  chan->break_delim_us = delim_us;
  chan->break_deadline_us = pcan_timestamp_us() + low_us;
  chan->phase = LIN_PHASE_BREAK_LOW;
}

static void lin_poll_break_and_tx(struct lin_channel *chan)
{
  UART_Type *uart = chan->uart;
  uint32_t now_us = pcan_timestamp_us();
  uint32_t now_ms = pcan_timestamp_millis();

  if (chan->phase == LIN_PHASE_BREAK_LOW)
  {
    if ((int32_t)(now_us - chan->break_deadline_us) >= 0)
    {
      gpio_write_pin(HPM_GPIO0, BOARD_UART_LIN_TX_PORT, BOARD_UART_LIN_TX_PIN, 1U);
      chan->break_deadline_us = now_us + chan->break_delim_us;
      chan->phase = LIN_PHASE_BREAK_DELIM;
    }
    return;
  }

  if (chan->phase == LIN_PHASE_BREAK_DELIM)
  {
    if ((int32_t)(now_us - chan->break_deadline_us) >= 0)
    {
      init_uart_pins(uart);
      uart_reset_rx_fifo(uart);
      lin_flush_rx(chan);

      if (chan->mode == PLIN_LIN_MODE_SLAVE)
      {
        uart_enable_irq(uart, uart_intr_rx_data_avail_or_timeout | uart_intr_rx_line_stat);
      }

      if (chan->tx_len == 0U)
      {
        /* Wakeup pulse only */
        lin_frame_done(chan);
        return;
      }

      if (chan->rx_expected > 0U)
      {
        uart_enable_irq(uart, uart_intr_rx_data_avail_or_timeout | uart_intr_rx_line_stat);
      }

      for (uint8_t i = 0U; i < chan->tx_len; i++)
      {
        uart_write_byte(uart, chan->tx_buf[i]);
      }

      chan->tx_echo_skip = chan->tx_len;
      chan->tx_deadline_ms = now_ms + 30U;
      chan->phase = LIN_PHASE_TX;
    }
    return;
  }

  if (chan->phase == LIN_PHASE_TX)
  {
    bool tx_empty = uart_check_status(uart, uart_stat_transmitter_empty);
    bool tx_timeout = ((int32_t)(now_ms - chan->tx_deadline_ms) >= 0);

    if (!tx_empty && !tx_timeout)
    {
      return;
    }

    if (chan->frame_report != 0U)
    {
      lin_flush_rx(chan);
      uint8_t len = chan->frame_len;
      const uint8_t *data = ((len > 0U) && (chan->frame_error == 0U)) ? &chan->tx_buf[2] : NULL;
      uint8_t cs = ((len > 0U) && (chan->frame_error == 0U)) ? chan->tx_buf[2U + len] : 0U;

      lin_report(chan, chan->frame_pid, len, chan->frame_dir,
                 chan->frame_cs, chan->frame_error, cs, data);
      lin_frame_done(chan);
    }
    else if (chan->rx_expected > 0U)
    {
      uint16_t baud = (chan->baudrate > 0U) ? chan->baudrate : 19200U;
      uint32_t frame_ms = 6U + ((uint32_t)(chan->frame_len + 2U) * 12000U) / baud;
      chan->phase = LIN_PHASE_RX;
      chan->rx_count = 0U;
      chan->rx_deadline_ms = now_ms + frame_ms;
    }
    else
    {
      lin_frame_done(chan);
    }
    return;
  }

  if (chan->phase == LIN_PHASE_TX_SLAVE)
  {
    bool tx_empty = uart_check_status(uart, uart_stat_transmitter_empty);
    bool tx_timeout = ((int32_t)(now_ms - chan->tx_deadline_ms) >= 0);

    if (!tx_empty && !tx_timeout)
    {
      return;
    }

    lin_report(chan, chan->frame_pid, chan->frame_len, 1U,
               chan->frame_cs, chan->frame_error,
               chan->tx_buf[chan->frame_len], chan->tx_buf);
    lin_flush_rx(chan);
    chan->parse_state = LIN_PARSE_WAIT_BREAK;
    lin_frame_done(chan);
  }
}

static void lin_master_write(struct lin_channel *chan)
{
  const struct lin_tx_request *req = &chan->txq[chan->txq_tail];
  uint8_t id = (uint8_t)(req->pid & 0x3FU);
  uint8_t length = (req->length > 8U) ? 8U : req->length;
  uint8_t channel = (uint8_t)(chan - lin_channels);
  uint16_t baud = (chan->baudrate > 0U) ? chan->baudrate : 19200U;
  uint32_t bit_us = 1000000U / baud;
  uint32_t break_us = (uint32_t)plin_protocol_get_break_len(channel) * bit_us;

  chan->txq_tail = (uint8_t)((chan->txq_tail + 1U) % LIN_TXQ_LEN);

  chan->frame_id = id;
  chan->frame_pid = req->pid;
  chan->frame_len = length;
  chan->frame_dir = req->direction;
  chan->frame_cs = req->checksum_type;
  chan->frame_error = 0U;
  chan->frame_report = 0U;
  chan->rx_expected = 0U;
  chan->rx_count = 0U;

  chan->tx_buf[0] = 0x55U;
  chan->tx_buf[1] = req->pid;

  if (req->direction == 1U)
  {
    uint8_t cs = lin_checksum(id, req->pid, req->checksum_type, req->data, length);
    if (length > 0U)
    {
      memcpy(&chan->tx_buf[2], req->data, length);
    }
    chan->tx_buf[2U + length] = cs;
    chan->tx_len = (uint8_t)(3U + length);
    chan->frame_report = 1U;
  }
  else
  {
    chan->tx_len = 2U;
    chan->rx_expected = (uint8_t)(length + 1U);
  }

  lin_start_break_pulse(chan, break_us, (bit_us < 50U) ? 50U : bit_us);
}

static void lin_master_start(struct lin_channel *chan)
{
  uint8_t channel = (uint8_t)(chan - lin_channels);
  uint8_t count = plin_protocol_get_slot_count(channel);
  uint8_t slot[PLIN_SLOT_SIZE];
  uint8_t entry[PLIN_FRAME_ENTRY_SIZE];

  if (count == 0U)
  {
    chan->active = 0U;
    lin_flush_rx(chan);
    return;
  }

  if (plin_protocol_get_slot(channel, chan->slot_index, slot) == 0)
  {
    chan->slot_index = 0U;
    return;
  }

  uint8_t id = (uint8_t)(slot[8] & 0x3FU);
  uint16_t delay = (uint16_t)((uint16_t)slot[6] | ((uint16_t)slot[7] << 8U));
  if (delay < 4U)
  {
    delay = 4U;
  }

  chan->slot_delay = delay;
  chan->slot_index = (uint8_t)((chan->slot_index + 1U) % count);

  if (slot[4] != 0U)
  {
    chan->next_slot_ms = pcan_timestamp_millis() + delay;
    return;
  }

  if (plin_protocol_get_frame(channel, id, entry) == 0)
  {
    chan->next_slot_ms = pcan_timestamp_millis() + delay;
    return;
  }

  uint8_t length = (entry[1] > 8U) ? 8U : entry[1];
  uint8_t direction = entry[2];
  uint8_t checksum_type = entry[3];
  uint16_t flags = (uint16_t)((uint16_t)entry[4] | ((uint16_t)entry[5] << 8U));

  if (direction == 0U)
  {
    chan->next_slot_ms = pcan_timestamp_millis() + delay;
    return;
  }

  uint8_t pid = lin_pid(id);
  uint16_t baud = (chan->baudrate > 0U) ? chan->baudrate : 19200U;
  uint32_t bit_us = 1000000U / baud;
  uint32_t break_us = (uint32_t)plin_protocol_get_break_len(channel) * bit_us;

  chan->frame_id = id;
  chan->frame_pid = pid;
  chan->frame_len = length;
  chan->frame_dir = direction;
  chan->frame_cs = checksum_type;
  chan->frame_error = 0U;
  chan->frame_report = 0U;
  chan->rx_expected = 0U;
  chan->rx_count = 0U;

  chan->tx_buf[0] = 0x55U;
  chan->tx_buf[1] = pid;

  if ((direction == 1U) && ((flags & 0x0001U) != 0U))
  {
    uint8_t data[8] = {0};
    if (length > 0U)
    {
      memcpy(data, &entry[6], length);
      memcpy(&chan->tx_buf[2], data, length);
    }
    chan->tx_buf[2U + length] = lin_checksum(id, pid, checksum_type, data, length);
    chan->tx_len = (uint8_t)(3U + length);
    chan->frame_report = 1U;
  }
  else if (direction == 1U)
  {
    chan->tx_len = 2U;
    chan->frame_report = 1U;
    chan->frame_len = 0U;
    chan->frame_error = 0x0008U;
  }
  else
  {
    chan->tx_len = 2U;
    chan->rx_expected = (uint8_t)(length + 1U);
  }

  lin_start_break_pulse(chan, break_us, (bit_us < 50U) ? 50U : bit_us);
}

static void lin_master_poll(struct lin_channel *chan)
{
  uint8_t byte = 0U;

  if (chan->phase == LIN_PHASE_RX)
  {
    while ((chan->tx_echo_skip > 0U) && (lin_pop_rx(chan, &byte) != 0))
    {
      chan->tx_echo_skip--;
    }

    while ((chan->tx_echo_skip == 0U) &&
           (chan->rx_count < chan->rx_expected) &&
           (lin_pop_rx(chan, &byte) != 0))
    {
      if (chan->rx_count < 8U)
      {
        chan->rx_data[chan->rx_count] = byte;
      }
      else
      {
        /* 9th byte for length == 8 (checksum) stored via checksum var below */
      }
      if (chan->rx_count == (uint8_t)(chan->rx_expected - 1U))
      {
        uint8_t expect = lin_checksum(chan->frame_id, chan->frame_pid, chan->frame_cs,
                                      chan->rx_data, chan->frame_len);
        lin_report(chan, chan->frame_pid, chan->frame_len, chan->frame_dir,
                   chan->frame_cs, (expect == byte) ? 0U : 0x0020U, byte,
                   chan->rx_data);
        lin_frame_done(chan);
        return;
      }
      chan->rx_count++;
    }

    if ((int32_t)(pcan_timestamp_millis() - chan->rx_deadline_ms) >= 0)
    {
      lin_report(chan, chan->frame_pid, 0U, chan->frame_dir, chan->frame_cs,
                 0x0008U, 0U, NULL);
      lin_frame_done(chan);
    }
    return;
  }

  if (chan->phase != LIN_PHASE_IDLE)
  {
    return;
  }

  if (chan->txq_head != chan->txq_tail)
  {
    lin_master_write(chan);
    return;
  }

  if (chan->active == 0U)
  {
    return;
  }

  if ((int32_t)(pcan_timestamp_millis() - chan->next_slot_ms) < 0)
  {
    return;
  }

  lin_master_start(chan);
}

static void lin_slave_start_response(struct lin_channel *chan, uint8_t id,
                                     const uint8_t *entry)
{
  uint8_t length = (entry[1] > 8U) ? 8U : entry[1];
  uint8_t checksum_type = entry[3];
  uint8_t pid = lin_pid(id);
  uint8_t cs;

  if (length > 0U)
  {
    memcpy(chan->tx_buf, &entry[6], length);
  }
  cs = lin_checksum(id, pid, checksum_type, chan->tx_buf, length);
  chan->tx_buf[length] = cs;

  chan->frame_id = id;
  chan->frame_pid = pid;
  chan->frame_len = length;
  chan->frame_dir = 1U;
  chan->frame_cs = checksum_type;
  chan->frame_error = 0U;
  chan->tx_len = (uint8_t)(length + 1U);

  for (uint8_t i = 0U; i < chan->tx_len; i++)
  {
    uart_write_byte(chan->uart, chan->tx_buf[i]);
  }
  chan->tx_deadline_ms = pcan_timestamp_millis() + 30U;
  chan->phase = LIN_PHASE_TX_SLAVE;
}

static void lin_slave_poll(struct lin_channel *chan)
{
  uint8_t channel = (uint8_t)(chan - lin_channels);
  uint8_t byte = 0U;

  if (chan->phase != LIN_PHASE_IDLE)
  {
    return;
  }

  if (chan->break_flag != 0U)
  {
    chan->break_flag = 0U;
    chan->parse_state = LIN_PARSE_WAIT_SYNC;
    chan->parse_count = 0U;
  }

  while (lin_pop_rx(chan, &byte) != 0)
  {
    chan->last_rx_ms = pcan_timestamp_millis();

    switch (chan->parse_state)
    {
      case LIN_PARSE_WAIT_SYNC:
        chan->parse_state = (byte == 0x55U) ? LIN_PARSE_WAIT_PID : LIN_PARSE_WAIT_BREAK;
        break;

      case LIN_PARSE_WAIT_PID:
      {
        uint8_t id = (uint8_t)(byte & 0x3FU);
        uint8_t entry[PLIN_FRAME_ENTRY_SIZE];

        chan->pid = byte;
        chan->parse_state = LIN_PARSE_WAIT_BREAK;
        chan->parse_count = 0U;

        if (byte != lin_pid(id))
        {
          break;
        }

        if (plin_protocol_get_frame(channel, id, entry) != 0)
        {
          uint8_t length = (entry[1] > 8U) ? 8U : entry[1];
          uint16_t flags = (uint16_t)((uint16_t)entry[4] | ((uint16_t)entry[5] << 8U));

          if (entry[2] == 1U)
          {
            if ((flags & 0x0001U) != 0U)
            {
              lin_slave_start_response(chan, id, entry);
              return;
            }
          }
          else
          {
            chan->rx_len = length;
            chan->parse_state = (length > 0U) ? LIN_PARSE_DATA : LIN_PARSE_CHECKSUM;
          }
        }
        break;
      }

      case LIN_PARSE_DATA:
        chan->rx_data[chan->parse_count++] = byte;
        if (chan->parse_count >= chan->rx_len)
        {
          chan->parse_state = LIN_PARSE_CHECKSUM;
        }
        break;

      case LIN_PARSE_CHECKSUM:
      {
        uint8_t id = (uint8_t)(chan->pid & 0x3FU);
        uint8_t entry[PLIN_FRAME_ENTRY_SIZE];

        if (plin_protocol_get_frame(channel, id, entry) != 0)
        {
          uint8_t expect = lin_checksum(id, chan->pid, entry[3],
                                        chan->rx_data, chan->rx_len);
          lin_report(chan, chan->pid, chan->rx_len, entry[2], entry[3],
                     (expect == byte) ? 0U : 0x0020U, byte,
                     chan->rx_data);
        }
        chan->parse_state = LIN_PARSE_WAIT_BREAK;
        break;
      }

      default:
        chan->parse_state = LIN_PARSE_WAIT_BREAK;
        break;
    }
  }

  if ((chan->parse_state != LIN_PARSE_WAIT_BREAK) &&
      ((uint32_t)(pcan_timestamp_millis() - chan->last_rx_ms) > 20U))
  {
    chan->parse_state = LIN_PARSE_WAIT_BREAK;
  }
}

static void lin_uart_configure(struct lin_channel *chan, uint16_t baudrate)
{
  UART_Type *uart = chan->uart;
  if ((chan->is_physical == 0U) || (uart == NULL))
  {
    return;
  }

  uart_disable_irq(uart, uart_intr_rx_data_avail_or_timeout |
                         uart_intr_rx_line_stat |
                         uart_intr_tx_slot_avail);

  uart_config_t cfg = {0};
  uart_default_config(uart, &cfg);
  cfg.src_freq_in_hz = clock_get_frequency(BOARD_UART_LIN_CLK_NAME);
  cfg.baudrate = baudrate;
  cfg.rx_fifo_level = uart_rx_fifo_trg_not_empty;
  cfg.tx_fifo_level = uart_tx_fifo_trg_not_full;
  (void)uart_init(uart, &cfg);

  init_uart_pins(uart);
  uart_reset_all_fifo(uart);
  lin_flush_rx(chan);

  if (chan->mode != PLIN_LIN_MODE_NONE)
  {
    uart_enable_irq(uart, uart_intr_rx_data_avail_or_timeout | uart_intr_rx_line_stat);
  }
}

SDK_DECLARE_EXT_ISR_M(BOARD_UART_LIN_IRQ, plin_uart3_isr)
void plin_uart3_isr(void)
{
  struct lin_channel *chan = &lin_channels[0];
  UART_Type *uart = chan->uart;

  for (uint8_t iter = 0U; iter < 32U; iter++)
  {
    uint8_t irq_id = uart_get_irq_id(uart);
    if ((irq_id & 0x01U) != 0U)
    {
      break;
    }

    uint32_t lsr = uart_get_status(uart);
    if ((lsr & uart_stat_line_break) != 0U)
    {
      chan->break_flag = 1U;
      if ((lsr & uart_stat_data_ready) != 0U)
      {
        (void)uart_read_byte(uart); /* discard break 0x00 byte */
        lsr = uart_get_status(uart);
      }
      if (chan->mode == PLIN_LIN_MODE_SLAVE)
      {
        chan->rx_head = 0U;
        chan->rx_tail = 0U;
      }
    }

    while ((lsr & uart_stat_data_ready) != 0U)
    {
      uint8_t byte = uart_read_byte(uart);
      lin_push_rx(chan, byte);
      lsr = uart_get_status(uart);
      if ((lsr & uart_stat_line_break) != 0U)
      {
        chan->break_flag = 1U;
      }
    }
  }
}

void plin_lin_init(void)
{
  board_init_uart(BOARD_UART_LIN);
#if defined(BOARD_SUPPORT_LIN_TRANSCEIVER_CONTROL) && (BOARD_SUPPORT_LIN_TRANSCEIVER_CONTROL == 1)
  board_lin_transceiver_control(false);
#endif

  intc_m_enable_irq_with_priority(BOARD_UART_LIN_IRQ, 1U);

  for (uint8_t i = 0U; i < PLIN_CHANNELS; i++)
  {
    struct lin_channel *chan = &lin_channels[i];
    chan->mode = PLIN_LIN_MODE_NONE;
    chan->baudrate = 19200U;
    chan->active = 0U;
    chan->phase = LIN_PHASE_IDLE;
    chan->slot_index = 0U;
    chan->next_slot_ms = 0U;
    chan->parse_state = LIN_PARSE_WAIT_BREAK;
    chan->sequence = 0U;
    chan->txq_head = 0U;
    chan->txq_tail = 0U;
    lin_uart_configure(chan, 19200U);
  }
}

void plin_lin_set_config(uint8_t channel, uint16_t baudrate, uint8_t mode)
{
  if (channel >= PLIN_CHANNELS)
  {
    return;
  }

  struct lin_channel *chan = &lin_channels[channel];
  if ((baudrate < 1000U) || (baudrate > 20000U))
  {
    baudrate = 19200U;
  }

  chan->baudrate = baudrate;
  chan->mode = mode;
  chan->active = 0U;
  chan->phase = LIN_PHASE_IDLE;
  chan->sequence = 0U;
  chan->parse_state = LIN_PARSE_WAIT_BREAK;
  chan->txq_head = 0U;
  chan->txq_tail = 0U;

  if (chan->is_physical != 0U)
  {
#if defined(BOARD_SUPPORT_LIN_TRANSCEIVER_CONTROL) && (BOARD_SUPPORT_LIN_TRANSCEIVER_CONTROL == 1)
    board_lin_transceiver_control(mode != PLIN_LIN_MODE_NONE);
#endif
    lin_uart_configure(chan, baudrate);
  }
}

void plin_lin_set_active(uint8_t channel, uint8_t active)
{
  if (channel >= PLIN_CHANNELS)
  {
    return;
  }

  struct lin_channel *chan = &lin_channels[channel];
  chan->active = (active != 0U) ? 1U : 0U;
  chan->phase = LIN_PHASE_IDLE;
  chan->slot_index = 0U;
  chan->next_slot_ms = pcan_timestamp_millis();
  if (chan->is_physical != 0U)
  {
    init_uart_pins(chan->uart);
    lin_flush_rx(chan);
  }
}

void plin_lin_send_wakeup(uint8_t channel)
{
  if (channel >= PLIN_CHANNELS)
  {
    return;
  }

  struct lin_channel *chan = &lin_channels[channel];
  if ((chan->is_physical == 0U) || (chan->mode == PLIN_LIN_MODE_NONE) ||
      (chan->phase != LIN_PHASE_IDLE))
  {
    return;
  }

  chan->tx_len = 0U;
  chan->frame_report = 0U;
  chan->rx_expected = 0U;
  lin_start_break_pulse(chan, LIN_WAKEUP_US, 100U);
}

void plin_lin_write(uint8_t channel, uint8_t pid, uint8_t length, uint8_t direction,
                    uint8_t checksum_type, const uint8_t *data)
{
  if ((channel >= PLIN_CHANNELS) || (data == NULL))
  {
    return;
  }

  struct lin_channel *chan = &lin_channels[channel];
  if ((chan->is_physical == 0U) || (chan->mode != PLIN_LIN_MODE_MASTER))
  {
    return;
  }

  uint8_t next = (uint8_t)((chan->txq_head + 1U) % LIN_TXQ_LEN);
  if (next == chan->txq_tail)
  {
    chan->txq_tail = (uint8_t)((chan->txq_tail + 1U) % LIN_TXQ_LEN);
  }

  struct lin_tx_request *req = &chan->txq[chan->txq_head];
  req->pid = pid;
  req->length = (length > 8U) ? 8U : length;
  req->direction = direction;
  req->checksum_type = checksum_type;
  if (req->length > 0U)
  {
    memcpy(req->data, data, req->length);
  }
  chan->txq_head = next;
}

void plin_lin_poll(void)
{
  struct lin_channel *chan = &lin_channels[0];
  if ((chan->is_physical == 0U) || (chan->mode == PLIN_LIN_MODE_NONE))
  {
    return;
  }

  if ((chan->phase == LIN_PHASE_BREAK_LOW) ||
      (chan->phase == LIN_PHASE_BREAK_DELIM) ||
      (chan->phase == LIN_PHASE_TX) ||
      (chan->phase == LIN_PHASE_TX_SLAVE))
  {
    lin_poll_break_and_tx(chan);
  }

  if (chan->mode == PLIN_LIN_MODE_MASTER)
  {
    if ((chan->active != 0U) || (chan->txq_head != chan->txq_tail) ||
        (chan->phase != LIN_PHASE_IDLE))
    {
      lin_master_poll(chan);
    }
    else if (chan->phase == LIN_PHASE_IDLE)
    {
      lin_flush_rx(chan);
    }
  }
  else if (chan->mode == PLIN_LIN_MODE_SLAVE)
  {
    lin_slave_poll(chan);
  }
}
