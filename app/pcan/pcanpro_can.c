#include <string.h>
#include "board.h"
#include "hpm_soc.h"
#include "hpm_clock_drv.h"
#include "hpm_mcan_drv.h"
#include "hpm_mcan_regs.h"
#include "pcanpro_timestamp.h"
#include "pcanpro_can.h"

#define CAN_TX_FIFO_SIZE  (256U)
#define CAN_RX_POLL_BATCH (32U)

struct t_can_dev
{
  MCAN_Type *mcan;
  uint8_t    is_physical;
  uint8_t    bus_active;
  uint8_t    silent_mode;
  uint8_t    loopback_mode;
  uint8_t    iso_mode;
  uint8_t    force_20ab;
  uint8_t    last_status;
  uint32_t   clk_hz;

  uint32_t   tx_msgs;
  uint32_t   tx_errs;
  uint32_t   tx_ovfs;
  uint32_t   rx_msgs;
  uint32_t   rx_errs;
  uint32_t   rx_ovfs;

  struct t_can_msg tx_fifo[CAN_TX_FIFO_SIZE];
  uint32_t   tx_head;
  uint32_t   tx_tail;

  struct t_can_bitrate nom_timing;
  struct t_can_bitrate data_timing;

  int  (*rx_isr)(uint8_t bus, struct t_can_msg *pmsg);
  int  (*tx_isr)(uint8_t bus, struct t_can_msg *pmsg);
  void (*err_handler)(int bus, uint32_t status);
  void (*status_cb)(uint8_t bus, uint8_t status);
};

#if defined(MCAN_SOC_MSG_BUF_IN_AHB_RAM) && (MCAN_SOC_MSG_BUF_IN_AHB_RAM == 1)
ATTR_PLACE_AT(".ahb_sram") static uint32_t s_mcan_msg_buf[MCAN_MSG_BUF_SIZE_IN_WORDS];
#endif

static struct t_can_dev can_dev_array[CAN_BUS_TOTAL] =
{
  [CAN_BUS_1] = {
    .mcan = BOARD_APP_CAN_BASE,
    .is_physical = 1U,
    .iso_mode = 1U,
    .nom_timing = { .brp = 1U, .tseg1 = 63U, .tseg2 = 16U, .sjw = 16U },   /* 1 Mbit/s @ 80 MHz */
    .data_timing = { .brp = 1U, .tseg1 = 15U, .tseg2 = 4U, .sjw = 4U },     /* 4 Mbit/s @ 80 MHz */
  },
  [CAN_BUS_2] = {
    .mcan = NULL,
    .is_physical = 0U,
    .iso_mode = 1U,
    .nom_timing = { .brp = 1U, .tseg1 = 63U, .tseg2 = 16U, .sjw = 16U },
    .data_timing = { .brp = 1U, .tseg1 = 15U, .tseg2 = 4U, .sjw = 4U },
  },
};

static uint32_t clamp_u32(uint32_t val, uint32_t min_v, uint32_t max_v)
{
  if (val < min_v)
  {
    return min_v;
  }
  if (val > max_v)
  {
    return max_v;
  }
  return val;
}

static uint8_t len_to_dlc(uint8_t len)
{
  if (len <= 8U)
  {
    return len;
  }
  if (len <= 12U)
  {
    return 9U;
  }
  if (len <= 16U)
  {
    return 10U;
  }
  if (len <= 20U)
  {
    return 11U;
  }
  if (len <= 24U)
  {
    return 12U;
  }
  if (len <= 32U)
  {
    return 13U;
  }
  if (len <= 48U)
  {
    return 14U;
  }
  return 15U;
}

static uint8_t dlc_to_len(uint8_t dlc)
{
  static const uint8_t dlc_bytes[16] =
  {
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
    8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U
  };
  return dlc_bytes[dlc & 0x0FU];
}

static void get_precalculated_nominal(uint32_t bitrate, uint32_t *brp,
                                      uint32_t *tseg1, uint32_t *tseg2, uint32_t *sjw)
{
  switch (bitrate)
  {
    case 1000000U: *brp = 1U; *tseg1 = 63U; *tseg2 = 16U; *sjw = 16U; break;
    case 800000U:  *brp = 1U; *tseg1 = 79U; *tseg2 = 20U; *sjw = 20U; break;
    default:
    case 500000U:  *brp = 2U; *tseg1 = 63U; *tseg2 = 16U; *sjw = 16U; break;
    case 250000U:  *brp = 4U; *tseg1 = 63U; *tseg2 = 16U; *sjw = 16U; break;
    case 125000U:  *brp = 8U; *tseg1 = 63U; *tseg2 = 16U; *sjw = 16U; break;
    case 100000U:  *brp = 10U; *tseg1 = 63U; *tseg2 = 16U; *sjw = 16U; break;
    case 83333U:   *brp = 60U; *tseg1 = 13U; *tseg2 = 2U; *sjw = 2U; break;
    case 50000U:   *brp = 100U; *tseg1 = 13U; *tseg2 = 2U; *sjw = 2U; break;
    case 20000U:   *brp = 250U; *tseg1 = 13U; *tseg2 = 2U; *sjw = 2U; break;
    case 10000U:   *brp = 500U; *tseg1 = 13U; *tseg2 = 2U; *sjw = 2U; break;
  }
}

static void get_precalculated_data(uint32_t bitrate, uint32_t *brp,
                                   uint32_t *tseg1, uint32_t *tseg2, uint32_t *sjw)
{
  switch (bitrate)
  {
    case 5000000U: *brp = 1U; *tseg1 = 11U; *tseg2 = 4U; *sjw = 4U; break;
    case 4000000U: *brp = 1U; *tseg1 = 15U; *tseg2 = 4U; *sjw = 4U; break;
    default:
    case 2000000U: *brp = 2U; *tseg1 = 15U; *tseg2 = 4U; *sjw = 4U; break;
    case 1000000U: *brp = 4U; *tseg1 = 15U; *tseg2 = 4U; *sjw = 4U; break;
    case 500000U:  *brp = 8U; *tseg1 = 15U; *tseg2 = 4U; *sjw = 4U; break;
  }
}

static uint32_t mcan3_hw_init_once(MCAN_Type *base)
{
  static uint8_t done = 0U;
  static uint32_t cached_clk = 0U;

  if (done != 0U)
  {
    return cached_clk;
  }
  done = 1U;

#if defined(MCAN_SOC_MSG_BUF_IN_AHB_RAM) && (MCAN_SOC_MSG_BUF_IN_AHB_RAM == 1)
  mcan_msg_buf_attr_t attr = {
    .ram_base = (uint32_t)&s_mcan_msg_buf[0],
    .ram_size = sizeof(s_mcan_msg_buf),
  };
  (void)mcan_set_msg_buf_attr(base, &attr);
#endif

  init_can_pins(base);
  cached_clk = board_init_can_clock(base);

  /* Ensure 80 MHz exact clock on MCAN3 for PCAN bit-timing parity */
  uint32_t pll1_hz = clock_get_frequency(clk_pll1clk0);
  if ((pll1_hz >= PCAN_FDCAN_CLOCK_HZ) && ((pll1_hz % PCAN_FDCAN_CLOCK_HZ) == 0U))
  {
    uint32_t div = pll1_hz / PCAN_FDCAN_CLOCK_HZ;
    if ((div >= 1U) && (div <= 16U))
    {
      clock_add_to_group(clock_can3, 0);
      clock_set_source_divider(clock_can3, clk_src_pll1_clk0, div);
      cached_clk = clock_get_frequency(clock_can3);
    }
  }

  return cached_clk;
}

static void can_apply_config(struct t_can_dev *dev)
{
  if ((dev->is_physical == 0U) || (dev->mcan == NULL))
  {
    return;
  }

  MCAN_Type *base = dev->mcan;
  dev->clk_hz = mcan3_hw_init_once(base);

  base->CCCR |= MCAN_CCCR_INIT_MASK;
  while ((base->CCCR & MCAN_CCCR_INIT_MASK) == 0U)
  {
  }

  if (dev->bus_active == 0U)
  {
    return;
  }

  mcan_config_t cfg;
  mcan_get_default_config(base, &cfg);

  cfg.enable_canfd = (dev->force_20ab == 0U);
  cfg.enable_non_iso_mode = (dev->iso_mode == 0U);
  cfg.disable_auto_retransmission = false;

  if (dev->loopback_mode != 0U)
  {
    cfg.mode = mcan_mode_loopback_internal;
  }
  else if (dev->silent_mode != 0U)
  {
    cfg.mode = mcan_mode_listen_only;
  }
  else
  {
    cfg.mode = mcan_mode_normal;
  }

  cfg.use_lowlevel_timing_setting = true;
  cfg.can_timing.prescaler = (uint16_t)clamp_u32(dev->nom_timing.brp, 1U, 512U);
  cfg.can_timing.num_seg1 = (uint16_t)clamp_u32(dev->nom_timing.tseg1, 1U, 256U);
  cfg.can_timing.num_seg2 = (uint8_t)clamp_u32(dev->nom_timing.tseg2, 1U, 128U);
  cfg.can_timing.num_sjw = (uint8_t)clamp_u32(dev->nom_timing.sjw, 1U, cfg.can_timing.num_seg2);
  cfg.can_timing.enable_tdc = false;

  uint32_t dbrp = clamp_u32(dev->data_timing.brp, 1U, 32U);
  uint32_t dtseg1 = clamp_u32(dev->data_timing.tseg1, 1U, 32U);
  uint32_t dtseg2 = clamp_u32(dev->data_timing.tseg2, 1U, 16U);
  uint32_t dsjw = clamp_u32(dev->data_timing.sjw, 1U, dtseg2);

  cfg.canfd_timing.prescaler = (uint16_t)dbrp;
  cfg.canfd_timing.num_seg1 = (uint16_t)dtseg1;
  cfg.canfd_timing.num_seg2 = (uint8_t)dtseg2;
  cfg.canfd_timing.num_sjw = (uint8_t)dsjw;

  if (cfg.enable_canfd && (dbrp <= 2U))
  {
    uint32_t ssp = dbrp * (1U + dtseg1);
    if (ssp > 127U)
    {
      ssp = 127U;
    }
    cfg.enable_tdc = true;
    cfg.canfd_timing.enable_tdc = true;
    cfg.tdc_config.ssp_offset = (uint8_t)ssp;
    cfg.tdc_config.filter_window_length = 0U;
  }
  else
  {
    cfg.enable_tdc = false;
    cfg.canfd_timing.enable_tdc = false;
  }

  mcan_get_default_ram_config(base, &cfg.ram_config, cfg.enable_canfd);
  cfg.ram_config.enable_std_filter = false;
  cfg.ram_config.std_filter_elem_count = 0U;
  cfg.ram_config.enable_ext_filter = false;
  cfg.ram_config.ext_filter_elem_count = 0U;
  cfg.all_filters_config.std_id_filter_list.mcan_filter_elem_count = 0U;
  cfg.all_filters_config.ext_id_filter_list.mcan_filter_elem_count = 0U;
  cfg.all_filters_config.global_filter_config.accept_non_matching_std_frame_option =
      MCAN_ACCEPT_NON_MATCHING_FRAME_OPTION_IN_RXFIFO0;
  cfg.all_filters_config.global_filter_config.accept_non_matching_ext_frame_option =
      MCAN_ACCEPT_NON_MATCHING_FRAME_OPTION_IN_RXFIFO0;
  cfg.all_filters_config.global_filter_config.reject_remote_std_frame = false;
  cfg.all_filters_config.global_filter_config.reject_remote_ext_frame = false;

  (void)mcan_init(base, &cfg, dev->clk_hz);
}

uint32_t pcan_can_msg_time(const struct t_can_msg *pmsg, uint32_t nt, uint32_t dt)
{
  if (pmsg == NULL)
  {
    return 0U;
  }

  uint32_t data_bits = ((uint32_t)pmsg->size) * 8U;
  if ((pmsg->flags & MSG_FLAG_FD) != 0U)
  {
    uint32_t hdr_bits = ((pmsg->flags & MSG_FLAG_EXT) != 0U) ? 48U : 29U;
    if ((pmsg->flags & MSG_FLAG_BRS) != 0U)
    {
      return hdr_bits * nt + (data_bits + 28U) * dt;
    }
    return (hdr_bits + data_bits + 28U) * nt;
  }

  uint32_t hdr_bits = ((pmsg->flags & MSG_FLAG_EXT) != 0U) ? 67U : 47U;
  return (hdr_bits + data_bits) * nt;
}

int pcan_can_stats(int bus, struct t_can_stats *p_stats)
{
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL) || (p_stats == NULL))
  {
    return 0;
  }

  const struct t_can_dev *dev = &can_dev_array[bus];
  p_stats->tx_msgs = dev->tx_msgs;
  p_stats->tx_errs = dev->tx_errs;
  p_stats->rx_msgs = dev->rx_msgs;
  p_stats->rx_errs = dev->rx_errs;
  p_stats->rx_ovfs = dev->rx_ovfs;
  return (int)sizeof(*p_stats);
}

int pcan_can_init_ex(int bus, uint32_t bitrate)
{
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL))
  {
    return -1;
  }

  struct t_can_dev *dev = &can_dev_array[bus];
  uint32_t brp = 0U, tseg1 = 0U, tseg2 = 0U, sjw = 0U;

  get_precalculated_nominal(bitrate, &brp, &tseg1, &tseg2, &sjw);
  dev->nom_timing.brp = (uint16_t)brp;
  dev->nom_timing.tseg1 = (uint16_t)tseg1;
  dev->nom_timing.tseg2 = (uint8_t)tseg2;
  dev->nom_timing.sjw = (uint8_t)sjw;

  get_precalculated_data(2000000U, &brp, &tseg1, &tseg2, &sjw);
  dev->data_timing.brp = (uint16_t)brp;
  dev->data_timing.tseg1 = (uint16_t)tseg1;
  dev->data_timing.tseg2 = (uint8_t)tseg2;
  dev->data_timing.sjw = (uint8_t)sjw;

  dev->bus_active = 0U;
  dev->silent_mode = 0U;
  dev->loopback_mode = 0U;
  dev->force_20ab = 0U;
  dev->tx_head = 0U;
  dev->tx_tail = 0U;
  dev->last_status = 0U;

  can_apply_config(dev);
  return 0;
}

void pcan_can_set_silent(int bus, uint8_t silent_mode)
{
  if ((bus >= 0) && (bus < CAN_BUS_TOTAL))
  {
    can_dev_array[bus].silent_mode = (silent_mode != 0U) ? 1U : 0U;
    can_apply_config(&can_dev_array[bus]);
  }
}

void pcan_can_set_iso_mode(int bus, uint8_t iso_mode)
{
  if ((bus >= 0) && (bus < CAN_BUS_TOTAL))
  {
    can_dev_array[bus].iso_mode = (iso_mode != 0U) ? 1U : 0U;
    can_apply_config(&can_dev_array[bus]);
  }
}

void pcan_can_set_20ab_mode(int bus, uint8_t force_classic)
{
  if ((bus >= 0) && (bus < CAN_BUS_TOTAL))
  {
    can_dev_array[bus].force_20ab = (force_classic != 0U) ? 1U : 0U;
    can_apply_config(&can_dev_array[bus]);
  }
}

void pcan_can_set_loopback(int bus, uint8_t loopback)
{
  if ((bus >= 0) && (bus < CAN_BUS_TOTAL))
  {
    can_dev_array[bus].loopback_mode = (loopback != 0U) ? 1U : 0U;
    can_apply_config(&can_dev_array[bus]);
  }
}

void pcan_can_set_bus_active(int bus, uint16_t mode)
{
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL))
  {
    return;
  }

  struct t_can_dev *dev = &can_dev_array[bus];
  dev->bus_active = (mode != 0U) ? 1U : 0U;
  dev->last_status = 0U;
  pcan_can_tx_abort(bus);
  can_apply_config(dev);
}

void pcan_can_tx_abort(int bus)
{
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL))
  {
    return;
  }

  struct t_can_dev *dev = &can_dev_array[bus];
  dev->tx_head = 0U;
  dev->tx_tail = 0U;

  if ((dev->is_physical != 0U) && (dev->mcan != NULL))
  {
    dev->mcan->TXBCR = MCAN_TXBCR_CR_SET(0xFFFFFFFFU);
  }
}

void pcan_can_set_bitrate(int bus, uint32_t bitrate, int is_data_bitrate)
{
  uint32_t brp = 0U, tseg1 = 0U, tseg2 = 0U, sjw = 0U;

  if ((bus < 0) || (bus >= CAN_BUS_TOTAL) || (bitrate == 0U))
  {
    return;
  }

  if (is_data_bitrate != 0)
  {
    get_precalculated_data(bitrate, &brp, &tseg1, &tseg2, &sjw);
    (void)pcan_can_set_data_timing(bus, brp, tseg1, tseg2, sjw);
  }
  else
  {
    get_precalculated_nominal(bitrate, &brp, &tseg1, &tseg2, &sjw);
    (void)pcan_can_set_nominal_timing(bus, brp, tseg1, tseg2, sjw);
  }
}

void pcan_can_set_bitrate_ex(int bus, uint16_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw)
{
  (void)pcan_can_set_nominal_timing(bus, brp, tseg1, tseg2, sjw);
}

int pcan_can_set_nominal_timing(int bus, uint32_t brp, uint32_t tseg1, uint32_t tseg2, uint32_t sjw)
{
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL))
  {
    return -1;
  }

  struct t_can_dev *dev = &can_dev_array[bus];
  dev->nom_timing.brp = (uint16_t)clamp_u32(brp, 1U, 512U);
  dev->nom_timing.tseg1 = (uint16_t)clamp_u32(tseg1, 1U, 256U);
  dev->nom_timing.tseg2 = (uint8_t)clamp_u32(tseg2, 1U, 128U);
  dev->nom_timing.sjw = (uint8_t)clamp_u32(sjw, 1U, dev->nom_timing.tseg2);

  can_apply_config(dev);
  return 0;
}

int pcan_can_set_data_timing(int bus, uint32_t brp, uint32_t tseg1, uint32_t tseg2, uint32_t sjw)
{
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL))
  {
    return -1;
  }

  struct t_can_dev *dev = &can_dev_array[bus];
  dev->data_timing.brp = (uint16_t)clamp_u32(brp, 1U, 32U);
  dev->data_timing.tseg1 = (uint16_t)clamp_u32(tseg1, 1U, 32U);
  dev->data_timing.tseg2 = (uint8_t)clamp_u32(tseg2, 1U, 16U);
  dev->data_timing.sjw = (uint8_t)clamp_u32(sjw, 1U, dev->data_timing.tseg2);

  can_apply_config(dev);
  return 0;
}

int pcan_can_write(int bus, struct t_can_msg *p_msg)
{
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL) || (p_msg == NULL))
  {
    return -1;
  }

  struct t_can_dev *dev = &can_dev_array[bus];
  if (dev->is_physical == 0U)
  {
    return 0;
  }

  uint32_t next_head = (dev->tx_head + 1U) % CAN_TX_FIFO_SIZE;
  if (next_head == dev->tx_tail)
  {
    dev->tx_ovfs++;
    return -1;
  }

  dev->tx_fifo[dev->tx_head] = *p_msg;
  dev->tx_head = next_head;
  return 0;
}

static void can_flush_tx_fifo(int bus)
{
  struct t_can_dev *dev = &can_dev_array[bus];
  if ((dev->is_physical == 0U) || (dev->mcan == NULL) || (dev->bus_active == 0U))
  {
    return;
  }

  MCAN_Type *base = dev->mcan;

  while (dev->tx_tail != dev->tx_head)
  {
    const struct t_can_msg *p_msg = &dev->tx_fifo[dev->tx_tail];
    mcan_tx_frame_t tx;
    memset(&tx, 0, sizeof(tx));

    if ((p_msg->flags & MSG_FLAG_EXT) != 0U)
    {
      tx.use_ext_id = 1U;
      tx.ext_id = p_msg->id & 0x1FFFFFFFU;
    }
    else
    {
      tx.use_ext_id = 0U;
      tx.std_id = p_msg->id & 0x7FFU;
    }

    uint8_t copy_len = (p_msg->size <= CAN_PAYLOAD_MAX_SIZE) ? p_msg->size : CAN_PAYLOAD_MAX_SIZE;
    if ((dev->force_20ab == 0U) && ((p_msg->flags & MSG_FLAG_FD) != 0U))
    {
      tx.canfd_frame = 1U;
      tx.rtr = 0U;
      tx.bitrate_switch = ((p_msg->flags & MSG_FLAG_BRS) != 0U) ? 1U : 0U;
      tx.error_state_indicator = ((p_msg->flags & MSG_FLAG_ESI) != 0U) ? 1U : 0U;
    }
    else
    {
      if (copy_len > 8U)
      {
        copy_len = 8U;
      }
      tx.canfd_frame = 0U;
      tx.rtr = ((p_msg->flags & MSG_FLAG_RTR) != 0U) ? 1U : 0U;
    }

    tx.dlc = len_to_dlc(copy_len);
    if ((copy_len > 0U) && (tx.rtr == 0U))
    {
      memcpy(tx.data_8, p_msg->data, copy_len);
    }

    if (mcan_transmit_via_txfifo_nonblocking(base, &tx, NULL) != status_success)
    {
      break;
    }

    dev->tx_msgs++;
    if (dev->tx_isr != NULL)
    {
      struct t_can_msg echo = *p_msg;
      echo.timestamp = pcan_timestamp_us();
      (void)dev->tx_isr((uint8_t)bus, &echo);
    }

    dev->tx_tail = (dev->tx_tail + 1U) % CAN_TX_FIFO_SIZE;
  }
}

static void can_poll_rx_fifo(int bus)
{
  struct t_can_dev *dev = &can_dev_array[bus];
  if ((dev->is_physical == 0U) || (dev->mcan == NULL) || (dev->bus_active == 0U))
  {
    return;
  }

  MCAN_Type *base = dev->mcan;

  for (uint32_t i = 0U; i < CAN_RX_POLL_BATCH; i++)
  {
    if (MCAN_RXF0S_F0FL_GET(base->RXF0S) == 0U)
    {
      break;
    }

    mcan_rx_message_t rx;
    if (mcan_read_rxfifo(base, 0U, &rx) != status_success)
    {
      dev->rx_errs++;
      break;
    }

    struct t_can_msg msg = {0};
    msg.timestamp = pcan_timestamp_us();
    msg.id = (rx.use_ext_id != 0U) ? rx.ext_id : rx.std_id;
    msg.flags = (rx.use_ext_id != 0U) ? MSG_FLAG_EXT : MSG_FLAG_STD;

    if (rx.rtr != 0U)
    {
      msg.flags |= MSG_FLAG_RTR;
    }
    if (rx.canfd_frame != 0U)
    {
      msg.flags |= MSG_FLAG_FD;
      if (rx.bitrate_switch != 0U)
      {
        msg.flags |= MSG_FLAG_BRS;
      }
      if (rx.error_state_indicator != 0U)
      {
        msg.flags |= MSG_FLAG_ESI;
      }
    }

    msg.size = dlc_to_len((uint8_t)rx.dlc);
    if ((msg.size > 0U) && ((msg.flags & MSG_FLAG_RTR) == 0U))
    {
      memcpy(msg.data, rx.data_8, msg.size);
    }

    if (dev->rx_isr != NULL)
    {
      if (dev->rx_isr((uint8_t)bus, &msg) < 0)
      {
        dev->rx_ovfs++;
        break;
      }
    }
    dev->rx_msgs++;
  }

  if ((mcan_get_interrupt_flags(base) & MCAN_IR_RF0L_MASK) != 0U)
  {
    mcan_clear_interrupt_flags(base, MCAN_IR_RF0L_MASK);
    dev->rx_ovfs++;
  }
}

static void can_poll_errors(int bus)
{
  struct t_can_dev *dev = &can_dev_array[bus];
  if ((dev->is_physical == 0U) || (dev->mcan == NULL) || (dev->bus_active == 0U))
  {
    return;
  }

  uint32_t psr = dev->mcan->PSR;
  /* Bits aligned to upper nibble of ucan_status_msg.channel_p_w_b (P=0x02, W=0x04, BO=0x08) */
  uint8_t status = 0U;

  if ((psr & MCAN_PSR_EP_MASK) != 0U)
  {
    status |= 0x02U;
  }
  if ((psr & MCAN_PSR_EW_MASK) != 0U)
  {
    status |= 0x04U;
  }
  if ((psr & MCAN_PSR_BO_MASK) != 0U)
  {
    status |= 0x08U;
  }

  if (status != dev->last_status)
  {
    dev->last_status = status;
    if (dev->status_cb != NULL)
    {
      dev->status_cb((uint8_t)bus, status);
    }
    if (dev->err_handler != NULL)
    {
      dev->err_handler(bus, status);
    }
  }
}

void pcan_can_poll(void)
{
  static uint32_t err_last_check = 0U;
  uint32_t ts_ms = pcan_timestamp_millis();

  can_poll_rx_fifo(CAN_BUS_1);
  can_flush_tx_fifo(CAN_BUS_1);

  if ((uint32_t)(ts_ms - err_last_check) >= 250U)
  {
    err_last_check = ts_ms;
    can_poll_errors(CAN_BUS_1);
  }
}

void pcan_can_install_rx_callback(int bus, int (*rx_isr)(uint8_t, struct t_can_msg *))
{
  if ((bus >= 0) && (bus < CAN_BUS_TOTAL))
  {
    can_dev_array[bus].rx_isr = rx_isr;
  }
}

void pcan_can_install_tx_callback(int bus, int (*tx_isr)(uint8_t, struct t_can_msg *))
{
  if ((bus >= 0) && (bus < CAN_BUS_TOTAL))
  {
    can_dev_array[bus].tx_isr = tx_isr;
  }
}

void pcan_can_install_err_callback(int bus, void (*cb)(int, uint32_t))
{
  if ((bus >= 0) && (bus < CAN_BUS_TOTAL))
  {
    can_dev_array[bus].err_handler = cb;
  }
}

void pcan_can_install_status_callback(int bus, void (*cb)(uint8_t, uint8_t))
{
  if ((bus >= 0) && (bus < CAN_BUS_TOTAL))
  {
    can_dev_array[bus].status_cb = cb;
  }
}

int pcan_can_set_filter_mask(int bus, int num, int format, uint32_t id, uint32_t mask)
{
  (void)bus;
  (void)num;
  (void)format;
  (void)id;
  (void)mask;
  return 0;
}

int pcan_can_filter_init_stdid_list(int bus, const uint16_t *id_list, int id_len)
{
  (void)bus;
  (void)id_list;
  (void)id_len;
  return 0;
}
