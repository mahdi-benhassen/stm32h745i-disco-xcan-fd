#include <string.h>
#include "at32f45x.h"
#include "pcanpro_timestamp.h"
#include "pcanpro_can.h"

#define CAN_TX_FIFO_SIZE  (128U)
#define CAN_RX_POLL_BATCH (16U)

struct t_can_dev
{
  can_type *can_x;
  uint32_t tx_msgs;
  uint32_t tx_errs;
  uint32_t tx_ovfs;

  uint32_t rx_msgs;
  uint32_t rx_errs;
  uint32_t rx_ovfs;

  struct t_can_msg tx_fifo[CAN_TX_FIFO_SIZE];
  uint32_t tx_head;
  uint32_t tx_tail;
  uint8_t  last_status;
  uint8_t  silent_mode;
  uint8_t  loopback_mode;
  uint8_t  iso_mode;
  uint8_t  force_20ab;
  uint8_t  bus_active;
  uint8_t  is_physical;

  struct t_can_bitrate nom_timing;
  struct t_can_bitrate data_timing;

  int  (*rx_isr)(uint8_t bus, struct t_can_msg *pmsg);
  int  (*tx_isr)(uint8_t bus, struct t_can_msg *pmsg);
  void (*err_handler)(int bus, uint32_t status);
  void (*status_cb)(uint8_t bus, uint8_t status);
};

static struct t_can_dev can_dev_array[CAN_BUS_TOTAL] =
{
  [CAN_BUS_1] = {
    .can_x = CAN1,
    .is_physical = 1U,
    .iso_mode = 1U,
    .nom_timing = { .brp = 2U, .tseg1 = 63U, .tseg2 = 16U, .sjw = 16U },   /* 1 Mbit/s @ 160 MHz */
    .data_timing = { .brp = 2U, .tseg1 = 15U, .tseg2 = 4U, .sjw = 4U },     /* 4 Mbit/s @ 160 MHz */
  },
  [CAN_BUS_2] = {
    .can_x = NULL,
    .is_physical = 0U,
    .iso_mode = 1U,
    .nom_timing = { .brp = 2U, .tseg1 = 63U, .tseg2 = 16U, .sjw = 16U },
    .data_timing = { .brp = 2U, .tseg1 = 15U, .tseg2 = 4U, .sjw = 4U },
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

static can_data_length_type len_to_can_dlc(uint8_t len)
{
  static const can_data_length_type std_dlc[9] =
  {
    CAN_DLC_BYTES_0, CAN_DLC_BYTES_1, CAN_DLC_BYTES_2,
    CAN_DLC_BYTES_3, CAN_DLC_BYTES_4, CAN_DLC_BYTES_5,
    CAN_DLC_BYTES_6, CAN_DLC_BYTES_7, CAN_DLC_BYTES_8
  };

  if (len <= 8U)
  {
    return std_dlc[len];
  }
  if (len <= 12U)
  {
    return CAN_DLC_BYTES_12;
  }
  if (len <= 16U)
  {
    return CAN_DLC_BYTES_16;
  }
  if (len <= 20U)
  {
    return CAN_DLC_BYTES_20;
  }
  if (len <= 24U)
  {
    return CAN_DLC_BYTES_24;
  }
  if (len <= 32U)
  {
    return CAN_DLC_BYTES_32;
  }
  if (len <= 48U)
  {
    return CAN_DLC_BYTES_48;
  }
  return CAN_DLC_BYTES_64;
}

static uint8_t can_dlc_to_len(can_data_length_type dlc)
{
  static const uint8_t dlc_bytes[16] =
  {
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
    8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U
  };

  uint32_t idx = (uint32_t)dlc & 0x0FU;
  return dlc_bytes[idx];
}

static void get_precalculated_nominal(uint32_t bitrate, uint32_t *brp,
                                      uint32_t *tseg1, uint32_t *tseg2, uint32_t *sjw)
{
  /* 160 MHz CAN_PCLK nominal bit timings (2x 80 MHz reference) */
  switch (bitrate)
  {
    case 1000000U:
      *brp = 2U;
      *tseg1 = 63U;
      *tseg2 = 16U;
      *sjw = 16U;
      break;
    case 800000U:
      *brp = 2U;
      *tseg1 = 79U;
      *tseg2 = 20U;
      *sjw = 20U;
      break;
    default:
    case 500000U:
      *brp = 4U;
      *tseg1 = 63U;
      *tseg2 = 16U;
      *sjw = 16U;
      break;
    case 250000U:
      *brp = 8U;
      *tseg1 = 63U;
      *tseg2 = 16U;
      *sjw = 16U;
      break;
    case 125000U:
      *brp = 16U;
      *tseg1 = 63U;
      *tseg2 = 16U;
      *sjw = 16U;
      break;
    case 100000U:
      *brp = 20U;
      *tseg1 = 63U;
      *tseg2 = 16U;
      *sjw = 16U;
      break;
    case 83333U:
      *brp = 120U;
      *tseg1 = 13U;
      *tseg2 = 2U;
      *sjw = 2U;
      break;
    case 50000U:
      *brp = 200U;
      *tseg1 = 13U;
      *tseg2 = 2U;
      *sjw = 2U;
      break;
    case 20000U:
      *brp = 500U;
      *tseg1 = 13U;
      *tseg2 = 2U;
      *sjw = 2U;
      break;
    case 10000U:
      *brp = 500U;
      *tseg1 = 27U;
      *tseg2 = 4U;
      *sjw = 4U;
      break;
  }
}

static void get_precalculated_data(uint32_t bitrate, uint32_t *brp,
                                   uint32_t *tseg1, uint32_t *tseg2, uint32_t *sjw)
{
  /* 160 MHz CAN_PCLK data phase bit timings */
  switch (bitrate)
  {
    case 5000000U:
      *brp = 2U;
      *tseg1 = 11U;
      *tseg2 = 4U;
      *sjw = 4U;
      break;
    case 4000000U:
      *brp = 2U;
      *tseg1 = 15U;
      *tseg2 = 4U;
      *sjw = 4U;
      break;
    default:
    case 2000000U:
      *brp = 4U;
      *tseg1 = 15U;
      *tseg2 = 4U;
      *sjw = 4U;
      break;
    case 1000000U:
      *brp = 8U;
      *tseg1 = 15U;
      *tseg2 = 4U;
      *sjw = 4U;
      break;
    case 500000U:
      *brp = 16U;
      *tseg1 = 15U;
      *tseg2 = 4U;
      *sjw = 4U;
      break;
  }
}

static void can1_gpio_and_clock_init(void)
{
  static uint8_t initialized = 0U;
  if (initialized != 0U)
  {
    return;
  }
  initialized = 1U;

  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_CAN1_PERIPH_CLOCK, TRUE);
  crm_can_clock_select(CRM_CAN1, CRM_CAN_CLOCK_SOURCE_PCLK);

  gpio_default_para_init(&gpio_init_struct);

  /* PD0 = CAN1_RX, PD1 = CAN1_TX (AF9) */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1;
  gpio_init(GPIOD, &gpio_init_struct);
  gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE0, GPIO_MUX_9);
  gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE1, GPIO_MUX_9);

  /* PB7 = CAN_STB on AT-START-F457: drive LOW to keep transceiver in normal active mode */
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pins = GPIO_PINS_7;
  gpio_init(GPIOB, &gpio_init_struct);
  gpio_bits_reset(GPIOB, GPIO_PINS_7);
}

static void scale_timing_for_div(const struct t_can_bitrate *src, uint32_t div,
                                 uint32_t max_bts1, uint32_t max_bts2,
                                 uint32_t *out_bts1, uint32_t *out_bts2, uint32_t *out_sjw)
{
  uint32_t src_tq = 1U + (uint32_t)src->tseg1 + (uint32_t)src->tseg2;
  uint32_t total_cycles = (uint32_t)src->brp * src_tq;
  uint32_t target_tq = (total_cycles + (div / 2U)) / div;
  target_tq = clamp_u32(target_tq, 3U, max_bts1 + max_bts2);

  /* Preserve nominal sample point (1 + tseg1) / src_tq */
  uint32_t bts1 = ((target_tq * (1U + (uint32_t)src->tseg1)) + (src_tq / 2U)) / src_tq;
  bts1 = clamp_u32(bts1, 2U, max_bts1);
  if (bts1 >= target_tq)
  {
    bts1 = target_tq - 1U;
  }

  uint32_t bts2 = clamp_u32(target_tq - bts1, 1U, max_bts2);
  if ((bts1 + bts2) != target_tq)
  {
    bts1 = clamp_u32(target_tq - bts2, 2U, max_bts1);
  }

  uint32_t sjw = (((uint32_t)src->sjw * (uint32_t)src->brp) + (div / 2U)) / div;
  sjw = clamp_u32(sjw, 1U, bts2);

  *out_bts1 = bts1;
  *out_bts2 = bts2;
  *out_sjw = sjw;
}

static void can_apply_config(struct t_can_dev *dev)
{
  if ((dev->is_physical == 0U) || (dev->can_x == NULL))
  {
    return;
  }

  can_type *can_x = dev->can_x;
  can1_gpio_and_clock_init();

  can_software_reset(can_x, TRUE);

  /* AT32 CAN1 shares one bittime_div (1..32) across nominal and data phases */
  uint32_t nom_cycles = (uint32_t)dev->nom_timing.brp *
                        (1U + (uint32_t)dev->nom_timing.tseg1 + (uint32_t)dev->nom_timing.tseg2);
  uint32_t data_cycles = (uint32_t)dev->data_timing.brp *
                         (1U + (uint32_t)dev->data_timing.tseg1 + (uint32_t)dev->data_timing.tseg2);

  uint32_t chosen_div = 0U;
  for (uint32_t div = 1U; div <= 32U; div++)
  {
    uint32_t nom_tq = nom_cycles / div;
    uint32_t data_tq = data_cycles / div;

    if ((nom_tq < 3U) || (nom_tq > 641U) || (data_tq < 3U) || (data_tq > 385U))
    {
      continue;
    }

    if (chosen_div == 0U)
    {
      chosen_div = div;
    }

    if (((nom_cycles % div) == 0U) && ((data_cycles % div) == 0U))
    {
      uint32_t nom_bts1 = 0U;
      uint32_t nom_bts2 = 0U;
      uint32_t nom_sjw = 0U;
      uint32_t data_bts1 = 0U;
      uint32_t data_bts2 = 0U;
      uint32_t data_sjw = 0U;
      scale_timing_for_div(&dev->nom_timing, div, 513U, 128U, &nom_bts1, &nom_bts2, &nom_sjw);
      scale_timing_for_div(&dev->data_timing, div, 257U, 128U, &data_bts1, &data_bts2, &data_sjw);
      if (((nom_bts1 + nom_bts2) == nom_tq) && ((data_bts1 + data_bts2) == data_tq))
      {
        chosen_div = div;
        break;
      }
    }
  }

  if (chosen_div == 0U)
  {
    chosen_div = 1U;
    for (uint32_t div = 1U; div <= 32U; div++)
    {
      if ((nom_cycles / div) <= 641U)
      {
        chosen_div = div;
        break;
      }
    }
  }

  uint32_t nom_bts1 = 0U;
  uint32_t nom_bts2 = 0U;
  uint32_t nom_sjw = 0U;
  uint32_t data_bts1 = 0U;
  uint32_t data_bts2 = 0U;
  uint32_t data_sjw = 0U;

  scale_timing_for_div(&dev->nom_timing, chosen_div, 513U, 128U, &nom_bts1, &nom_bts2, &nom_sjw);
  scale_timing_for_div(&dev->data_timing, chosen_div, 257U, 128U, &data_bts1, &data_bts2, &data_sjw);

  can_bittime_type bittime;
  can_bittime_default_para_init(&bittime);
  bittime.bittime_div = (uint8_t)chosen_div;
  bittime.ac_bts1_size = (uint16_t)nom_bts1;
  bittime.ac_bts2_size = (uint8_t)nom_bts2;
  bittime.ac_rsaw_size = (uint8_t)nom_sjw;
  bittime.fd_bts1_size = (uint16_t)data_bts1;
  bittime.fd_bts2_size = (uint8_t)data_bts2;
  bittime.fd_rsaw_size = (uint8_t)data_sjw;
  /* SSP offset in TQ units per Artery SDK (fd_bts1_size + 1) */
  bittime.fd_ssp_offset = ((data_bts1 + 1U) <= 255U) ? (uint8_t)(data_bts1 + 1U) : 0U;
  can_bittime_set(can_x, &bittime);

  can_fd_iso_mode_enable(can_x, (dev->iso_mode != 0U) ? TRUE : FALSE);
  can_stb_transmit_mode_set(can_x, CAN_STB_TRANSMIT_BY_FIFO);

  /* Accept all standard/extended classic & CAN-FD frames via Filter 0 */
  can_filter_config_type filter_cfg;
  can_filter_default_para_init(&filter_cfg);
  can_filter_set(can_x, CAN_FILTER_NUM_0, &filter_cfg);
  can_filter_enable(can_x, CAN_FILTER_NUM_0, TRUE);

  if (dev->bus_active != 0U)
  {
    can_software_reset(can_x, FALSE);
    can_retransmission_limit_set(can_x, CAN_RE_TRANS_TIMES_UNLIMIT);
    can_rearbitration_limit_set(can_x, CAN_RE_ARBI_TIMES_UNLIMIT);
    can_rxbuf_warning_set(can_x, 3U);

    if (dev->loopback_mode != 0U)
    {
      can_mode_set(can_x, CAN_MODE_INT_LOOPBACK);
    }
    else if (dev->silent_mode != 0U)
    {
      can_mode_set(can_x, CAN_MODE_LISTENONLY);
    }
    else
    {
      can_mode_set(can_x, CAN_MODE_COMMUNICATE);
    }
  }
}

uint32_t pcan_can_msg_time(const struct t_can_msg *pmsg, uint32_t nt, uint32_t dt)
{
  uint32_t data_bits;
  uint32_t hdr_bits;

  if (pmsg == NULL)
  {
    return 0U;
  }

  data_bits = ((uint32_t)pmsg->size) * 8U;
  if ((pmsg->flags & MSG_FLAG_FD) != 0U)
  {
    hdr_bits = ((pmsg->flags & MSG_FLAG_EXT) != 0U) ? 48U : 29U;
    if ((pmsg->flags & MSG_FLAG_BRS) != 0U)
    {
      return hdr_bits * nt + (data_bits + 28U) * dt;
    }
    return (hdr_bits + data_bits + 28U) * nt;
  }

  hdr_bits = ((pmsg->flags & MSG_FLAG_EXT) != 0U) ? 67U : 47U;
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
  uint32_t brp = 0U;
  uint32_t tseg1 = 0U;
  uint32_t tseg2 = 0U;
  uint32_t sjw = 0U;

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
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL))
  {
    return;
  }

  can_dev_array[bus].silent_mode = (silent_mode != 0U) ? 1U : 0U;
  can_apply_config(&can_dev_array[bus]);
}

void pcan_can_set_iso_mode(int bus, uint8_t iso_mode)
{
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL))
  {
    return;
  }

  can_dev_array[bus].iso_mode = (iso_mode != 0U) ? 1U : 0U;
  can_apply_config(&can_dev_array[bus]);
}

void pcan_can_set_20ab_mode(int bus, uint8_t force_classic)
{
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL))
  {
    return;
  }

  can_dev_array[bus].force_20ab = (force_classic != 0U) ? 1U : 0U;
}

void pcan_can_set_loopback(int bus, uint8_t loopback)
{
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL))
  {
    return;
  }

  can_dev_array[bus].loopback_mode = (loopback != 0U) ? 1U : 0U;
  can_apply_config(&can_dev_array[bus]);
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

  if ((dev->is_physical != 0U) && (dev->can_x != NULL))
  {
    can_transmit_abort(dev->can_x, CAN_ABORT_TRANSMIT_PTB);
    can_transmit_abort(dev->can_x, CAN_ABORT_TRANSMIT_STB);
  }
}

void pcan_can_set_bitrate(int bus, uint32_t bitrate, int is_data_bitrate)
{
  uint32_t brp = 0U;
  uint32_t tseg1 = 0U;
  uint32_t tseg2 = 0U;
  uint32_t sjw = 0U;

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
  dev->nom_timing.tseg1 = (uint16_t)clamp_u32(tseg1, 1U, 512U);
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
  dev->data_timing.tseg1 = (uint16_t)clamp_u32(tseg1, 1U, 256U);
  dev->data_timing.tseg2 = (uint8_t)clamp_u32(tseg2, 1U, 128U);
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
  if ((dev->is_physical == 0U) || (dev->can_x == NULL) || (dev->bus_active == 0U))
  {
    return;
  }

  can_type *can_x = dev->can_x;
  uint8_t sent_any = 0U;

  while (dev->tx_tail != dev->tx_head)
  {
    struct t_can_msg *p_msg = &dev->tx_fifo[dev->tx_tail];
    can_txbuf_type tx = {0};

    if ((p_msg->flags & MSG_FLAG_EXT) != 0U)
    {
      tx.id_type = CAN_ID_EXTENDED;
      tx.id = p_msg->id & 0x1FFFFFFFU;
    }
    else
    {
      tx.id_type = CAN_ID_STANDARD;
      tx.id = p_msg->id & 0x7FFU;
    }

    if ((dev->force_20ab == 0U) && ((p_msg->flags & MSG_FLAG_FD) != 0U))
    {
      tx.frame_type = CAN_FRAME_DATA;
      tx.fd_format = CAN_FORMAT_FD;
      tx.fd_rate_switch = ((p_msg->flags & MSG_FLAG_BRS) != 0U) ? CAN_BRS_ON : CAN_BRS_OFF;
    }
    else
    {
      tx.frame_type = ((p_msg->flags & MSG_FLAG_RTR) != 0U) ? CAN_FRAME_REMOTE : CAN_FRAME_DATA;
      tx.fd_format = CAN_FORMAT_CLASSIC;
      tx.fd_rate_switch = CAN_BRS_OFF;
    }

    uint8_t copy_len = (p_msg->size <= CAN_PAYLOAD_MAX_SIZE) ? p_msg->size : CAN_PAYLOAD_MAX_SIZE;
    tx.data_length = len_to_can_dlc(copy_len);
    if ((copy_len > 0U) && (tx.frame_type == CAN_FRAME_DATA))
    {
      memcpy(tx.data, p_msg->data, copy_len);
    }

    if (can_txbuf_write(can_x, CAN_TXBUF_STB, &tx) != SUCCESS)
    {
      break;
    }

    sent_any = 1U;
    dev->tx_msgs++;
    if (dev->tx_isr != NULL)
    {
      struct t_can_msg echo = *p_msg;
      echo.timestamp = pcan_timestamp_us();
      (void)dev->tx_isr((uint8_t)bus, &echo);
    }

    dev->tx_tail = (dev->tx_tail + 1U) % CAN_TX_FIFO_SIZE;
  }

  if (sent_any != 0U)
  {
    can_txbuf_transmit(can_x, CAN_TRANSMIT_STB_ALL);
  }
}

static void can_poll_rx_fifo(int bus)
{
  struct t_can_dev *dev = &can_dev_array[bus];
  if ((dev->is_physical == 0U) || (dev->can_x == NULL) || (dev->bus_active == 0U))
  {
    return;
  }

  can_type *can_x = dev->can_x;

  for (uint32_t i = 0U; i < CAN_RX_POLL_BATCH; i++)
  {
    can_rxbuf_type rx;
    if (can_rxbuf_read(can_x, &rx) != SUCCESS)
    {
      break;
    }

    struct t_can_msg msg = {0};
    msg.timestamp = pcan_timestamp_us();
    msg.id = rx.id;
    msg.flags = (rx.id_type == CAN_ID_EXTENDED) ? MSG_FLAG_EXT : MSG_FLAG_STD;

    if (rx.frame_type == CAN_FRAME_REMOTE)
    {
      msg.flags |= MSG_FLAG_RTR;
    }
    if (rx.fd_format == CAN_FORMAT_FD)
    {
      msg.flags |= MSG_FLAG_FD;
      if (rx.fd_rate_switch == CAN_BRS_ON)
      {
        msg.flags |= MSG_FLAG_BRS;
      }
      if (rx.fd_error_state == CAN_ESI_PASSIVE)
      {
        msg.flags |= MSG_FLAG_ESI;
      }
    }

    msg.size = can_dlc_to_len(rx.data_length);
    if ((msg.size > 0U) && ((msg.flags & MSG_FLAG_RTR) == 0U))
    {
      memcpy(msg.data, rx.data, msg.size);
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

  if (can_flag_get(can_x, CAN_ROIF_FLAG) == SET)
  {
    can_flag_clear(can_x, CAN_ROIF_FLAG);
    dev->rx_ovfs++;
  }
}

static void can_poll_errors(int bus)
{
  struct t_can_dev *dev = &can_dev_array[bus];
  if ((dev->is_physical == 0U) || (dev->can_x == NULL) || (dev->bus_active == 0U))
  {
    return;
  }

  can_type *can_x = dev->can_x;
  /* Bits aligned to upper nibble of ucan_status_msg.channel_p_w_b (P=0x02, W=0x04, BO=0x08) */
  uint8_t status = 0U;

  if (can_flag_get(can_x, CAN_EPASS_FLAG) == SET)
  {
    status |= 0x02U;
  }
  if (can_flag_get(can_x, CAN_EWARN_FLAG) == SET)
  {
    status |= 0x04U;
  }
  if (can_busoff_get(can_x) == SET)
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
