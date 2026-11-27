#include <string.h>
#include "fdcan.h"
#include "pcanpro_timestamp.h"
#include "pcanpro_can.h"

#define CAN_TX_FIFO_SIZE  (128U)
#define CAN_RX_POLL_BATCH (16U)

struct t_can_dev
{
  FDCAN_HandleTypeDef *hfdcan;
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

  int  (*rx_isr)(uint8_t bus, struct t_can_msg *pmsg);
  int  (*tx_isr)(uint8_t bus, struct t_can_msg *pmsg);
  void (*err_handler)(int bus, uint32_t status);
  void (*status_cb)(uint8_t bus, uint8_t status);
};

static struct t_can_dev can_dev_array[CAN_BUS_TOTAL] =
{
  [CAN_BUS_1] = { .hfdcan = &hfdcan1 },
#if defined(CAN_NODE_SINGLE_BOARD) && (CAN_NODE_SINGLE_BOARD != 0)
  /* Reserve FDCAN2 exclusively for the on-board can-node UDS/ISO-TP ECU in Option B. */
  [CAN_BUS_2] = { .hfdcan = NULL },
#else
  [CAN_BUS_2] = { .hfdcan = &hfdcan2 },
#endif
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

static uint32_t len_to_fdcan_dlc(uint8_t len)
{
  static const uint32_t std_dlc[9] =
  {
    FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
    FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
    FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8
  };

  if (len <= 8U)
  {
    return std_dlc[len];
  }
  if (len <= 12U)
  {
    return FDCAN_DLC_BYTES_12;
  }
  if (len <= 16U)
  {
    return FDCAN_DLC_BYTES_16;
  }
  if (len <= 20U)
  {
    return FDCAN_DLC_BYTES_20;
  }
  if (len <= 24U)
  {
    return FDCAN_DLC_BYTES_24;
  }
  if (len <= 32U)
  {
    return FDCAN_DLC_BYTES_32;
  }
  if (len <= 48U)
  {
    return FDCAN_DLC_BYTES_48;
  }
  return FDCAN_DLC_BYTES_64;
}

static uint8_t fdcan_dlc_to_len(uint32_t dlc)
{
  static const uint8_t dlc_bytes[16] =
  {
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
    8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U
  };

  if (dlc < 16U)
  {
    return dlc_bytes[dlc];
  }
  return 64U;
}

static void get_precalculated_nominal(uint32_t bitrate, uint32_t *brp,
                                      uint32_t *tseg1, uint32_t *tseg2, uint32_t *sjw)
{
  /* 80 MHz FDCAN kernel clock: 87.5% SP (16 TQ) for <= 500k, 80% SP (10 TQ) for 800k/1M */
  switch (bitrate)
  {
    case 1000000U:
      *brp = 8U;
      *tseg1 = 7U;
      *tseg2 = 2U;
      *sjw = 2U;
      break;
    case 800000U:
      *brp = 10U;
      *tseg1 = 7U;
      *tseg2 = 2U;
      *sjw = 2U;
      break;
    default:
    case 500000U:
      *brp = 10U;
      *tseg1 = 13U;
      *tseg2 = 2U;
      *sjw = 2U;
      break;
    case 250000U:
      *brp = 20U;
      *tseg1 = 13U;
      *tseg2 = 2U;
      *sjw = 2U;
      break;
    case 125000U:
      *brp = 40U;
      *tseg1 = 13U;
      *tseg2 = 2U;
      *sjw = 2U;
      break;
    case 100000U:
      *brp = 50U;
      *tseg1 = 13U;
      *tseg2 = 2U;
      *sjw = 2U;
      break;
    case 50000U:
      *brp = 100U;
      *tseg1 = 13U;
      *tseg2 = 2U;
      *sjw = 2U;
      break;
    case 20000U:
      *brp = 250U;
      *tseg1 = 13U;
      *tseg2 = 2U;
      *sjw = 2U;
      break;
    case 10000U:
      *brp = 500U;
      *tseg1 = 13U;
      *tseg2 = 2U;
      *sjw = 2U;
      break;
  }
}

static void get_precalculated_data(uint32_t bitrate, uint32_t *brp,
                                   uint32_t *tseg1, uint32_t *tseg2, uint32_t *sjw)
{
  /* 80 MHz FDCAN kernel clock data phase timings */
  switch (bitrate)
  {
    case 5000000U:
      *brp = 1U;
      *tseg1 = 11U;
      *tseg2 = 4U;
      *sjw = 4U;
      break;
    case 4000000U:
      *brp = 1U;
      *tseg1 = 15U;
      *tseg2 = 4U;
      *sjw = 4U;
      break;
    default:
    case 2000000U:
      *brp = 2U;
      *tseg1 = 15U;
      *tseg2 = 4U;
      *sjw = 4U;
      break;
    case 1000000U:
      *brp = 4U;
      *tseg1 = 15U;
      *tseg2 = 4U;
      *sjw = 4U;
      break;
    case 500000U:
      *brp = 8U;
      *tseg1 = 15U;
      *tseg2 = 4U;
      *sjw = 4U;
      break;
  }
}

static int pcan_can_reinit(int bus)
{
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL))
  {
    return -1;
  }

  struct t_can_dev *p_dev = &can_dev_array[bus];
  FDCAN_HandleTypeDef *hfdcan = p_dev->hfdcan;
  if (hfdcan == NULL)
  {
    return -1;
  }

  if (hfdcan->State == HAL_FDCAN_STATE_BUSY)
  {
    (void)HAL_FDCAN_Stop(hfdcan);
  }

  if (p_dev->silent_mode != 0U)
  {
    hfdcan->Init.Mode = FDCAN_MODE_BUS_MONITORING;
  }
  else if (p_dev->loopback_mode != 0U)
  {
    hfdcan->Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
  }
  else
  {
    hfdcan->Init.Mode = FDCAN_MODE_NORMAL;
  }

  hfdcan->Init.FrameFormat = (p_dev->force_20ab != 0U) ? FDCAN_FRAME_CLASSIC : FDCAN_FRAME_FD_BRS;

  if (HAL_FDCAN_Init(hfdcan) != HAL_OK)
  {
    return -1;
  }

  if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_FILTER_REMOTE,
                                   FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    return -1;
  }

  if (p_dev->iso_mode != 0U)
  {
    (void)HAL_FDCAN_EnableISOMode(hfdcan);
  }
  else
  {
    (void)HAL_FDCAN_DisableISOMode(hfdcan);
  }

  if ((hfdcan->Init.FrameFormat == FDCAN_FRAME_FD_BRS) && (hfdcan->Init.DataPrescaler <= 2U))
  {
    uint32_t tdco = hfdcan->Init.DataPrescaler * hfdcan->Init.DataTimeSeg1;
    if (tdco > 0x7FU)
    {
      tdco = 0x7FU;
    }
    (void)HAL_FDCAN_ConfigTxDelayCompensation(hfdcan, tdco, 0U);
    (void)HAL_FDCAN_EnableTxDelayCompensation(hfdcan);
  }
  else
  {
    (void)HAL_FDCAN_DisableTxDelayCompensation(hfdcan);
  }

  if (p_dev->bus_active != 0U)
  {
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
    {
      return -1;
    }
  }

  return 0;
}

uint32_t pcan_can_msg_time(const struct t_can_msg *pmsg, uint32_t nt, uint32_t dt)
{
  const uint32_t data_bits = ((uint32_t)pmsg->size) << 3;
  const uint32_t control_bits = (pmsg->flags & MSG_FLAG_EXT) ? 67U : 47U;

  if ((pmsg->flags & MSG_FLAG_BRS) != 0U)
  {
    return (control_bits * nt) + (data_bits * dt);
  }
  return (control_bits + data_bits) * nt;
}

int pcan_can_set_filter_mask(int bus, int num, int format, uint32_t id, uint32_t mask)
{
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL) || (num < 0))
  {
    return -1;
  }

  FDCAN_HandleTypeDef *hfdcan = can_dev_array[bus].hfdcan;
  if (hfdcan == NULL)
  {
    return -1;
  }

  FDCAN_FilterTypeDef filter = { 0 };
  if (format == MSG_FLAG_EXT)
  {
    if ((uint32_t)num >= hfdcan->Init.ExtFiltersNbr)
    {
      return -1;
    }
    filter.IdType = FDCAN_EXTENDED_ID;
    filter.FilterID1 = id & 0x1FFFFFFFU;
    filter.FilterID2 = mask & 0x1FFFFFFFU;
  }
  else
  {
    if ((uint32_t)num >= hfdcan->Init.StdFiltersNbr)
    {
      return -1;
    }
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterID1 = id & 0x7FFU;
    filter.FilterID2 = mask & 0x7FFU;
  }

  filter.FilterIndex = (uint32_t)num;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

  if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
  {
    return -1;
  }
  return 0;
}

int pcan_can_filter_init_stdid_list(int bus, const uint16_t *id_list, int id_len)
{
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL) || (id_list == NULL) || (id_len < 0))
  {
    return -1;
  }

  FDCAN_HandleTypeDef *hfdcan = can_dev_array[bus].hfdcan;
  if (hfdcan == NULL)
  {
    return -1;
  }

  int count = 0;
  for (int i = 0; (i + 1) < id_len; i += 2)
  {
    uint32_t idx = (uint32_t)(i >> 1);
    if (idx >= hfdcan->Init.StdFiltersNbr)
    {
      break;
    }
    FDCAN_FilterTypeDef filter =
    {
      .IdType = FDCAN_STANDARD_ID,
      .FilterIndex = idx,
      .FilterType = FDCAN_FILTER_DUAL,
      .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
      .FilterID1 = id_list[i] & 0x7FFU,
      .FilterID2 = id_list[i + 1] & 0x7FFU,
    };
    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
    {
      return -1;
    }
    count += 2;
  }

  if (((id_len & 1) != 0) && (((uint32_t)id_len >> 1) < hfdcan->Init.StdFiltersNbr))
  {
    FDCAN_FilterTypeDef filter =
    {
      .IdType = FDCAN_STANDARD_ID,
      .FilterIndex = (uint32_t)(id_len >> 1),
      .FilterType = FDCAN_FILTER_DUAL,
      .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
      .FilterID1 = id_list[id_len - 1] & 0x7FFU,
      .FilterID2 = id_list[id_len - 1] & 0x7FFU,
    };
    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
    {
      return -1;
    }
    ++count;
  }

  return count;
}

static int can_send_hw(struct t_can_dev *p_dev, const struct t_can_msg *p_msg)
{
  FDCAN_HandleTypeDef *hfdcan = p_dev->hfdcan;
  if ((hfdcan == NULL) || (hfdcan->State != HAL_FDCAN_STATE_BUSY))
  {
    return -1;
  }

  if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0U)
  {
    return -1;
  }

  FDCAN_TxHeaderTypeDef hdr = { 0 };
  if ((p_msg->flags & MSG_FLAG_EXT) != 0U)
  {
    hdr.IdType = FDCAN_EXTENDED_ID;
    hdr.Identifier = p_msg->id & 0x1FFFFFFFU;
  }
  else
  {
    hdr.IdType = FDCAN_STANDARD_ID;
    hdr.Identifier = p_msg->id & 0x7FFU;
  }

  uint8_t size = p_msg->size;
  if ((p_msg->flags & MSG_FLAG_FD) != 0U && (p_dev->force_20ab == 0U))
  {
    hdr.FDFormat = FDCAN_FD_CAN;
    hdr.TxFrameType = FDCAN_DATA_FRAME;
    hdr.BitRateSwitch = (p_msg->flags & MSG_FLAG_BRS) ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    hdr.ErrorStateIndicator = (p_msg->flags & MSG_FLAG_ESI) ? FDCAN_ESI_PASSIVE : FDCAN_ESI_ACTIVE;
  }
  else
  {
    if (size > 8U)
    {
      size = 8U;
    }
    hdr.FDFormat = FDCAN_CLASSIC_CAN;
    hdr.TxFrameType = (p_msg->flags & MSG_FLAG_RTR) ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    hdr.BitRateSwitch = FDCAN_BRS_OFF;
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  }

  hdr.DataLength = len_to_fdcan_dlc(size);
  hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  hdr.MessageMarker = 0U;

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &hdr, p_msg->data) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

static void pcan_can_flush_tx(int bus)
{
  struct t_can_dev *p_dev = &can_dev_array[bus];

  while (p_dev->tx_head != p_dev->tx_tail)
  {
    struct t_can_msg *p_msg = &p_dev->tx_fifo[p_dev->tx_tail];
    if (can_send_hw(p_dev, p_msg) < 0)
    {
      break;
    }

    ++p_dev->tx_msgs;
    if (p_dev->tx_isr != NULL)
    {
      (void)p_dev->tx_isr((uint8_t)bus, p_msg);
    }

    p_dev->tx_tail = (p_dev->tx_tail + 1U) & (CAN_TX_FIFO_SIZE - 1U);
  }
}

int pcan_can_write(int bus, struct t_can_msg *p_msg)
{
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL) || (p_msg == NULL))
  {
    return -1;
  }

  struct t_can_dev *p_dev = &can_dev_array[bus];
  uint32_t tx_head_next = (p_dev->tx_head + 1U) & (CAN_TX_FIFO_SIZE - 1U);

  if (tx_head_next == p_dev->tx_tail)
  {
    ++p_dev->tx_ovfs;
    return -1;
  }

  p_dev->tx_fifo[p_dev->tx_head] = *p_msg;
  p_dev->tx_head = tx_head_next;
  return 0;
}

void pcan_can_install_rx_callback(int bus, int (*cb)(uint8_t, struct t_can_msg *))
{
  if ((bus >= 0) && (bus < CAN_BUS_TOTAL))
  {
    can_dev_array[bus].rx_isr = cb;
  }
}

void pcan_can_install_tx_callback(int bus, int (*cb)(uint8_t, struct t_can_msg *))
{
  if ((bus >= 0) && (bus < CAN_BUS_TOTAL))
  {
    can_dev_array[bus].tx_isr = cb;
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

int pcan_can_init_ex(int bus, uint32_t bitrate)
{
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL))
  {
    return -1;
  }

  struct t_can_dev *p_dev = &can_dev_array[bus];
  FDCAN_HandleTypeDef *hfdcan = p_dev->hfdcan;
  if (hfdcan == NULL)
  {
    return -1;
  }

  uint32_t brp, tseg1, tseg2, sjw;
  get_precalculated_nominal(bitrate, &brp, &tseg1, &tseg2, &sjw);

  hfdcan->Init.AutoRetransmission = ENABLE;
  hfdcan->Init.TransmitPause = DISABLE;
  hfdcan->Init.ProtocolException = DISABLE;
  hfdcan->Init.NominalPrescaler = brp;
  hfdcan->Init.NominalTimeSeg1 = tseg1;
  hfdcan->Init.NominalTimeSeg2 = tseg2;
  hfdcan->Init.NominalSyncJumpWidth = sjw;

  get_precalculated_data(2000000U, &brp, &tseg1, &tseg2, &sjw);
  hfdcan->Init.DataPrescaler = brp;
  hfdcan->Init.DataTimeSeg1 = tseg1;
  hfdcan->Init.DataTimeSeg2 = tseg2;
  hfdcan->Init.DataSyncJumpWidth = sjw;

  p_dev->bus_active = 0U;
  p_dev->silent_mode = 0U;
  p_dev->loopback_mode = 0U;
  p_dev->force_20ab = 0U;
  p_dev->tx_head = 0U;
  p_dev->tx_tail = 0U;
  p_dev->last_status = 0U;

  return pcan_can_reinit(bus);
}

void pcan_can_set_silent(int bus, uint8_t silent_mode)
{
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL))
  {
    return;
  }
  can_dev_array[bus].silent_mode = (silent_mode != 0U) ? 1U : 0U;
  (void)pcan_can_reinit(bus);
}

void pcan_can_set_iso_mode(int bus, uint8_t iso_mode)
{
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL))
  {
    return;
  }
  can_dev_array[bus].iso_mode = (iso_mode != 0U) ? 1U : 0U;
  (void)pcan_can_reinit(bus);
}

void pcan_can_set_20ab_mode(int bus, uint8_t force_classic)
{
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL))
  {
    return;
  }
  can_dev_array[bus].force_20ab = (force_classic != 0U) ? 1U : 0U;
  (void)pcan_can_reinit(bus);
}

void pcan_can_set_loopback(int bus, uint8_t loopback)
{
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL))
  {
    return;
  }
  can_dev_array[bus].loopback_mode = (loopback != 0U) ? 1U : 0U;
  (void)pcan_can_reinit(bus);
}

void pcan_can_tx_abort(int bus)
{
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL))
  {
    return;
  }

  struct t_can_dev *p_dev = &can_dev_array[bus];
  p_dev->tx_tail = p_dev->tx_head;

  if ((p_dev->hfdcan != NULL) && (p_dev->hfdcan->State == HAL_FDCAN_STATE_BUSY))
  {
    (void)HAL_FDCAN_AbortTxRequest(p_dev->hfdcan, 0xFFFFFFFFU);
  }
}

void pcan_can_set_bus_active(int bus, uint16_t mode)
{
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL))
  {
    return;
  }

  struct t_can_dev *p_dev = &can_dev_array[bus];
  p_dev->bus_active = (mode != 0U) ? 1U : 0U;
  pcan_can_tx_abort(bus);
  p_dev->last_status = 0U;
  (void)pcan_can_reinit(bus);
}

int pcan_can_set_nominal_timing(int bus, uint32_t brp, uint32_t tseg1, uint32_t tseg2, uint32_t sjw)
{
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL))
  {
    return -1;
  }

  FDCAN_HandleTypeDef *hfdcan = can_dev_array[bus].hfdcan;
  if (hfdcan == NULL)
  {
    return -1;
  }

  tseg2 = clamp_u32(tseg2, 1U, 128U);
  tseg1 = clamp_u32(tseg1, 1U, 256U);
  sjw = clamp_u32(sjw, 1U, (tseg2 < 128U) ? tseg2 : 128U);
  brp = clamp_u32(brp, 1U, 512U);

  hfdcan->Init.NominalPrescaler = brp;
  hfdcan->Init.NominalTimeSeg1 = tseg1;
  hfdcan->Init.NominalTimeSeg2 = tseg2;
  hfdcan->Init.NominalSyncJumpWidth = sjw;

  return pcan_can_reinit(bus);
}

int pcan_can_set_data_timing(int bus, uint32_t brp, uint32_t tseg1, uint32_t tseg2, uint32_t sjw)
{
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL))
  {
    return -1;
  }

  FDCAN_HandleTypeDef *hfdcan = can_dev_array[bus].hfdcan;
  if (hfdcan == NULL)
  {
    return -1;
  }

  tseg2 = clamp_u32(tseg2, 1U, 16U);
  tseg1 = clamp_u32(tseg1, 1U, 32U);
  sjw = clamp_u32(sjw, 1U, (tseg2 < 16U) ? tseg2 : 16U);
  brp = clamp_u32(brp, 1U, 32U);

  hfdcan->Init.DataPrescaler = brp;
  hfdcan->Init.DataTimeSeg1 = tseg1;
  hfdcan->Init.DataTimeSeg2 = tseg2;
  hfdcan->Init.DataSyncJumpWidth = sjw;

  return pcan_can_reinit(bus);
}

void pcan_can_set_bitrate(int bus, uint32_t bitrate, int is_data_bitrate)
{
  uint32_t brp, tseg1, tseg2, sjw;

  if ((bus < 0) || (bus >= CAN_BUS_TOTAL))
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

int pcan_can_stats(int bus, struct t_can_stats *p_stats)
{
  if ((bus < 0) || (bus >= CAN_BUS_TOTAL) || (p_stats == NULL))
  {
    return 0;
  }

  struct t_can_dev *p_dev = &can_dev_array[bus];
  p_stats->tx_msgs = p_dev->tx_msgs;
  p_stats->tx_errs = p_dev->tx_errs;
  p_stats->rx_msgs = p_dev->rx_msgs;
  p_stats->rx_errs = p_dev->rx_errs;
  p_stats->rx_ovfs = p_dev->rx_ovfs;

  return (int)sizeof(struct t_can_stats);
}

static void poll_rx_fifo(int bus, struct t_can_dev *p_dev)
{
  FDCAN_HandleTypeDef *hfdcan = p_dev->hfdcan;

  for (uint32_t n = 0U; n < CAN_RX_POLL_BATCH; n++)
  {
    if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) == 0U)
    {
      break;
    }

    FDCAN_RxHeaderTypeDef hdr;
    struct t_can_msg msg = { 0 };
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &hdr, msg.data) != HAL_OK)
    {
      ++p_dev->rx_errs;
      break;
    }

    msg.id = hdr.Identifier;
    if (hdr.IdType == FDCAN_EXTENDED_ID)
    {
      msg.flags |= MSG_FLAG_EXT;
    }
    if (hdr.RxFrameType == FDCAN_REMOTE_FRAME)
    {
      msg.flags |= MSG_FLAG_RTR;
    }
    if (hdr.FDFormat == FDCAN_FD_CAN)
    {
      msg.flags |= MSG_FLAG_FD;
      if (hdr.BitRateSwitch == FDCAN_BRS_ON)
      {
        msg.flags |= MSG_FLAG_BRS;
      }
      if (hdr.ErrorStateIndicator == FDCAN_ESI_PASSIVE)
      {
        msg.flags |= MSG_FLAG_ESI;
      }
    }

    msg.size = fdcan_dlc_to_len(hdr.DataLength);
    msg.timestamp = pcan_timestamp_us();

    if (p_dev->rx_isr != NULL)
    {
      if (p_dev->rx_isr((uint8_t)bus, &msg) < 0)
      {
        ++p_dev->rx_ovfs;
        break;
      }
    }
    ++p_dev->rx_msgs;
  }

  if (__HAL_FDCAN_GET_FLAG(hfdcan, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST) != 0U)
  {
    __HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST);
    ++p_dev->rx_ovfs;
  }
}

void pcan_can_poll(void)
{
  static uint32_t err_last_check = 0U;
  uint32_t ts_ms = pcan_timestamp_millis();

  for (int i = 0; i < CAN_BUS_TOTAL; i++)
  {
    struct t_can_dev *p_dev = &can_dev_array[i];
    if ((p_dev->hfdcan == NULL) || (p_dev->hfdcan->State != HAL_FDCAN_STATE_BUSY))
    {
      continue;
    }

    poll_rx_fifo(i, p_dev);
    pcan_can_flush_tx(i);
  }

  if ((uint32_t)(ts_ms - err_last_check) >= 250U)
  {
    err_last_check = ts_ms;
    for (int i = 0; i < CAN_BUS_TOTAL; i++)
    {
      struct t_can_dev *p_dev = &can_dev_array[i];
      if ((p_dev->hfdcan == NULL) || (p_dev->bus_active == 0U))
      {
        continue;
      }

      FDCAN_ProtocolStatusTypeDef psr;
      if (HAL_FDCAN_GetProtocolStatus(p_dev->hfdcan, &psr) != HAL_OK)
      {
        continue;
      }

      /* Bits aligned to upper nibble of ucan_status_msg.channel_p_w_b (P=0x02, W=0x04, BO=0x08). */
      uint8_t status = 0U;
      if (psr.ErrorPassive != 0U)
      {
        status |= 0x02U;
      }
      if (psr.Warning != 0U)
      {
        status |= 0x04U;
      }
      if (psr.BusOff != 0U)
      {
        status |= 0x08U;
      }

      if (status != p_dev->last_status)
      {
        p_dev->last_status = status;
        if (p_dev->status_cb != NULL)
        {
          p_dev->status_cb((uint8_t)i, status);
        }
        if (p_dev->err_handler != NULL)
        {
          p_dev->err_handler(i, status);
        }
      }
    }
  }
}
