/* SPDX-License-Identifier: MIT */
#include "can_node_fdcan.h"

#include <string.h>

#define FDCAN_POLL_BATCH_MAX    16U
#define FDCAN_BUSOFF_RECOVER_US 50000U
#define FDCAN_MSG_RAM_STD_MAX   8U
#define FDCAN_MSG_RAM_EXT_MAX   4U
#define FDCAN_MSG_RAM_RX0_MAX   32U
#define FDCAN_MSG_RAM_TX_MAX    16U

static uint32_t clamp_u32(uint32_t val, uint32_t min_v, uint32_t max_v)
{
    if (val < min_v) {
        return min_v;
    }
    if (val > max_v) {
        return max_v;
    }
    return val;
}

static uint32_t len_to_hal_dlc(uint8_t len)
{
    const uint8_t padded = can_fd_pad_len(len);

    switch (padded) {
    case 0U:
        return FDCAN_DLC_BYTES_0;
    case 1U:
        return FDCAN_DLC_BYTES_1;
    case 2U:
        return FDCAN_DLC_BYTES_2;
    case 3U:
        return FDCAN_DLC_BYTES_3;
    case 4U:
        return FDCAN_DLC_BYTES_4;
    case 5U:
        return FDCAN_DLC_BYTES_5;
    case 6U:
        return FDCAN_DLC_BYTES_6;
    case 7U:
        return FDCAN_DLC_BYTES_7;
    case 8U:
        return FDCAN_DLC_BYTES_8;
    case 12U:
        return FDCAN_DLC_BYTES_12;
    case 16U:
        return FDCAN_DLC_BYTES_16;
    case 20U:
        return FDCAN_DLC_BYTES_20;
    case 24U:
        return FDCAN_DLC_BYTES_24;
    case 32U:
        return FDCAN_DLC_BYTES_32;
    case 48U:
        return FDCAN_DLC_BYTES_48;
    default:
        return FDCAN_DLC_BYTES_64;
    }
}

static uint8_t hal_dlc_to_len(uint32_t hal_dlc)
{
    switch (hal_dlc) {
    case FDCAN_DLC_BYTES_0:
        return 0U;
    case FDCAN_DLC_BYTES_1:
        return 1U;
    case FDCAN_DLC_BYTES_2:
        return 2U;
    case FDCAN_DLC_BYTES_3:
        return 3U;
    case FDCAN_DLC_BYTES_4:
        return 4U;
    case FDCAN_DLC_BYTES_5:
        return 5U;
    case FDCAN_DLC_BYTES_6:
        return 6U;
    case FDCAN_DLC_BYTES_7:
        return 7U;
    case FDCAN_DLC_BYTES_8:
        return 8U;
    case FDCAN_DLC_BYTES_12:
        return 12U;
    case FDCAN_DLC_BYTES_16:
        return 16U;
    case FDCAN_DLC_BYTES_20:
        return 20U;
    case FDCAN_DLC_BYTES_24:
        return 24U;
    case FDCAN_DLC_BYTES_32:
        return 32U;
    case FDCAN_DLC_BYTES_48:
        return 48U;
    default:
        return 64U;
    }
}

/* Pre-computed nominal bit timings for 80 MHz FDCAN kernel clock (PLL1Q). */
static void lookup_nominal_timing(uint32_t bitrate, uint32_t *brp, uint32_t *tseg1, uint32_t *tseg2,
                                  uint32_t *sjw)
{
    switch (bitrate) {
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
    case 500000U:
    default:
        /* 80 MHz / (10 * (1 + 13 + 2)) = 500 kbps @ 87.5% SP. */
        *brp = 10U;
        *tseg1 = 13U;
        *tseg2 = 2U;
        *sjw = 2U;
        break;
    }
}

/* Pre-computed data-phase bit timings for 80 MHz FDCAN kernel clock (PLL1Q). */
static void lookup_data_timing(uint32_t bitrate, uint32_t *brp, uint32_t *tseg1, uint32_t *tseg2,
                               uint32_t *sjw)
{
    switch (bitrate) {
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
    case 2000000U:
    default:
        /* 80 MHz / (2 * (1 + 15 + 4)) = 2.0 Mbps @ 80.0% SP, SJW = 4. */
        *brp = 2U;
        *tseg1 = 15U;
        *tseg2 = 4U;
        *sjw = 4U;
        break;
    }
}

static bool configure_rx_filters(FDCAN_HandleTypeDef *hfdcan, const can_node_fdcan_config_t *config)
{
    if (config->accept_all) {
        return HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0,
                                            FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE,
                                            FDCAN_FILTER_REMOTE) == HAL_OK;
    }

    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE,
                                     FDCAN_FILTER_REMOTE) != HAL_OK) {
        return false;
    }

    FDCAN_FilterTypeDef filter = {0};
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    if (config->extended_ids) {
        filter.IdType = FDCAN_EXTENDED_ID;
        filter.FilterID1 = config->filter_rx_id & CAN_ID_EXT_MAX;
        filter.FilterID2 = CAN_ID_EXT_MAX;
    } else {
        filter.IdType = FDCAN_STANDARD_ID;
        filter.FilterID1 = config->filter_rx_id & CAN_ID_STD_MAX;
        filter.FilterID2 = CAN_ID_STD_MAX;
    }

    return HAL_FDCAN_ConfigFilter(hfdcan, &filter) == HAL_OK;
}

static void pending_reset(can_node_fdcan_t *port)
{
    port->pending_head = 0U;
    port->pending_count = 0U;
}

static bool pending_push(can_node_fdcan_t *port, uint8_t marker, uint64_t token)
{
    if (port->pending_count >= CAN_NODE_FDCAN_PENDING_MAX) {
        return false;
    }
    const uint32_t tail = (port->pending_head + port->pending_count) % CAN_NODE_FDCAN_PENDING_MAX;
    port->pending[tail].marker = marker;
    port->pending[tail].token = token;
    port->pending_count++;
    return true;
}

static bool pending_pop_marker(can_node_fdcan_t *port, uint8_t marker, uint64_t *out_token)
{
    while (port->pending_count > 0U) {
        const can_node_fdcan_pending_t head = port->pending[port->pending_head];
        port->pending_head = (port->pending_head + 1U) % CAN_NODE_FDCAN_PENDING_MAX;
        port->pending_count--;

        if (head.marker == marker) {
            *out_token = head.token;
            return true;
        }
        port->stats.tx_marker_mismatch++;
    }
    return false;
}

bool can_node_fdcan_init(can_node_fdcan_t *port, FDCAN_HandleTypeDef *hfdcan,
                         const can_node_fdcan_config_t *config,
                         const can_node_fdcan_callbacks_t *callbacks)
{
    if ((port == NULL) || (hfdcan == NULL) || (hfdcan->Instance == NULL) || (config == NULL) ||
        (callbacks == NULL)) {
        return false;
    }

    (void)memset(port, 0, sizeof(*port));
    port->hfdcan = hfdcan;
    port->config = *config;
    port->callbacks = *callbacks;
    port->next_token = 1U;
    port->stats.min_tx_gap_us = UINT32_MAX;

    if (hfdcan->State == HAL_FDCAN_STATE_BUSY) {
        (void)HAL_FDCAN_Stop(hfdcan);
    }

    uint32_t nbrp = 0U;
    uint32_t nt1 = 0U;
    uint32_t nt2 = 0U;
    uint32_t nsjw = 0U;
    lookup_nominal_timing(config->nominal_bitrate, &nbrp, &nt1, &nt2, &nsjw);

    uint32_t dbrp = 0U;
    uint32_t dt1 = 0U;
    uint32_t dt2 = 0U;
    uint32_t dsjw = 0U;
    lookup_data_timing(config->data_bitrate, &dbrp, &dt1, &dt2, &dsjw);

    if (config->can_fd) {
        hfdcan->Init.FrameFormat = config->can_fd_brs ? FDCAN_FRAME_FD_BRS : FDCAN_FRAME_FD_NO_BRS;
    } else {
        hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    }
    hfdcan->Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan->Init.AutoRetransmission = ENABLE;
    hfdcan->Init.TransmitPause = DISABLE;
    hfdcan->Init.ProtocolException = DISABLE;
    hfdcan->Init.NominalPrescaler = nbrp;
    hfdcan->Init.NominalTimeSeg1 = nt1;
    hfdcan->Init.NominalTimeSeg2 = nt2;
    hfdcan->Init.NominalSyncJumpWidth = nsjw;
    hfdcan->Init.DataPrescaler = dbrp;
    hfdcan->Init.DataTimeSeg1 = dt1;
    hfdcan->Init.DataTimeSeg2 = dt2;
    hfdcan->Init.DataSyncJumpWidth = dsjw;

    /* Keep CubeMX MessageRAMOffset (0 for FDCAN1, 1280 for FDCAN2); total = 912 <= 1280 words. */
    hfdcan->Init.StdFiltersNbr = FDCAN_MSG_RAM_STD_MAX;
    hfdcan->Init.ExtFiltersNbr = FDCAN_MSG_RAM_EXT_MAX;
    hfdcan->Init.RxFifo0ElmtsNbr = FDCAN_MSG_RAM_RX0_MAX;
    hfdcan->Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
    hfdcan->Init.RxFifo1ElmtsNbr = 0U;
    hfdcan->Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan->Init.RxBuffersNbr = 0U;
    hfdcan->Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    hfdcan->Init.TxEventsNbr = CAN_NODE_FDCAN_PENDING_MAX;
    hfdcan->Init.TxBuffersNbr = 0U;
    hfdcan->Init.TxFifoQueueElmtsNbr = FDCAN_MSG_RAM_TX_MAX;
    hfdcan->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    hfdcan->Init.TxElmtSize = FDCAN_DATA_BYTES_64;

    if (HAL_FDCAN_Init(hfdcan) != HAL_OK) {
        return false;
    }

    if (!configure_rx_filters(hfdcan, config)) {
        return false;
    }

    if (HAL_FDCAN_EnableISOMode(hfdcan) != HAL_OK) {
        return false;
    }

    if ((hfdcan->Init.FrameFormat == FDCAN_FRAME_FD_BRS) && (hfdcan->Init.DataPrescaler <= 2U)) {
        const uint32_t tdco =
            clamp_u32(hfdcan->Init.DataPrescaler * hfdcan->Init.DataTimeSeg1, 1U, 0x7FU);
        (void)HAL_FDCAN_ConfigTxDelayCompensation(hfdcan, tdco, 0U);
        (void)HAL_FDCAN_EnableTxDelayCompensation(hfdcan);
    } else {
        (void)HAL_FDCAN_DisableTxDelayCompensation(hfdcan);
    }

    return HAL_FDCAN_Start(hfdcan) == HAL_OK;
}

bool can_node_fdcan_send(can_node_fdcan_t *port, const can_frame_t *frame, uint64_t *out_token)
{
    if ((port == NULL) || (port->hfdcan == NULL) || (frame == NULL) || (out_token == NULL)) {
        return false;
    }
    if ((port->hfdcan->State != HAL_FDCAN_STATE_BUSY) || !can_frame_is_valid(frame)) {
        return false;
    }
    if ((port->pending_count >= CAN_NODE_FDCAN_PENDING_MAX) ||
        (HAL_FDCAN_GetTxFifoFreeLevel(port->hfdcan) == 0U)) {
        port->stats.tx_fifo_full++;
        return false;
    }

    FDCAN_TxHeaderTypeDef hdr = {0};
    if ((frame->flags & CAN_FLAG_EXT) != 0U) {
        hdr.IdType = FDCAN_EXTENDED_ID;
        hdr.Identifier = frame->id & CAN_ID_EXT_MAX;
    } else {
        hdr.IdType = FDCAN_STANDARD_ID;
        hdr.Identifier = frame->id & CAN_ID_STD_MAX;
    }

    if ((frame->flags & CAN_FLAG_FD) != 0U) {
        hdr.FDFormat = FDCAN_FD_CAN;
        hdr.TxFrameType = FDCAN_DATA_FRAME;
        hdr.BitRateSwitch = ((frame->flags & CAN_FLAG_BRS) != 0U) ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    } else {
        hdr.FDFormat = FDCAN_CLASSIC_CAN;
        hdr.TxFrameType =
            ((frame->flags & CAN_FLAG_RTR) != 0U) ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
        hdr.BitRateSwitch = FDCAN_BRS_OFF;
    }

    const uint8_t marker = port->next_marker;
    port->next_marker = (uint8_t)((port->next_marker + 1U) & 0xFFU);

    hdr.DataLength = len_to_hal_dlc(frame->dlc);
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    hdr.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
    hdr.MessageMarker = (uint32_t)marker;

    uint8_t payload[CAN_FD_MAX_DL] = {0};
    if (frame->dlc > 0U) {
        (void)memcpy(payload, frame->data, frame->dlc);
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(port->hfdcan, &hdr, payload) != HAL_OK) {
        port->stats.tx_fifo_full++;
        return false;
    }

    const uint64_t token = port->next_token++;
    (void)pending_push(port, marker, token);
    port->stats.tx_queued++;
    *out_token = token;
    return true;
}

void can_node_fdcan_poll_tx_events(can_node_fdcan_t *port, uint64_t now_us)
{
    if ((port == NULL) || (port->hfdcan == NULL) || (port->hfdcan->Instance == NULL)) {
        return;
    }

    if (__HAL_FDCAN_GET_FLAG(port->hfdcan, FDCAN_FLAG_TX_EVT_FIFO_ELT_LOST) != 0U) {
        __HAL_FDCAN_CLEAR_FLAG(port->hfdcan, FDCAN_FLAG_TX_EVT_FIFO_ELT_LOST);
        port->stats.tx_events_lost++;
    }

    for (uint32_t n = 0U; n < FDCAN_POLL_BATCH_MAX; n++) {
        if ((port->hfdcan->Instance->TXEFS & FDCAN_TXEFS_EFFL) == 0U) {
            break;
        }

        FDCAN_TxEventFifoTypeDef evt = {0};
        if (HAL_FDCAN_GetTxEvent(port->hfdcan, &evt) != HAL_OK) {
            break;
        }

        if (evt.EventType == FDCAN_TX_EVENT) {
            uint64_t token = 0U;
            port->stats.tx_events++;

            if (port->have_tx_confirm && (now_us >= port->last_tx_confirm_us)) {
                const uint64_t gap = now_us - port->last_tx_confirm_us;
                const uint32_t gap32 = (gap > UINT32_MAX) ? UINT32_MAX : (uint32_t)gap;
                port->stats.last_tx_gap_us = gap32;
                if (gap32 < port->stats.min_tx_gap_us) {
                    port->stats.min_tx_gap_us = gap32;
                }
            }
            port->last_tx_confirm_us = now_us;
            port->have_tx_confirm = true;

            if (pending_pop_marker(port, (uint8_t)evt.MessageMarker, &token) &&
                (port->callbacks.on_tx_done != NULL)) {
                port->callbacks.on_tx_done(port->callbacks.ctx, token, now_us);
            }
        }
    }
}

void can_node_fdcan_poll_rx(can_node_fdcan_t *port, uint64_t now_us)
{
    if ((port == NULL) || (port->hfdcan == NULL)) {
        return;
    }

    if (__HAL_FDCAN_GET_FLAG(port->hfdcan, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST) != 0U) {
        __HAL_FDCAN_CLEAR_FLAG(port->hfdcan, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST);
        port->stats.rx_fifo_lost++;
    }

    for (uint32_t n = 0U; n < FDCAN_POLL_BATCH_MAX; n++) {
        if (HAL_FDCAN_GetRxFifoFillLevel(port->hfdcan, FDCAN_RX_FIFO0) == 0U) {
            break;
        }

        FDCAN_RxHeaderTypeDef hdr = {0};
        can_frame_t frame = {0};
        if (HAL_FDCAN_GetRxMessage(port->hfdcan, FDCAN_RX_FIFO0, &hdr, frame.data) != HAL_OK) {
            break;
        }

        uint8_t flags = 0U;
        frame.id = hdr.Identifier;
        frame.dlc = hal_dlc_to_len(hdr.DataLength);
        if (hdr.IdType == FDCAN_EXTENDED_ID) {
            flags = (uint8_t)(flags | CAN_FLAG_EXT);
        }
        if (hdr.FDFormat == FDCAN_FD_CAN) {
            flags = (uint8_t)(flags | CAN_FLAG_FD);
            if (hdr.BitRateSwitch == FDCAN_BRS_ON) {
                flags = (uint8_t)(flags | CAN_FLAG_BRS);
            }
        } else {
            if (hdr.RxFrameType == FDCAN_REMOTE_FRAME) {
                flags = (uint8_t)(flags | CAN_FLAG_RTR);
            }
        }
        frame.flags = flags;

        port->stats.rx_frames++;
        if (port->callbacks.on_rx_frame != NULL) {
            port->callbacks.on_rx_frame(port->callbacks.ctx, &frame, now_us);
        }
    }
}

void can_node_fdcan_poll_health(can_node_fdcan_t *port, uint64_t now_us)
{
    if ((port == NULL) || (port->hfdcan == NULL) || (port->hfdcan->Instance == NULL)) {
        return;
    }

    FDCAN_ProtocolStatusTypeDef psr = {0};
    FDCAN_ErrorCountersTypeDef ecr = {0};

    if (HAL_FDCAN_GetProtocolStatus(port->hfdcan, &psr) == HAL_OK) {
        port->stats.error_warning = (psr.Warning != 0U);
        port->stats.error_passive = (psr.ErrorPassive != 0U);
        port->stats.bus_off = (psr.BusOff != 0U);
        port->stats.tdc_value = (uint8_t)psr.TDCvalue;

        if (psr.BusOff != 0U) {
            if (!port->in_bus_off) {
                port->in_bus_off = true;
                port->bus_off_since_us = now_us;
            } else if ((now_us - port->bus_off_since_us) >= FDCAN_BUSOFF_RECOVER_US) {
                /* Start non-blocking ISO 11898-1 Bus-Off recovery (128 x 11 recessive bits). */
                pending_reset(port);
                (void)HAL_FDCAN_AbortTxRequest(port->hfdcan, 0xFFFFFFFFU);
                CLEAR_BIT(port->hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
                port->stats.bus_off_recoveries++;
                port->in_bus_off = false;
            } else {
                /* Waiting for hold-off window before clearing CCCR.INIT. */
            }
        } else {
            port->in_bus_off = false;
        }
    }

    if (HAL_FDCAN_GetErrorCounters(port->hfdcan, &ecr) == HAL_OK) {
        port->stats.tec = (uint8_t)ecr.TxErrorCnt;
        port->stats.rec = (uint8_t)ecr.RxErrorCnt;
    }
}

const can_node_fdcan_stats_t *can_node_fdcan_stats(const can_node_fdcan_t *port)
{
    return (port != NULL) ? &port->stats : NULL;
}
