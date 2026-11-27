/* SPDX-License-Identifier: MIT */
/**
 * @file
 * STM32H7 HAL FDCAN adapter bridging FDCAN1/FDCAN2 hardware, TX Event FIFO hardware
 * confirmations, Transceiver Delay Compensation (TDC), acceptance filters, and Bus-Off
 * recovery to leoembeddeder/can-node's can_frame_t interface.
 */
#ifndef STM32_CAN_NODE_FDCAN_H
#define STM32_CAN_NODE_FDCAN_H

#include <stdbool.h>
#include <stdint.h>

#include "can/can_frame.h"
#include "fdcan.h"

/** Capacity of the software pending TX confirmation tracker (matches FDCAN TX Event FIFO). */
#define CAN_NODE_FDCAN_PENDING_MAX 16U

typedef struct {
    uint32_t nominal_bitrate; /**< Nominal/arbitration bitrate in bps (e.g. 500000U) */
    uint32_t data_bitrate;    /**< Data-phase bitrate in bps for CAN-FD BRS (e.g. 2000000U) */
    uint32_t filter_rx_id;    /**< Accepted UDS physical/diagnostic CAN ID */
    bool extended_ids;        /**< true: 29-bit extended filter, false: 11-bit standard */
    bool can_fd;              /**< true: FDCAN_FRAME_FD_BRS / FDCAN_FRAME_FD_NO_BRS */
    bool can_fd_brs;          /**< true: enable BRS + TDC (requires can_fd) */
    bool accept_all;          /**< true: bypass ID filter and accept all non-remote frames */
} can_node_fdcan_config_t;

typedef struct {
    void *ctx;
    void (*on_rx_frame)(void *ctx, const can_frame_t *frame, uint64_t now_us);
    void (*on_tx_done)(void *ctx, uint64_t token, uint64_t now_us);
} can_node_fdcan_callbacks_t;

typedef struct {
    uint32_t tx_queued;
    uint32_t tx_fifo_full;
    uint32_t tx_events;
    uint32_t tx_events_lost;
    uint32_t tx_marker_mismatch;
    uint32_t rx_frames;
    uint32_t rx_fifo_lost;
    uint32_t bus_off_recoveries;
    uint32_t last_tx_gap_us;
    uint32_t min_tx_gap_us;
    uint8_t tec;
    uint8_t rec;
    uint8_t tdc_value;
    bool error_warning;
    bool error_passive;
    bool bus_off;
} can_node_fdcan_stats_t;

typedef struct {
    uint8_t marker;
    uint64_t token;
} can_node_fdcan_pending_t;

typedef struct {
    FDCAN_HandleTypeDef *hfdcan;
    can_node_fdcan_config_t config;
    can_node_fdcan_callbacks_t callbacks;
    can_node_fdcan_pending_t pending[CAN_NODE_FDCAN_PENDING_MAX];
    uint32_t pending_head;
    uint32_t pending_count;
    uint64_t next_token;
    uint8_t next_marker;
    uint64_t last_tx_confirm_us;
    bool have_tx_confirm;
    uint64_t bus_off_since_us;
    bool in_bus_off;
    can_node_fdcan_stats_t stats;
} can_node_fdcan_t;

/** Configure Message RAM (with TX Event FIFO), bitrates, TDC, filters, and start FDCAN. */
bool can_node_fdcan_init(can_node_fdcan_t *port, FDCAN_HandleTypeDef *hfdcan,
                         const can_node_fdcan_config_t *config,
                         const can_node_fdcan_callbacks_t *callbacks);

/** Queue `frame` into the hardware TX FIFO with TX Event tracking; outputs unique `out_token`. */
bool can_node_fdcan_send(can_node_fdcan_t *port, const can_frame_t *frame, uint64_t *out_token);

/** Drain hardware TX Event FIFO and dispatch matched confirmations at `now_us`. */
void can_node_fdcan_poll_tx_events(can_node_fdcan_t *port, uint64_t now_us);

/** Drain hardware RX FIFO0 and dispatch received frames at `now_us`. */
void can_node_fdcan_poll_rx(can_node_fdcan_t *port, uint64_t now_us);

/** Poll protocol error counters/status and run non-blocking Bus-Off recovery. */
void can_node_fdcan_poll_health(can_node_fdcan_t *port, uint64_t now_us);

/** Return live FDCAN port diagnostics and counters. */
const can_node_fdcan_stats_t *can_node_fdcan_stats(const can_node_fdcan_t *port);

#endif /* STM32_CAN_NODE_FDCAN_H */
