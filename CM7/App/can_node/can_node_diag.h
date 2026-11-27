/* SPDX-License-Identifier: MIT */
/**
 * @file
 * STM32H7 diagnostic coordinator binding leoembeddeder/can-node's lib/isotp (ISO 15765-2:2016)
 * and lib/uds (ISO 14229-1:2020) with an 8192-byte RX reassembly buffer, double-buffered UDS
 * responses, N_As confirmation timeout filtering, and ECUReset confirmation gating.
 */
#ifndef STM32_CAN_NODE_DIAG_H
#define STM32_CAN_NODE_DIAG_H

#include <stdbool.h>
#include <stdint.h>

#include "can/can_frame.h"
#include "isotp/isotp.h"
#include "uds/uds.h"

#define CAN_NODE_DIAG_RESPONSES 2U
#define CAN_NODE_DIAG_HANDOUTS  (2U * ISOTP_PIPELINE_DEPTH)

typedef struct {
    void *ctx;
    bool (*send)(void *ctx, const can_frame_t *frame, uint64_t *token);
} can_node_diag_port_t;

typedef struct {
    uint32_t frames_in;
    uint32_t frames_out;
    uint32_t frames_refused;
    uint32_t confirms;
    uint32_t confirms_passed;
    uint32_t confirms_late;
    uint32_t confirms_unknown;
    uint32_t confirms_forgotten;
    uint32_t requests;
    uint32_t max_rx_msg_len;
    uint32_t last_rx_msg_len;
    uint32_t responses;
    uint32_t responses_sent;
    uint32_t responses_failed;
    uint32_t responses_dropped;
    uint32_t resets;
} can_node_diag_stats_t;

typedef struct {
    uint8_t data[UDS_RESPONSE_MAX];
    uint32_t len;
    bool reset;
} can_node_diag_response_t;

typedef struct {
    uint64_t token;
    uint64_t handout_us;
} can_node_diag_handout_t;

typedef struct {
    can_node_diag_port_t port;
    isotp_t tp;
    uds_server_t uds;
    can_node_diag_response_t responses[CAN_NODE_DIAG_RESPONSES];
    uint8_t scratch[UDS_RESPONSE_MAX];
    can_node_diag_handout_t handouts[CAN_NODE_DIAG_HANDOUTS];
    uint64_t call_us;
    uint64_t reset_deadline_us;
    uint32_t response_head;
    uint32_t response_count;
    uint32_t handout_head;
    uint32_t handout_count;
    uint32_t ready;
    isotp_result_t tx_result;
    bool sending;
    bool tx_ended;
    bool reset_due;
    can_node_diag_stats_t stats;
} can_node_diag_t;

/** Initialize ISO-TP and UDS server with caller-provided `rx_buf` (`rx_buf_size` >= UDS_REQUEST_MAX). */
bool can_node_diag_init(can_node_diag_t *d, const can_node_diag_port_t *port,
                        const uds_backend_t *backend, uint8_t *rx_buf, uint32_t rx_buf_size);

/** Feed a received CAN/CAN-FD frame into ISO-TP -> UDS at timestamp `now_us`. */
void can_node_diag_on_frame(can_node_diag_t *d, const can_frame_t *frame, uint64_t now_us);

/** Feed a hardware TX Event confirmation token into ISO-TP at timestamp `now_us`. */
void can_node_diag_on_tx_done(can_node_diag_t *d, uint64_t token, uint64_t now_us);

/** Advance ISO-TP STmin/timeouts, UDS S3server/ResponsePending timers, and response queue. */
void can_node_diag_poll(can_node_diag_t *d, uint64_t now_us);

/** Earliest microsecond deadline at which can_node_diag_poll() has time-driven work. */
uint64_t can_node_diag_next_deadline(const can_node_diag_t *d);

/** True once an accepted ECUReset positive response has finished sending (or hit 2 x N_As). */
bool can_node_diag_reset_due(const can_node_diag_t *d);

/** True when neither ISO-TP TX nor ISO-TP RX is mid-transaction (safe for telemetry print). */
bool can_node_diag_is_idle(const can_node_diag_t *d);

const can_node_diag_stats_t *can_node_diag_stats(const can_node_diag_t *d);
const isotp_stats_t *can_node_diag_isotp_stats(const can_node_diag_t *d);
void can_node_diag_uds_stats(const can_node_diag_t *d, uds_stats_t *out);
uint8_t can_node_diag_session(const can_node_diag_t *d);

#endif /* STM32_CAN_NODE_DIAG_H */
