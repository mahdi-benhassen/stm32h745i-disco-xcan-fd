/* SPDX-License-Identifier: MIT */
#include "can_node_diag.h"

#include <stddef.h>
#include <string.h>

#include "can_node_config.h"

#define DIAG_READY_MARK 0x48374447U /* "H7DG" */

static bool diag_ready(const can_node_diag_t *d)
{
    return (d != NULL) && (d->ready == DIAG_READY_MARK);
}

static void forget_oldest_handout(can_node_diag_t *d)
{
    d->handout_head = (d->handout_head + 1U) % CAN_NODE_DIAG_HANDOUTS;
    d->handout_count--;
}

static void record_handout(can_node_diag_t *d, uint64_t token)
{
    if (d->handout_count == CAN_NODE_DIAG_HANDOUTS) {
        forget_oldest_handout(d);
        d->stats.confirms_forgotten++;
    }

    can_node_diag_handout_t *entry =
        &d->handouts[(d->handout_head + d->handout_count) % CAN_NODE_DIAG_HANDOUTS];
    entry->token = token;
    entry->handout_us = d->call_us;
    d->handout_count++;
}

static can_node_diag_response_t *free_response_slot(can_node_diag_t *d)
{
    can_node_diag_response_t *slot = NULL;
    if (d->response_count < CAN_NODE_DIAG_RESPONSES) {
        slot = &d->responses[(d->response_head + d->response_count) % CAN_NODE_DIAG_RESPONSES];
    }
    return slot;
}

static void take_uds_output(can_node_diag_t *d, can_node_diag_response_t *slot,
                            const uds_output_t *out)
{
    bool reset_now = false;

    if (out->len != 0U) {
        d->stats.responses++;
        if (slot != NULL) {
            slot->len = out->len;
            slot->reset = out->reset;
            d->response_count++;
        } else {
            d->stats.responses_dropped++;
            reset_now = out->reset;
        }
    } else {
        reset_now = out->reset;
    }

    if (out->reset) {
        d->stats.resets++;
        if (d->reset_deadline_us == UINT64_MAX) {
            d->reset_deadline_us = d->call_us + CAN_NODE_RESET_DEADLINE_US;
        }
    }

    if (reset_now) {
        d->reset_due = true;
    }
}

static void end_tx(can_node_diag_t *d)
{
    const can_node_diag_response_t *done = &d->responses[d->response_head];

    if (d->tx_result == ISOTP_RESULT_OK) {
        d->stats.responses_sent++;
    } else {
        d->stats.responses_failed++;
    }

    if (done->reset) {
        d->reset_due = true;
    }

    d->response_head = (d->response_head + 1U) % CAN_NODE_DIAG_RESPONSES;
    d->response_count--;
    d->sending = false;
    d->tx_ended = false;
}

static void settle(can_node_diag_t *d)
{
    if (d->call_us >= d->reset_deadline_us) {
        d->reset_due = true;
    }

    for (uint32_t i = 0U; i <= CAN_NODE_DIAG_RESPONSES; i++) {
        if (d->sending && d->tx_ended) {
            end_tx(d);
        }

        if (!d->sending && (d->response_count > 0U) && !d->reset_due) {
            const can_node_diag_response_t *next = &d->responses[d->response_head];
            if (isotp_send(&d->tp, next->data, next->len, d->call_us) == ISOTP_OK) {
                d->sending = true;
            } else {
                d->tx_result = ISOTP_RESULT_ERROR;
                d->sending = true;
                d->tx_ended = true;
            }
        }
    }
}

static bool on_send_frame(void *context, const can_frame_t *frame)
{
    can_node_diag_t *d = (can_node_diag_t *)context;
    uint64_t token = 0U;
    const bool taken = d->port.send(d->port.ctx, frame, &token);

    if (taken) {
        d->stats.frames_out++;
        record_handout(d, token);
    } else {
        d->stats.frames_refused++;
    }
    return taken;
}

static bool on_indication(void *context, isotp_result_t result, const uint8_t *data,
                          uint32_t length)
{
    can_node_diag_t *d = (can_node_diag_t *)context;

    d->stats.last_rx_msg_len = length;
    if (length > d->stats.max_rx_msg_len) {
        d->stats.max_rx_msg_len = length;
    }

    if (result == ISOTP_RESULT_OK) {
        can_node_diag_response_t *slot = free_response_slot(d);
        uds_output_t out = {.len = 0U, .reset = false};

        d->stats.requests++;
        (void)uds_request(&d->uds, data, length, d->call_us,
                          (slot != NULL) ? slot->data : d->scratch, UDS_RESPONSE_MAX, &out);
        take_uds_output(d, slot, &out);
    }

    return false;
}

static void on_confirm(void *context, isotp_result_t result)
{
    can_node_diag_t *d = (can_node_diag_t *)context;

    d->tx_result = result;
    d->tx_ended = true;
}

bool can_node_diag_init(can_node_diag_t *d, const can_node_diag_port_t *port,
                        const uds_backend_t *backend, uint8_t *rx_buf, uint32_t rx_buf_size)
{
    bool ok = (d != NULL) && (port != NULL) && (port->send != NULL) && (backend != NULL) &&
              (rx_buf != NULL) && (rx_buf_size >= UDS_REQUEST_MAX);

    if (ok) {
        const isotp_config_t config = {
            .tx_id = CAN_NODE_TX_ID,
            .rx_id = CAN_NODE_RX_ID,
            .n_as_us = CAN_NODE_ISOTP_TIMEOUT_US,
            .n_ar_us = CAN_NODE_ISOTP_TIMEOUT_US,
            .n_bs_us = CAN_NODE_ISOTP_TIMEOUT_US,
            .n_cr_us = CAN_NODE_ISOTP_TIMEOUT_US,
            .n_br_us = 0U,
            .n_cs_us = CAN_NODE_ISOTP_N_CS_US,
            .extended_ids = (CAN_NODE_EXTENDED_IDS != 0U),
            .can_fd = (CAN_NODE_CAN_FD != 0U),
            .can_fd_brs = (CAN_NODE_CAN_FD_BRS != 0U),
            .tx_dl = (uint8_t)CAN_NODE_TX_DL,
            .rx_bs = (uint8_t)CAN_NODE_ISOTP_BS,
            .rx_stmin = (uint8_t)CAN_NODE_ISOTP_STMIN,
            .rx_wft_max = 0U,
            .tx_wft_max = (uint8_t)CAN_NODE_ISOTP_TX_WFT_MAX,
        };
        const isotp_callbacks_t callbacks = {
            .send_frame = on_send_frame,
            .ff_indication = NULL,
            .indication = on_indication,
            .confirm = on_confirm,
            .context = d,
        };

        (void)memset(d, 0, sizeof(*d));
        d->port = *port;
        d->reset_deadline_us = UINT64_MAX;
        d->tx_result = ISOTP_RESULT_OK;

        ok = isotp_init(&d->tp, &config, &callbacks, rx_buf, rx_buf_size) &&
             uds_init(&d->uds, backend);
        if (ok) {
            d->ready = DIAG_READY_MARK;
        }
    }

    return ok;
}

void can_node_diag_on_frame(can_node_diag_t *d, const can_frame_t *frame, uint64_t now_us)
{
    if (diag_ready(d) && (frame != NULL)) {
        d->call_us = now_us;
        d->stats.frames_in++;
        (void)isotp_on_frame(&d->tp, frame, now_us);
        settle(d);
    }
}

void can_node_diag_on_tx_done(can_node_diag_t *d, uint64_t token, uint64_t now_us)
{
    if (diag_ready(d)) {
        d->call_us = now_us;
        d->stats.confirms++;

        while ((d->handout_count > 0U) && (d->handouts[d->handout_head].token < token)) {
            forget_oldest_handout(d);
            d->stats.confirms_forgotten++;
        }

        if ((d->handout_count > 0U) && (d->handouts[d->handout_head].token == token)) {
            const uint64_t handout_us = d->handouts[d->handout_head].handout_us;
            forget_oldest_handout(d);

            if (now_us < (handout_us + CAN_NODE_ISOTP_TIMEOUT_US)) {
                (void)isotp_on_tx_done(&d->tp, now_us);
                d->stats.confirms_passed++;
            } else {
                d->stats.confirms_late++;
            }
        } else {
            d->stats.confirms_unknown++;
        }

        settle(d);
    }
}

void can_node_diag_poll(can_node_diag_t *d, uint64_t now_us)
{
    if (diag_ready(d)) {
        can_node_diag_response_t *slot = NULL;
        uds_output_t out = {.len = 0U, .reset = false};

        d->call_us = now_us;
        isotp_poll(&d->tp, now_us);
        settle(d);

        slot = free_response_slot(d);
        (void)uds_poll(&d->uds, now_us, (slot != NULL) ? slot->data : d->scratch, UDS_RESPONSE_MAX,
                       &out);
        take_uds_output(d, slot, &out);
        settle(d);
    }
}

uint64_t can_node_diag_next_deadline(const can_node_diag_t *d)
{
    uint64_t deadline = UINT64_MAX;
    if (diag_ready(d)) {
        const uint64_t tp_dl = isotp_next_deadline(&d->tp);
        const uint64_t uds_dl = uds_next_deadline(&d->uds);
        const uint64_t rst_dl = d->reset_due ? UINT64_MAX : d->reset_deadline_us;

        deadline = (tp_dl < uds_dl) ? tp_dl : uds_dl;
        deadline = (rst_dl < deadline) ? rst_dl : deadline;
    }
    return deadline;
}

bool can_node_diag_reset_due(const can_node_diag_t *d)
{
    return diag_ready(d) && d->reset_due;
}

bool can_node_diag_is_idle(const can_node_diag_t *d)
{
    return diag_ready(d) && !isotp_tx_busy(&d->tp) && !isotp_rx_busy(&d->tp) && !uds_busy(&d->uds);
}

const can_node_diag_stats_t *can_node_diag_stats(const can_node_diag_t *d)
{
    return diag_ready(d) ? &d->stats : NULL;
}

const isotp_stats_t *can_node_diag_isotp_stats(const can_node_diag_t *d)
{
    return diag_ready(d) ? isotp_stats(&d->tp) : NULL;
}

void can_node_diag_uds_stats(const can_node_diag_t *d, uds_stats_t *out)
{
    if (out != NULL) {
        if (diag_ready(d)) {
            uds_get_stats(&d->uds, out);
        } else {
            (void)memset(out, 0, sizeof(*out));
        }
    }
}

uint8_t can_node_diag_session(const can_node_diag_t *d)
{
    return diag_ready(d) ? uds_session(&d->uds) : (uint8_t)UDS_SESSION_DEFAULT;
}
