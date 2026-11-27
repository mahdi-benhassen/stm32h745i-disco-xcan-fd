/* SPDX-License-Identifier: MIT */
#include "can_node_app.h"

#include <string.h>

#include "can_node_backend.h"
#include "can_node_config.h"
#include "can_node_diag.h"
#include "can_node_fdcan.h"
#include "can_node_time.h"
#include "can_node_uart.h"
#include "fdcan.h"
#include "main.h"

/* Place the 8 KB ISO-TP RX reassembly buffer in AXI-SRAM D1 (.ram_d1_bss). */
__attribute__((section(".ram_d1_bss"),
               aligned(32))) static uint8_t s_rx_buffer[CAN_NODE_ISOTP_RX_BUF_SIZE];

typedef struct {
    can_node_fdcan_t fdcan;
    can_node_backend_t backend;
    can_node_diag_t diag;
    uint64_t next_health_us;
    uint64_t next_telemetry_us;
    uint64_t activity_until_us;
    uint64_t next_heartbeat_us;
    uint32_t loop_max_us;
    bool initialized;
    bool heartbeat_on;
} can_node_app_t;

static can_node_app_t s_app;

static FDCAN_HandleTypeDef *selected_hfdcan(void)
{
#if CAN_NODE_SINGLE_BOARD
    return &hfdcan2;
#else
    return &hfdcan1;
#endif
}

static bool port_diag_send(void *ctx, const can_frame_t *frame, uint64_t *token)
{
    can_node_app_t *app = (can_node_app_t *)ctx;
    const bool sent = can_node_fdcan_send(&app->fdcan, frame, token);
    if (sent) {
        app->activity_until_us = can_node_time_us64() + 50000U;
    }
    return sent;
}

static void port_on_rx_frame(void *ctx, const can_frame_t *frame, uint64_t now_us)
{
    can_node_app_t *app = (can_node_app_t *)ctx;
    app->activity_until_us = now_us + 50000U;
    can_node_diag_on_frame(&app->diag, frame, now_us);
}

static void port_on_tx_done(void *ctx, uint64_t token, uint64_t now_us)
{
    can_node_app_t *app = (can_node_app_t *)ctx;
    can_node_diag_on_tx_done(&app->diag, token, now_us);
}

static void update_leds(can_node_app_t *app, uint64_t now_us)
{
#if !CAN_NODE_SINGLE_BOARD
    if (now_us >= app->next_heartbeat_us) {
        app->heartbeat_on = !app->heartbeat_on;
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,
                          app->heartbeat_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
        app->next_heartbeat_us = now_us + 500000U;
    }

    const bool active = (now_us < app->activity_until_us);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, active ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
    (void)app;
    (void)now_us;
#endif
}

static void emit_banner(void)
{
    can_node_uart_printf("\r\n[can-node] STM32H745I-DISCO ISO-TP CAN-FD + UDS Ready\r\n");
    can_node_uart_printf(
        "[can-node] mode=%s fdcan=%s nom=%lu data=%lu can_fd=%u brs=%u tx_dl=%u\r\n",
        (CAN_NODE_SINGLE_BOARD != 0U) ? "OptionB(SingleBoard)" : "OptionA(Standalone)",
        (CAN_NODE_SINGLE_BOARD != 0U) ? "FDCAN2" : "FDCAN1",
        (unsigned long)CAN_NODE_NOMINAL_BITRATE, (unsigned long)CAN_NODE_DATA_BITRATE,
        (unsigned)CAN_NODE_CAN_FD, (unsigned)CAN_NODE_CAN_FD_BRS, (unsigned)CAN_NODE_TX_DL);
    can_node_uart_printf(
        "[can-node] rx_id=0x%03lX tx_id=0x%03lX rx_buf=%u uds_req_max=%u slot=0x%08lX/%u\r\n",
        (unsigned long)CAN_NODE_RX_ID, (unsigned long)CAN_NODE_TX_ID,
        (unsigned)CAN_NODE_ISOTP_RX_BUF_SIZE, (unsigned)UDS_REQUEST_MAX,
        (unsigned long)CAN_NODE_DL_SLOT_ADDR, (unsigned)CAN_NODE_DL_SLOT_SIZE);
    can_node_uart_flush(50U);
}

static void emit_telemetry(const can_node_app_t *app)
{
    const can_node_fdcan_stats_t *fs = can_node_fdcan_stats(&app->fdcan);
    const can_node_diag_stats_t *ds = can_node_diag_stats(&app->diag);
    const isotp_stats_t *is = can_node_diag_isotp_stats(&app->diag);
    uds_stats_t us;

    if ((fs == NULL) || (ds == NULL) || (is == NULL)) {
        return;
    }

    can_node_diag_uds_stats(&app->diag, &us);
    can_node_uart_printf(
        "[can-node] sess=0x%02X loopMax=%luus rxF=%lu txEvt=%lu maxRx=%lu req=%lu rsp=%lu "
        "pend=%lu tec=%u rec=%u\r\n",
        (unsigned)can_node_diag_session(&app->diag), (unsigned long)app->loop_max_us,
        (unsigned long)fs->rx_frames, (unsigned long)fs->tx_events,
        (unsigned long)ds->max_rx_msg_len, (unsigned long)ds->requests,
        (unsigned long)ds->responses_sent, (unsigned long)us.response_pending, (unsigned)fs->tec,
        (unsigned)fs->rec);
}

void can_node_app_init(void)
{
    (void)memset(&s_app, 0, sizeof(s_app));

    can_node_time_init();
    can_node_uart_init();

    const can_node_fdcan_config_t fdcan_cfg = {
        .nominal_bitrate = CAN_NODE_NOMINAL_BITRATE,
        .data_bitrate = CAN_NODE_DATA_BITRATE,
        .filter_rx_id = CAN_NODE_RX_ID,
        .extended_ids = (CAN_NODE_EXTENDED_IDS != 0U),
        .can_fd = (CAN_NODE_CAN_FD != 0U),
        .can_fd_brs = (CAN_NODE_CAN_FD_BRS != 0U),
        .accept_all = false,
    };
    const can_node_fdcan_callbacks_t fdcan_cbs = {
        .ctx = &s_app,
        .on_rx_frame = port_on_rx_frame,
        .on_tx_done = port_on_tx_done,
    };

    uds_backend_t uds_cbs = {0};
    const can_node_diag_port_t diag_port = {
        .ctx = &s_app,
        .send = port_diag_send,
    };

    const bool ok = can_node_fdcan_init(&s_app.fdcan, selected_hfdcan(), &fdcan_cfg, &fdcan_cbs) &&
                    can_node_backend_init(&s_app.backend, &uds_cbs) &&
                    can_node_diag_init(&s_app.diag, &diag_port, &uds_cbs, s_rx_buffer,
                                       CAN_NODE_ISOTP_RX_BUF_SIZE);
    if (!ok) {
        Error_Handler();
    }

    const uint64_t now_us = can_node_time_us64();
    s_app.next_health_us = now_us + CAN_NODE_HEALTH_PERIOD_US;
    s_app.next_telemetry_us = now_us + CAN_NODE_TELEMETRY_PERIOD_US;
    s_app.next_heartbeat_us = now_us + 500000U;
    s_app.initialized = true;

    emit_banner();
}

void can_node_app_poll(void)
{
    if (!s_app.initialized) {
        return;
    }

    const uint64_t t_start = can_node_time_us64();

    /* 1. Hardware TX Event confirmations first (unlocks next STmin / pipeline slot). */
    can_node_fdcan_poll_tx_events(&s_app.fdcan, t_start);

    /* 2. Hardware RX FIFO0 -> ISO-TP -> UDS server. */
    can_node_fdcan_poll_rx(&s_app.fdcan, t_start);

    /* 3. Advance ISO-TP timers (STmin, N_Bs/N_Cr) and UDS timers (S3server, NRC 0x78). */
    can_node_diag_poll(&s_app.diag, t_start);

    /* 4. Execute requested hardReset cleanly after response confirmation (or 2 x N_As). */
    if (can_node_diag_reset_due(&s_app.diag)) {
        can_node_uart_write_str("[can-node] ECUReset confirmed -> NVIC_SystemReset()\r\n");
        can_node_uart_flush(20U);
        NVIC_SystemReset();
    }

    /* 5. Periodic FDCAN health / Bus-Off recovery and Option A board LEDs. */
    if (t_start >= s_app.next_health_us) {
        can_node_fdcan_poll_health(&s_app.fdcan, t_start);
        s_app.next_health_us = t_start + CAN_NODE_HEALTH_PERIOD_US;
    }
    update_leds(&s_app, t_start);

    /* 6. Idle-only periodic UART telemetry and single-byte TX FIFO pump. */
#if CAN_NODE_UART_TELEMETRY
    if ((t_start >= s_app.next_telemetry_us) && can_node_diag_is_idle(&s_app.diag)) {
        emit_telemetry(&s_app);
        s_app.next_telemetry_us = t_start + CAN_NODE_TELEMETRY_PERIOD_US;
    }
    can_node_uart_poll();
#endif

    const uint64_t t_end = can_node_time_us64();
    if (t_end >= t_start) {
        const uint64_t elapsed = t_end - t_start;
        const uint32_t elapsed32 = (elapsed > UINT32_MAX) ? UINT32_MAX : (uint32_t)elapsed;
        if (elapsed32 > s_app.loop_max_us) {
            s_app.loop_max_us = elapsed32;
        }
    }
}
