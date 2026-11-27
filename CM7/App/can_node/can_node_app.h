/* SPDX-License-Identifier: MIT */
/**
 * @file
 * Top-level STM32H745 CM7 entrypoints for the leoembeddeder/can-node ISO-TP + UDS port.
 * Selects FDCAN1 (Option A) or FDCAN2 (Option B alongside PCAN on FDCAN1) at compile time,
 * drives hardware TX Event confirmations, RX polling, microsecond STmin/UDS deadlines, Bus-Off
 * recovery, LEDs (Option A), and idle-time ST-LINK VCP telemetry.
 */
#ifndef STM32_CAN_NODE_APP_H
#define STM32_CAN_NODE_APP_H

/** Initialize TIM2 microsecond IRQ, UART ring, FDCAN hardware port, UDS backend, and ISO-TP. */
void can_node_app_init(void);

/** Run one non-blocking poll cycle (drain TX Events, drain RX, advance ISO-TP/UDS, pump UART). */
void can_node_app_poll(void);

#endif /* STM32_CAN_NODE_APP_H */
