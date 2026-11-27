/* SPDX-License-Identifier: MIT */
/**
 * @file
 * Non-blocking USART3 debug telemetry ring buffer (ST-LINK VCP, 115200 8N1).
 *
 * Drains at most 1 byte per superloop tick directly into USART3->TDR when TXE is set,
 * keeping worst-case poll latency below 1 us so ISO-TP STmin = 100 us (0xF1) is never delayed
 * by UART output.
 */
#ifndef STM32_CAN_NODE_UART_H
#define STM32_CAN_NODE_UART_H

#include <stdint.h>

/** Initialize the non-blocking USART3 TX ring buffer. */
void can_node_uart_init(void);

/** Enqueue a NUL-terminated ASCII string into the TX ring (drops excess cleanly). */
void can_node_uart_write_str(const char *str);

/** Format and enqueue an ASCII string into the TX ring (up to 192 bytes per call). */
void can_node_uart_printf(const char *fmt, ...);

/** Drain at most one byte into USART3->TDR if TX FIFO/TDR has room. */
void can_node_uart_poll(void);

/** Drain buffered bytes synchronously up to `timeout_ms` (use only at boot or pre-reset). */
void can_node_uart_flush(uint32_t timeout_ms);

#endif /* STM32_CAN_NODE_UART_H */
