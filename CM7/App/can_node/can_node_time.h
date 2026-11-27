/* SPDX-License-Identifier: MIT */
/**
 * @file
 * Monotonic 64-bit microsecond clock built on CubeMX's 32-bit TIM2 (1.0 us/tick) plus
 * a hardware update interrupt (TIM2_IRQHandler) for race-free 32-to-64-bit extension.
 *
 * Unlike poll-only wrap detection, the IRQ-backed upper 32 bits remain monotonic even across
 * arbitrary idle gaps > 71.5 minutes, and can_node_time_us64() stays safe even if called from
 * multiple contexts.
 */
#ifndef STM32_CAN_NODE_TIME_H
#define STM32_CAN_NODE_TIME_H

#include <stdint.h>

/** Arm TIM2 overflow interrupt and start TIM2 if not already running (preserves CNT). */
void can_node_time_init(void);

/** Return a monotonic 64-bit microsecond timestamp (1.0 us resolution). */
uint64_t can_node_time_us64(void);

/** Return the raw 32-bit TIM2 microsecond counter. */
uint32_t can_node_time_us32(void);

#endif /* STM32_CAN_NODE_TIME_H */
