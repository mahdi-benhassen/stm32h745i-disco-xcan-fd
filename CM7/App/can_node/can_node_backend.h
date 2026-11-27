/* SPDX-License-Identifier: MIT */
/**
 * @file
 * STM32H745 verification backend for leoembeddeder/can-node's lib/uds (all 9 callbacks):
 *   - 64 KB RAM_D1 virtual download slot (0x00010000, size <= 65536) with async 60 ms erase
 *     (exercising NRC 0x78 ResponsePending at 40 ms), CRC-32/ISO-HDLC check, and trial/confirm
 *     state transitions.
 *   - 32 automotive DTCs (DTC_EVENTS_MAX = 32U) pre-seeded across confirmed, pending, and
 *     passed states so 0x19 01/02/0A exercise FD SF (CAN_DL 12..48) and Multi-Frame (131 B).
 *   - Boot info DIDs (0xF189 / 0xFD00) and ECUReset (0x11 01).
 */
#ifndef STM32_CAN_NODE_BACKEND_H
#define STM32_CAN_NODE_BACKEND_H

#include <stdbool.h>
#include <stdint.h>

#include "dtc/dtc.h"
#include "uds/uds.h"

typedef struct {
    dtc_state_t dtc;
    uint32_t erase_size;
    uint32_t written_bytes;
    uint32_t running_crc;
    uint32_t last_crc32;
    uint32_t last_dl_size;
    uint32_t erase_start_us;
    uint32_t download_count;
    uint8_t confirm_state;
    uint8_t active_slot;
    bool erasing;
    bool slot_erased;
    bool reset_requested;
} can_node_backend_t;

/** Initialize virtual slot, seed 32 DTCs, and populate all 9 callbacks in `out_cbs`. */
bool can_node_backend_init(can_node_backend_t *be, uds_backend_t *out_cbs);

#endif /* STM32_CAN_NODE_BACKEND_H */
