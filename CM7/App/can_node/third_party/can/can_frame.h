/* SPDX-License-Identifier: MIT */
/**
 * @file
 * A classical CAN or CAN FD frame as the rest of the project passes it around, and the two
 * properties of a frame that do not depend on any controller: whether it is well formed, and how
 * it ranks in arbitration.
 */
#ifndef FCN_CAN_FRAME_H
#define FCN_CAN_FRAME_H

#include <stdbool.h>
#include <stdint.h>

#define CAN_MAX_DLC    8U
#define CAN_FD_MAX_DL  64U
#define CAN_ID_STD_MAX 0x7FFU
#define CAN_ID_EXT_MAX 0x1FFFFFFFU

#define CAN_FLAG_EXT 0x01U /**< extended (29-bit) identifier */
#define CAN_FLAG_RTR 0x02U /**< remote transmission request: carries a DLC but no data */
#define CAN_FLAG_FD  0x04U /**< CAN FD frame (up to 64 data bytes; never with RTR) */
#define CAN_FLAG_BRS 0x08U /**< CAN FD bit rate switch (only valid with CAN_FLAG_FD) */

typedef struct {
    uint32_t id;   /**< identifier, right-aligned: 11 bits, or 29 bits with CAN_FLAG_EXT */
    uint8_t flags; /**< CAN_FLAG_* */
    /** payload byte length (CAN_DL): 0..8 classical; 0..8,12,16,20,24,32,48,64 with CAN_FLAG_FD */
    uint8_t dlc;
    uint8_t data[CAN_FD_MAX_DL]; /**< data bytes 0..dlc-1; the rest is ignored */
} can_frame_t;

/** True if the identifier, flags, and payload length are valid for classical CAN or CAN FD. */
bool can_frame_is_valid(const can_frame_t *frame);

/** True if `len` is a valid CAN FD payload length (0..8, 12, 16, 20, 24, 32, 48, 64). */
bool can_fd_len_is_valid(uint8_t len);

/** Round `len` (0..64) up to the smallest valid CAN FD payload length >= `len` (clamped to 64). */
uint8_t can_fd_pad_len(uint8_t len);

/** Convert a 4-bit CAN FD wire DLC (0..15) to byte length (0..64); >15 returns 64. */
uint8_t can_fd_dlc_to_len(uint8_t raw_dlc);

/** Convert a byte length (0..64) to a 4-bit CAN FD wire DLC (0..15), rounding up. */
uint8_t can_fd_len_to_dlc(uint8_t len);

/**
 * Rank of a frame in bitwise arbitration: of two frames that start at the same time, the one
 * with the SMALLER key wins the bus.
 *
 * The key is the arbitration field as it appears on the wire, where a dominant bit (0) beats
 * a recessive bit (1): the 11 base identifier bits, then RTR (base format) or SRR (extended,
 * always recessive), then IDE, then for extended frames the 18 identifier extension bits and
 * RTR. A base-format frame therefore beats an extended frame with the same base identifier,
 * and a data frame beats a remote frame with the same identifier.
 */
uint32_t can_frame_arbitration_key(const can_frame_t *frame);

#endif /* FCN_CAN_FRAME_H */
