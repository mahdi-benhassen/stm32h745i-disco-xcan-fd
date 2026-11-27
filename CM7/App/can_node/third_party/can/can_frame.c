/* SPDX-License-Identifier: MIT */
#include "can/can_frame.h"

#include <stddef.h>

/* The flags this code knows how to represent. A frame with any other bit set is rejected rather
 * than accepted with that bit ignored. */
#define FRAME_KNOWN_FLAGS (CAN_FLAG_EXT | CAN_FLAG_RTR | CAN_FLAG_FD | CAN_FLAG_BRS)

#define CAN_FD_DLC_COUNT 16U

/* ISO 11898-1:2015 Table of DLC (0..15) to CAN FD payload byte length (CAN_DL). */
static const uint8_t s_can_fd_dlc_len[CAN_FD_DLC_COUNT] = {
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U,
};

/* A 29-bit identifier is transmitted in two parts (Bosch CAN Specification 2.0, Part B,
 * section 3.2.1): its upper 11 bits ID-28..ID-18, the base identifier, and its lower 18 bits
 * ID-17..ID-0, the identifier extension. */
#define FRAME_ID_EXTENSION_BITS 18U
#define FRAME_ID_EXTENSION_MASK 0x3FFFFU

/*
 * The arbitration key holds the bits that decide arbitration in the order in which they are
 * transmitted, the first one in bit 31:
 *
 *     key bit     31 ........ 21    20     19    18 ........ 1     0
 *     base        ID-28 .. ID-18    RTR    IDE     0 ....... 0     0
 *     extended    ID-28 .. ID-18    SRR    IDE   ID-17 .. ID-0    RTR
 *
 * SRR is always 1; IDE is 0 in a base frame and 1 in an extended frame; RTR is 1 in a remote
 * frame. On the bus a dominant 0 overwrites a recessive 1 and the node that sent the 1 backs
 * off, so of two frames the one whose key is the smaller unsigned number keeps the bus:
 * comparing two keys is the bitwise arbitration, first bit first.
 *
 * The arbitration field of a base frame ends with RTR; its IDE bit already belongs to the
 * control field. IDE is part of the key all the same, because it is the bit at which a base
 * remote frame wins against an extended frame with the same base identifier: RTR and SRR are
 * both recessive there. Nothing after IDE separates two base frames, so the remaining key
 * bits of a base frame are 0 and frames with the same arbitration field have the same key.
 */
#define KEY_BASE_ID_SHIFT   21U
#define KEY_RTR_OR_SRR      0x00100000U
#define KEY_IDE             0x00080000U
#define KEY_EXTENSION_SHIFT 1U
#define KEY_EXTENDED_RTR    0x00000001U

bool can_fd_len_is_valid(uint8_t len)
{
    return (len <= CAN_FD_MAX_DL) && (can_fd_pad_len(len) == len);
}

uint8_t can_fd_pad_len(uint8_t len)
{
    uint8_t padded = CAN_FD_MAX_DL;
    bool found = false;

    for (uint8_t i = 0U; (i < CAN_FD_DLC_COUNT) && !found; i++) {
        if (s_can_fd_dlc_len[i] >= len) {
            padded = s_can_fd_dlc_len[i];
            found = true;
        }
    }

    return padded;
}

uint8_t can_fd_dlc_to_len(uint8_t raw_dlc)
{
    const uint8_t idx = (raw_dlc < CAN_FD_DLC_COUNT) ? raw_dlc : (uint8_t)(CAN_FD_DLC_COUNT - 1U);

    return s_can_fd_dlc_len[idx];
}

uint8_t can_fd_len_to_dlc(uint8_t len)
{
    uint8_t dlc = (uint8_t)(CAN_FD_DLC_COUNT - 1U);
    bool found = false;

    for (uint8_t i = 0U; (i < CAN_FD_DLC_COUNT) && !found; i++) {
        if (s_can_fd_dlc_len[i] >= len) {
            dlc = i;
            found = true;
        }
    }

    return dlc;
}

bool can_frame_is_valid(const can_frame_t *frame)
{
    bool valid = false;

    if (frame != NULL) {
        /* "Fits its format" is taken literally: 11 or 29 bits. The Bosch specification also
         * says that the seven most significant identifier bits must not all be recessive,
         * which would exclude the base identifiers 0x7F0 to 0x7FF. The header does not ask
         * for that (CAN_ID_STD_MAX is 0x7FF), and neither controllers nor Linux SocketCAN
         * refuse those identifiers, so they are accepted here. */
        const bool extended = (frame->flags & CAN_FLAG_EXT) != 0U;
        const bool is_fd = (frame->flags & CAN_FLAG_FD) != 0U;
        const bool is_rtr = (frame->flags & CAN_FLAG_RTR) != 0U;
        const bool is_brs = (frame->flags & CAN_FLAG_BRS) != 0U;
        const uint32_t id_max = extended ? CAN_ID_EXT_MAX : CAN_ID_STD_MAX;

        if ((frame->id <= id_max) && ((frame->flags & ~FRAME_KNOWN_FLAGS) == 0U)) {
            if (is_fd) {
                valid = (!is_rtr) && can_fd_len_is_valid(frame->dlc);
            } else {
                valid = (!is_brs) && (frame->dlc <= CAN_MAX_DLC);
            }
        }
    }

    return valid;
}

uint32_t can_frame_arbitration_key(const can_frame_t *frame)
{
    /* The header leaves the key of an invalid frame open. Here an identifier that is wider
     * than its format is cut down to it, so that a stray high bit cannot spill into a
     * neighbouring field of the key; flags other than EXT and RTR are not looked at. `frame`
     * must not be NULL: a key cannot report an error. */
    const bool extended = (frame->flags & CAN_FLAG_EXT) != 0U;
    const bool remote = (frame->flags & CAN_FLAG_RTR) != 0U;
    uint32_t key;

    if (extended) {
        const uint32_t base_id = (frame->id >> FRAME_ID_EXTENSION_BITS) & CAN_ID_STD_MAX;
        const uint32_t extension = frame->id & FRAME_ID_EXTENSION_MASK;

        key = (base_id << KEY_BASE_ID_SHIFT) | KEY_RTR_OR_SRR | KEY_IDE |
              (extension << KEY_EXTENSION_SHIFT);

        if (remote) {
            key |= KEY_EXTENDED_RTR;
        }
    } else {
        key = (frame->id & CAN_ID_STD_MAX) << KEY_BASE_ID_SHIFT;

        if (remote) {
            key |= KEY_RTR_OR_SRR;
        }
    }

    return key;
}
