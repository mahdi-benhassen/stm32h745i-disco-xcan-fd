/* SPDX-License-Identifier: MIT */
/**
 * @file
 * ISO 15765-2:2016 (ISO-TP, DoCAN transport layer) for one connection on classical CAN and
 * CAN FD: messages of 1 to UINT32_MAX bytes cut into frames of TX_DL (8..64) bytes and put
 * together again, with flow control and the protocol's timers. Pure logic with time passed in:
 * no operating system, no clock of its own, no allocation; frames leave through a callback, and
 * the caller reports frames received and frames confirmed sent.
 *
 * WHAT IT IS. One isotp_t is one connection between two nodes: frames it sends carry
 * config.tx_id, frames it accepts carry config.rx_id (on the node 0x7E8 and 0x7E0). It is a
 * sender and a receiver at the same time, and the two are independent, as in the standard: a
 * message can be received while one is being sent. Normal addressing (the whole first byte of
 * every frame is protocol control information, the N_PCI), physical addressing, classical CAN
 * (CAN_DL 8) and CAN FD (TX_DL 8, 12, 16, 20, 24, 32, 48, 64), and 32-bit FF_DL escape
 * sequences (> 4095 bytes).
 *
 * NOT IN THIS MODULE. Extended and mixed addressing (an address byte before the N_PCI),
 * functional addressing (one request to many receivers, single frames only), and unpadded
 * classical CAN transmission (every classical frame sent is padded to 8 bytes with ISOTP_PADDING;
 * CAN FD SF > 7 and the last CF are padded to the smallest valid discrete CAN_DL per Clause
 * 10.4.2.3).
 *
 * FRAMES (ISO 15765-2:2016 Table 9, normal addressing; "d" is message data, "p" the padding
 * byte ISOTP_PADDING, 0xCC):
 *
 *   SingleFrame  SF (CAN_DL <= 8)    byte 0: 0x0L (L = 1..7), bytes 1..7: d or p
 *   SingleFrame  SF (CAN_DL > 8)     byte 0: 0x00, byte 1: SF_DL (8..62), bytes 2..CAN_DL-1: d/p
 *   FirstFrame   FF (FF_DL <= 4095)  byte 0: 0x1H, byte 1: LL, bytes 2..TX_DL-1: d
 *   FirstFrame   FF (FF_DL > 4095)   byte 0: 0x10, byte 1: 0x00, bytes 2..5: FF_DL (BE32),
 *                                    bytes 6..TX_DL-1: d
 *   ConsecutiveFrame CF              byte 0: 0x2N (N = SN 0..15), bytes 1..TX_DL-1: d or p
 *   FlowControl  FC                  byte 0: 0x3S (S = FS 0..2), byte 1: BS, byte 2: STmin,
 *                                    bytes 3..7: p
 *
 *   Note (Table 12 & Table 13): SF_DL 1..7 always uses the 1-byte PCI with CAN_DL = 8 even when
 *   CAN FD is enabled; SF_DL >= 8 uses the 2-byte escape PCI padded to the smallest valid CAN FD
 *   length {12, 16, 20, 24, 32, 48, 64} covering (2 + SF_DL). Table 14 sets FF_DLmin to 8 when
 *   CAN_DL = 8 and (RX_DL - 1) / (TX_DL - 1) when CAN_DL > 8.
 *
 * RECEIVED FRAMES ARE CHECKED before they are acted on, and a frame that fails a check is
 * ignored: it changes nothing but a counter of isotp_stats_t.ignored (isotp_ignored_t says
 * which; the call still acts on the timers of its time):
 *   - not for this connection: invalid CAN/CAN FD frame, another identifier or format, a remote
 *     frame, or a CAN FD frame on a classical-CAN instance;
 *   - shorter (or mismatched CAN FD length) than its N_PCI needs;
 *   - a length the protocol does not allow (Table 12, Table 13, Table 14, escape FF <= 4095);
 *   - a reserved N_PCI type (0x4..0xF);
 *   - a CF that no reception expects now, an FC that no transmission waits for.
 */
#ifndef FCN_ISOTP_H
#define FCN_ISOTP_H

#include <stdbool.h>
#include <stdint.h>

#include "can/can_frame.h"

/** Longest message using the 12-bit FirstFrame length field. */
#define ISOTP_FF_DL_12BIT_MAX 4095U

/** Maximum message length supported by the 32-bit escape FirstFrame (UINT32_MAX). */
#define ISOTP_MAX_LENGTH UINT32_MAX

/** Classical CAN (CAN_DL = 8) payload capacities per frame type (normal addressing). */
#define ISOTP_SF_DATA 7U
#define ISOTP_FF_DATA 6U
#define ISOTP_CF_DATA 7U

/** CAN FD (64-byte) maximum SingleFrame and 12-bit FirstFrame payload capacities. */
#define ISOTP_FD_MAX_SF_DATA (CAN_FD_MAX_DL - 2U)
#define ISOTP_FD_MAX_FF_DATA (CAN_FD_MAX_DL - 2U)

/** The byte that fills unused trailing bytes up to the target CAN_DL. */
#define ISOTP_PADDING 0xCCU

/** The standard's value of the timeouts N_As, N_Ar, N_Bs and N_Cr: 1000 ms. */
#define ISOTP_TIMEOUT_US 1000000U

/** The STmin a sender uses instead of a reserved value: 0x7F, 127 ms. */
#define ISOTP_STMIN_RESERVED_US 127000U

/** Frames handed out and not yet confirmed, at most. */
#define ISOTP_PIPELINE_DEPTH 4U

/** Returned by isotp_next_deadline() when nothing is timed. */
#define ISOTP_NO_DEADLINE UINT64_MAX

/** The outcome of a transmission or a reception: the standard's N_Result. */
typedef enum {
    ISOTP_RESULT_OK = 0,       /**< N_OK: the message went through */
    ISOTP_RESULT_TIMEOUT_A,    /**< N_TIMEOUT_A: N_As (sender) or N_Ar (receiver) ran out */
    ISOTP_RESULT_TIMEOUT_BS,   /**< N_TIMEOUT_Bs: sender, no FC in time */
    ISOTP_RESULT_TIMEOUT_CR,   /**< N_TIMEOUT_Cr: receiver, no CF in time */
    ISOTP_RESULT_WRONG_SN,     /**< N_WRONG_SN: receiver, a CF with an unexpected SN */
    ISOTP_RESULT_INVALID_FS,   /**< N_INVALID_FS: sender, an FC with a reserved FS */
    ISOTP_RESULT_UNEXP_PDU,    /**< N_UNEXP_PDU: receiver, an SF or FF during a reception */
    ISOTP_RESULT_WFT_OVRN,     /**< N_WFT_OVRN: receiver, N_WFTmax WAITs were not enough;
                                    sender, more than tx_wft_max WAITs (an extension) */
    ISOTP_RESULT_BUFFER_OVFLW, /**< N_BUFFER_OVFLW: sender, the receiver answered OVFLW */
    ISOTP_RESULT_ERROR,        /**< N_ERROR: the send_frame callback refused a frame */
    ISOTP_RESULTS              /**< the number of results, not a result */
} isotp_result_t;

/** Kinds of frame, for the counters of frames in and out. */
typedef enum {
    ISOTP_FRAME_SF = 0,
    ISOTP_FRAME_FF, /**< escape FFs included */
    ISOTP_FRAME_CF,
    ISOTP_FRAME_FC_CTS,
    ISOTP_FRAME_FC_WAIT,
    ISOTP_FRAME_FC_OVFLW,
    ISOTP_FRAME_FC_RESERVED, /**< an FC with a reserved FS: received only, never sent */
    ISOTP_FRAME_KINDS        /**< the number of kinds, not a kind */
} isotp_frame_kind_t;

/** Why a received frame was ignored. */
typedef enum {
    ISOTP_IGNORED_FOREIGN = 0,   /**< invalid frame, wrong id/format/mode, or remote frame */
    ISOTP_IGNORED_SHORT,         /**< frame length below or mismatched for what the N_PCI needs */
    ISOTP_IGNORED_LENGTH,        /**< invalid SF_DL/CAN_DL pair (Table 12/13) or FF_DL (Table 14) */
    ISOTP_IGNORED_RESERVED_TYPE, /**< N_PCI type 0x4..0xF */
    ISOTP_IGNORED_UNEXPECTED_CF, /**< a CF no reception expects now */
    ISOTP_IGNORED_UNEXPECTED_FC, /**< an FC no transmission waits for */
    ISOTP_IGNORED_REASONS        /**< the number of reasons, not a reason */
} isotp_ignored_t;

/** What isotp_send() answers. */
typedef enum {
    ISOTP_OK = 0, /**< the message was taken; its confirmation follows through `confirm` */
    ISOTP_E_ARG,  /**< a NULL pointer, length 0, or an uninitialised instance */
    ISOTP_E_BUSY  /**< a transmission is in progress, or the call came from a callback */
} isotp_status_t;

/**
 * The parameters of a connection. isotp_init() refuses a configuration with a value outside the
 * ranges given here.
 */
typedef struct {
    uint32_t tx_id;   /**< identifier of the frames sent: 0..0x7FF, or 0..0x1FFFFFFF if extended */
    uint32_t rx_id;   /**< identifier of the frames accepted; same range, not tx_id */
    uint32_t n_as_us; /**< N_As, 1..UINT32_MAX us; the standard: ISOTP_TIMEOUT_US */
    uint32_t n_ar_us; /**< N_Ar, 1..UINT32_MAX us; the standard: ISOTP_TIMEOUT_US */
    uint32_t n_bs_us; /**< N_Bs, 1..UINT32_MAX us; the standard: ISOTP_TIMEOUT_US */
    uint32_t n_cr_us; /**< N_Cr, 1..UINT32_MAX us; the standard: ISOTP_TIMEOUT_US */
    /** N_Br: the longest the receiver waits for a kept buffer before it sends an FC WAIT, and
     *  the time between WAITs, 0..UINT32_MAX us. 0: no wait. */
    uint32_t n_br_us;
    /** N_Cs performance target, any value: CFs handed out later are counted. */
    uint32_t n_cs_us;
    bool extended_ids; /**< both identifiers have 29 bits (CAN_FLAG_EXT) */
    bool can_fd;       /**< true: use CAN FD frames (CAN_FLAG_FD); false: classical CAN */
    bool can_fd_brs;   /**< true: set CAN_FLAG_BRS on transmitted frames (requires can_fd) */
    uint8_t tx_dl; /**< TX_DL: 0 defaults (8 classical / 64 FD); else 8 or {12,16,20,24,32,48,64} */
    uint8_t rx_bs; /**< BS of this receiver's FC CTS, 0..255 (0: one block) */
    uint8_t rx_stmin;   /**< STmin of this receiver's FC CTS: 0x00..0x7F or 0xF1..0xF9 */
    uint8_t rx_wft_max; /**< N_WFTmax: FC WAITs this receiver sends in a row, 0..255 */
    uint8_t tx_wft_max; /**< FC WAITs this sender accepts in a row, 0..255 */
} isotp_config_t;

/** Hands a frame to the bus. Returns true if the transmit path took it (a confirmation will
 *  follow), false if not: the transmission or reception it belongs to then ends at once with
 *  ISOTP_RESULT_ERROR (an OVFLW is simply not sent). `frame` is valid during the call only. */
typedef bool (*isotp_send_frame_fn)(void *context, const can_frame_t *frame);

/** N_USData_FF.indication: an FF that fits started a reception of `length` bytes. */
typedef void (*isotp_ff_indication_fn)(void *context, uint32_t length);

/**
 * N_USData.indication: a reception ended. With ISOTP_RESULT_OK, `data` points to the message of
 * `length` bytes in the receive buffer; return true to keep the buffer until isotp_rx_release(),
 * false to give it back when the callback returns. With any other result `data` is NULL and
 * `length` is the FF_DL of the message that was lost.
 */
typedef bool (*isotp_indication_fn)(void *context, isotp_result_t result, const uint8_t *data,
                                    uint32_t length);

/** N_USData.confirm: the transmission that isotp_send() started ended with `result`. */
typedef void (*isotp_confirm_fn)(void *context, isotp_result_t result);

/** The callbacks of an instance. send_frame and indication are required. */
typedef struct {
    isotp_send_frame_fn send_frame;
    isotp_ff_indication_fn ff_indication; /**< may be NULL */
    isotp_indication_fn indication;
    isotp_confirm_fn confirm; /**< may be NULL */
    void *context;            /**< passed to every callback as it is */
} isotp_callbacks_t;

/** The counters of an instance. Arrays are indexed by the enums above. */
typedef struct {
    uint32_t tx_requests;                    /**< messages isotp_send() took */
    uint32_t tx_results[ISOTP_RESULTS];      /**< confirmations given, by result */
    uint32_t ff_indications;                 /**< N_USData_FF.indication given */
    uint32_t rx_results[ISOTP_RESULTS];      /**< indications given, by result; [OK]: messages */
    uint32_t frames_out[ISOTP_FRAME_KINDS];  /**< frames the send_frame callback took */
    uint32_t frames_in[ISOTP_FRAME_KINDS];   /**< frames received and processed */
    uint32_t ignored[ISOTP_IGNORED_REASONS]; /**< frames received and ignored, by reason */
    uint32_t rx_overflow;    /**< FFs that did not fit the receive buffer: answered OVFLW */
    uint32_t rx_sf_too_long; /**< SFs longer than the receive buffer: dropped */
    uint32_t rx_sf_busy;     /**< SFs that came while the buffer was kept: dropped */
    /** OVFLW answers not sent: a new SF or FF came while one was due, or send_frame refused it */
    uint32_t fc_dropped;
    uint32_t stmin_reserved;    /**< FC CTS with a reserved STmin: 127 ms used */
    uint32_t send_refused;      /**< frames the send_frame callback refused */
    uint32_t confirmed;         /**< confirmations that matched a frame handed out */
    uint32_t confirm_unmatched; /**< isotp_on_tx_done() with no frame handed out */
    uint32_t unconfirmed;       /**< frames given up: their N_As or N_Ar ran out first */
    /** frames taken as sent because the peer's answer came before their confirmation */
    uint32_t proven;
    uint32_t outstanding;     /**< frames handed out and not yet confirmed or given up, now */
    uint32_t reentry_refused; /**< calls refused because they came from a callback */
    uint32_t n_br_late;       /**< FCs handed out more than n_br_us after what called for them */
    uint32_t n_cs_late;       /**< CFs handed out more than n_cs_us after their reference */
    uint32_t n_br_max_us;     /**< the longest N_Br measured, saturating */
    uint32_t n_cs_max_us;     /**< the longest N_Cs measured, saturating */
} isotp_stats_t;

/* Private to isotp.c; in the header only so that an instance can be allocated statically. */
typedef enum {
    ISOTP_TX_IDLE = 0,
    ISOTP_TX_DUE,     /* the frame `frame` is to be handed out (waits for room in the pipeline) */
    ISOTP_TX_SENT,    /* handed out; its confirmation (`awaited`) is awaited, N_As runs */
    ISOTP_TX_WAIT_FC, /* waiting for an FC; N_Bs runs out at `timer_us` */
    ISOTP_TX_STMIN    /* the next CF is due at `timer_us` */
} isotp_tx_state_t;

typedef enum {
    ISOTP_RX_IDLE = 0,
    ISOTP_RX_FC_DUE,      /* the FC `fc` is to be handed out (waits for room in the pipeline) */
    ISOTP_RX_FC_SENT,     /* the FC CTS or WAIT is out; its confirmation is awaited, N_Ar runs */
    ISOTP_RX_WAIT_BUFFER, /* the buffer is kept; the next FC is due at `timer_us` */
    ISOTP_RX_CF           /* taking CFs; N_Cr runs out at `timer_us` */
} isotp_rx_state_t;

/* A frame handed out and not confirmed: the oldest is handouts[0]. */
typedef struct {
    uint64_t deadline_us; /* its N_As or N_Ar runs out: it is given up */
    uint32_t serial;      /* which hand-out it was */
    bool rx_side;         /* an FC of the receiver; otherwise a frame of the sender */
} isotp_handout_t;

typedef struct {
    const uint8_t *data; /* the message, as given to isotp_send() */
    uint64_t timer_us;   /* WAIT_FC: N_Bs runs out; STMIN: the next CF is due */
    uint64_t since_us;   /* the reference of N_Cs for the next CF */
    uint64_t cf_done_us; /* confirmation of the previous CF, for STmin across a block */
    uint32_t awaited;    /* SENT: the serial of the frame handed out */
    uint32_t stmin_us;   /* STmin of the current block */
    uint32_t length;     /* of the message */
    uint32_t offset;     /* bytes handed out so far */
    isotp_tx_state_t state;
    isotp_frame_kind_t frame; /* DUE, SENT: SF, FF or CF */
    uint8_t sn;               /* SN of the next CF */
    uint8_t bs;               /* BS of the current block */
    uint8_t block_count;      /* CFs handed out in the current block */
    uint8_t wft_count;        /* FC WAITs in a row */
    bool cf_proven;           /* the previous CF was proven by an FC, not confirmed */
    bool sent_cf;             /* at least one CF has been handed out for this message */
} isotp_tx_t;

typedef struct {
    uint64_t timer_us; /* RX_CF: N_Cr runs out; WAIT_BUFFER: the next FC is due */
    uint64_t since_us; /* the reference of N_Br for the next FC */
    uint32_t awaited;  /* FC_SENT: the serial of the FC handed out */
    uint32_t length;   /* FF_DL of the message being received */
    uint32_t offset;   /* bytes stored */
    isotp_rx_state_t state;
    isotp_frame_kind_t fc;               /* FC_DUE, FC_SENT: FC_CTS, FC_WAIT or FC_OVFLW */
    uint8_t rx_dl;                       /* RX_DL established by the FirstFrame (8..64) */
    uint8_t stash_len;                   /* data bytes carried in the FirstFrame */
    uint8_t sn;                          /* SN expected next */
    uint8_t block_count;                 /* CFs taken in the current block */
    uint8_t wft_count;                   /* FC WAITs handed out for this message */
    bool held;                           /* the upper layer keeps the buffer */
    uint8_t stash[ISOTP_FD_MAX_FF_DATA]; /* FF payload while the buffer is kept */
} isotp_rx_t;

typedef struct {
    isotp_config_t config;
    isotp_callbacks_t callbacks;
    uint8_t *buffer;
    isotp_tx_t tx;
    isotp_rx_t rx;
    isotp_handout_t handouts[ISOTP_PIPELINE_DEPTH];
    isotp_stats_t stats;
    uint32_t serial;      /* serial of the next hand-out */
    uint32_t buffer_size; /* usable bytes of the receive buffer */
    uint8_t handout_count;
    bool ready;       /* initialised */
    bool in_callback; /* a callback is running */
} isotp_t;

/**
 * Prepare `tp` for a connection: copies `config` and `callbacks`, and keeps `rx_buffer` of
 * `rx_size` bytes to receive messages. Nothing is in progress, the buffer is free and every
 * counter is 0.
 */
bool isotp_init(isotp_t *tp, const isotp_config_t *config, const isotp_callbacks_t *callbacks,
                uint8_t *rx_buffer, uint32_t rx_size);

/** N_USData.request: send `length` bytes at `data` at time `now_us`. */
isotp_status_t isotp_send(isotp_t *tp, const uint8_t *data, uint32_t length, uint64_t now_us);

/** A frame arrived at time `now_us`: process it. */
bool isotp_on_frame(isotp_t *tp, const can_frame_t *frame, uint64_t now_us);

/** The oldest frame handed out and not yet confirmed has been sent at `now_us`. */
bool isotp_on_tx_done(isotp_t *tp, uint64_t now_us);

/** Act on the timers that have run out by `now_us` and hand out frames that are due. */
void isotp_poll(isotp_t *tp, uint64_t now_us);

/** Give back the receive buffer that `indication` kept. */
bool isotp_rx_release(isotp_t *tp, uint64_t now_us);

/** The earliest time at which isotp_poll() has something to do, or ISOTP_NO_DEADLINE. */
uint64_t isotp_next_deadline(const isotp_t *tp);

/** True while a transmission is in progress (isotp_send() would answer ISOTP_E_BUSY). */
bool isotp_tx_busy(const isotp_t *tp);

/** True while a reception is in progress. */
bool isotp_rx_busy(const isotp_t *tp);

/** True while `indication` keeps the receive buffer. */
bool isotp_rx_held(const isotp_t *tp);

/** The counters, live: valid until the next call on `tp`. NULL if `tp` is NULL. */
const isotp_stats_t *isotp_stats(const isotp_t *tp);

/** The separation time an STmin byte asks for, in microseconds. */
uint32_t isotp_stmin_us(uint8_t stmin);

/** True if `stmin` is not a reserved value (0x00..0x7F, 0xF1..0xF9). Any context. */
bool isotp_stmin_valid(uint8_t stmin);

/** Names for logs and statistics. Any context. */
const char *isotp_result_name(isotp_result_t result);
const char *isotp_frame_kind_name(isotp_frame_kind_t kind);
const char *isotp_ignored_name(isotp_ignored_t reason);

#endif /* FCN_ISOTP_H */
