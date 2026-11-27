/* SPDX-License-Identifier: MIT */
/**
 * @file
 * The node's UDS server (ISO 14229-1:2020, unified diagnostic services): the diagnostic
 * sessions and their timing, the general response behaviour, the services of phase 5, and the
 * download of a new image into lib/boot's slot writer. Transport-independent: a complete
 * request message comes in as bytes (uds_request()), and the server answers with the bytes of
 * one response, or with none, written into a buffer of the caller's. In the node the
 * diagnostic task puts lib/isotp (ISO 15765-2) in front of it. docs/diagnostics.md has a table
 * for every service; this header says what the code does.
 *
 * Pure C: no operating system, no allocation, no globals and no clock. Time comes in with every
 * call as a monotonic count of microseconds on the caller's clock (uint64_t: it does not wrap
 * in practice). It must not go backwards from one call to the next; a time earlier than one
 * the server has seen counts as that time.
 *
 * Where the standard leaves a choice, the choice made here is marked CHOICE. Clause numbers of
 * the standard are not cited; the rules are named.
 *
 * SESSIONS. The server starts in the defaultSession (0x01). DiagnosticSessionControl (0x10)
 * moves it, in these transitions only (a refused one is answered with
 * subFunctionNotSupportedInActiveSession 0x7E):
 *
 *   active \ requested   default 0x01   programming 0x02   extended 0x03
 *   default              yes            0x7E               yes
 *   programming          yes            yes                0x7E
 *   extended             yes            yes                yes
 *
 * CHOICE: the programming session is entered from the extended session, as the programming
 * sequence of docs/diagnostics.md does, and is left only for the default session (by the
 * request, by S3server, or by a reset). Every accepted DiagnosticSessionControl starts its
 * session afresh, also when it names the active one: a download in progress is abandoned (THE
 * DOWNLOAD) and the results of the routines are forgotten. lib/boot makes an abandoned download
 * safe: the slot stays WRITING, which the bootloader never starts, and the next eraseMemory
 * begins it again.
 *
 * TIMING (times from the call that delivered the request, its `now_us`):
 *
 *   P2server      UDS_P2_SERVER_MS 50 ms. The response, final or responsePending, must start
 *                 within it: reported in the DiagnosticSessionControl response, 1 ms
 *                 resolution, as 0x0032.
 *   P2*server     UDS_P2_STAR_SERVER_MS 5000 ms. The time a tester waits after a
 *                 responsePending: reported in that response, 10 ms resolution, as 0x01F4.
 *   responsePending
 *                 A request still pending UDS_PENDING_FIRST_MS (40 ms) after it was received
 *                 is answered with requestCorrectlyReceived-ResponsePending (NRC 0x78), and
 *                 again UDS_PENDING_EVERY_MS (2000 ms) after the last 0x78 was produced, until
 *                 the final response. The first leaves 10 ms of P2server for the caller's
 *                 latency and the frame; each next one comes 3000 ms before the tester's
 *                 P2*server runs out. A backend that finishes before 40 ms causes none.
 *   S3server      UDS_S3_SERVER_MS 5000 ms. In a non-default session, a server that has
 *                 received no request for S3server since it finished the last one returns to
 *                 the default session (counted in s3_timeouts), as if a request for it had been
 *                 accepted, and abandons a download. The timer runs from the call that
 *                 produced the last request's final response (or would have, had it not been
 *                 suppressed), and is expired at exactly UDS_S3_SERVER_MS after it. It does not
 *                 run while a request is pending, nor in the default session. CHOICE: it starts
 *                 at the response the server hands out, not at the transport's confirmation
 *                 that it went out; the difference is the transmission of one response.
 *
 * uds_request() and uds_poll() both check S3server first, so the outcome does not depend on
 * whether the caller polled in between. uds_next_deadline() names the next moment at which
 * time alone gives uds_poll() something to do.
 *
 * THE GENERAL RESPONSE BEHAVIOUR. A request is checked in this order, as the standard's
 * general server response behaviour has it (its SecurityAccess and Authentication steps fall
 * away: neither service is implemented). The first check that fails decides the negative
 * response, 7F <SID> <NRC>, and no later check is made:
 *
 *   1  no earlier request is still being processed               busyRepeatRequest 0x21
 *   2  the SID is one of the server's services                   serviceNotSupported 0x11
 *   3  the service is supported in the active session            serviceNotSupportedInActiveSession 0x7F
 *   for a service with a sub-function:
 *   4  the request has at least 2 bytes                          incorrectMessageLengthOrInvalidFormat 0x13
 *   5  the sub-function is supported (bit 7 removed first where it is the SPRMIB)
 *                                                                subFunctionNotSupported 0x12
 *   6  the sub-function is supported in the active session       subFunctionNotSupportedInActiveSession 0x7E
 *   7  the checks of the service, in the order its handler documents (uds.c, uds_program.c)
 *      and docs/diagnostics.md tabulates
 *   8  the positive response fits the caller's buffer            responseTooLong 0x14
 *
 * A request of 0 bytes has no SID: it is ignored (requests_empty), answers nothing and does not
 * restart S3server. CHOICE (check 1): the server processes one request at a time. A request
 * that arrives while one is pending is answered with 0x21 at once, whatever it is, and is not
 * kept; the pending one goes on undisturbed, its responsePending cadence included. 0x21 says
 * that the server is too busy for the request now and that the client is to send it again.
 *
 * SUPPRESSPOSRSPMSGINDICATIONBIT. Bit 7 of the sub-function of DiagnosticSessionControl,
 * ECUReset, RoutineControl and TesterPresent. With it set, the service is carried out (the
 * session changes, the reset is asked for, the routine runs) and the positive response is not
 * sent; a negative response is sent as always. Once a responsePending has gone out for the
 * request, the final positive response is sent all the same, because the client now waits for
 * one. CHOICE: ReadDTCInformation has a sub-function but no such bit here; its whole byte is the
 * reportType, so 0x81, 0x82 and 0x8A are sub-functions not supported (0x12): a read whose answer
 * is suppressed has no purpose.
 *
 * THE BACKEND (uds_backend_t). Everything the node itself has to do goes through it: the reset,
 * the boot information behind two DIDs, the DTC snapshot and the clear, and the flash work of a
 * download and of the confirm. Every function is called first with UDS_CALL_START, from the call
 * that processes the request, and answers UDS_BE_DONE, with its result in its out-parameters, or
 * UDS_BE_PENDING. A PENDING operation is called again with UDS_CALL_POLL, with the same
 * arguments, from every uds_poll() until it answers DONE; meanwhile the server sends
 * responsePending as TIMING says, and calls no other backend function. Out-parameters are read
 * only from the call that answers DONE. An NRC a backend returns must be one the server names
 * (uds_nrc_code(), not 0x78); any other value is sent as generalReject 0x10. The functions run
 * in the caller's context, inside uds_request() and uds_poll().
 *
 * THE DOWNLOAD (uds_dl_state_t), all of it in the programmingSession:
 *
 *   IDLE      --eraseMemory (0x31 01 FF00, address A, size N) DONE BOOT_OK-->   ERASED(A, N)
 *   ERASED    --RequestDownload (0x34, the same A and N)-->                        TRANSFER(A, N)
 *   TRANSFER  --TransferData (0x36) blocks, counter from 1, until N bytes-->       TRANSFER
 *   TRANSFER  --RequestTransferExit (0x37), all N bytes taken, DONE BOOT_BUSY or BOOT_OK-->  EXITED
 *   EXITED    --checkProgrammingDependencies (0x31 01 FF01) DONE BOOT_OK-->        CHECKED
 *
 * The erase is the slot writer's boot_slot_by_address(), boot_writer_begin() and the erase steps
 * until BOOT_OK; TransferData is boot_writer_write() at the offset the blocks so far add up to;
 * RequestTransferExit is the first boot_writer_finish_step() (the last partial page, and the
 * start of the validation, which answers BOOT_BUSY); the check routine is the finish steps until
 * BOOT_OK (the validation and the COMMIT record: the slot is then TRIAL). All of that is the
 * backend's (uds_backend_t).
 *
 * An eraseMemory in TRANSFER is refused (conditionsNotCorrect 0x22) and the download goes on.
 * In every other state it is accepted: what was there ends (from EXITED counted in `aborted`),
 * nothing is erased for a download until the erase completes (ERASED), and an erase the backend
 * refuses leaves IDLE. In TRANSFER and EXITED, a refusal of the backend (a write, the exit, the
 * check) and transferDataSuspended (0x71) end the download: IDLE, counted in `failed`. An
 * accepted DiagnosticSessionControl and an S3server timeout end it too: IDLE, from TRANSFER or
 * EXITED counted in `aborted`. The other refusals (a wrong counter, a malformed block, an exit
 * before the last byte, a check before the exit) leave the state as it was.
 *
 * The blockSequenceCounter starts at 1 and wraps from 0xFF to 0x00. A block whose counter is
 * that of the last block accepted is answered positively and not written again: the client
 * repeats a block whose response it lost (the standard's rule); its data are not compared with
 * what was written. maxNumberOfBlockLength (UDS_MAX_BLOCK_LENGTH, in the RequestDownload
 * response) counts the SID and the counter: 1026 bytes, 1024 of data. docs/diagnostics.md says
 * why.
 *
 * LIB/BOOT'S STATUS AS AN NRC (uds_nrc_for_boot()), for every backend operation that answers
 * with a boot_status_t:
 *
 *   BOOT_OK, BOOT_BUSY               none: the operation succeeded (BOOT_BUSY only where the
 *                                    operation ends in it: RequestTransferExit)
 *   BOOT_E_ARG, BOOT_E_SIZE          requestOutOfRange 0x31 (no slot at the address, a size of
 *                                    0 or beyond the slot)
 *   BOOT_E_STATE                     conditionsNotCorrect 0x22 (the running slot, a trial
 *                                    pending, the factory state; no trial to confirm)
 *   BOOT_E_SEQUENCE                  requestSequenceError 0x24
 *   BOOT_E_INVALID, BOOT_E_FLASH,    generalProgrammingFailure 0x72
 *   BOOT_E_POWER_LOST, any other
 *
 * A BOOT_BUSY where the operation must end in BOOT_OK (an erase, a write, the check, the
 * confirm) is generalProgrammingFailure 0x72 too.
 *
 * STATISTICS (uds_get_stats()): requests by service, negative responses by code,
 * responsePending sent, positive responses sent and suppressed, session changes, S3server
 * timeouts, resets asked for, the longest time a request was pending, and the download's
 * progress and durations. Counters wrap after 2^32.
 *
 * NOT IMPLEMENTED (docs/diagnostics.md says what a product adds): SecurityAccess (0x27),
 * Authentication (0x29), CommunicationControl (0x28), ControlDTCSetting (0x85),
 * WriteDataByIdentifier (0x2E), RequestUpload (0x35), stopRoutine, compressed or encrypted
 * downloads (dataFormatIdentifier other than 0x00), more than one DID per request, DTC groups
 * other than all (0xFFFFFF), and a MemorySelection in ClearDiagnosticInformation. Functional
 * addressing is the transport's matter; this server takes every request as physical.
 *
 * CONTEXT. The functions keep no data of their own, call no operating system and allocate
 * nothing. One uds_server_t belongs to one task (in the node, the diagnostic task), which makes
 * every call on it; the backend is called from those calls. Each call takes the time of the
 * request's checks, one backend call, and building one response of at most UDS_RESPONSE_MAX
 * bytes (with the copy of a request of at most UDS_REQUEST_MAX bytes).
 */
#ifndef FCN_UDS_H
#define FCN_UDS_H

#include <stdbool.h>
#include <stdint.h>

#include "boot/boot.h"
#include "dtc/dtc.h"

/* ------------------------------------------------------------------------------------ timing */

#define UDS_P2_SERVER_MS      50U   /**< P2server_max, reported in the DSC response */
#define UDS_P2_STAR_SERVER_MS 5000U /**< P2*server_max, reported in the DSC response */
#define UDS_PENDING_FIRST_MS  40U   /**< the first responsePending, after the request */
#define UDS_PENDING_EVERY_MS  2000U /**< each next one, after the one before */
#define UDS_S3_SERVER_MS      5000U /**< the session's timeout without a request */

/** uds_next_deadline() when no time-driven work is due. */
#define UDS_NO_DEADLINE UINT64_MAX

/* ---------------------------------------------------------------------------------- sessions */

#define UDS_SESSION_DEFAULT     0x01U /**< defaultSession */
#define UDS_SESSION_PROGRAMMING 0x02U /**< programmingSession */
#define UDS_SESSION_EXTENDED    0x03U /**< extendedDiagnosticSession */

/* ------------------------------------------------------------------- service identifiers */

#define UDS_SID_SESSION_CONTROL  0x10U /**< DiagnosticSessionControl */
#define UDS_SID_ECU_RESET        0x11U /**< ECUReset */
#define UDS_SID_CLEAR_DTC        0x14U /**< ClearDiagnosticInformation */
#define UDS_SID_READ_DTC         0x19U /**< ReadDTCInformation */
#define UDS_SID_READ_DID         0x22U /**< ReadDataByIdentifier */
#define UDS_SID_ROUTINE_CONTROL  0x31U /**< RoutineControl */
#define UDS_SID_REQUEST_DOWNLOAD 0x34U /**< RequestDownload */
#define UDS_SID_TRANSFER_DATA    0x36U /**< TransferData */
#define UDS_SID_TRANSFER_EXIT    0x37U /**< RequestTransferExit */
#define UDS_SID_TESTER_PRESENT   0x3EU /**< TesterPresent */
#define UDS_SID_NEGATIVE         0x7FU /**< the first byte of a negative response */

/** A positive response's SID: the request's plus this. */
#define UDS_POSITIVE_OFFSET 0x40U

/** The services in the server's table: the length of uds_stats_t.requests. */
#define UDS_SERVICE_COUNT 10U

/* -------------------------------------------------------------- negative response codes */

#define UDS_NRC_NONE                       0x00U /**< no NRC: the positive response */
#define UDS_NRC_GENERAL_REJECT             0x10U /**< generalReject */
#define UDS_NRC_SERVICE_NOT_SUPPORTED      0x11U /**< serviceNotSupported */
#define UDS_NRC_SUBFUNCTION_UNSUPPORTED    0x12U /**< subFunctionNotSupported */
#define UDS_NRC_INCORRECT_LENGTH           0x13U /**< incorrectMessageLengthOrInvalidFormat */
#define UDS_NRC_RESPONSE_TOO_LONG          0x14U /**< responseTooLong */
#define UDS_NRC_BUSY_REPEAT_REQUEST        0x21U /**< busyRepeatRequest */
#define UDS_NRC_CONDITIONS_NOT_CORRECT     0x22U /**< conditionsNotCorrect */
#define UDS_NRC_REQUEST_SEQUENCE_ERROR     0x24U /**< requestSequenceError */
#define UDS_NRC_REQUEST_OUT_OF_RANGE       0x31U /**< requestOutOfRange */
#define UDS_NRC_DOWNLOAD_NOT_ACCEPTED      0x70U /**< uploadDownloadNotAccepted */
#define UDS_NRC_TRANSFER_SUSPENDED         0x71U /**< transferDataSuspended */
#define UDS_NRC_PROGRAMMING_FAILURE        0x72U /**< generalProgrammingFailure */
#define UDS_NRC_WRONG_BLOCK_COUNTER        0x73U /**< wrongBlockSequenceCounter */
#define UDS_NRC_RESPONSE_PENDING           0x78U /**< requestCorrectlyReceived-ResponsePending */
#define UDS_NRC_SUBFUNCTION_NOT_IN_SESSION 0x7EU /**< subFunctionNotSupportedInActiveSession */
#define UDS_NRC_SERVICE_NOT_IN_SESSION     0x7FU /**< serviceNotSupportedInActiveSession */

/** The negative response codes the server sends, 0x78 not among them: the length of
 *  uds_stats_t.nrc, in the order of uds_nrc_code(). */
#define UDS_NRC_KINDS 15U

/* -------------------------------------------------------------------------- sub-functions */

/** The suppressPosRspMsgIndicationBit of a sub-function byte. */
#define UDS_SPRMIB 0x80U

#define UDS_RESET_HARD          0x01U /**< ECUReset hardReset */
#define UDS_TESTER_PRESENT_ZERO 0x00U /**< TesterPresent zeroSubFunction */
#define UDS_ROUTINE_START       0x01U /**< RoutineControl startRoutine */
#define UDS_ROUTINE_STOP        0x02U /**< RoutineControl stopRoutine: not supported */
#define UDS_ROUTINE_RESULTS     0x03U /**< RoutineControl requestRoutineResults */
#define UDS_DTC_COUNT_BY_MASK   0x01U /**< reportNumberOfDTCByStatusMask */
#define UDS_DTC_BY_MASK         0x02U /**< reportDTCByStatusMask */
#define UDS_DTC_SUPPORTED       0x0AU /**< reportSupportedDTC */

/* -------------------------------------------------------------------------- data, routines */

#define UDS_DID_ACTIVE_SESSION   0xF186U /**< ActiveDiagnosticSessionDataIdentifier */
#define UDS_DID_SOFTWARE_VERSION 0xF189U /**< vehicleManufacturerECUSoftwareVersionNumber */
#define UDS_DID_BOOT_INFO        0xFD00U /**< boot information, system-supplier range */

#define UDS_RID_ERASE_MEMORY  0xFF00U /**< eraseMemory */
#define UDS_RID_CHECK_PROG    0xFF01U /**< checkProgrammingDependencies */
#define UDS_RID_CONFIRM_TRIAL 0xF000U /**< confirm the running trial, system-supplier range */

/** The routines in the server's table. */
#define UDS_ROUTINE_COUNT 3U

/** The routineStatusRecord of every positive RoutineControl response: the routine's outcome,
 *  then the NRC its start was answered with (UDS_NRC_NONE after a success). */
#define UDS_ROUTINE_STATUS_SIZE 2U
#define UDS_ROUTINE_OK          0x00U /**< the routine completed successfully */
#define UDS_ROUTINE_FAILED      0x01U /**< its start was answered with the NRC that follows */

/* ----------------------------------------------------------------------- the DTC services */

#define UDS_DTC_FORMAT_ISO14229 0x01U     /**< DTCFormatIdentifier ISO_14229-1_DTCFormat */
#define UDS_DTC_GROUP_ALL       0xFFFFFFU /**< groupOfDTC: all groups */

/* ---------------------------------------------------------------------------- the download */

#define UDS_DATA_FORMAT_PLAIN 0x00U /**< dataFormatIdentifier: no compression, no encryption */
/** addressAndLengthFormatIdentifier: a 4-byte memorySize, a 4-byte memoryAddress. */
#define UDS_ADDR_LEN_FORMAT 0x44U
#ifndef UDS_BLOCK_DATA_MAX
#define UDS_BLOCK_DATA_MAX 1024U /**< data bytes in one TransferData request at most */
#endif
/** maxNumberOfBlockLength: a whole TransferData request, its SID and counter included. */
#define UDS_MAX_BLOCK_LENGTH (UDS_BLOCK_DATA_MAX + 2U)

/* ------------------------------------------------------------------------ the boot information */

#define UDS_BOOT_INFO_VERSION 1U  /**< the layout version, byte 0 of DID 0xFD00 */
#define UDS_BOOT_INFO_SIZE    12U /**< bytes of the record of DID 0xFD00 */
#define UDS_BOOT_NO_SLOT      0xFFU

/* ------------------------------------------------------------------------------ the buffers */

/** The longest request the server keeps: a whole TransferData block. A longer request is
 *  answered (its service's length check refuses it) without being kept. */
#define UDS_REQUEST_MAX UDS_MAX_BLOCK_LENGTH

/** The longest response: reportDTCByStatusMask or reportSupportedDTC with every DTC, the SID,
 *  the reportType, the availability mask, and 4 bytes per DTC. A caller's buffer of this size
 *  takes every response. */
#define UDS_RESPONSE_MAX (3U + (4U * DTC_EVENTS_MAX))

/** A negative response: 7F, the SID, the NRC. A smaller buffer takes nothing. */
#define UDS_NEGATIVE_LEN 3U

/* ------------------------------------------------------------------------------------ types */

/** Which call of a backend operation this is (see THE BACKEND). */
typedef enum {
    UDS_CALL_START = 0, /**< the first, from the call that processes the request */
    UDS_CALL_POLL       /**< a later one, from uds_poll(), after the operation answered PENDING */
} uds_call_t;

/** What a backend operation answers. */
typedef enum {
    UDS_BE_DONE = 0, /**< finished: its result is in the out-parameters */
    UDS_BE_PENDING   /**< not yet: call again with UDS_CALL_POLL */
} uds_be_t;

/** The boot information of the running image, from the bootloader's hand-off (plat_retained.h,
 *  THE HAND-OFF) and the confirm. The values are passed through into DID 0xFD00 as they are
 *  given; without a hand-off the fields from it are written as "none" (uds_backend_t). */
typedef struct {
    bool handoff;           /**< the bootloader started this image and left a hand-off */
    uint8_t slot;           /**< the slot started: 0 (A) or 1 (B) */
    uint8_t mode;           /**< the hand-off's mode: 1 CONFIRMED, 2 TRIAL */
    uint8_t attempt;        /**< the attempt of a trial, 1..3; 0 for the confirmed image */
    uint8_t event;          /**< the bootloader's select event: lib/boot's boot_event_t values */
    uint8_t event_slot;     /**< the slot that event acted on, or UDS_BOOT_NO_SLOT */
    uint8_t confirm_state;  /**< BMS_BootInfo's ConfirmState (app/update.h): 0 NOT_NEEDED,
                                1 WAITING, 2 CONFIRMING, 3 CONFIRMED, 4 FAILED */
    uint32_t image_version; /**< the image's version, major << 24 | minor << 16 | patch */
} uds_boot_info_t;

/**
 * What the node does for the server (see THE BACKEND). `ctx` is passed to every function. Each
 * function answers UDS_BE_DONE or UDS_BE_PENDING; `nrc` out-parameters are UDS_NRC_NONE for
 * success, `status` out-parameters lib/boot's result.
 */
typedef struct {
    void *ctx;
    /** ECUReset with `reset_type` (UDS_RESET_HARD): *nrc UDS_NRC_NONE accepts it, and the
     *  server answers positively and reports the reset in uds_output_t; the caller makes it
     *  after the response has gone out. Any NRC refuses it (conditionsNotCorrect 0x22, say). */
    uds_be_t (*ecu_reset)(void *ctx, uds_call_t call, uint8_t reset_type, uint8_t *nrc);
    /** The boot information, for DIDs 0xF189 and 0xFD00. */
    uds_be_t (*boot_info)(void *ctx, uds_call_t call, uds_boot_info_t *info, uint8_t *nrc);
    /** A consistent snapshot of the DTCs, for ReadDTCInformation: *state must stay valid and
     *  unchanged until the call returns to the caller (the response is built inside it). */
    uds_be_t (*dtc_snapshot)(void *ctx, uds_call_t call, const dtc_state_t **state, uint8_t *nrc);
    /** ClearDiagnosticInformation of `group` (UDS_DTC_GROUP_ALL): DONE only once the clear has
     *  been made, so that the positive response follows it. */
    uds_be_t (*dtc_clear)(void *ctx, uds_call_t call, uint32_t group, uint8_t *nrc);
    /** eraseMemory for the image of `size` bytes at `address` (the start of a slot, in the
     *  device's address space): boot_slot_by_address(), boot_writer_begin(), the erase steps
     *  until BOOT_OK. */
    uds_be_t (*erase)(void *ctx, uds_call_t call, uint32_t address, uint32_t size,
                      boot_status_t *status);
    /** TransferData: boot_writer_write() of `len` bytes at image offset `offset`. `data` stays
     *  valid and unchanged until the operation answers DONE (it is the server's copy). */
    uds_be_t (*write)(void *ctx, uds_call_t call, uint32_t offset, const uint8_t *data,
                      uint32_t len, boot_status_t *status);
    /** RequestTransferExit: the first boot_writer_finish_step(), BOOT_BUSY when it succeeds. */
    uds_be_t (*transfer_exit)(void *ctx, uds_call_t call, boot_status_t *status);
    /** checkProgrammingDependencies: boot_writer_finish_step() until BOOT_OK (the validation
     *  and the COMMIT record). */
    uds_be_t (*check)(void *ctx, uds_call_t call, boot_status_t *status);
    /** The confirm routine: boot_confirm() of the running trial; BOOT_E_STATE when there is no
     *  trial to confirm. */
    uds_be_t (*confirm)(void *ctx, uds_call_t call, boot_status_t *status);
} uds_backend_t;

/** Where the download stands (see THE DOWNLOAD). */
typedef enum {
    UDS_DL_IDLE = 0, /**< nothing erased for a download */
    UDS_DL_ERASED,   /**< eraseMemory done for an address and size */
    UDS_DL_TRANSFER, /**< RequestDownload accepted: TransferData */
    UDS_DL_EXITED,   /**< RequestTransferExit accepted: the check routine is next */
    UDS_DL_CHECKED   /**< the check routine committed the image */
} uds_dl_state_t;

/** The download in the statistics. */
typedef struct {
    uds_dl_state_t state;
    uint32_t address;         /**< of the last erase accepted; 0 before */
    uint32_t size;            /**< bytes, of the last erase accepted; 0 before */
    uint32_t received;        /**< bytes taken in the last download (since its RequestDownload) */
    uint32_t blocks;          /**< blocks written in it */
    uint32_t repeated_blocks; /**< blocks answered as repeats in it, not written again */
    uint8_t counter;          /**< its last blockSequenceCounter accepted; 0 before its first */
    uint32_t erases;          /**< eraseMemory routines completed */
    uint32_t started;         /**< RequestDownload accepted */
    uint32_t exited;          /**< RequestTransferExit accepted */
    uint32_t committed;       /**< checkProgrammingDependencies passed */
    uint32_t aborted;         /**< downloads ended in TRANSFER or EXITED by a session change,
                                   S3server or an erase */
    uint32_t failed;          /**< downloads ended by 0x71 or a refusal of the backend */
    uint64_t erase_us;        /**< the last completed eraseMemory, request to final response */
    uint64_t transfer_us;     /**< RequestDownload to the RequestTransferExit response, the last */
    uint64_t check_us;        /**< the last completed check routine, request to response */
} uds_download_stats_t;

/** The server's counters (see STATISTICS). */
typedef struct {
    uint32_t requests[UDS_SERVICE_COUNT]; /**< by service, in the order of uds_service_sid();
                                               counted at their arrival, busy or not */
    uint32_t requests_unsupported;        /**< with a SID not in the table (0x11) */
    uint32_t requests_empty;              /**< of 0 bytes, ignored */
    uint32_t nrc[UDS_NRC_KINDS];          /**< negative responses by code, in the order of
                                               uds_nrc_code() */
    uint32_t response_pending;            /**< responsePending (0x78) sent */
    uint32_t positive;                    /**< positive responses sent */
    uint32_t suppressed;                  /**< positive responses suppressed (SPRMIB) */
    uint32_t session_changes;             /**< session changes, by request or S3server */
    uint32_t s3_timeouts;                 /**< S3server timeouts */
    uint32_t resets;                      /**< ECUResets accepted */
    uint64_t pending_max_us;              /**< the longest a request was pending */
    uds_download_stats_t download;
} uds_stats_t;

/** What a call hands back to the caller. */
typedef struct {
    uint32_t len; /**< the bytes of the response in the caller's buffer; 0: nothing to send */
    /** ECUReset was accepted: reset once these `len` bytes have gone out (at once if 0). */
    bool reset;
} uds_output_t;

/* Private to lib/uds; in the header so that a server can be allocated statically. */
typedef struct {
    bool valid;  /* the routine ran since its session began */
    uint8_t nrc; /* what its start was answered with: UDS_NRC_NONE after a success */
} uds_routine_result_t;

/* Private to lib/uds: the download (see THE DOWNLOAD). */
typedef struct {
    uds_dl_state_t state;
    uint32_t address;
    uint32_t size;
    uint32_t received;
    uint32_t blocks;
    uint32_t repeated;
    uint8_t counter;     /* the last counter accepted */
    bool have_block;     /* a block has been accepted since RequestDownload */
    uint64_t started_us; /* the time of the RequestDownload */
} uds_download_t;

/* Private to lib/uds: the server. Read it through the functions below. */
typedef struct {
    uds_backend_t backend;
    uint32_t ready;               /* a mark set by uds_init() */
    uint8_t session;              /* UDS_SESSION_* */
    bool s3_running;              /* in a non-default session, between two requests */
    uint64_t s3_start_us;         /* when it started */
    uint64_t last_us;             /* the latest time a call gave */
    bool busy;                    /* a request is pending */
    uint8_t service;              /* its index in the service table */
    uint8_t subfunction;          /* its sub-function, SPRMIB removed; 0 without one */
    bool suppress;                /* its positive response is to be suppressed */
    bool pending_sent;            /* a responsePending went out for it */
    uint64_t request_us;          /* when it arrived */
    uint64_t next_pending_us;     /* when the next responsePending is due */
    uint32_t req_len;             /* its length */
    uint8_t req[UDS_REQUEST_MAX]; /* the request in progress, as far as it fits */
    uds_download_t dl;
    uds_routine_result_t routine[UDS_ROUTINE_COUNT];
    uds_stats_t stats;
} uds_server_t;

/* -------------------------------------------------------------------------------- functions */

/**
 * Prepare `s`, in the default session with nothing pending and every counter 0, to call
 * `backend` (copied; every function in it must be set). The memory of `s` may hold anything
 * before. Returns false, preparing nothing, for a NULL pointer or a backend function that is
 * NULL. The owner of `s`, before any other call on it.
 */
bool uds_init(uds_server_t *s, const uds_backend_t *backend);

/**
 * A request message of `len` bytes at `req` has arrived, at `now_us` (when its last byte came).
 * The server checks it (THE GENERAL RESPONSE BEHAVIOUR), carries it out or starts it, and
 * writes what is to be sent now into `rsp` (`rsp_cap` bytes, at least UDS_NEGATIVE_LEN;
 * UDS_RESPONSE_MAX takes every response): the final response, a negative response, or, for a
 * request whose backend answered PENDING, nothing yet (uds_poll() goes on with it). `req` is
 * not used after the call: a pending request is kept in the server (up to UDS_REQUEST_MAX
 * bytes). `out` says how many bytes to send and whether to reset after them.
 *
 * Returns false, doing nothing (out->len 0), for a NULL pointer (`req` may be NULL with a `len`
 * of 0), an `rsp_cap` below UDS_NEGATIVE_LEN, or a server uds_init() did not prepare;
 * otherwise true. The owner of `s` (the diagnostic task).
 */
bool uds_request(uds_server_t *s, const uint8_t *req, uint32_t len, uint64_t now_us, uint8_t *rsp,
                 uint32_t rsp_cap, uds_output_t *out);

/**
 * Time has passed, or a pending backend operation may have finished: at `now_us`, check
 * S3server, poll the pending operation, and write into `rsp` what is to be sent now: the final
 * response, a responsePending that is due, or nothing (out->len 0). Call it when the backend's
 * operation may have finished (in the node, when the flash task reports a job done) and no
 * later than uds_next_deadline(); calling it more often changes nothing.
 *
 * Returns false, doing nothing, as uds_request() does; otherwise true. The owner of `s`.
 */
bool uds_poll(uds_server_t *s, uint64_t now_us, uint8_t *rsp, uint32_t rsp_cap, uds_output_t *out);

/**
 * The next time at which uds_poll() has time-driven work: the next responsePending of a
 * pending request, or the expiry of S3server; UDS_NO_DEADLINE if there is none (or for a NULL
 * or unprepared `s`). The completion of a pending backend operation is not a deadline. Any
 * context that may read `s`.
 */
uint64_t uds_next_deadline(const uds_server_t *s);

/** Whether a request is pending (its backend operation answered PENDING); false for a NULL or
 *  unprepared `s`. Any context that may read `s`. */
bool uds_busy(const uds_server_t *s);

/** The active session (UDS_SESSION_*); UDS_SESSION_DEFAULT for a NULL or unprepared `s`. Any
 *  context that may read `s`. */
uint8_t uds_session(const uds_server_t *s);

/** A copy of the counters (see STATISTICS); zeros for a NULL or unprepared `s`. A NULL `out`
 *  does nothing. Any context that may read `s`. */
void uds_get_stats(const uds_server_t *s, uds_stats_t *out);

/** The NRC for a lib/boot status (see LIB/BOOT'S STATUS AS AN NRC): UDS_NRC_NONE for BOOT_OK
 *  and BOOT_BUSY. Any context. */
uint8_t uds_nrc_for_boot(boot_status_t status);

/** The SID of entry `index` of uds_stats_t.requests, 0..UDS_SERVICE_COUNT - 1; 0 beyond. Any
 *  context. */
uint8_t uds_service_sid(uint32_t index);

/** The code of entry `index` of uds_stats_t.nrc, 0..UDS_NRC_KINDS - 1; 0 beyond. Any
 *  context. */
uint8_t uds_nrc_code(uint32_t index);

/** The standard's name of a service ("DiagnosticSessionControl"), of an NRC
 *  ("conditionsNotCorrect"; "requestCorrectlyReceivedResponsePending" for 0x78), of a session
 *  ("programmingSession") or of a download state ("transfer"); "?" for a value the server does
 *  not know. Any context. */
const char *uds_service_name(uint8_t sid);
const char *uds_nrc_name(uint8_t nrc);
const char *uds_session_name(uint8_t session);
const char *uds_dl_state_name(uds_dl_state_t state);

#endif /* FCN_UDS_H */
