/* SPDX-License-Identifier: MIT */
#include <stddef.h>

#include "uds/uds.h"
#include "uds/uds_internal.h"

/*
 * RoutineControl (0x31) and the download: RequestDownload (0x34), TransferData (0x36) and
 * RequestTransferExit (0x37), with the download's state machine (uds.h, THE DOWNLOAD). The
 * flash work is the backend's; what is decided here is when each step may come, at which
 * offset a block goes, and which NRC a refusal of lib/boot becomes.
 *
 * What holds between two calls, in addition to uds.c's:
 *
 *   - in ERASED, TRANSFER, EXITED and CHECKED, dl.address and dl.size are those of the
 *     eraseMemory that completed, and dl.size >= 1 (lib/boot refuses a size of 0);
 *   - in TRANSFER and EXITED, dl.received <= dl.size, and dl.received is the sum of the data of
 *     the blocks written since the RequestDownload, each written at the offset the blocks
 *     before it add up to; in EXITED, dl.received == dl.size;
 *   - dl.have_block says that dl.counter is the counter of a block accepted since the
 *     RequestDownload; dl.counter is 0 until then, so the first block expected is 1.
 */

/* Requests of RoutineControl: 31 <type> <RID, 2 bytes> <routineControlOptionRecord>. */
#define ROUTINE_BASE_LEN 4U
/* eraseMemory: the option record is 0x44, the address (4 bytes) and the size (4 bytes). */
#define ERASE_FORMAT_AT 4U
#define ERASE_ADDR_AT   5U
#define ERASE_SIZE_AT   9U
#define ERASE_LEN       13U

/* RequestDownload: 34 <dataFormatIdentifier> 0x44 <address, 4> <size, 4>. */
#define DOWNLOAD_MIN_LEN  3U
#define DOWNLOAD_DFI_AT   1U
#define DOWNLOAD_ALFID_AT 2U
#define DOWNLOAD_ADDR_AT  3U
#define DOWNLOAD_SIZE_AT  7U
#define DOWNLOAD_LEN      11U
/* lengthFormatIdentifier of its response: maxNumberOfBlockLength in 2 bytes (high nibble). */
#define DOWNLOAD_LFID 0x20U

/* TransferData: 36 <blockSequenceCounter> <data>. */
#define TRANSFER_HEAD_LEN 2U

_Static_assert(UDS_ADDR_LEN_FORMAT == 0x44U, "4 bytes of address and 4 of size, as parsed here");
_Static_assert(ERASE_LEN == (ERASE_SIZE_AT + 4U), "the erase request ends with its size");
_Static_assert(DOWNLOAD_LEN == (DOWNLOAD_SIZE_AT + 4U), "RequestDownload ends with its size");
_Static_assert(UDS_MAX_BLOCK_LENGTH <= 0xFFFFU, "maxNumberOfBlockLength is reported in 2 bytes");
_Static_assert(TRANSFER_HEAD_LEN == 2U, "a block is the SID, the counter, then its data");
/* UDS_REQUEST_MAX is a whole block (uds.h); every other service's request is shorter. */
_Static_assert(UDS_REQUEST_MAX >= ERASE_LEN, "the longest request of another service is kept");
_Static_assert(UDS_ROUTINE_STATUS_SIZE == 2U, "the outcome and the NRC");

/* The routines, in the order of uds_server_t.routine. */
#define ROUTINE_ERASE   0U
#define ROUTINE_CHECK   1U
#define ROUTINE_CONFIRM 2U

typedef struct {
    uint16_t rid;
    uint8_t sessions; /* UDS_IN_*: where the routine is supported */
} routine_t;

static const routine_t k_routines[UDS_ROUTINE_COUNT] = {
    {UDS_RID_ERASE_MEMORY, UDS_IN_PROGRAMMING},
    {UDS_RID_CHECK_PROG, UDS_IN_PROGRAMMING},
    {UDS_RID_CONFIRM_TRIAL, UDS_IN_EXTENDED},
};

_Static_assert((ROUTINE_ERASE < UDS_ROUTINE_COUNT) && (ROUTINE_CHECK < UDS_ROUTINE_COUNT) &&
                   (ROUTINE_CONFIRM < UDS_ROUTINE_COUNT),
               "every routine has an entry");

/* ----------------------------------------------------------------------- the download's state */

void uds_download_end(uds_server_t *s, bool failed)
{
    if ((s->dl.state == UDS_DL_TRANSFER) || (s->dl.state == UDS_DL_EXITED)) {
        if (failed) {
            s->stats.download.failed++;
        } else {
            s->stats.download.aborted++;
        }
    }

    s->dl.state = UDS_DL_IDLE;
}

void uds_program_session_begins(uds_server_t *s)
{
    uds_download_end(s, false);

    for (uint32_t i = 0U; i < UDS_ROUTINE_COUNT; i++) {
        s->routine[i].valid = false;
        s->routine[i].nrc = UDS_NRC_NONE;
    }
}

/* The NRC of a backend operation that must end in BOOT_OK (uds.h, LIB/BOOT'S STATUS AS AN
 * NRC): a BOOT_BUSY there is a failure. */
static uint8_t nrc_unless_ok(boot_status_t status)
{
    return (status == BOOT_OK) ? (uint8_t)UDS_NRC_NONE : (uint8_t)UDS_NRC_PROGRAMMING_FAILURE;
}

static uint8_t nrc_of_final(boot_status_t status)
{
    const uint8_t nrc = uds_nrc_for_boot(status);

    return (nrc != UDS_NRC_NONE) ? nrc : nrc_unless_ok(status);
}

/* ----------------------------------------------------------------------------- RoutineControl */

static uint32_t routine_index(uint16_t rid)
{
    uint32_t index = UDS_ROUTINE_COUNT;

    for (uint32_t i = 0U; i < UDS_ROUTINE_COUNT; i++) {
        if (k_routines[i].rid == rid) {
            index = i;
        }
    }

    return index;
}

/*
 * A list of checks in which the first that fails decides: each step is its own statement,
 * `if ((nrc == UDS_NRC_NONE) && <the step fails>)`, so that a step is evaluated only once the
 * steps before it have passed (it may rely on them, a length on its way to a byte that it then
 * reads, say) and two steps may give the same NRC.
 */

/* The checks of eraseMemory's start, in this order: an option record (0x13); its
 * addressAndLengthFormatIdentifier is 0x44 (0x31); 13 bytes (0x13); no download in TRANSFER
 * (0x22). */
static uint8_t erase_checks(const uds_server_t *s)
{
    uint8_t nrc = UDS_NRC_NONE;

    if (s->req_len <= ERASE_FORMAT_AT) {
        nrc = UDS_NRC_INCORRECT_LENGTH;
    }
    if ((nrc == UDS_NRC_NONE) && (s->req[ERASE_FORMAT_AT] != UDS_ADDR_LEN_FORMAT)) {
        nrc = UDS_NRC_REQUEST_OUT_OF_RANGE;
    }
    if ((nrc == UDS_NRC_NONE) && (s->req_len != ERASE_LEN)) {
        nrc = UDS_NRC_INCORRECT_LENGTH;
    }
    if ((nrc == UDS_NRC_NONE) && (s->dl.state == UDS_DL_TRANSFER)) {
        nrc = UDS_NRC_CONDITIONS_NOT_CORRECT;
    }

    return nrc;
}

/* The checks of a startRoutine that its routine makes before it runs: eraseMemory's above;
 * for the others 4 bytes (0x13), and for the check routine a download in EXITED (0x24). */
static uint8_t start_checks(const uds_server_t *s, uint32_t routine)
{
    uint8_t nrc = UDS_NRC_NONE;

    if (routine == ROUTINE_ERASE) {
        nrc = erase_checks(s);
    } else if (s->req_len != ROUTINE_BASE_LEN) {
        nrc = UDS_NRC_INCORRECT_LENGTH;
    } else if ((routine == ROUTINE_CHECK) && (s->dl.state != UDS_DL_EXITED)) {
        nrc = UDS_NRC_REQUEST_SEQUENCE_ERROR;
    } else {
        /* the routine may run */
    }

    return nrc;
}

/* The routine's backend operation. */
static uds_be_t run_routine(uds_server_t *s, uint32_t routine, uds_call_t call,
                            boot_status_t *status)
{
    uds_be_t be = UDS_BE_DONE;

    if (routine == ROUTINE_ERASE) {
        be = s->backend.erase(s->backend.ctx, call, uds_get32(s->req, ERASE_ADDR_AT),
                              uds_get32(s->req, ERASE_SIZE_AT), status);
    } else if (routine == ROUTINE_CHECK) {
        be = s->backend.check(s->backend.ctx, call, status);
    } else {
        be = s->backend.confirm(s->backend.ctx, call, status);
    }

    return (be == UDS_BE_DONE) ? UDS_BE_DONE : UDS_BE_PENDING;
}

/* What a routine that ran leaves in the download (uds.h, THE DOWNLOAD). */
static void routine_done(uds_server_t *s, uint32_t routine, uint8_t nrc, uint64_t now)
{
    const uint64_t took = now - s->request_us;

    if (routine == ROUTINE_ERASE) {
        if (nrc == UDS_NRC_NONE) {
            s->dl.state = UDS_DL_ERASED;
            s->dl.address = uds_get32(s->req, ERASE_ADDR_AT);
            s->dl.size = uds_get32(s->req, ERASE_SIZE_AT);
            s->stats.download.erases++;
            s->stats.download.erase_us = took;
        }
    } else if (routine == ROUTINE_CHECK) {
        if (nrc == UDS_NRC_NONE) {
            s->dl.state = UDS_DL_CHECKED;
            s->stats.download.committed++;
            s->stats.download.check_us = took;
        } else {
            uds_download_end(s, true);
        }
    } else {
        /* The confirm leaves the download as it is. */
    }
}

/* startRoutine: the routine's checks, then its operation. Positive:
 * 71 01 <RID> <UDS_ROUTINE_OK> <UDS_NRC_NONE>. A routine that ran keeps its outcome for
 * requestRoutineResults; one refused by its checks did not run. */
static uds_be_t start_routine(uds_server_t *s, uint32_t routine, uds_call_t call, uint64_t now,
                              uds_reply_t *r)
{
    uds_be_t be = UDS_BE_DONE;
    uint8_t nrc = UDS_NRC_NONE;

    if (call == UDS_CALL_START) {
        nrc = start_checks(s, routine);

        if ((nrc == UDS_NRC_NONE) && (routine == ROUTINE_ERASE)) {
            /* Whatever was erased or written before is begun again: nothing is erased for a
             * download until this erase completes. */
            uds_download_end(s, false);
        }
    }

    if (nrc != UDS_NRC_NONE) {
        uds_refuse(r, nrc);
    } else {
        boot_status_t status = BOOT_E_FLASH;

        be = run_routine(s, routine, call, &status);

        if (be == UDS_BE_DONE) {
            nrc = nrc_of_final(status);
            s->routine[routine].valid = true;
            s->routine[routine].nrc = nrc;
            routine_done(s, routine, nrc, now);

            if (nrc != UDS_NRC_NONE) {
                uds_refuse(r, nrc);
            } else {
                uds_put8(r, (uint8_t)(UDS_SID_ROUTINE_CONTROL + UDS_POSITIVE_OFFSET));
                uds_put8(r, UDS_ROUTINE_START);
                uds_put16(r, k_routines[routine].rid);
                uds_put8(r, UDS_ROUTINE_OK);
                uds_put8(r, UDS_NRC_NONE);
            }
        }
    }

    return be;
}

/* requestRoutineResults: 4 bytes (0x13); the routine has run in this session (0x24).
 * Positive: 71 03 <RID> <UDS_ROUTINE_OK or UDS_ROUTINE_FAILED> <the NRC of its start>. */
static void routine_results(uds_server_t *s, uint32_t routine, uds_reply_t *r)
{
    if (s->req_len != ROUTINE_BASE_LEN) {
        uds_refuse(r, UDS_NRC_INCORRECT_LENGTH);
    } else if (!s->routine[routine].valid) {
        uds_refuse(r, UDS_NRC_REQUEST_SEQUENCE_ERROR);
    } else {
        const uint8_t nrc = s->routine[routine].nrc;

        uds_put8(r, (uint8_t)(UDS_SID_ROUTINE_CONTROL + UDS_POSITIVE_OFFSET));
        uds_put8(r, UDS_ROUTINE_RESULTS);
        uds_put16(r, k_routines[routine].rid);
        uds_put8(r, (nrc == UDS_NRC_NONE) ? (uint8_t)UDS_ROUTINE_OK : (uint8_t)UDS_ROUTINE_FAILED);
        uds_put8(r, nrc);
    }
}

/* RoutineControl (0x31): startRoutine or requestRoutineResults (uds.c checked the sub-function
 * and its session). Checks: at least 4 bytes (0x13); a routine of the server (0x31); the
 * routine supported in the active session (0x31); then the sub-function's checks. */
uds_be_t uds_svc_routine(uds_server_t *s, uds_call_t call, uint64_t now_us, uds_reply_t *r)
{
    uds_be_t be = UDS_BE_DONE;
    uint8_t nrc = UDS_NRC_NONE;
    uint32_t routine = UDS_ROUTINE_COUNT;

    if (s->req_len < ROUTINE_BASE_LEN) {
        nrc = UDS_NRC_INCORRECT_LENGTH;
    } else {
        routine = routine_index(uds_get16(s->req, 2U));

        /* A routine the server does not have, or not in this session: both out of range,
         * as ReadDataByIdentifier answers a DID that is not supported in the session. */
        if ((routine >= UDS_ROUTINE_COUNT) ||
            ((k_routines[routine].sessions & uds_session_bit(s->session)) == 0U)) {
            nrc = UDS_NRC_REQUEST_OUT_OF_RANGE;
        }
    }

    if (nrc != UDS_NRC_NONE) {
        uds_refuse(r, nrc);
    } else if (s->subfunction == UDS_ROUTINE_RESULTS) {
        routine_results(s, routine, r);
    } else {
        be = start_routine(s, routine, call, now_us, r);
    }

    return be;
}

/* ---------------------------------------------------------------------------- the download */

/* RequestDownload (0x34). Checks, in this order: at least 3 bytes (0x13); the
 * addressAndLengthFormatIdentifier is 0x44 (0x31); 11 bytes (0x13); the dataFormatIdentifier is
 * 0x00 (0x31); no download in TRANSFER (0x22); an eraseMemory completed (ERASED, else 0x70);
 * its address and size (0x31). Positive: 74 20 <maxNumberOfBlockLength, 2 bytes>. */
uds_be_t uds_svc_request_download(uds_server_t *s, uds_call_t call, uint64_t now_us, uds_reply_t *r)
{
    uint8_t nrc = UDS_NRC_NONE;

    (void)call;

    if (s->req_len < DOWNLOAD_MIN_LEN) {
        nrc = UDS_NRC_INCORRECT_LENGTH;
    }
    if ((nrc == UDS_NRC_NONE) && (s->req[DOWNLOAD_ALFID_AT] != UDS_ADDR_LEN_FORMAT)) {
        nrc = UDS_NRC_REQUEST_OUT_OF_RANGE;
    }
    if ((nrc == UDS_NRC_NONE) && (s->req_len != DOWNLOAD_LEN)) {
        nrc = UDS_NRC_INCORRECT_LENGTH;
    }
    if ((nrc == UDS_NRC_NONE) && (s->req[DOWNLOAD_DFI_AT] != UDS_DATA_FORMAT_PLAIN)) {
        nrc = UDS_NRC_REQUEST_OUT_OF_RANGE;
    }
    if ((nrc == UDS_NRC_NONE) && (s->dl.state == UDS_DL_TRANSFER)) {
        nrc = UDS_NRC_CONDITIONS_NOT_CORRECT;
    }
    if ((nrc == UDS_NRC_NONE) && (s->dl.state != UDS_DL_ERASED)) {
        nrc = UDS_NRC_DOWNLOAD_NOT_ACCEPTED;
    }
    if ((nrc == UDS_NRC_NONE) && ((uds_get32(s->req, DOWNLOAD_ADDR_AT) != s->dl.address) ||
                                  (uds_get32(s->req, DOWNLOAD_SIZE_AT) != s->dl.size))) {
        nrc = UDS_NRC_REQUEST_OUT_OF_RANGE;
    }

    if (nrc != UDS_NRC_NONE) {
        uds_refuse(r, nrc);
    } else {
        s->dl.state = UDS_DL_TRANSFER;
        s->dl.received = 0U;
        s->dl.blocks = 0U;
        s->dl.repeated = 0U;
        s->dl.counter = 0U;
        s->dl.have_block = false;
        s->dl.started_us = now_us;
        s->stats.download.started++;
        uds_put8(r, (uint8_t)(UDS_SID_REQUEST_DOWNLOAD + UDS_POSITIVE_OFFSET));
        uds_put8(r, DOWNLOAD_LFID);
        uds_put16(r, (uint16_t)UDS_MAX_BLOCK_LENGTH);
    }

    return UDS_BE_DONE;
}

static void transfer_positive(uds_reply_t *r, uint8_t counter)
{
    uds_put8(r, (uint8_t)(UDS_SID_TRANSFER_DATA + UDS_POSITIVE_OFFSET));
    uds_put8(r, counter);
}

/* The checks of a block that is no repeat, in this order: not every byte taken already
 * (0x24); the next counter (0x73); some data (0x13); no more data than the bytes left (0x71,
 * which ends the download). */
static uint8_t new_block_checks(uds_server_t *s)
{
    uint8_t nrc = UDS_NRC_NONE;

    if (s->dl.received == s->dl.size) {
        nrc = UDS_NRC_REQUEST_SEQUENCE_ERROR;
    } else if (s->req[1] != (uint8_t)(s->dl.counter + 1U)) {
        nrc = UDS_NRC_WRONG_BLOCK_COUNTER;
    } else if (s->req_len == TRANSFER_HEAD_LEN) {
        nrc = UDS_NRC_INCORRECT_LENGTH;
    } else if ((s->req_len - TRANSFER_HEAD_LEN) > (s->dl.size - s->dl.received)) {
        nrc = UDS_NRC_TRANSFER_SUSPENDED;
        uds_download_end(s, true);
    } else {
        /* the next block, and it fits */
    }

    return nrc;
}

/* The checks of a TransferData block, in this order (uds.h, THE DOWNLOAD): at least 2 bytes
 * (0x13); a download in TRANSFER (0x24); at most maxNumberOfBlockLength bytes (0x31); then the
 * counter of the last block accepted is a repeat (*repeat set, no NRC, no further check); any
 * other block goes through new_block_checks(). */
static uint8_t transfer_checks(uds_server_t *s, bool *repeat)
{
    uint8_t nrc = UDS_NRC_NONE;

    *repeat = false;

    if (s->req_len < TRANSFER_HEAD_LEN) {
        nrc = UDS_NRC_INCORRECT_LENGTH;
    }
    if ((nrc == UDS_NRC_NONE) && (s->dl.state != UDS_DL_TRANSFER)) {
        nrc = UDS_NRC_REQUEST_SEQUENCE_ERROR;
    }
    if ((nrc == UDS_NRC_NONE) && (s->req_len > UDS_MAX_BLOCK_LENGTH)) {
        nrc = UDS_NRC_REQUEST_OUT_OF_RANGE;
    }
    if (nrc == UDS_NRC_NONE) {
        if (s->dl.have_block && (s->req[1] == s->dl.counter)) {
            *repeat = true;
        } else {
            nrc = new_block_checks(s);
        }
    }

    return nrc;
}

/* TransferData (0x36): the checks above, then the backend's write of the block's data at the
 * offset dl.received. Positive: 76 <counter>. A refusal of the backend ends the download. */
uds_be_t uds_svc_transfer_data(uds_server_t *s, uds_call_t call, uint64_t now_us, uds_reply_t *r)
{
    uds_be_t be = UDS_BE_DONE;
    uint8_t nrc = UDS_NRC_NONE;
    bool repeat = false;

    (void)now_us;

    if (call == UDS_CALL_START) {
        nrc = transfer_checks(s, &repeat);
    }

    if (nrc != UDS_NRC_NONE) {
        uds_refuse(r, nrc);
    } else if (repeat) {
        s->dl.repeated++;
        transfer_positive(r, s->req[1]);
    } else {
        const uint32_t len = s->req_len - TRANSFER_HEAD_LEN;
        boot_status_t status = BOOT_E_FLASH;

        be = s->backend.write(s->backend.ctx, call, s->dl.received, &s->req[TRANSFER_HEAD_LEN], len,
                              &status);

        if (be != UDS_BE_DONE) {
            be = UDS_BE_PENDING;
        } else {
            nrc = nrc_of_final(status);

            if (nrc != UDS_NRC_NONE) {
                uds_download_end(s, true);
                uds_refuse(r, nrc);
            } else {
                s->dl.received += len;
                s->dl.blocks++;
                s->dl.counter = s->req[1];
                s->dl.have_block = true;
                transfer_positive(r, s->req[1]);
            }
        }
    }

    return be;
}

/* RequestTransferExit (0x37). Checks: 1 byte, no transferRequestParameterRecord (0x13); a
 * download in TRANSFER (0x24); every byte of its size taken (0x24). Then the backend's first
 * finish step, which succeeds with BOOT_BUSY (or BOOT_OK). Positive: 77. A refusal ends the
 * download. */
uds_be_t uds_svc_transfer_exit(uds_server_t *s, uds_call_t call, uint64_t now_us, uds_reply_t *r)
{
    uds_be_t be = UDS_BE_DONE;
    uint8_t nrc = UDS_NRC_NONE;

    if (call == UDS_CALL_START) {
        if (s->req_len != 1U) {
            nrc = UDS_NRC_INCORRECT_LENGTH;
        } else if ((s->dl.state != UDS_DL_TRANSFER) || (s->dl.received != s->dl.size)) {
            /* no download, or not every byte of it taken */
            nrc = UDS_NRC_REQUEST_SEQUENCE_ERROR;
        } else {
            /* the download is complete */
        }
    }

    if (nrc != UDS_NRC_NONE) {
        uds_refuse(r, nrc);
    } else {
        boot_status_t status = BOOT_E_FLASH;

        be = s->backend.transfer_exit(s->backend.ctx, call, &status);

        if (be != UDS_BE_DONE) {
            be = UDS_BE_PENDING;
        } else {
            nrc = uds_nrc_for_boot(status);

            if (nrc != UDS_NRC_NONE) {
                uds_download_end(s, true);
                uds_refuse(r, nrc);
            } else {
                s->dl.state = UDS_DL_EXITED;
                s->stats.download.exited++;
                s->stats.download.transfer_us = now_us - s->dl.started_us;
                uds_put8(r, (uint8_t)(UDS_SID_TRANSFER_EXIT + UDS_POSITIVE_OFFSET));
            }
        }
    }

    return be;
}
