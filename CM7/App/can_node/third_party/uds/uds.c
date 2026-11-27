/* SPDX-License-Identifier: MIT */
#include "uds/uds.h"

#include <stddef.h>
#include <string.h>

#include "uds/uds_internal.h"

/*
 * What the server does is described in uds.h. This file holds the core (the service table,
 * the general response behaviour, the timing of responsePending and S3server, the statistics)
 * and the services that are not about programming: DiagnosticSessionControl, ECUReset,
 * ClearDiagnosticInformation, ReadDTCInformation, ReadDataByIdentifier and TesterPresent.
 * uds_program.c has RoutineControl and the download.
 *
 * What holds for a uds_server_t between any two calls, from a successful uds_init() on:
 *
 *   - `ready` is READY_MARK and every backend function is set (server_usable());
 *   - `session` is one of UDS_SESSION_*; `s3_running` only in a non-default session;
 *   - `req` holds the first min(req_len, UDS_REQUEST_MAX) bytes of the request processed last.
 *     A handler reads byte i only after a check that req_len > i; no service but TransferData
 *     reads beyond byte 12, and TransferData reads its data only after the check req_len <=
 *     UDS_MAX_BLOCK_LENGTH, which is UDS_REQUEST_MAX: every byte read is in the copy;
 *   - while `busy`, `service` is the index of a table entry with a handler, and the request has
 *     passed every check its handler makes at UDS_CALL_START. Nothing those checks looked at
 *     changes while `busy`: every request that arrives meanwhile is answered with
 *     busyRepeatRequest and not kept, and S3server does not run.
 */

/* The mark of a prepared server. */
#define READY_MARK 0x55445331U

#define US_PER_MS 1000U

/* The times of uds.h, in microseconds. */
#define PENDING_FIRST_US ((uint64_t)UDS_PENDING_FIRST_MS * US_PER_MS)
#define PENDING_EVERY_US ((uint64_t)UDS_PENDING_EVERY_MS * US_PER_MS)
#define S3_US            ((uint64_t)UDS_S3_SERVER_MS * US_PER_MS)

/* P2*server in the DiagnosticSessionControl response has a resolution of 10 ms. */
#define P2_STAR_RESOLUTION_MS 10U

/* A sub-function byte without its SPRMIB. */
#define SUBFUNCTION_MASK 0x7FU
_Static_assert((SUBFUNCTION_MASK | UDS_SPRMIB) == 0xFFU, "the byte is the SPRMIB and the value");

/* An image version's parts (lib/image's encoding, docs/update.md). */
#define VERSION_MAJOR_SHIFT 24U
#define VERSION_MINOR_SHIFT 16U
#define VERSION_PART_MASK   0xFFU
#define VERSION_PATCH_MASK  0xFFFFU

_Static_assert(UDS_P2_SERVER_MS <= 0xFFFFU, "P2server is reported in 16 bits of 1 ms");
_Static_assert((UDS_P2_STAR_SERVER_MS % P2_STAR_RESOLUTION_MS) == 0U,
               "P2*server is reported in units of 10 ms");
_Static_assert((UDS_P2_STAR_SERVER_MS / P2_STAR_RESOLUTION_MS) <= 0xFFFFU,
               "P2*server is reported in 16 bits of 10 ms");
_Static_assert(UDS_PENDING_FIRST_MS < UDS_P2_SERVER_MS,
               "the first responsePending comes before P2server runs out");
_Static_assert(UDS_PENDING_EVERY_MS < UDS_P2_STAR_SERVER_MS,
               "each next responsePending comes before P2*server runs out");
/* UDS_RESPONSE_MAX is the DTC list with every DTC (uds.h); every other response is shorter. */
_Static_assert(UDS_RESPONSE_MAX >= (3U + UDS_BOOT_INFO_SIZE), "DID 0xFD00 fits");
_Static_assert(UDS_RESPONSE_MAX >= (3U + 13U), "DID 0xF189, 255.255.65535, fits");
_Static_assert(UDS_RESPONSE_MAX >= 6U, "the DiagnosticSessionControl response fits");
_Static_assert(UDS_NEGATIVE_LEN == 3U, "7F, the SID, the NRC");
_Static_assert(DTC_EVENTS_MAX <= 0xFFFFU, "the DTC count of 0x19 0x01 has 16 bits");

/* One sub-function of a service: its value (SPRMIB removed) and the sessions it is in. */
typedef struct {
    uint8_t value;
    uint8_t sessions;
} subfunction_t;

/* One entry of the service table. */
typedef struct {
    uint8_t sid;
    uint8_t sessions;          /* UDS_IN_*: the sessions the service is supported in */
    bool suppressible;         /* bit 7 of the sub-function is the SPRMIB */
    uint8_t subfunction_count; /* 0: no sub-function */
    const subfunction_t *subfunctions;
    uds_handler_t handler;
} service_t;

static uds_be_t svc_session(uds_server_t *s, uds_call_t call, uint64_t now_us, uds_reply_t *r);
static uds_be_t svc_reset(uds_server_t *s, uds_call_t call, uint64_t now_us, uds_reply_t *r);
static uds_be_t svc_clear_dtc(uds_server_t *s, uds_call_t call, uint64_t now_us, uds_reply_t *r);
static uds_be_t svc_read_dtc(uds_server_t *s, uds_call_t call, uint64_t now_us, uds_reply_t *r);
static uds_be_t svc_read_did(uds_server_t *s, uds_call_t call, uint64_t now_us, uds_reply_t *r);
static uds_be_t svc_tester_present(uds_server_t *s, uds_call_t call, uint64_t now_us,
                                   uds_reply_t *r);

/* DiagnosticSessionControl: the sub-function is the session asked for, and its sessions are
 * those it may be asked from (uds.h, SESSIONS). */
#define SESSION_SUBFUNCTIONS 3U
static const subfunction_t k_session_subfunctions[SESSION_SUBFUNCTIONS] = {
    {UDS_SESSION_DEFAULT, UDS_IN_ALL},
    {UDS_SESSION_PROGRAMMING, UDS_IN_PROGRAMMING | UDS_IN_EXTENDED},
    {UDS_SESSION_EXTENDED, UDS_IN_DEFAULT | UDS_IN_EXTENDED},
};

#define RESET_SUBFUNCTIONS 1U
static const subfunction_t k_reset_subfunctions[RESET_SUBFUNCTIONS] = {
    {UDS_RESET_HARD, UDS_IN_ALL},
};

#define DTC_SUBFUNCTIONS 3U
static const subfunction_t k_dtc_subfunctions[DTC_SUBFUNCTIONS] = {
    {UDS_DTC_COUNT_BY_MASK, UDS_IN_DEFAULT | UDS_IN_EXTENDED},
    {UDS_DTC_BY_MASK, UDS_IN_DEFAULT | UDS_IN_EXTENDED},
    {UDS_DTC_SUPPORTED, UDS_IN_DEFAULT | UDS_IN_EXTENDED},
};

#define ROUTINE_SUBFUNCTIONS 2U
static const subfunction_t k_routine_subfunctions[ROUTINE_SUBFUNCTIONS] = {
    {UDS_ROUTINE_START, UDS_IN_PROGRAMMING | UDS_IN_EXTENDED},
    {UDS_ROUTINE_RESULTS, UDS_IN_PROGRAMMING | UDS_IN_EXTENDED},
};

#define TESTER_SUBFUNCTIONS 1U
static const subfunction_t k_tester_subfunctions[TESTER_SUBFUNCTIONS] = {
    {UDS_TESTER_PRESENT_ZERO, UDS_IN_ALL},
};

/* The service table, in the order of the SIDs and of uds_stats_t.requests. */
static const service_t k_services[UDS_SERVICE_COUNT] = {
    {UDS_SID_SESSION_CONTROL, UDS_IN_ALL, true, SESSION_SUBFUNCTIONS, k_session_subfunctions,
     svc_session},
    {UDS_SID_ECU_RESET, UDS_IN_ALL, true, RESET_SUBFUNCTIONS, k_reset_subfunctions, svc_reset},
    {UDS_SID_CLEAR_DTC, UDS_IN_DEFAULT | UDS_IN_EXTENDED, false, 0U, NULL, svc_clear_dtc},
    {UDS_SID_READ_DTC, UDS_IN_DEFAULT | UDS_IN_EXTENDED, false, DTC_SUBFUNCTIONS,
     k_dtc_subfunctions, svc_read_dtc},
    {UDS_SID_READ_DID, UDS_IN_ALL, false, 0U, NULL, svc_read_did},
    {UDS_SID_ROUTINE_CONTROL, UDS_IN_PROGRAMMING | UDS_IN_EXTENDED, true, ROUTINE_SUBFUNCTIONS,
     k_routine_subfunctions, uds_svc_routine},
    {UDS_SID_REQUEST_DOWNLOAD, UDS_IN_PROGRAMMING, false, 0U, NULL, uds_svc_request_download},
    {UDS_SID_TRANSFER_DATA, UDS_IN_PROGRAMMING, false, 0U, NULL, uds_svc_transfer_data},
    {UDS_SID_TRANSFER_EXIT, UDS_IN_PROGRAMMING, false, 0U, NULL, uds_svc_transfer_exit},
    {UDS_SID_TESTER_PRESENT, UDS_IN_ALL, true, TESTER_SUBFUNCTIONS, k_tester_subfunctions,
     svc_tester_present},
};

/* The NRCs the server sends, 0x78 not among them, in the order of uds_stats_t.nrc. */
static const uint8_t k_nrc_codes[UDS_NRC_KINDS] = {
    UDS_NRC_GENERAL_REJECT,          UDS_NRC_SERVICE_NOT_SUPPORTED,
    UDS_NRC_SUBFUNCTION_UNSUPPORTED, UDS_NRC_INCORRECT_LENGTH,
    UDS_NRC_RESPONSE_TOO_LONG,       UDS_NRC_BUSY_REPEAT_REQUEST,
    UDS_NRC_CONDITIONS_NOT_CORRECT,  UDS_NRC_REQUEST_SEQUENCE_ERROR,
    UDS_NRC_REQUEST_OUT_OF_RANGE,    UDS_NRC_DOWNLOAD_NOT_ACCEPTED,
    UDS_NRC_TRANSFER_SUSPENDED,      UDS_NRC_PROGRAMMING_FAILURE,
    UDS_NRC_WRONG_BLOCK_COUNTER,     UDS_NRC_SUBFUNCTION_NOT_IN_SESSION,
    UDS_NRC_SERVICE_NOT_IN_SESSION,
};

/* ------------------------------------------------------------------------------- helpers */

void uds_put8(uds_reply_t *r, uint8_t value)
{
    if (r->len < r->cap) {
        r->buf[r->len] = value;
        r->len++;
    } else {
        r->overflow = true;
    }
}

void uds_put16(uds_reply_t *r, uint16_t value)
{
    uds_put8(r, (uint8_t)(value >> 8U));
    uds_put8(r, (uint8_t)(value & 0xFFU));
}

void uds_put32(uds_reply_t *r, uint32_t value)
{
    uds_put16(r, (uint16_t)(value >> 16U));
    uds_put16(r, (uint16_t)(value & 0xFFFFU));
}

void uds_refuse(uds_reply_t *r, uint8_t nrc)
{
    r->nrc = nrc;
}

uint16_t uds_get16(const uint8_t *bytes, uint32_t at)
{
    return (uint16_t)(((uint16_t)bytes[at] << 8U) | (uint16_t)bytes[at + 1U]);
}

uint32_t uds_get32(const uint8_t *bytes, uint32_t at)
{
    return ((uint32_t)uds_get16(bytes, at) << 16U) | (uint32_t)uds_get16(bytes, at + 2U);
}

uint64_t uds_add_sat(uint64_t a, uint64_t b)
{
    return (b > (UINT64_MAX - a)) ? UINT64_MAX : (a + b);
}

uint8_t uds_session_bit(uint8_t session)
{
    uint8_t bit = 0U;

    switch (session) {
    case UDS_SESSION_DEFAULT:
        bit = UDS_IN_DEFAULT;
        break;
    case UDS_SESSION_PROGRAMMING:
        bit = UDS_IN_PROGRAMMING;
        break;
    case UDS_SESSION_EXTENDED:
        bit = UDS_IN_EXTENDED;
        break;
    default:
        /* not a session: no bit */
        break;
    }

    return bit;
}

static bool backend_complete(const uds_backend_t *b)
{
    return (b->ecu_reset != NULL) && (b->boot_info != NULL) && (b->dtc_snapshot != NULL) &&
           (b->dtc_clear != NULL) && (b->erase != NULL) && (b->write != NULL) &&
           (b->transfer_exit != NULL) && (b->check != NULL) && (b->confirm != NULL);
}

static bool server_usable(const uds_server_t *s)
{
    return (s != NULL) && (s->ready == READY_MARK) && backend_complete(&s->backend);
}

/* The index of `nrc` in k_nrc_codes, or UDS_NRC_KINDS. */
static uint32_t nrc_index(uint8_t nrc)
{
    uint32_t index = UDS_NRC_KINDS;

    for (uint32_t i = 0U; i < UDS_NRC_KINDS; i++) {
        if (k_nrc_codes[i] == nrc) {
            index = i;
        }
    }

    return index;
}

/* An NRC from a backend: one the server names, or generalReject (uds.h, THE BACKEND). */
static uint8_t backend_nrc(uint8_t nrc)
{
    return (nrc_index(nrc) < UDS_NRC_KINDS) ? nrc : UDS_NRC_GENERAL_REJECT;
}

/* The table entry of `sid`, as an index, or UDS_SERVICE_COUNT. */
static uint32_t service_index(uint8_t sid)
{
    uint32_t index = UDS_SERVICE_COUNT;

    for (uint32_t i = 0U; i < UDS_SERVICE_COUNT; i++) {
        if (k_services[i].sid == sid) {
            index = i;
        }
    }

    return index;
}

/* The caller's time, never earlier than a time it gave before (uds.h). */
static uint64_t take_time(uds_server_t *s, uint64_t now_us)
{
    uint64_t now = now_us;

    if (now < s->last_us) {
        now = s->last_us;
    } else {
        s->last_us = now;
    }

    return now;
}

/* Write the negative response 7F <sid> <nrc> into `rsp` and count it; its length. `nrc` is
 * 0x78 or a code of k_nrc_codes (every NRC from a backend went through backend_nrc()). */
static uint32_t negative(uds_server_t *s, uint8_t *rsp, uint8_t sid, uint8_t nrc)
{
    const uint32_t index = nrc_index(nrc);

    rsp[0] = UDS_SID_NEGATIVE;
    rsp[1] = sid;
    rsp[2] = nrc;

    if (nrc == UDS_NRC_RESPONSE_PENDING) {
        s->stats.response_pending++;
    } else if (index < UDS_NRC_KINDS) {
        s->stats.nrc[index]++;
    } else {
        /* Not reached: see above. */
    }

    return UDS_NEGATIVE_LEN;
}

/* Enter `session` as a session begins (uds.h, SESSIONS): counted if it is another one; the
 * download ends, the routines' results are forgotten; S3server is started by the response. */
static void session_begins(uds_server_t *s, uint8_t session)
{
    if (session != s->session) {
        s->stats.session_changes++;
    }

    s->session = session;
    s->s3_running = false;
    uds_program_session_begins(s);
}

/* S3server (uds.h, TIMING). */
static void check_s3(uds_server_t *s, uint64_t now)
{
    if ((!s->busy) && s->s3_running && ((now - s->s3_start_us) >= S3_US)) {
        s->stats.s3_timeouts++;
        session_begins(s, UDS_SESSION_DEFAULT);
    }
}

static void reply_init(uds_reply_t *r, uint8_t *rsp, uint32_t rsp_cap)
{
    r->buf = rsp;
    r->cap = rsp_cap;
    r->len = 0U;
    r->overflow = false;
    r->nrc = UDS_NRC_NONE;
    r->reset = false;
}

/* The request is done: hand out its response, which `r` built in `rsp` (uds.h, THE GENERAL
 * RESPONSE BEHAVIOUR, step 8, and SUPPRESSPOSRSPMSGINDICATIONBIT), and start S3server. */
static void finish(uds_server_t *s, const uds_reply_t *r, uint8_t *rsp, uint64_t now,
                   uds_output_t *out)
{
    const uint64_t pending = now - s->request_us;
    uint8_t nrc = r->nrc;

    if ((nrc == UDS_NRC_NONE) && r->overflow) {
        nrc = UDS_NRC_RESPONSE_TOO_LONG;
    }

    if (nrc != UDS_NRC_NONE) {
        out->len = negative(s, rsp, s->req[0], nrc);
    } else {
        if (s->suppress && (!s->pending_sent)) {
            out->len = 0U;
            s->stats.suppressed++;
        } else {
            out->len = r->len;
            s->stats.positive++;
        }

        if (r->reset) {
            out->reset = true;
            s->stats.resets++;
        }
    }

    if (pending > s->stats.pending_max_us) {
        s->stats.pending_max_us = pending;
    }

    s->busy = false;
    s->s3_running = (s->session != UDS_SESSION_DEFAULT);
    s->s3_start_us = now;
}

/* Checks 2 to 6 of THE GENERAL RESPONSE BEHAVIOUR for a request kept in `s`; sets
 * s->subfunction and s->suppress. UDS_NRC_NONE if the request goes on to its handler. */
static uint8_t general_checks(uds_server_t *s, uint32_t index)
{
    uint8_t nrc = UDS_NRC_NONE;

    if (index >= UDS_SERVICE_COUNT) {
        nrc = UDS_NRC_SERVICE_NOT_SUPPORTED;
    } else if ((k_services[index].sessions & uds_session_bit(s->session)) == 0U) {
        nrc = UDS_NRC_SERVICE_NOT_IN_SESSION;
    } else if (k_services[index].subfunction_count == 0U) {
        nrc = UDS_NRC_NONE;
    } else if (s->req_len < 2U) {
        nrc = UDS_NRC_INCORRECT_LENGTH;
    } else {
        const service_t *svc = &k_services[index];
        uint8_t value = s->req[1];
        uint8_t sessions = 0U;
        bool found = false;

        if (svc->suppressible) {
            s->suppress = ((value & UDS_SPRMIB) != 0U);
            value = (uint8_t)(value & SUBFUNCTION_MASK);
        }

        for (uint32_t i = 0U; i < svc->subfunction_count; i++) {
            if (svc->subfunctions[i].value == value) {
                found = true;
                sessions = svc->subfunctions[i].sessions;
            }
        }

        if (!found) {
            nrc = UDS_NRC_SUBFUNCTION_UNSUPPORTED;
        } else if ((sessions & uds_session_bit(s->session)) == 0U) {
            nrc = UDS_NRC_SUBFUNCTION_NOT_IN_SESSION;
        } else {
            s->subfunction = value;
        }
    }

    return nrc;
}

/* A request while none is pending: keep it, check it, start its handler. */
static void start_request(uds_server_t *s, uint32_t index, const uint8_t *req, uint32_t len,
                          uint64_t now, uint8_t *rsp, uint32_t rsp_cap, uds_output_t *out)
{
    uds_reply_t reply;
    const uint32_t kept = (len < UDS_REQUEST_MAX) ? len : UDS_REQUEST_MAX;

    (void)memcpy(s->req, req, kept);
    s->req_len = len;
    s->service = (uint8_t)index;
    s->subfunction = 0U;
    s->suppress = false;
    s->pending_sent = false;
    s->request_us = now;
    s->next_pending_us = uds_add_sat(now, PENDING_FIRST_US);
    s->s3_running = false;
    reply_init(&reply, rsp, rsp_cap);

    const uint8_t nrc = general_checks(s, index);

    if (nrc != UDS_NRC_NONE) {
        uds_refuse(&reply, nrc);
        finish(s, &reply, rsp, now, out);
    } else if (k_services[index].handler(s, UDS_CALL_START, now, &reply) == UDS_BE_PENDING) {
        s->busy = true;
    } else {
        finish(s, &reply, rsp, now, out);
    }
}

/* --------------------------------------------------------------- the services of this file */

/* DiagnosticSessionControl (0x10): 2 bytes. The session begins, and the response reports the
 * timing: 50 <session> <P2server_max, 2 bytes, 1 ms> <P2*server_max, 2 bytes, 10 ms>. */
static uds_be_t svc_session(uds_server_t *s, uds_call_t call, uint64_t now_us, uds_reply_t *r)
{
    (void)call;
    (void)now_us;

    if (s->req_len != 2U) {
        uds_refuse(r, UDS_NRC_INCORRECT_LENGTH);
    } else {
        session_begins(s, s->subfunction);
        uds_put8(r, (uint8_t)(UDS_SID_SESSION_CONTROL + UDS_POSITIVE_OFFSET));
        uds_put8(r, s->subfunction);
        uds_put16(r, (uint16_t)UDS_P2_SERVER_MS);
        uds_put16(r, (uint16_t)(UDS_P2_STAR_SERVER_MS / P2_STAR_RESOLUTION_MS));
    }

    return UDS_BE_DONE;
}

/* ECUReset (0x11), hardReset: 2 bytes; the backend accepts or refuses; positive 51 01, and the
 * reset is reported with it. */
static uds_be_t svc_reset(uds_server_t *s, uds_call_t call, uint64_t now_us, uds_reply_t *r)
{
    uds_be_t be = UDS_BE_DONE;

    (void)now_us;

    if ((call == UDS_CALL_START) && (s->req_len != 2U)) {
        uds_refuse(r, UDS_NRC_INCORRECT_LENGTH);
    } else {
        uint8_t nrc = UDS_NRC_NONE;

        be = s->backend.ecu_reset(s->backend.ctx, call, s->subfunction, &nrc);

        if (be != UDS_BE_DONE) {
            be = UDS_BE_PENDING;
        } else if (nrc != UDS_NRC_NONE) {
            uds_refuse(r, backend_nrc(nrc));
        } else {
            uds_put8(r, (uint8_t)(UDS_SID_ECU_RESET + UDS_POSITIVE_OFFSET));
            uds_put8(r, s->subfunction);
            r->reset = true;
        }
    }

    return be;
}

/* The groupOfDTC of a ClearDiagnosticInformation request of at least 4 bytes. */
static uint32_t dtc_group(const uds_server_t *s)
{
    return ((uint32_t)s->req[1] << 16U) | (uint32_t)uds_get16(s->req, 2U);
}

/* ClearDiagnosticInformation (0x14): 14 <groupOfDTC, 3 bytes>. Checks: 4 or 5 bytes (0x13);
 * the group is all groups, 0xFFFFFF (0x31); no MemorySelection, a fifth byte (0x31). Then the
 * backend's clear; positive 54 once it is done. */
static uds_be_t svc_clear_dtc(uds_server_t *s, uds_call_t call, uint64_t now_us, uds_reply_t *r)
{
    uds_be_t be = UDS_BE_DONE;
    uint8_t nrc = UDS_NRC_NONE;

    (void)now_us;

    if (call == UDS_CALL_START) {
        if ((s->req_len != 4U) && (s->req_len != 5U)) {
            nrc = UDS_NRC_INCORRECT_LENGTH;
        } else if ((dtc_group(s) != UDS_DTC_GROUP_ALL) || (s->req_len == 5U)) {
            /* A group other than all, or a MemorySelection (a user-defined DTC memory, which
             * the server does not have): both out of range. */
            nrc = UDS_NRC_REQUEST_OUT_OF_RANGE;
        } else {
            /* all groups, no MemorySelection */
        }
    }

    if (nrc != UDS_NRC_NONE) {
        uds_refuse(r, nrc);
    } else {
        uint8_t backend = UDS_NRC_NONE;

        be = s->backend.dtc_clear(s->backend.ctx, call, UDS_DTC_GROUP_ALL, &backend);

        if (be != UDS_BE_DONE) {
            be = UDS_BE_PENDING;
        } else if (backend != UDS_NRC_NONE) {
            uds_refuse(r, backend_nrc(backend));
        } else {
            uds_put8(r, (uint8_t)(UDS_SID_CLEAR_DTC + UDS_POSITIVE_OFFSET));
        }
    }

    return be;
}

static void put_dtc(uds_reply_t *r, const dtc_entry_t *entry)
{
    uds_put8(r, (uint8_t)((entry->dtc >> 16U) & 0xFFU));
    uds_put16(r, (uint16_t)(entry->dtc & 0xFFFFU));
    uds_put8(r, (uint8_t)(entry->status & DTC_STATUS_AVAILABILITY_MASK));
}

/* The positive response of ReadDTCInformation from the snapshot `state`. */
static void dtc_response(const uds_server_t *s, const dtc_state_t *state, uds_reply_t *r)
{
    const uint8_t report = s->subfunction;

    uds_put8(r, (uint8_t)(UDS_SID_READ_DTC + UDS_POSITIVE_OFFSET));
    uds_put8(r, report);
    uds_put8(r, (uint8_t)DTC_STATUS_AVAILABILITY_MASK);

    if (report == UDS_DTC_COUNT_BY_MASK) {
        const uint8_t mask = (uint8_t)(s->req[2] & DTC_STATUS_AVAILABILITY_MASK);

        uds_put8(r, UDS_DTC_FORMAT_ISO14229);
        uds_put16(r, dtc_count_by_status_mask(state, mask));
    } else if (report == UDS_DTC_BY_MASK) {
        const uint8_t mask = (uint8_t)(s->req[2] & DTC_STATUS_AVAILABILITY_MASK);
        uint16_t cursor = 0U;
        dtc_entry_t entry;

        while (dtc_next_by_status_mask(state, mask, &cursor, &entry)) {
            put_dtc(r, &entry);
        }
    } else {
        /* reportSupportedDTC: every DTC of the table, whatever its status. */
        bool more = true;

        for (uint16_t event = 0U; more && (event < DTC_EVENTS_MAX); event++) {
            dtc_entry_t entry;

            more = dtc_get(state, event, &entry);
            if (more) {
                put_dtc(r, &entry);
            }
        }
    }
}

/* ReadDTCInformation (0x19): 19 01 <mask>, 19 02 <mask>, 19 0A. Checks: the length of the
 * reportType, 3 or 2 bytes (0x13). Then the backend's snapshot. Positive:
 *   59 01 <availability mask> <DTCFormatIdentifier 0x01> <count, 2 bytes>
 *   59 02 <availability mask> { <DTC, 3 bytes> <status> } for each DTC that matches the mask
 *   59 0A <availability mask> { <DTC, 3 bytes> <status> } for each DTC */
static uds_be_t svc_read_dtc(uds_server_t *s, uds_call_t call, uint64_t now_us, uds_reply_t *r)
{
    uds_be_t be = UDS_BE_DONE;
    uint32_t length = 3U; /* reportType and DTCStatusMask */

    (void)now_us;

    if (s->subfunction == UDS_DTC_SUPPORTED) {
        length = 2U; /* reportType alone */
    }

    if ((call == UDS_CALL_START) && (s->req_len != length)) {
        uds_refuse(r, UDS_NRC_INCORRECT_LENGTH);
    } else {
        const dtc_state_t *state = NULL;
        uint8_t nrc = UDS_NRC_NONE;

        be = s->backend.dtc_snapshot(s->backend.ctx, call, &state, &nrc);

        if (be != UDS_BE_DONE) {
            be = UDS_BE_PENDING;
        } else if (nrc != UDS_NRC_NONE) {
            uds_refuse(r, backend_nrc(nrc));
        } else if (state == NULL) {
            uds_refuse(r, UDS_NRC_GENERAL_REJECT);
        } else {
            dtc_response(s, state, r);
        }
    }

    return be;
}

/* "major.minor.patch" in decimal, no leading zeros, no terminator. */
static void put_decimal(uds_reply_t *r, uint32_t value)
{
    uint8_t digits[5];
    uint32_t count = 0U;
    uint32_t rest = value;

    do {
        digits[count] = (uint8_t)((uint8_t)'0' + (uint8_t)(rest % 10U));
        count++;
        rest /= 10U;
    } while ((rest != 0U) && (count < 5U));

    while (count > 0U) {
        count--;
        uds_put8(r, digits[count]);
    }
}

static void put_version(uds_reply_t *r, uint32_t version)
{
    put_decimal(r, (version >> VERSION_MAJOR_SHIFT) & VERSION_PART_MASK);
    uds_put8(r, (uint8_t)'.');
    put_decimal(r, (version >> VERSION_MINOR_SHIFT) & VERSION_PART_MASK);
    uds_put8(r, (uint8_t)'.');
    put_decimal(r, version & VERSION_PATCH_MASK);
}

/* The record of DID 0xFD00 (docs/diagnostics.md): without a hand-off the hand-off's fields are
 * written as none. */
static void put_boot_info(uds_reply_t *r, const uds_boot_info_t *info)
{
    uds_put8(r, UDS_BOOT_INFO_VERSION);
    if (info->handoff) {
        uds_put8(r, 1U);
        uds_put8(r, info->slot);
        uds_put8(r, info->mode);
        uds_put8(r, info->attempt);
        uds_put8(r, info->confirm_state);
        uds_put8(r, info->event);
        uds_put8(r, info->event_slot);
        uds_put32(r, info->image_version);
    } else {
        uds_put8(r, 0U);
        uds_put8(r, UDS_BOOT_NO_SLOT);
        uds_put8(r, 0U);
        uds_put8(r, 0U);
        uds_put8(r, info->confirm_state);
        uds_put8(r, 0U);
        uds_put8(r, UDS_BOOT_NO_SLOT);
        uds_put32(r, 0U);
    }
}

/* ReadDataByIdentifier (0x22): 22 <DID, 2 bytes>, one DID. Checks: 3 bytes (0x13: also for
 * more than one DID, more than the server takes at a time); a DID of the server (0x31). F186
 * is the server's own; F189 and FD00 come from the backend's boot information, and F189 is
 * refused (0x22) without a hand-off: there is no image header to give a version. Positive:
 * 62 <DID> <data>. */
static uds_be_t svc_read_did(uds_server_t *s, uds_call_t call, uint64_t now_us, uds_reply_t *r)
{
    uds_be_t be = UDS_BE_DONE;
    const uint16_t did = (s->req_len >= 3U) ? uds_get16(s->req, 1U) : 0U;

    (void)now_us;

    if ((call == UDS_CALL_START) && (s->req_len != 3U)) {
        uds_refuse(r, UDS_NRC_INCORRECT_LENGTH);
    } else if (did == UDS_DID_ACTIVE_SESSION) {
        uds_put8(r, (uint8_t)(UDS_SID_READ_DID + UDS_POSITIVE_OFFSET));
        uds_put16(r, did);
        uds_put8(r, s->session);
    } else if ((did == UDS_DID_SOFTWARE_VERSION) || (did == UDS_DID_BOOT_INFO)) {
        uds_boot_info_t info;
        uint8_t nrc = UDS_NRC_NONE;

        (void)memset(&info, 0, sizeof(info));
        be = s->backend.boot_info(s->backend.ctx, call, &info, &nrc);

        if (be != UDS_BE_DONE) {
            be = UDS_BE_PENDING;
        } else if (nrc != UDS_NRC_NONE) {
            uds_refuse(r, backend_nrc(nrc));
        } else if ((did == UDS_DID_SOFTWARE_VERSION) && (!info.handoff)) {
            uds_refuse(r, UDS_NRC_CONDITIONS_NOT_CORRECT);
        } else {
            uds_put8(r, (uint8_t)(UDS_SID_READ_DID + UDS_POSITIVE_OFFSET));
            uds_put16(r, did);
            if (did == UDS_DID_SOFTWARE_VERSION) {
                put_version(r, info.image_version);
            } else {
                put_boot_info(r, &info);
            }
        }
    } else {
        uds_refuse(r, UDS_NRC_REQUEST_OUT_OF_RANGE);
    }

    return be;
}

/* TesterPresent (0x3E), zeroSubFunction: 2 bytes; positive 7E 00. Its work is S3server's
 * restart, which every request makes. The signature is uds_handler_t's, the type of every
 * handler in the service table: `s` cannot be const although this handler only reads it. */
/* cppcheck-suppress constParameterPointer */
static uds_be_t svc_tester_present(uds_server_t *s, uds_call_t call, uint64_t now_us,
                                   uds_reply_t *r)
{
    (void)call;
    (void)now_us;

    if (s->req_len != 2U) {
        uds_refuse(r, UDS_NRC_INCORRECT_LENGTH);
    } else {
        uds_put8(r, (uint8_t)(UDS_SID_TESTER_PRESENT + UDS_POSITIVE_OFFSET));
        uds_put8(r, s->subfunction);
    }

    return UDS_BE_DONE;
}

/* ---------------------------------------------------------------------------- the interface */

bool uds_init(uds_server_t *s, const uds_backend_t *backend)
{
    bool ok = (s != NULL) && (backend != NULL);

    if (ok) {
        ok = backend_complete(backend);
    }

    if (ok) {
        (void)memset(s, 0, sizeof(*s));
        s->backend = *backend;
        s->session = UDS_SESSION_DEFAULT;
        s->dl.state = UDS_DL_IDLE;
        s->stats.download.state = UDS_DL_IDLE;
        s->ready = READY_MARK;
    }

    return ok;
}

static bool call_valid(const uds_server_t *s, const uint8_t *rsp, uint32_t rsp_cap,
                       uds_output_t *out)
{
    bool ok = (out != NULL);

    if (ok) {
        out->len = 0U;
        out->reset = false;
        ok = server_usable(s) && (rsp != NULL) && (rsp_cap >= UDS_NEGATIVE_LEN);
    }

    return ok;
}

bool uds_request(uds_server_t *s, const uint8_t *req, uint32_t len, uint64_t now_us, uint8_t *rsp,
                 uint32_t rsp_cap, uds_output_t *out)
{
    bool ok = call_valid(s, rsp, rsp_cap, out);

    if (ok) {
        ok = (req != NULL) || (len == 0U);
    }

    if (ok) {
        const uint64_t now = take_time(s, now_us);

        check_s3(s, now);

        if (len == 0U) {
            s->stats.requests_empty++;
        } else {
            const uint32_t index = service_index(req[0]);

            if (index < UDS_SERVICE_COUNT) {
                s->stats.requests[index]++;
            } else {
                s->stats.requests_unsupported++;
            }

            if (s->busy) {
                out->len = negative(s, rsp, req[0], UDS_NRC_BUSY_REPEAT_REQUEST);
            } else {
                start_request(s, index, req, len, now, rsp, rsp_cap, out);
            }
        }
    }

    return ok;
}

bool uds_poll(uds_server_t *s, uint64_t now_us, uint8_t *rsp, uint32_t rsp_cap, uds_output_t *out)
{
    const bool ok = call_valid(s, rsp, rsp_cap, out);

    if (ok) {
        const uint64_t now = take_time(s, now_us);

        check_s3(s, now);

        if (s->busy) {
            uds_reply_t reply;

            reply_init(&reply, rsp, rsp_cap);

            if (k_services[s->service].handler(s, UDS_CALL_POLL, now, &reply) == UDS_BE_DONE) {
                finish(s, &reply, rsp, now, out);
            } else if (now >= s->next_pending_us) {
                out->len = negative(s, rsp, s->req[0], UDS_NRC_RESPONSE_PENDING);
                s->pending_sent = true;
                s->next_pending_us = uds_add_sat(now, PENDING_EVERY_US);
            } else {
                out->len = 0U;
            }
        }
    }

    return ok;
}

uint64_t uds_next_deadline(const uds_server_t *s)
{
    uint64_t deadline = UDS_NO_DEADLINE;

    if (server_usable(s)) {
        if (s->busy) {
            deadline = s->next_pending_us;
        } else if (s->s3_running) {
            deadline = uds_add_sat(s->s3_start_us, S3_US);
        } else {
            /* nothing is due: UDS_NO_DEADLINE */
        }
    }

    return deadline;
}

bool uds_busy(const uds_server_t *s)
{
    return server_usable(s) && s->busy;
}

uint8_t uds_session(const uds_server_t *s)
{
    return server_usable(s) ? s->session : (uint8_t)UDS_SESSION_DEFAULT;
}

void uds_get_stats(const uds_server_t *s, uds_stats_t *out)
{
    if (out != NULL) {
        if (server_usable(s)) {
            *out = s->stats;
            out->download.state = s->dl.state;
            out->download.address = s->dl.address;
            out->download.size = s->dl.size;
            out->download.received = s->dl.received;
            out->download.blocks = s->dl.blocks;
            out->download.repeated_blocks = s->dl.repeated;
            out->download.counter = s->dl.counter;
        } else {
            (void)memset(out, 0, sizeof(*out));
        }
    }
}

uint8_t uds_nrc_for_boot(boot_status_t status)
{
    uint8_t nrc = UDS_NRC_PROGRAMMING_FAILURE;

    switch (status) {
    case BOOT_OK:
    case BOOT_BUSY:
        nrc = UDS_NRC_NONE;
        break;
    case BOOT_E_ARG:
    case BOOT_E_SIZE:
        nrc = UDS_NRC_REQUEST_OUT_OF_RANGE;
        break;
    case BOOT_E_STATE:
        nrc = UDS_NRC_CONDITIONS_NOT_CORRECT;
        break;
    case BOOT_E_SEQUENCE:
        nrc = UDS_NRC_REQUEST_SEQUENCE_ERROR;
        break;
    case BOOT_E_INVALID:
    case BOOT_E_FLASH:
    case BOOT_E_POWER_LOST:
    case BOOT_E_LAYOUT:
    case BOOT_E_NOTHING_BOOTABLE:
    default:
        /* generalProgrammingFailure, as initialised */
        break;
    }

    return nrc;
}

uint8_t uds_service_sid(uint32_t index)
{
    return (index < UDS_SERVICE_COUNT) ? k_services[index].sid : 0U;
}

uint8_t uds_nrc_code(uint32_t index)
{
    return (index < UDS_NRC_KINDS) ? k_nrc_codes[index] : 0U;
}

const char *uds_service_name(uint8_t sid)
{
    const char *name = "?";

    switch (sid) {
    case UDS_SID_SESSION_CONTROL:
        name = "DiagnosticSessionControl";
        break;
    case UDS_SID_ECU_RESET:
        name = "ECUReset";
        break;
    case UDS_SID_CLEAR_DTC:
        name = "ClearDiagnosticInformation";
        break;
    case UDS_SID_READ_DTC:
        name = "ReadDTCInformation";
        break;
    case UDS_SID_READ_DID:
        name = "ReadDataByIdentifier";
        break;
    case UDS_SID_ROUTINE_CONTROL:
        name = "RoutineControl";
        break;
    case UDS_SID_REQUEST_DOWNLOAD:
        name = "RequestDownload";
        break;
    case UDS_SID_TRANSFER_DATA:
        name = "TransferData";
        break;
    case UDS_SID_TRANSFER_EXIT:
        name = "RequestTransferExit";
        break;
    case UDS_SID_TESTER_PRESENT:
        name = "TesterPresent";
        break;
    default:
        /* a value the server does not know: "?" */
        break;
    }

    return name;
}

const char *uds_nrc_name(uint8_t nrc)
{
    const char *name = "?";

    switch (nrc) {
    case UDS_NRC_GENERAL_REJECT:
        name = "generalReject";
        break;
    case UDS_NRC_SERVICE_NOT_SUPPORTED:
        name = "serviceNotSupported";
        break;
    case UDS_NRC_SUBFUNCTION_UNSUPPORTED:
        name = "subFunctionNotSupported";
        break;
    case UDS_NRC_INCORRECT_LENGTH:
        name = "incorrectMessageLengthOrInvalidFormat";
        break;
    case UDS_NRC_RESPONSE_TOO_LONG:
        name = "responseTooLong";
        break;
    case UDS_NRC_BUSY_REPEAT_REQUEST:
        name = "busyRepeatRequest";
        break;
    case UDS_NRC_CONDITIONS_NOT_CORRECT:
        name = "conditionsNotCorrect";
        break;
    case UDS_NRC_REQUEST_SEQUENCE_ERROR:
        name = "requestSequenceError";
        break;
    case UDS_NRC_REQUEST_OUT_OF_RANGE:
        name = "requestOutOfRange";
        break;
    case UDS_NRC_DOWNLOAD_NOT_ACCEPTED:
        name = "uploadDownloadNotAccepted";
        break;
    case UDS_NRC_TRANSFER_SUSPENDED:
        name = "transferDataSuspended";
        break;
    case UDS_NRC_PROGRAMMING_FAILURE:
        name = "generalProgrammingFailure";
        break;
    case UDS_NRC_WRONG_BLOCK_COUNTER:
        name = "wrongBlockSequenceCounter";
        break;
    case UDS_NRC_RESPONSE_PENDING:
        name = "requestCorrectlyReceivedResponsePending";
        break;
    case UDS_NRC_SUBFUNCTION_NOT_IN_SESSION:
        name = "subFunctionNotSupportedInActiveSession";
        break;
    case UDS_NRC_SERVICE_NOT_IN_SESSION:
        name = "serviceNotSupportedInActiveSession";
        break;
    default:
        /* a value the server does not know: "?" */
        break;
    }

    return name;
}

const char *uds_session_name(uint8_t session)
{
    const char *name = "?";

    switch (session) {
    case UDS_SESSION_DEFAULT:
        name = "defaultSession";
        break;
    case UDS_SESSION_PROGRAMMING:
        name = "programmingSession";
        break;
    case UDS_SESSION_EXTENDED:
        name = "extendedDiagnosticSession";
        break;
    default:
        /* a value the server does not know: "?" */
        break;
    }

    return name;
}

const char *uds_dl_state_name(uds_dl_state_t state)
{
    const char *name;

    switch (state) {
    case UDS_DL_IDLE:
        name = "idle";
        break;
    case UDS_DL_ERASED:
        name = "erased";
        break;
    case UDS_DL_TRANSFER:
        name = "transfer";
        break;
    case UDS_DL_EXITED:
        name = "exited";
        break;
    case UDS_DL_CHECKED:
        name = "checked";
        break;
    default:
        name = "?";
        break;
    }

    return name;
}
