/* SPDX-License-Identifier: MIT */
/*
 * Private to lib/uds: what uds.c (the core, the session and DTC services, ReadDataByIdentifier,
 * TesterPresent) and uds_program.c (RoutineControl and the download services) share. Not for
 * any other module.
 */
#ifndef FCN_UDS_INTERNAL_H
#define FCN_UDS_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#include "uds/uds.h"

/* The sessions in which a service, a sub-function or a routine is supported, as a bit mask. */
#define UDS_IN_DEFAULT     0x01U
#define UDS_IN_PROGRAMMING 0x02U
#define UDS_IN_EXTENDED    0x04U
#define UDS_IN_ALL         (UDS_IN_DEFAULT | UDS_IN_PROGRAMMING | UDS_IN_EXTENDED)

/* The response a handler builds: positive bytes into the caller's buffer, or an NRC. */
typedef struct {
    uint8_t *buf;  /* the caller's buffer */
    uint32_t cap;  /* its size, at least UDS_NEGATIVE_LEN */
    uint32_t len;  /* the positive response's bytes so far */
    bool overflow; /* a byte did not fit: the response is too long for the buffer */
    uint8_t nrc;   /* UDS_NRC_NONE: positive; otherwise the negative response's code */
    bool reset;    /* the positive response is to be followed by the reset */
} uds_reply_t;

/* A service's handler: the checks of the service at UDS_CALL_START, then its work, a backend
 * operation's first call or its next poll. Answers UDS_BE_DONE with the response in `r`, or
 * UDS_BE_PENDING when the backend did (nothing in `r` then). */
typedef uds_be_t (*uds_handler_t)(uds_server_t *s, uds_call_t call, uint64_t now_us,
                                  uds_reply_t *r);

/* Append one byte, a 16-bit or a 32-bit number (big-endian, as every UDS parameter) to the
 * positive response; a byte beyond the buffer sets `overflow` instead. */
void uds_put8(uds_reply_t *r, uint8_t value);
void uds_put16(uds_reply_t *r, uint16_t value);
void uds_put32(uds_reply_t *r, uint32_t value);

/* Answer with the negative response `nrc`. */
void uds_refuse(uds_reply_t *r, uint8_t nrc);

/* The big-endian number at bytes[at] (2 or 4 bytes). The caller has checked the length. */
uint16_t uds_get16(const uint8_t *bytes, uint32_t at);
uint32_t uds_get32(const uint8_t *bytes, uint32_t at);

/* `a` + `b`, at most UINT64_MAX. */
uint64_t uds_add_sat(uint64_t a, uint64_t b);

/* The session bit (UDS_IN_*) of a session (UDS_SESSION_*); 0 for any other value. */
uint8_t uds_session_bit(uint8_t session);

/* End the download: IDLE. From TRANSFER or EXITED it is counted as failed (`failed`) or, for a
 * session change, S3server or a new erase, as aborted. */
void uds_download_end(uds_server_t *s, bool failed);

/* A session begins (DiagnosticSessionControl, S3server): the download ends and the routines'
 * results are forgotten. */
void uds_program_session_begins(uds_server_t *s);

/* The handlers of uds_program.c. */
uds_be_t uds_svc_routine(uds_server_t *s, uds_call_t call, uint64_t now_us, uds_reply_t *r);
uds_be_t uds_svc_request_download(uds_server_t *s, uds_call_t call, uint64_t now_us,
                                  uds_reply_t *r);
uds_be_t uds_svc_transfer_data(uds_server_t *s, uds_call_t call, uint64_t now_us, uds_reply_t *r);
uds_be_t uds_svc_transfer_exit(uds_server_t *s, uds_call_t call, uint64_t now_us, uds_reply_t *r);

#endif /* FCN_UDS_INTERNAL_H */
