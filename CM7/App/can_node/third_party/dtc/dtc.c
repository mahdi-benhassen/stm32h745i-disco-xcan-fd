/* SPDX-License-Identifier: MIT */
#include "dtc/dtc.h"

#include <stddef.h>

/*
 * What the module does is described in dtc.h. This file adds how; where the header names a
 * CHOICE, the function that implements it says so.
 *
 * What holds for a dtc_state_t between any two calls, from a successful dtc_init() on:
 *
 *   - `events` points to a table that dtc_init() accepted, and 1 <= event_count <=
 *     DTC_EVENTS_MAX. These two are what state_usable() checks before anything is read or
 *     written, because the state is caller memory and event_count bounds every array access;
 *   - every counter lies within the thresholds of its event, which is what keeps the
 *     arithmetic below in range (see debounce() and fault_detection_counter());
 *   - every status byte satisfies the implications listed in dtc.h;
 *   - failed_cycles is 0 while PDTC is 0, and at least 1 while TFTOC is 1;
 *   - every record is NO_EVENT or an event id below event_count. dtc_recorded() checks it
 *     anyway before it uses one as an index.
 */

/* A record that names no event. */
#define NO_EVENT ((uint16_t)0xFFFFU)

/* The largest count of cycles with a failure; the count saturates there. */
#define FAILED_CYCLES_MAX ((uint8_t)0xFFU)

/* What a FAILED sets, and what every test result resets (the test has completed). */
#define SET_BY_FAILED                                                                              \
    ((uint8_t)(DTC_STATUS_TF | DTC_STATUS_TFTOC | DTC_STATUS_PDTC | DTC_STATUS_TFSLC))
#define RESET_BY_COMPLETION ((uint8_t)(DTC_STATUS_TNCSLC | DTC_STATUS_TNCTOC))

/* What a PASSED resets: TF, and the bits that say the test has not completed. */
#define RESET_BY_PASSED ((uint8_t)(DTC_STATUS_TF | DTC_STATUS_TNCSLC | DTC_STATUS_TNCTOC))

_Static_assert(DTC_EVENTS_MAX >= 1U, "a table has at least one event");
_Static_assert(DTC_EVENTS_MAX < 0xFFFFU, "event ids are uint16_t, and NO_EVENT is none of them");
_Static_assert(DTC_STATUS_AFTER_CLEAR == 0x50U, "TNCSLC and TNCTOC, the status after a clear");
_Static_assert((DTC_STATUS_TF | DTC_STATUS_TFTOC | DTC_STATUS_PDTC | DTC_STATUS_CDTC |
                DTC_STATUS_TNCSLC | DTC_STATUS_TFSLC | DTC_STATUS_TNCTOC | DTC_STATUS_WIR) ==
                   DTC_STATUS_AVAILABILITY_MASK,
               "the eight bits make up the whole byte, and every one is implemented");
_Static_assert(DTC_NUMBER_MAX < 0xFFFFFFU, "0xFFFFFF is the groupOfDTC for all groups");
/* A counter within int16_t plus or minus a step within uint16_t fits an int32_t (debounce()). */
_Static_assert(((int32_t)INT16_MAX + (int32_t)UINT16_MAX) <= INT32_MAX,
               "counter + step fits an int32_t");
_Static_assert(((int32_t)INT16_MIN - (int32_t)UINT16_MAX) >= INT32_MIN,
               "counter - step fits an int32_t");
/* The FDC is an int8_t, so its negative half has -DTC_FDC_MIN = 128 steps, and a counter times
 * 128 fits an int32_t (fault_detection_counter()). */
_Static_assert((DTC_FDC_MAX == (int32_t)INT8_MAX) && (DTC_FDC_MIN == (int32_t)INT8_MIN),
               "the FDC range is that of an int8_t");
_Static_assert(((int32_t)INT16_MIN) >= (INT32_MIN / 128), "counter * 128 fits an int32_t");

/* What one report leads to. */
typedef enum {
    RESULT_NONE,   /* evidence only: no test result */
    RESULT_FAILED, /* the test result FAILED */
    RESULT_PASSED  /* the test result PASSED */
} result_t;

static bool entry_valid(const dtc_event_config_t *entry)
{
    return (entry->dtc >= DTC_NUMBER_MIN) && (entry->dtc <= DTC_NUMBER_MAX) &&
           (entry->failed_threshold >= 1) && (entry->passed_threshold <= -1) &&
           (entry->step_up >= 1U) && (entry->step_down >= 1U) &&
           (entry->confirmation_threshold >= 1U);
}

/* Every entry within its ranges, and no DTC number twice. */
static bool table_valid(const dtc_event_config_t *events, uint16_t count)
{
    bool valid = (count >= 1U) && (count <= DTC_EVENTS_MAX);

    if (valid) {
        for (uint16_t i = 0U; i < count; i++) {
            if (!entry_valid(&events[i])) {
                valid = false;
            }

            for (uint16_t j = 0U; j < i; j++) {
                if (events[j].dtc == events[i].dtc) {
                    valid = false;
                }
            }
        }
    }

    return valid;
}

/* Whether the state can be used: initialised, with an event count that bounds the arrays. A
 * zeroed dtc_state_t has no table and is refused. */
static bool state_usable(const dtc_state_t *state)
{
    return (state != NULL) && (state->events != NULL) && (state->event_count >= 1U) &&
           (state->event_count <= DTC_EVENTS_MAX);
}

static bool report_valid(dtc_report_t report)
{
    bool valid = false;

    switch (report) {
    case DTC_REPORT_PASSED:
    case DTC_REPORT_FAILED:
    case DTC_REPORT_PREPASSED:
    case DTC_REPORT_PREFAILED:
        valid = true;
        break;
    default:
        valid = false;
        break;
    }

    return valid;
}

/* A status byte changed: count it (dtc_status_change_count()). Unsigned, so it wraps. */
static void note_change(dtc_state_t *state, uint8_t before, uint8_t after)
{
    if (before != after) {
        state->status_changes++;
    }
}

/*
 * The counter-based debouncing of dtc.h, REPORTS AND DEBOUNCING. The arithmetic is done in
 * int32_t, where a counter within int16_t and a step within uint16_t cannot overflow
 * (_Static_assert above); the result is clamped to the thresholds, so it fits the int16_t
 * again.
 */
static result_t debounce(const dtc_event_config_t *config, dtc_event_state_t *event,
                         dtc_report_t report)
{
    const int32_t failed = (int32_t)config->failed_threshold;
    const int32_t passed = (int32_t)config->passed_threshold;
    int32_t counter = (int32_t)event->counter;
    result_t result = RESULT_NONE;

    switch (report) {
    case DTC_REPORT_PREFAILED:
        counter += (int32_t)config->step_up;
        if (counter >= failed) {
            counter = failed;
            result = RESULT_FAILED;
        }
        break;
    case DTC_REPORT_PREPASSED:
        /* CHOICE: the jump goes to 0, and the step of the same report follows it. */
        if (config->jump_down && (counter > 0)) {
            counter = 0;
        }
        counter -= (int32_t)config->step_down;
        if (counter <= passed) {
            counter = passed;
            result = RESULT_PASSED;
        }
        break;
    case DTC_REPORT_FAILED:
        counter = failed;
        result = RESULT_FAILED;
        break;
    case DTC_REPORT_PASSED:
    default: /* report_valid() has let nothing else through */
        counter = passed;
        result = RESULT_PASSED;
        break;
    }

    event->counter = (int16_t)counter;
    return result;
}

/* A bit of `event` rose: note it in the pair of records that follows that bit, the first
 * since the last clear (kept once set) and the most recent. */
static void note_record(uint16_t *first, uint16_t *most_recent, uint16_t event)
{
    if (*first == NO_EVENT) {
        *first = event;
    }

    *most_recent = event;
}

/* The test result FAILED (dtc.h, THE STATUS BYTE: bits 0 to 7). */
static void test_failed(dtc_state_t *state, uint16_t event)
{
    const dtc_event_config_t *config = &state->events[event];
    dtc_event_state_t *ev = &state->event[event];
    const uint8_t before = ev->status;
    uint8_t status = before;

    /* Bit 3: the first failure of an operation cycle makes it a cycle with a failure. TFTOC
     * tells it from the later ones, because nothing but the start of a cycle and a clear reset
     * it. The count saturates; with confirmation_threshold at most 255 that cannot keep a DTC
     * from being confirmed. */
    if (((before & DTC_STATUS_TFTOC) == 0U) && (ev->failed_cycles < FAILED_CYCLES_MAX)) {
        ev->failed_cycles++;
    }

    status |= SET_BY_FAILED;
    status &= (uint8_t)~RESET_BY_COMPLETION;

    /* Bits 3 and 7. CHOICE: the indicator is requested from the confirmation on. */
    if (ev->failed_cycles >= config->confirmation_threshold) {
        status |= DTC_STATUS_CDTC;

        if (config->warning_indicator) {
            status |= DTC_STATUS_WIR;
        }
    }

    ev->status = status;

    /* CHOICE: the records follow the rising edges of TF and CDTC. */
    if ((before & DTC_STATUS_TF) == 0U) {
        note_record(&state->first_failed, &state->most_recent_failed, event);
    }

    if (((before & DTC_STATUS_CDTC) == 0U) && ((status & DTC_STATUS_CDTC) != 0U)) {
        note_record(&state->first_confirmed, &state->most_recent_confirmed, event);
    }
}

/* The test result PASSED: TF reset, the test completed. Nothing else changes; in particular
 * PDTC waits for the end of the cycle. */
static void test_passed(dtc_event_state_t *event)
{
    event->status &= (uint8_t)~RESET_BY_PASSED;
}

/* dtc.h: the DTCFaultDetectionCounter, rounded toward zero (C division truncates). With the
 * counter within its thresholds the quotient lies within -128..127. */
static int8_t fault_detection_counter(const dtc_event_config_t *config, int16_t counter)
{
    int32_t fdc;

    if (counter >= 0) {
        fdc = ((int32_t)counter * DTC_FDC_MAX) / (int32_t)config->failed_threshold;
    } else {
        fdc = ((int32_t)counter * (-DTC_FDC_MIN)) / (-(int32_t)config->passed_threshold);
    }

    return (int8_t)fdc;
}

static void fill_entry(const dtc_state_t *state, uint16_t event, dtc_entry_t *entry)
{
    const dtc_event_config_t *config = &state->events[event];
    const dtc_event_state_t *ev = &state->event[event];

    entry->dtc = config->dtc;
    entry->counter = ev->counter;
    entry->event = event;
    entry->status = ev->status;
    entry->fault_detection_counter = fault_detection_counter(config, ev->counter);
    entry->failed_cycles = ev->failed_cycles;
}

static void forget_records(dtc_state_t *state)
{
    state->first_failed = NO_EVENT;
    state->first_confirmed = NO_EVENT;
    state->most_recent_failed = NO_EVENT;
    state->most_recent_confirmed = NO_EVENT;
}

bool dtc_init(dtc_state_t *state, const dtc_event_config_t *events, uint16_t count)
{
    bool accepted = false;

    if ((state != NULL) && (events != NULL) && table_valid(events, count)) {
        state->events = events;
        state->event_count = count;
        state->status_changes = 0U;
        forget_records(state);

        /* All DTC_EVENTS_MAX entries, not only `count` of them, so that the state holds
         * nothing from before. */
        for (uint32_t i = 0U; i < DTC_EVENTS_MAX; i++) {
            state->event[i].counter = 0;
            state->event[i].status = DTC_STATUS_AFTER_CLEAR;
            state->event[i].failed_cycles = 0U;
        }

        accepted = true;
    }

    return accepted;
}

bool dtc_report(dtc_state_t *state, uint16_t event, dtc_report_t report)
{
    bool accepted = false;

    if (state_usable(state) && (event < state->event_count) && report_valid(report)) {
        const uint8_t before = state->event[event].status;
        const result_t result = debounce(&state->events[event], &state->event[event], report);

        if (result == RESULT_FAILED) {
            test_failed(state, event);
        } else if (result == RESULT_PASSED) {
            test_passed(&state->event[event]);
        } else {
            /* evidence only: the status byte stays as it is */
        }

        note_change(state, before, state->event[event].status);
        accepted = true;
    }

    return accepted;
}

bool dtc_start_operation_cycle(dtc_state_t *state)
{
    bool accepted = false;

    if (state_usable(state)) {
        for (uint16_t i = 0U; i < state->event_count; i++) {
            dtc_event_state_t *ev = &state->event[i];
            const uint8_t before = ev->status;
            uint8_t status = before;

            /* The end of the cycle (bit 2): the test completed and never failed. The count of
             * cycles with a failure starts again with PDTC (bit 3, CHOICE). */
            if ((before & (uint8_t)(DTC_STATUS_TNCTOC | DTC_STATUS_TFTOC)) == 0U) {
                status &= (uint8_t)~DTC_STATUS_PDTC;
                ev->failed_cycles = 0U;
            }

            /* The start of the next (bits 1 and 6); TF and the bits about the time since the
             * last clear stay. CHOICE: the debouncing starts again from 0. */
            status &= (uint8_t)~DTC_STATUS_TFTOC;
            status |= DTC_STATUS_TNCTOC;
            ev->status = status;
            ev->counter = 0;
            note_change(state, before, status);
        }

        accepted = true;
    }

    return accepted;
}

bool dtc_clear_all(dtc_state_t *state)
{
    bool accepted = false;

    if (state_usable(state)) {
        for (uint16_t i = 0U; i < state->event_count; i++) {
            dtc_event_state_t *ev = &state->event[i];
            const uint8_t before = ev->status;

            ev->status = DTC_STATUS_AFTER_CLEAR;
            ev->counter = 0;
            ev->failed_cycles = 0U;
            note_change(state, before, ev->status);
        }

        forget_records(state);
        accepted = true;
    }

    return accepted;
}

bool dtc_get(const dtc_state_t *state, uint16_t event, dtc_entry_t *entry)
{
    bool found = false;

    if (state_usable(state) && (entry != NULL) && (event < state->event_count)) {
        fill_entry(state, event, entry);
        found = true;
    }

    return found;
}

uint16_t dtc_count_by_status_mask(const dtc_state_t *state, uint8_t mask)
{
    uint16_t count = 0U;

    if (state_usable(state)) {
        for (uint16_t i = 0U; i < state->event_count; i++) {
            if ((state->event[i].status & mask) != 0U) {
                count++;
            }
        }
    }

    return count;
}

bool dtc_next_by_status_mask(const dtc_state_t *state, uint8_t mask, uint16_t *cursor,
                             dtc_entry_t *entry)
{
    bool found = false;

    if (state_usable(state) && (cursor != NULL) && (entry != NULL)) {
        uint16_t i = *cursor;

        while ((!found) && (i < state->event_count)) {
            if ((state->event[i].status & mask) != 0U) {
                fill_entry(state, i, entry);
                found = true;
            }

            i++;
        }

        *cursor = i;
    }

    return found;
}

bool dtc_recorded(const dtc_state_t *state, dtc_record_t record, dtc_entry_t *entry)
{
    bool found = false;

    if (state_usable(state) && (entry != NULL)) {
        uint16_t event = NO_EVENT;

        switch (record) {
        case DTC_RECORD_FIRST_FAILED:
            event = state->first_failed;
            break;
        case DTC_RECORD_FIRST_CONFIRMED:
            event = state->first_confirmed;
            break;
        case DTC_RECORD_MOST_RECENT_FAILED:
            event = state->most_recent_failed;
            break;
        case DTC_RECORD_MOST_RECENT_CONFIRMED:
            event = state->most_recent_confirmed;
            break;
        default:
            event = NO_EVENT;
            break;
        }

        /* NO_EVENT is above every event id, so this also tells "none" from a record. */
        if (event < state->event_count) {
            fill_entry(state, event, entry);
            found = true;
        }
    }

    return found;
}

uint32_t dtc_status_change_count(const dtc_state_t *state)
{
    uint32_t changes = 0U;

    if (state_usable(state)) {
        changes = state->status_changes;
    }

    return changes;
}
