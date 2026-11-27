/* SPDX-License-Identifier: MIT */
/**
 * @file
 * Diagnostic trouble codes (DTCs): from the reports of the node's fault monitors to the status
 * byte of every DTC, in the form in which a UDS tester reads it.
 *
 * Sources: ISO 14229-1 (Unified diagnostic services), Annex D, the definitions of the
 * DTCStatusMask and statusOfDTC bits, and the sub-functions of ReadDTCInformation (0x19) and
 * ClearDiagnosticInformation (0x14) that the read access below is shaped for; the AUTOSAR
 * "Specification of Diagnostic Event Manager" (Dem) for the counter-based debouncing, the
 * report values (Dem_EventStatusType) and the operation cycle. The Dem is referred to by the
 * names of its configuration parameters and functions, not by requirement numbers. Where the
 * norm leaves a choice, the choice made here is marked CHOICE.
 *
 * EVENTS AND DTCS. The caller describes its monitors in a table of dtc_event_config_t, one
 * entry per event; the event id is the index of the entry. Every event has a DTC of its own
 * (no combined DTCs), so the status of an event and the status of its DTC are the same byte. A
 * DTC number has 24 bits, 0x000001..0xFFFFFE: the DTCHighByte, DTCMiddleByte and DTCLowByte of
 * a UDS response, most significant first. 0x000000 is what an entry left out of a table with
 * designated initialisers holds, and 0xFFFFFF is the groupOfDTC with which
 * ClearDiagnosticInformation means all groups, so neither names a DTC here. Which number
 * stands for which fault is the project's definition, not this module's.
 *
 * REPORTS AND DEBOUNCING (the Dem's counter-based debounce algorithm). A monitor reports one of
 * four values, dtc_report_t, which has the values of Dem_EventStatusType. Each event has a
 * debounce counter, a signed number between the passed_threshold (below 0) and the
 * failed_threshold (above 0) of its entry; it is 0 after dtc_init():
 *
 *   PREFAILED  counter += step_up (DemDebounceCounterIncrementStepSize). At or above
 *              failed_threshold (DemDebounceCounterFailedThreshold) it is set to that threshold,
 *              and the test result is FAILED.
 *   PREPASSED  if jump_down is set and the counter is above 0, it first jumps to 0
 *              (DemDebounceCounterJumpDown with a DemDebounceCounterJumpDownValue of 0). Then
 *              counter -= step_down (DemDebounceCounterDecrementStepSize). At or below
 *              passed_threshold (DemDebounceCounterPassedThreshold) it is set to that
 *              threshold, and the test result is PASSED.
 *   FAILED     the monitor has matured the fault itself: counter = failed_threshold, the test
 *              result is FAILED.
 *   PASSED     likewise: counter = passed_threshold, the test result is PASSED.
 *
 * CHOICE: after a jump down, the step of the same report is still applied, so that an event
 * that comes from FAILED needs as many PREPASSED reports to pass as it needs from 0. A counter
 * at a threshold stays there: one more PREFAILED at failed_threshold gives FAILED again, one more
 * PREPASSED at passed_threshold gives PASSED again. CHOICE: dtc_start_operation_cycle() and
 * dtc_clear_all() set the counter to 0, so that evidence collected before either does not count
 * after it.
 *
 * A pre-report that does not reach its threshold is evidence, not a test result: it moves the
 * counter and leaves the status byte alone. Below, "FAILED" and "PASSED" mean test results,
 * however they were reached, and "the test completed" means that one of them was reached.
 *
 * The counter is also given as the DTCFaultDetectionCounter of ISO 14229-1, -128..127, which
 * ReadDTCInformation 0x14 reports: mapped linearly, 0..failed_threshold to 0..127 and
 * passed_threshold..0 to -128..0. CHOICE: rounded toward zero, so that only a counter at its
 * threshold gives 127 or -128.
 *
 * THE STATUS BYTE (statusOfDTC, ISO 14229-1 Annex D). The rule for each bit, in the order of
 * the byte:
 *
 *   bit 0  testFailed (TF). Set by FAILED. Reset by PASSED and by a clear. The result of the
 *          most recent test: it is kept at the start of an operation cycle, which performs no
 *          test.
 *   bit 1  testFailedThisOperationCycle (TFTOC). Set by FAILED. Reset at the start of an
 *          operation cycle and by a clear.
 *   bit 2  pendingDTC (PDTC). Set by FAILED. Reset at the end of an operation cycle in which
 *          the test completed and never failed (TNCTOC and TFTOC both 0 when the cycle ends),
 *          and by a clear. A PASSED alone does not reset it.
 *   bit 3  confirmedDTC (CDTC). Set by FAILED once confirmation_threshold operation cycles
 *          have had a failure. Reset only by a clear: there is no aging.
 *          CHOICE (ISO leaves the confirmation criteria to the manufacturer; the number is the
 *          Dem's DemEventConfirmationThreshold): the cycles with a failure are counted since
 *          PDTC was last reset, so a cycle in which the test completed without a failure starts
 *          the count again, and a cycle in which the test did not complete does not. With a
 *          threshold of 1 the first FAILED confirms.
 *   bit 4  testNotCompletedSinceLastClear (TNCSLC). Set by dtc_init() and by a clear. Reset by
 *          FAILED and by PASSED.
 *   bit 5  testFailedSinceLastClear (TFSLC). Set by FAILED. Reset only by a clear: there is no
 *          aging.
 *   bit 6  testNotCompletedThisOperationCycle (TNCTOC). Set at the start of an operation cycle
 *          (dtc_init() starts the first) and by a clear. Reset by FAILED and by PASSED.
 *   bit 7  warningIndicatorRequested (WIR). Set together with CDTC for an event whose entry has
 *          warning_indicator; never set for any other. Reset only by a clear. CHOICE (ISO
 *          leaves the indicator's behaviour to the manufacturer): the indicator stays requested
 *          until the DTC is cleared. The Dem's healing, which switches it off after
 *          DemIndicatorHealingCycleCounterThreshold cycles that passed, is not implemented.
 *
 * After dtc_init() and after dtc_clear_all() every status byte is DTC_STATUS_AFTER_CLEAR
 * (TNCSLC and TNCTOC: 0x50). The module implements all eight bits, so the
 * DTCStatusAvailabilityMask that ReadDTCInformation reports is DTC_STATUS_AVAILABILITY_MASK,
 * 0xFF.
 *
 * From the rules it follows that at every moment TF, TFTOC, PDTC and CDTC each imply TFSLC;
 * TFTOC implies PDTC and not TNCTOC; WIR implies CDTC; TNCSLC implies TNCTOC and not TFSLC; and
 * the counter lies within the event's thresholds. The unit tests check this after every step
 * of their random runs.
 *
 * THE OPERATION CYCLE. One for all events: a run of the node from a reset. dtc_init() begins
 * the first. dtc_start_operation_cycle() ends the current cycle and begins the next one: for
 * every event, first the end (PDTC as described for bit 2), then the start (TFTOC reset, TNCTOC
 * set, the counter 0). In phase 3 the whole state is in RAM and a reset loses it, so the node
 * calls dtc_init() at its start and has no use for dtc_start_operation_cycle(). A node that
 * keeps the state in non-volatile memory (phase 4, which adds the restore) calls it at its
 * start, after the restore.
 *
 * CLEAR. dtc_clear_all() is ClearDiagnosticInformation for the groupOfDTC 0xFFFFFF (all
 * groups): every status byte becomes DTC_STATUS_AFTER_CLEAR, every counter and every count of
 * cycles with a failure 0, and the records below are forgotten. It does not begin an operation
 * cycle; TNCTOC is set because the norm sets it at a clear.
 *
 * RECORDS. Since the last clear (or dtc_init()), the module keeps the first and the most recent
 * DTC whose TF went from 0 to 1, and the first and the most recent whose CDTC went from 0 to 1:
 * what ReadDTCInformation 0x0B, 0x0D, 0x0C and 0x0E report, together with the DTC's present
 * status. CHOICE: a record changes when the bit rises, not with every FAILED of a DTC whose TF
 * is still set.
 *
 * READ ACCESS. A DTC matches a status mask if (statusOfDTC & mask) != 0, as ReadDTCInformation
 * 0x01 (count) and 0x02 (report) define it; a mask of 0 matches nothing. The DTCs are visited in
 * the order of the table, through a cursor that the caller keeps, so that a response can be
 * built in pieces without any state in this module.
 *
 * NOT IN PHASE 3. No aging: CDTC and TFSLC stay set until a clear. No storage in non-volatile
 * memory: the state is in RAM, and a reset loses it, confirmed DTCs included; keeping confirmed
 * DTCs across resets needs the flash of phase 4. Also not implemented: healing of WIR; snapshot
 * and extended data records; occurrence counters; enable and storage conditions; an event
 * memory with fewer entries than events, and displacement (every DTC has its status at all
 * times); time-based debouncing; jump up; more than one operation cycle; clearing a single DTC
 * or group.
 *
 * CONTEXT. The functions keep no data of their own, call no operating system, allocate nothing
 * and do not block. The caller owns the dtc_state_t and serialises all calls on one (in the
 * application, one task owns it). Under that condition any task or interrupt may call them.
 * dtc_report() and the reads of one event take constant time; dtc_init() at most
 * DTC_EVENTS_MAX * (DTC_EVENTS_MAX - 1) / 2 comparisons of DTC numbers; the other functions
 * visit every event once.
 */
#ifndef FCN_DTC_H
#define FCN_DTC_H

#include <stdbool.h>
#include <stdint.h>

/** Events a dtc_state_t has room for. */
#define DTC_EVENTS_MAX 32U

/** The range of a DTC number (see EVENTS AND DTCS). */
#define DTC_NUMBER_MIN 0x000001U
#define DTC_NUMBER_MAX 0xFFFFFEU

/** The bits of statusOfDTC (ISO 14229-1 Annex D), named as the Dem's DEM_UDS_STATUS_*. */
#define DTC_STATUS_TF     0x01U /**< bit 0 testFailed */
#define DTC_STATUS_TFTOC  0x02U /**< bit 1 testFailedThisOperationCycle */
#define DTC_STATUS_PDTC   0x04U /**< bit 2 pendingDTC */
#define DTC_STATUS_CDTC   0x08U /**< bit 3 confirmedDTC */
#define DTC_STATUS_TNCSLC 0x10U /**< bit 4 testNotCompletedSinceLastClear */
#define DTC_STATUS_TFSLC  0x20U /**< bit 5 testFailedSinceLastClear */
#define DTC_STATUS_TNCTOC 0x40U /**< bit 6 testNotCompletedThisOperationCycle */
#define DTC_STATUS_WIR    0x80U /**< bit 7 warningIndicatorRequested */

/** The status byte of every DTC after dtc_init() and dtc_clear_all(). */
#define DTC_STATUS_AFTER_CLEAR (DTC_STATUS_TNCSLC | DTC_STATUS_TNCTOC)

/** The DTCStatusAvailabilityMask: the module implements every bit of the status byte. */
#define DTC_STATUS_AVAILABILITY_MASK 0xFFU

/** The range of the DTCFaultDetectionCounter (ISO 14229-1). */
#define DTC_FDC_MAX 127
#define DTC_FDC_MIN (-128)

/** What a monitor reports: Dem_EventStatusType, with its values. */
typedef enum {
    DTC_REPORT_PASSED = 0x00,    /**< matured by the monitor: the test passed */
    DTC_REPORT_FAILED = 0x01,    /**< matured by the monitor: the test failed */
    DTC_REPORT_PREPASSED = 0x02, /**< evidence that the test passes: counter -= step_down */
    DTC_REPORT_PREFAILED = 0x03  /**< evidence that the test fails: counter += step_up */
} dtc_report_t;

/** The DTCs the module remembers since the last clear (see RECORDS). */
typedef enum {
    DTC_RECORD_FIRST_FAILED,         /**< the first whose TF rose (ReadDTCInformation 0x0B) */
    DTC_RECORD_FIRST_CONFIRMED,      /**< the first whose CDTC rose (0x0C) */
    DTC_RECORD_MOST_RECENT_FAILED,   /**< the most recent whose TF rose (0x0D) */
    DTC_RECORD_MOST_RECENT_CONFIRMED /**< the most recent whose CDTC rose (0x0E) */
} dtc_record_t;

/**
 * One event: its DTC and how its monitor's reports are debounced. Constant data, in a table
 * that the dtc_state_t points to. dtc_init() refuses a table with a value outside these ranges:
 * dtc DTC_NUMBER_MIN..DTC_NUMBER_MAX and different from every other entry's; failed_threshold
 * 1..32767; passed_threshold -32768..-1; step_up and step_down 1..65535 (a step beyond the
 * distance to a threshold ends at the threshold); confirmation_threshold 1..255.
 */
typedef struct {
    uint32_t dtc;                   /**< the DTC number */
    int16_t failed_threshold;       /**< counter value that is FAILED (above 0) */
    int16_t passed_threshold;       /**< counter value that is PASSED (below 0) */
    uint16_t step_up;               /**< added by a PREFAILED */
    uint16_t step_down;             /**< subtracted by a PREPASSED */
    uint8_t confirmation_threshold; /**< operation cycles with a failure until CDTC (bit 3) */
    bool jump_down;                 /**< a PREPASSED above 0 jumps to 0 first */
    bool warning_indicator;         /**< the DTC requests a warning indicator when confirmed */
} dtc_event_config_t;

/** What the read functions give about one DTC: a copy, taken at the time of the call. */
typedef struct {
    uint32_t dtc;                   /**< the DTC number */
    int16_t counter;                /**< debounce counter, passed..failed_threshold */
    uint16_t event;                 /**< the event id: index of its entry in the table */
    uint8_t status;                 /**< statusOfDTC: DTC_STATUS_* bits */
    int8_t fault_detection_counter; /**< DTCFaultDetectionCounter, DTC_FDC_MIN..DTC_FDC_MAX */
    uint8_t failed_cycles;          /**< cycles with a failure toward CDTC (bit 3), 0..255 */
} dtc_entry_t;

/* Private to dtc.c; in the header only so that the state can be allocated statically. */
typedef struct {
    int16_t counter;       /* the debounce counter */
    uint8_t status;        /* statusOfDTC */
    uint8_t failed_cycles; /* cycles with a failure since PDTC was last reset, saturating */
} dtc_event_state_t;

/* Private to dtc.c. The records hold an event id, or UINT16_MAX for none. */
typedef struct {
    const dtc_event_config_t *events; /* the table given to dtc_init() */
    uint32_t status_changes;          /* status bytes changed since dtc_init(); wraps */
    uint16_t event_count;             /* entries of the table, 1..DTC_EVENTS_MAX */
    uint16_t first_failed;
    uint16_t first_confirmed;
    uint16_t most_recent_failed;
    uint16_t most_recent_confirmed;
    dtc_event_state_t event[DTC_EVENTS_MAX]; /* the first event_count are used */
} dtc_state_t;

/**
 * Prepare `state` for the `count` events of `events`, to which it keeps a pointer: the table
 * has to stay valid and unchanged as long as `state` is used (a const table). Every DTC gets
 * the status DTC_STATUS_AFTER_CLEAR and a counter of 0, there are no records, the count of
 * status changes is 0, and the first operation cycle begins. The memory of `state` may hold
 * anything before. Any context (see CONTEXT); in the application, the owner of the state calls
 * it once before any other task can reach the state.
 *
 * Returns false, with `state` unchanged, if a pointer is NULL, `count` is not
 * 1..DTC_EVENTS_MAX, or an entry is outside the ranges given at dtc_event_config_t. Otherwise
 * true.
 */
bool dtc_init(dtc_state_t *state, const dtc_event_config_t *events, uint16_t count);

/**
 * The monitor of event `event` (0..count - 1) reports `report`: debounce it, and if a test
 * result is reached, update the status byte and the records as described above.
 *
 * Returns false, changing nothing, if `state` is NULL or not initialised (a zeroed dtc_state_t
 * is refused), `event` is not below the number of events, or `report` is not a value of
 * dtc_report_t. Otherwise true. Any context (see CONTEXT).
 */
bool dtc_report(dtc_state_t *state, uint16_t event, dtc_report_t report);

/**
 * End the current operation cycle and begin the next (see THE OPERATION CYCLE). Returns false,
 * changing nothing, if `state` is NULL or not initialised; otherwise true. Any context (see
 * CONTEXT).
 */
bool dtc_start_operation_cycle(dtc_state_t *state);

/**
 * ClearDiagnosticInformation for all groups (see CLEAR). Returns false, changing nothing, if
 * `state` is NULL or not initialised; otherwise true. Any context (see CONTEXT).
 */
bool dtc_clear_all(dtc_state_t *state);

/**
 * The DTC of event `event`. Returns false, leaving `entry` alone, if a pointer is NULL, `state`
 * is not initialised or `event` is not below the number of events. Any context (see CONTEXT).
 */
bool dtc_get(const dtc_state_t *state, uint16_t event, dtc_entry_t *entry);

/**
 * The number of DTCs whose status matches `mask`, (status & mask) != 0: the DTCCount of
 * ReadDTCInformation 0x01, 0..DTC_EVENTS_MAX. 0 if `state` is NULL or not initialised. Any
 * context (see CONTEXT).
 */
uint16_t dtc_count_by_status_mask(const dtc_state_t *state, uint8_t mask);

/**
 * The next DTC whose status matches `mask`, from event *cursor on, in the order of the table:
 * the list of ReadDTCInformation 0x02. Start with *cursor = 0 and call again until the result
 * is false. A match at event i fills `entry` and sets *cursor to i + 1; when nothing matches
 * any more, *cursor becomes the number of events (or stays, if it was beyond it) and the
 * result is false. Returns false, changing nothing, if a pointer is NULL or `state` is not
 * initialised. Any context (see CONTEXT). The list is a consistent picture only if the state
 * does not change between the calls that build it.
 */
bool dtc_next_by_status_mask(const dtc_state_t *state, uint8_t mask, uint16_t *cursor,
                             dtc_entry_t *entry);

/**
 * The DTC that `record` names (see RECORDS), with its present status. Returns false, leaving
 * `entry` alone, if no DTC has been recorded since the last clear, a pointer is NULL, `state`
 * is not initialised or `record` is not a value of dtc_record_t. Any context (see CONTEXT).
 */
bool dtc_recorded(const dtc_state_t *state, dtc_record_t record, dtc_entry_t *entry);

/**
 * How many times a status byte has changed since dtc_init(): a call that changes the status of
 * three DTCs adds 3. It wraps after 2^32 changes. A caller that acts on changes (a frame sent
 * at once when a status changes) compares it with the value it saw last. 0 if `state` is NULL
 * or not initialised. Any context (see CONTEXT).
 */
uint32_t dtc_status_change_count(const dtc_state_t *state);

#endif /* FCN_DTC_H */
