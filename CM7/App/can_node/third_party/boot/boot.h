/* SPDX-License-Identifier: MIT */
/**
 * @file
 * Boot control and the slot writer: which of two image slots the bootloader starts, how a new
 * image is written into the other one, and how it becomes the image that runs, or is rolled
 * back. Every state change is a write to flash that a loss of supply can cut at any point; the
 * design below keeps a consistent, bootable state through every such cut under the tear model of
 * lib/flash (flash/flash.h): a cut program leaves its units before the cut written, the unit at
 * the cut with some of its 1-to-0 transitions done and the rest untouched; a cut erase leaves
 * some of the sector's 0 bits set to 1. Either way, a cut only leaves bits at 1 that the finished
 * operation would have left at 0. Nothing here depends on the timing of the device.
 *
 * Pure C over a flash_dev_t: no operating system, no allocation, no globals. Image validation
 * is a callback (boot_validator_t), so that this module does not depend on the image format.
 *
 * THE LAYOUT is the caller's (boot_config_t): a run of 2..BOOT_META_SECTORS_MAX consecutive
 * metadata sectors, and two slots of whole sectors, A (slot 0) and B (slot 1), none of the three
 * regions overlapping. The host build and a target build pass their own tables.
 *
 * SLOT STATES (boot_slot_state_t). The metadata gives each slot one state, and every state but
 * EMPTY and WRITING names the image it applies to by its id (BOOT_IMAGE_ID_SIZE bytes from the
 * validator, see THE VALIDATOR):
 *
 *   EMPTY      nothing recorded (the factory state of slot B)
 *   WRITING    an update is writing the slot; not bootable
 *   TRIAL      a new image that validated when it was committed and has not confirmed itself;
 *              booted at most BOOT_MAX_ATTEMPTS times
 *   CONFIRMED  the active image. Exactly one slot is CONFIRMED in every record
 *   PREVIOUS   the image that was CONFIRMED before the active one; booted only when the active
 *              image does not validate
 *   REJECTED   a trial that ran out of attempts or did not validate, or an active image that
 *              stopped validating; never booted again, may be overwritten by an update
 *
 * An image counts as the one a state names only if the validator accepts the slot AND returns
 * the id the record holds for it. A slot rewritten behind the metadata's back (a tool, a bench)
 * therefore never passes for the image that was confirmed there.
 *
 * THE METADATA LOG. The metadata sectors form a ring that holds a log of ENTRIES. Entry i of
 * metadata sector s starts at meta_addr + s * sector_size + i * stride. An entry is, in this
 * order (F = max(8, unit_size) bytes, a power of two that divides the page size):
 *
 *   offset           size        content
 *   0                body_size   the RECORD (96 bytes, layout below), 0xFF-padded to whole units
 *   body_size        F           the COMMIT field
 *   body_size + F    3 x F       the ATTEMPT fields 0, 1, 2 (BOOT_MAX_ATTEMPTS)
 *
 * body_size is 96 rounded up to whole units (96 for units up to 32 bytes, 128 for 64). The
 * stride is the entry's size rounded up to a power of two if that fits in a page, otherwise to
 * whole pages, so that an entry never shares a page with the next one unless it is smaller than
 * a page; with the host geometry (4096-byte sectors, 256-byte pages, 8-byte units) an entry is
 * 128 bytes and a sector holds 32 of them.
 *
 * THE RECORD, little-endian, written byte by byte:
 *
 *   offset  size  field
 *   0       4     magic BOOT_RECORD_MAGIC, the bytes 'F' 'C' 'N' 'B'
 *   4       1     format version BOOT_RECORD_VERSION (1)
 *   5       1     event: the boot_event_t that wrote the record (ADOPT..FALLBACK)
 *   6       1     event slot: the slot that event acted on (0 or 1): the slot adopted, begun,
 *                 committed or confirmed; the trial's for ROLLBACK and REJECT; for FALLBACK
 *                 the slot fallen back to, whose PREVIOUS image is CONFIRMED again
 *   7       1     reserved, 0xFF
 *   8       4     sequence number, 1..0xFFFFFFFE: one more than the newest record before it
 *   12      40    slot A: state (1 byte, boot_slot_state_t), 3 reserved bytes 0xFF, image
 *                 version (4), image id (32). EMPTY and WRITING have version 0xFFFFFFFF and an
 *                 id of 0xFF bytes
 *   52      40    slot B, the same
 *   92      4     CRC-32/ISO-HDLC (lib/crc, equal to zlib.crc32) of bytes 0..91
 *
 * THE COMMIT FIELD holds the number of 0 bits in the body_size bytes of the record, as a 32-bit
 * little-endian number, then the same number with every bit inverted, then 0xFF bytes up to F.
 * It is programmed in its own operation, after the record has been programmed completely.
 *
 * WHICH RECORD COUNTS. An entry holds a valid record if its commit field reads exactly the
 * value computed from the record bytes as read, the CRC matches, and the record is well formed
 * (magic, version, reserved bytes, an event and states of the enumerations, exactly one
 * CONFIRMED slot, at most one TRIAL, version and id 0xFF for EMPTY and WRITING). The USED PART
 * of a metadata sector is its entries from entry 0 up to, not including, the first entry that
 * reads completely erased (all of its bytes 0xFF), or the whole sector. The valid record with
 * the highest sequence number in the used parts is the NEWEST, and it alone is the state of the
 * system. No valid record at all is the FACTORY state. Records are written in the order of the
 * entries (WRITING A RECORD), so nothing that counts lies beyond the end of a used part.
 *
 * Why the commit field holds a count of zeros: under the tear model a cut only leaves bits at 1
 * that should be 0, in the record, in the commit field, or both. Such damage lowers the number
 * of 0 bits in the record and can only raise the stored count (and its inverted copy), so the
 * two can never agree again: a Berger code (J. M. Berger, "A note on error detection codes for
 * asymmetric channels", Information and Control 4, 1961), which detects every error that only
 * turns 0s into 1s, not just most of them. So a torn record, a torn commit field and a record
 * damaged by a torn erase of its sector are each rejected with certainty; the CRC is there for
 * damage outside the tear model (a bit that flips the other way).
 *
 * THE ATTEMPT FIELDS belong to the record of their entry. The bootloader programs one field,
 * all 0x00, before each boot of the TRIAL image the record names, the first field that reads
 * erased. A field that does not read erased (all 0xFF) counts as used, whatever it holds, so a
 * torn attempt write counts against the image, never for it. A unit is never programmed twice.
 *
 * WRITING A RECORD (every state change). The new record goes into the entry that ends the used
 * part of the sector of the newest record (sector 0 in the factory state): its first erased
 * entry. If the used part is the whole sector, the next sector of the ring is erased first and
 * the record goes into its entry 0; that sector only ever holds entries older than the newest
 * record, so nothing that counts is lost. Then: one program of the record (one per page it
 * spans), one program of the commit field, and a read of the log to check that the new record is
 * now the newest. An entry is never written twice: a torn one stays as it is, part of the used
 * part, and the next record goes after it. The record numbered 0xFFFFFFFE is the last one: a
 * write after it is refused with BOOT_E_STATE, after more than 1.4 billion updates of three
 * records each.
 *
 * THE STATE MACHINE. c is the CONFIRMED slot, o the other one; "-" means nothing is written.
 *
 *   state of o    event                                   written            new state of o
 *   (factory)     select, A validates                     record: A CONF     EMPTY (c = A)
 *   (factory)     select, A does not, B does              record: B CONF     EMPTY (c = B)
 *   (factory)     select, neither validates               -                  (factory)
 *   any but TRIAL select, c validates                     -                  unchanged
 *   PREVIOUS      select, c does not validate, o does     record: FALLBACK   c' = o, REJECTED
 *   other         select, c does not validate             -                  unchanged
 *   TRIAL(k)      select, k < 3, o validates              attempt field k    TRIAL(k + 1)
 *   TRIAL(k)      select, k < 3, o does not validate      record: REJECT     REJECTED
 *   TRIAL(3)      select                                  record: ROLLBACK   REJECTED
 *   EMPTY, PREVIOUS, REJECTED  begin(o)                   record: BEGIN      WRITING
 *   WRITING       begin(o)                                -                  WRITING
 *   WRITING       finish: the image validates             record: COMMIT     TRIAL(0)
 *   WRITING       finish: it does not                     -                  WRITING
 *   TRIAL(k)      confirm(o), k >= 1                      record: CONFIRM    c' = o, PREVIOUS
 *
 * TRIAL(k) is a trial with k attempt fields used. After a ROLLBACK or REJECT, select goes on with
 * the row "select, c validates" (and its neighbours) of the new state. Refused, writing nothing:
 * begin(c), begin while o is TRIAL (a trial runs, or is about to, and c is its fallback), begin
 * and confirm in the factory state, confirm of a slot that is not TRIAL, and confirm of a TRIAL
 * that has used no attempt (it has never run, so the caller is not the trial). An image that
 * validates is booted at the row that names it: c as CONFIRMED, o as TRIAL with attempt k + 1.
 * select writes at most one record and boots at most once per call.
 *
 * WHAT A CUT LEAVES. A record write cut anywhere leaves either the old newest record (the new
 * entry is torn or its commit field incomplete, or the cut fell in the erase of the next sector)
 * or the new one (the commit field got all of its bits); never anything else. An attempt write
 * cut leaves the attempt counted or not counted; a cut before the boot means it was not booted.
 * Cuts while the image sectors are erased and written leave the slot WRITING, which select never
 * boots. So after any cut the state is the one before the operation or the one after it, and
 * select boots what that state says. docs/update.md goes through every step.
 *
 * WEAR. A record is 2 program operations (one more for every further page the record spans).
 * One metadata sector is erased every entries_per_sector records, so each of them once every
 * meta_sectors * entries_per_sector records. A complete update writes 3 records (BEGIN, COMMIT,
 * CONFIRM) and one attempt field per trial boot; a failed one BEGIN, COMMIT and ROLLBACK or
 * REJECT. The slot sectors the image covers are erased once per update of that slot. Booting a
 * CONFIRMED image writes nothing.
 *
 * THE VALIDATOR (boot_validator_t) checks the image stored in a region of the device, in steps
 * that each take a bounded time, so that a caller with deadlines can spread a long check (a
 * SHA-256 over a megabyte on a Cortex-M3) over many calls. start() begins a check of
 * [addr, addr + len): an image that would extend beyond it must be refused. step() returns
 * BOOT_CHECK_BUSY until it is done, then VALID with the image's id and version, INVALID, or
 * ERROR if a read of the device failed. Every BUSY step must advance by at least one byte: a
 * check that has returned BUSY more than len times is taken as INVALID. The id must tell images
 * apart, header included (with lib/image: a SHA-256 over the image header, which contains the
 * payload's digest, is a good choice). select validates in one call over the whole slot; the
 * slot writer one step per call over the bytes it has written.
 *
 * THE SLOT WRITER (boot_writer_t) writes a new image into the slot that is not CONFIRMED, in the
 * calls phase 5's UDS download makes (app/flash_job.h): boot_writer_begin() (the eraseMemory
 * routine, which names the slot by its start address, boot_slot_by_address(), and the image
 * size; the slot must be the one that is not CONFIRMED, o in THE STATE MACHINE; writes the BEGIN
 * record before the first erase), boot_writer_erase_step() until BOOT_OK (one sector per call,
 * responsePending in between), boot_writer_write() for every TransferData block (any length,
 * consecutive offsets; whole pages are programmed, a partial page waits in the writer), and
 * boot_writer_finish_step() until BOOT_OK (its first call at RequestTransferExit: the last
 * partial page and the start of the validation; the others in the checkProgrammingDependencies
 * routine: the validation one step per call, then the COMMIT record). Only the sectors the image
 * needs are erased, and each of them always, even if it reads erased.
 *
 * WHAT A CALLER HAS TO KNOW. The functions keep no data of their own and allocate nothing. One
 * boot_t per device, and all calls on it (and on the writers that use it) from one task, or
 * otherwise serialised by the caller: nothing here locks. Every operation reads the metadata
 * from the device first, so a boot_t is never out of date with the flash: one read of each entry
 * of the used parts and of the erased entry that ends each, then the last valid record of each
 * used part decoded, read backwards from its end (boot_describe() decodes every entry, to count
 * the records). Each call takes the time of the flash operations it makes: a record write is at
 * most one sector erase and three programs, then a read of the log; select adds one validation
 * of up to two slots.
 *
 * LIMITS. The tear model has no weak bits: a bit that reads 1 after a cut is taken as erased, and
 * a unit that reads all 0xFF after a cut program that did nothing is programmed again later (the
 * next record can go into such an entry, the next attempt into such a field). On a part with an
 * error-correcting code per unit, an interrupted program can leave check bits written under data
 * that reads 0xFF; the driver of such a part must report a unit it cannot read cleanly as not
 * erased (for example as 0x00 bytes) rather than as 0xFF. Retention errors, read disturb and
 * erase suspend are outside the model too. No anti-rollback (an older version may be
 * written and booted) and no authenticity (the id is a digest, not a signature): docs/update.md.
 */
#ifndef FCN_BOOT_H
#define FCN_BOOT_H

#include <stdbool.h>
#include <stdint.h>

#include "flash/flash.h"

/** The number of image slots: A (0) and B (1). */
#define BOOT_SLOTS 2U

/** How many times a TRIAL image is booted without confirming itself before it is rolled back. */
#define BOOT_MAX_ATTEMPTS 3U

/** "No slot" in a slot index of a result. */
#define BOOT_NO_SLOT 0xFFU

/** Bytes of an image id (see THE VALIDATOR). */
#define BOOT_IMAGE_ID_SIZE 32U

/** The number of metadata sectors a layout may have. */
#define BOOT_META_SECTORS_MIN 2U
#define BOOT_META_SECTORS_MAX 8U

/** The largest program unit and page this module works with: it keeps a page in RAM. */
#define BOOT_UNIT_MAX 64U
#define BOOT_PAGE_MAX 512U

/** The record before its padding to whole units, and the smallest commit or attempt field. */
#define BOOT_RECORD_SIZE 96U
#define BOOT_FIELD_MIN   8U

/** The record's first word, the bytes 'F' 'C' 'N' 'B' read little-endian, and its version. */
#define BOOT_RECORD_MAGIC   0x424E4346U
#define BOOT_RECORD_VERSION 1U

/** The result of an operation. */
typedef enum {
    BOOT_OK = 0,             /**< done */
    BOOT_BUSY,               /**< a step function: done with this step, call again */
    BOOT_E_ARG,              /**< a NULL pointer, a slot index >= BOOT_SLOTS, an uninitialised
                                  context, no validator where one is needed */
    BOOT_E_LAYOUT,           /**< boot_init(): the layout does not fit the device or this module */
    BOOT_E_STATE,            /**< refused in the present state (see THE STATE MACHINE), or no
                                  sequence number is left for a record (WRITING A RECORD) */
    BOOT_E_SEQUENCE,         /**< the slot writer was called out of order */
    BOOT_E_SIZE,             /**< an image size of 0 or beyond the slot, or data past it */
    BOOT_E_INVALID,          /**< the written image did not validate */
    BOOT_E_NOTHING_BOOTABLE, /**< select: no slot holds an image the metadata allows to boot */
    BOOT_E_FLASH,            /**< the device refused an operation, a read failed, or a record
                                  written did not read back as the newest one */
    BOOT_E_POWER_LOST        /**< the device lost its supply during the call */
} boot_status_t;

/** The state of a slot (see SLOT STATES). The values are the bytes stored in a record. */
typedef enum {
    BOOT_SLOT_EMPTY = 0,
    BOOT_SLOT_WRITING = 1,
    BOOT_SLOT_TRIAL = 2,
    BOOT_SLOT_CONFIRMED = 3,
    BOOT_SLOT_PREVIOUS = 4,
    BOOT_SLOT_REJECTED = 5
} boot_slot_state_t;

/** What wrote a record, and what a select wrote. The values are the bytes stored in a record. */
typedef enum {
    BOOT_EVENT_NONE = 0,     /**< select only: no record written */
    BOOT_EVENT_ADOPT = 1,    /**< select in the factory state recorded the image it found */
    BOOT_EVENT_BEGIN = 2,    /**< an update began: the slot is WRITING */
    BOOT_EVENT_COMMIT = 3,   /**< the written image validated: the slot is TRIAL */
    BOOT_EVENT_CONFIRM = 4,  /**< the trial confirmed itself: CONFIRMED, the old one PREVIOUS */
    BOOT_EVENT_ROLLBACK = 5, /**< select: the trial used all its attempts, it is REJECTED */
    BOOT_EVENT_REJECT = 6,   /**< select: the trial did not validate, it is REJECTED */
    BOOT_EVENT_FALLBACK = 7  /**< select: the active image did not validate; the PREVIOUS one is
                                  CONFIRMED again, the other REJECTED */
} boot_event_t;

/** The result of a validator step (see THE VALIDATOR). */
typedef enum {
    BOOT_CHECK_BUSY = 0, /**< not done: call step() again */
    BOOT_CHECK_VALID,    /**< the region holds a valid image; the info is filled */
    BOOT_CHECK_INVALID,  /**< it does not */
    BOOT_CHECK_ERROR     /**< a read of the device failed: nothing can be said */
} boot_check_t;

/** What the validator tells about a valid image. */
typedef struct {
    uint8_t id[BOOT_IMAGE_ID_SIZE]; /**< tells this image from every other one */
    uint32_t version;               /**< the image version, for diagnosis and the hand-off */
} boot_image_info_t;

/**
 * Image validation, supplied by the caller (see THE VALIDATOR). One check at a time per
 * validator: start() abandons a check that was not finished.
 */
typedef struct {
    void *ctx; /**< passed to both functions */
    /** Begin checking the image in [addr, addr + len) of `dev`. */
    void (*start)(void *ctx, const flash_dev_t *dev, uint32_t addr, uint32_t len);
    /** One bounded step; on BOOT_CHECK_VALID it fills `info`. */
    boot_check_t (*step)(void *ctx, boot_image_info_t *info);
} boot_validator_t;

/** A region of the device: whole sectors. */
typedef struct {
    uint32_t addr; /**< first byte, at a sector start */
    uint32_t size; /**< bytes, a multiple of the sector size, at least one sector */
} boot_region_t;

/** The device, the layout and the validator. Copied by boot_init(); `dev` must outlive it. */
typedef struct {
    const flash_dev_t *dev;
    uint32_t meta_addr;    /**< first metadata sector, at a sector start */
    uint32_t meta_sectors; /**< BOOT_META_SECTORS_MIN..BOOT_META_SECTORS_MAX consecutive sectors */
    boot_region_t slot[BOOT_SLOTS];
    /** Needed by boot_select() and the slot writer; both functions NULL if the caller uses
     *  neither (a node that only confirms). */
    boot_validator_t validator;
} boot_config_t;

/* Private to lib/boot; in the header so that a boot_t can be allocated statically. One slot
 * of a decoded record. */
typedef struct {
    boot_slot_state_t state;
    uint32_t version;
    uint8_t id[BOOT_IMAGE_ID_SIZE];
} boot_slot_record_t;

/* Private to lib/boot: a decoded record. */
typedef struct {
    uint32_t seq;
    boot_event_t event;
    uint8_t event_slot;
    boot_slot_record_t slot[BOOT_SLOTS];
} boot_record_t;

/* Private to lib/boot: the configuration, the sizes derived from it, and what the last read of
 * the metadata found. Read it through boot_describe(). */
typedef struct {
    boot_config_t config;
    uint32_t ready;              /* a mark set by boot_init() */
    uint32_t field_size;         /* F */
    uint32_t body_size;          /* the record padded to whole units */
    uint32_t entry_size;         /* body, commit and attempt fields */
    uint32_t stride;             /* from one entry to the next */
    uint32_t entries_per_sector; /* entries in one metadata sector */
    bool have_record;            /* a valid record was found */
    boot_record_t record;        /* the newest one */
    uint32_t record_sector;      /* where it is: metadata sector 0..meta_sectors - 1 */
    uint32_t record_index;       /* and entry */
    uint8_t attempts_used;       /* its attempt fields that do not read erased */
    uint8_t attempt_next;        /* its first erased attempt field, or BOOT_MAX_ATTEMPTS */
    uint32_t append_sector;      /* where the next record goes */
    uint32_t append_index;
    bool append_erases;     /* the next record erases append_sector first */
    uint32_t records_valid; /* valid records found (counted by boot_describe() only, else 0) */
    uint32_t entries_used;  /* entries in the used parts of the sectors */
} boot_t;

/** What boot_select() decided. */
typedef struct {
    uint8_t slot;     /**< the slot to boot, or BOOT_NO_SLOT */
    bool trial;       /**< it is booted as a TRIAL */
    uint8_t attempt;  /**< for a trial the attempt this boot is, 1..BOOT_MAX_ATTEMPTS; else 0 */
    uint32_t version; /**< the image version the validator returned for it; 0 if none */
    uint32_t addr;    /**< the slot's region, for the jump; 0 if none */
    uint32_t size;
    boot_event_t event; /**< the record this select wrote: NONE, ADOPT, ROLLBACK, REJECT or
                             FALLBACK (also set when the write itself failed) */
    uint8_t event_slot; /**< the slot that event acted on (ADOPT: the slot adopted; ROLLBACK,
                             REJECT: the trial's; FALLBACK: the slot fallen back to), or
                             BOOT_NO_SLOT */
    /** BOOT_OK, or why a record or attempt write of this select failed (BOOT_E_FLASH; or
     *  BOOT_E_STATE, no sequence number left). A failed write never makes select boot a TRIAL:
     *  it boots the CONFIRMED image instead. */
    boot_status_t write_status;
} boot_selection_t;

/** One slot in a description. */
typedef struct {
    boot_slot_state_t state; /**< from the newest record; EMPTY in the factory state */
    uint32_t version;        /**< as recorded; 0xFFFFFFFF for EMPTY and WRITING */
    uint8_t id[BOOT_IMAGE_ID_SIZE];
    uint32_t addr; /**< the slot's region */
    uint32_t size;
} boot_slot_info_t;

/** The state of the metadata, for logs, statistics and tools (boot_describe()). */
typedef struct {
    bool have_record;   /**< false: the factory state; the fields up to `slot` are then 0 or
                                BOOT_NO_SLOT */
    uint32_t seq;       /**< the newest record's sequence number */
    boot_event_t event; /**< and its event */
    uint8_t event_slot;
    uint32_t record_sector; /**< where it is: metadata sector and entry */
    uint32_t record_index;
    uint8_t active_slot;   /**< the CONFIRMED slot, or BOOT_NO_SLOT */
    uint8_t trial_slot;    /**< the TRIAL slot, or BOOT_NO_SLOT */
    uint8_t attempts_used; /**< of the trial: 0..BOOT_MAX_ATTEMPTS; 0 without one */
    uint8_t attempts_left; /**< BOOT_MAX_ATTEMPTS - attempts_used; 0 without a trial */
    boot_slot_info_t slot[BOOT_SLOTS];
    uint32_t records_valid; /**< valid records in the used parts of the log */
    uint32_t entries_used;  /**< entries in the used parts (valid, torn and old records) */
    uint32_t entries_total; /**< meta_sectors * entries_per_sector */
    uint32_t entries_per_sector;
    uint32_t entry_stride;  /**< bytes from one entry to the next */
    uint32_t field_size;    /**< F: bytes of the commit field and of each attempt field */
    uint32_t record_body;   /**< bytes of the record with its padding */
    bool next_write_erases; /**< the next record erases a metadata sector first */
    uint32_t next_sector;   /**< where the next record goes */
    uint32_t next_index;
} boot_description_t;

/** The phases of the slot writer. */
typedef enum {
    BOOT_WRITER_IDLE = 0,   /**< nothing begun, or refused at begin */
    BOOT_WRITER_ERASING,    /**< begun: erase steps to do */
    BOOT_WRITER_WRITING,    /**< erased: taking data */
    BOOT_WRITER_VALIDATING, /**< all data written: validation steps to do */
    BOOT_WRITER_COMMITTING, /**< validated: the COMMIT record is the next step */
    BOOT_WRITER_DONE,       /**< committed: the slot is TRIAL */
    BOOT_WRITER_FAILED      /**< a flash error, a validation that failed, or a state change by
                                 someone else: begin again */
} boot_writer_phase_t;

/* Private to lib/boot; in the header for static allocation. Read it through
 * boot_writer_progress(). */
typedef struct {
    boot_t *boot;
    boot_writer_phase_t phase;
    uint8_t slot;
    uint32_t image_size;
    uint32_t erase_total;   /* sectors the image needs */
    uint32_t erase_done;    /* of those, erased */
    uint32_t written;       /* bytes taken by boot_writer_write() */
    uint32_t page_offset;   /* slot offset of page[0] */
    uint32_t page_fill;     /* bytes in page[] */
    uint32_t check_steps;   /* validator steps that returned BUSY */
    boot_image_info_t info; /* from the validation */
    uint8_t page[BOOT_PAGE_MAX];
} boot_writer_t;

/** Where a slot writer stands (boot_writer_progress()). */
typedef struct {
    boot_writer_phase_t phase;
    uint8_t slot;            /**< the slot being written, or BOOT_NO_SLOT before a begin */
    uint32_t image_size;     /**< bytes announced at begin */
    uint32_t sectors_total;  /**< sectors the image needs */
    uint32_t sectors_erased; /**< of those, erased */
    uint32_t written;        /**< bytes taken */
} boot_writer_progress_t;

/**
 * Check `config` and prepare `b` for it. Reads nothing from the device: every operation reads
 * the metadata itself.
 *
 * Returns BOOT_E_ARG if a pointer is NULL (a NULL `b` is left alone; any other refusal leaves
 * `b` uninitialised), the device lacks an operation, or the validator has one function but
 * not the other. BOOT_E_LAYOUT if the geometry is not as flash/flash.h says, a unit is larger
 * than BOOT_UNIT_MAX, a page larger than BOOT_PAGE_MAX or smaller than a field, a region does
 * not start at a sector or does not end inside the device, a slot is not whole sectors, the
 * metadata has fewer or more sectors than allowed, two regions overlap, or a sector holds no
 * entry. Otherwise BOOT_OK. Any context; no flash access, so no time to speak of.
 */
boot_status_t boot_init(boot_t *b, const boot_config_t *config);

/**
 * The bootloader's decision: which slot to boot, and the state change that goes with it (see
 * THE STATE MACHINE). Validates the slot it boots, and the slots it has to decide about, through
 * the validator in one call per slot.
 *
 * Returns BOOT_OK with `sel` describing the boot; BOOT_E_NOTHING_BOOTABLE (sel->slot is
 * BOOT_NO_SLOT, sel->event may say what was written on the way); BOOT_E_FLASH if a read of the
 * metadata or the validator failed (nothing is booted: the device cannot be trusted);
 * BOOT_E_POWER_LOST if the device lost power; BOOT_E_ARG if a pointer is NULL, `b` is not
 * initialised or there is no validator. The bootloader, before the image runs. Takes the time of
 * up to two validations of a whole slot, one record write and one attempt write.
 */
boot_status_t boot_select(boot_t *b, boot_selection_t *sel);

/**
 * The running TRIAL image confirms itself: `slot` (the slot it was booted from, as the
 * bootloader's hand-off says) becomes CONFIRMED and the slot that was CONFIRMED becomes
 * PREVIOUS. One record write.
 *
 * Returns BOOT_OK once the record is written and reads back as the newest; BOOT_E_STATE if
 * there is no record, `slot` is not TRIAL, the trial has used no attempt, or no sequence number
 * is left; BOOT_E_ARG for a NULL or uninitialised `b` or a slot >= BOOT_SLOTS; BOOT_E_FLASH or
 * BOOT_E_POWER_LOST from the device (the state is then the one before the call or, after a cut,
 * possibly the confirmed one). The task that owns `b`, in the running image; one metadata erase
 * and two or three programs at most.
 */
boot_status_t boot_confirm(boot_t *b, uint8_t slot);

/**
 * Read the metadata and describe it into `d` (see boot_description_t). Returns BOOT_OK, or
 * BOOT_E_ARG for a NULL pointer or an uninitialised `b`, or BOOT_E_FLASH / BOOT_E_POWER_LOST if
 * a read failed, in which case `d` describes the factory state. The task that owns `b`; reads
 * the used parts of the log and decodes every record in them.
 */
boot_status_t boot_describe(boot_t *b, boot_description_t *d);

/**
 * The slot whose region starts at `address`, as a UDS RequestDownload names it: BOOT_OK with
 * *slot set, or BOOT_E_ARG (nothing set) if no slot starts there or a pointer is NULL or `b` is
 * not initialised. The address is in the device's address space, as in the layout; a target
 * that maps the device into the CPU's address space subtracts its base first. Any context.
 */
boot_status_t boot_slot_by_address(const boot_t *b, uint32_t address, uint8_t *slot);

/**
 * Begin writing an image of `image_size` bytes into `slot` with the writer `w`, which keeps a
 * pointer to `b`. Writes the BEGIN record, unless `slot` is WRITING already, before any sector
 * of the slot is erased (see THE SLOT WRITER). Any earlier use of `w` is abandoned.
 *
 * Returns BOOT_OK, the writer then in BOOT_WRITER_ERASING; BOOT_E_STATE in the factory state,
 * while a slot is TRIAL, or if `slot` is CONFIRMED; BOOT_E_SIZE for a size of 0 or beyond the
 * slot; BOOT_E_ARG for a NULL or uninitialised argument, a slot >= BOOT_SLOTS or no validator.
 * After a refusal the writer is IDLE; after BOOT_E_FLASH, BOOT_E_POWER_LOST or BOOT_E_STATE (no
 * sequence number left) from the record write it is FAILED (no sector of the slot has been
 * erased). The task that owns `b`; one record write.
 */
boot_status_t boot_writer_begin(boot_writer_t *w, boot_t *b, uint8_t slot, uint32_t image_size);

/**
 * Erase the next sector the image needs. Returns BOOT_BUSY while more remain, BOOT_OK when the
 * last one is erased (the writer is then WRITING). BOOT_E_SEQUENCE if the writer is not ERASING;
 * BOOT_E_STATE (writer FAILED) if the slot is no longer WRITING in the last metadata `b` read;
 * BOOT_E_FLASH or BOOT_E_POWER_LOST (writer FAILED) from the device; BOOT_E_ARG for a NULL or
 * unused writer. One sector erase.
 */
boot_status_t boot_writer_erase_step(boot_writer_t *w);

/**
 * Take `len` bytes of the image at image offset `offset`, which must be the number of bytes
 * taken so far. Every page that is complete is programmed; the rest waits in the writer.
 *
 * Returns BOOT_OK; BOOT_E_ARG for a NULL or unused writer, NULL data or a length of 0;
 * BOOT_E_SEQUENCE (nothing taken) if the writer is not WRITING or `offset` is not the next
 * offset; BOOT_E_SIZE (nothing taken) if the data would go past the image size; BOOT_E_STATE
 * (writer FAILED) if the slot is no longer WRITING; BOOT_E_FLASH or BOOT_E_POWER_LOST (writer
 * FAILED, part of the data programmed) from the device. Up to len / page_size + 1 programs.
 */
boot_status_t boot_writer_write(boot_writer_t *w, uint32_t offset, const uint8_t *data,
                                uint32_t len);

/**
 * One step of the end of the transfer: the first call programs the partial last page and starts
 * the validation of [slot start, slot start + image size); each later call is one validator
 * step; the call after the validator says VALID writes the COMMIT record (the slot becomes
 * TRIAL with no attempt used) and returns BOOT_OK. BOOT_BUSY until then.
 *
 * Errors: BOOT_E_SEQUENCE if fewer bytes than the image size were taken (the writer stays
 * WRITING) or the writer is not in a phase of the end; BOOT_E_INVALID (writer FAILED, slot still
 * WRITING) if the image does not validate; BOOT_E_STATE (FAILED) if the slot is no longer
 * WRITING; BOOT_E_FLASH (FAILED) if the device or the validator's reads failed; BOOT_E_POWER_LOST
 * (FAILED); BOOT_E_ARG for a NULL or unused writer. Each call is at most one program, one
 * validator step, or one record write.
 */
boot_status_t boot_writer_finish_step(boot_writer_t *w);

/** Where `w` stands. A NULL `w` or `p` does nothing. After a begin that was refused, or failed
 *  in its record write, the slot is BOOT_NO_SLOT; a writer that never saw a begin reads as its
 *  memory says (IDLE and slot 0 for a zeroed static one). The context that owns `w`. */
void boot_writer_progress(const boot_writer_t *w, boot_writer_progress_t *p);

/** Short names for logs and tools ("ok", "trial", "rollback", ...); "?" for a value outside the
 *  enumeration. Any context. */
const char *boot_status_name(boot_status_t status);
const char *boot_slot_state_name(boot_slot_state_t state);
const char *boot_event_name(boot_event_t event);
const char *boot_writer_phase_name(boot_writer_phase_t phase);

#endif /* FCN_BOOT_H */
