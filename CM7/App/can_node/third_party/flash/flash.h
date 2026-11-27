/* SPDX-License-Identifier: MIT */
/**
 * @file
 * The flash device interface: what the update code (lib/image, lib/boot, the slot writer)
 * expects from a NOR flash, whichever part and driver stand behind it.
 *
 * A device is a flash_dev_t: its geometry and three operations, called through function
 * pointers with the device's own context. The NOR model of lib/flash/nor.h implements it over
 * memory; the host platform drives that model over a mapped file (plat_flash.h); a target
 * build implements it with the driver of its part.
 *
 * GEOMETRY. Four sizes, each a power of two except `size`:
 *
 *     unit_size  divides  page_size  divides  sector_size  divides  size
 *
 * The device's bytes have the addresses 0 to size - 1. The erase unit is the sector: the
 * sectors start at the multiples of sector_size. The program unit is the unit: units start at
 * the multiples of unit_size, and one program writes whole units inside one page (a page is
 * the most one program writes; pages start at the multiples of page_size).
 *
 * THE RULES. They are the rules of a NOR flash, with one made stricter (write-once):
 *
 *  - Erased bytes read 0xFF. An erase sets one whole sector to 0xFF.
 *  - A program can only clear bits: the byte that results is the old byte AND the byte given
 *    (new = old & src). It writes whole units inside one page.
 *  - STRICT WRITE-ONCE: a unit is programmed at most once between two erases of its sector. A
 *    program of a unit that does not read all 0xFF is refused with FLASH_E_NOT_ERASED, and
 *    then nothing at all is changed, not even the other units of the call. Together with the
 *    rule before, this means that an accepted program leaves exactly the bytes it was given.
 *    The rule keeps a design that follows it valid on flash with an error-correcting code per
 *    program unit, which forbids programming a written unit again (its check bits would no
 *    longer match); a plain NOR part would allow it. Code above this interface must never
 *    rely on programming a unit twice, for example to clear one more bit of a flag.
 *  - Operations are synchronous: a call returns when the operation is over. How long that
 *    takes is the device's business; a device that waits for its part lets the caller's task
 *    block meanwhile (on the host, plat_flash.h). A caller that must stay responsive splits
 *    its work into operations (one sector erase per call, one page per program).
 *  - A loss of supply can cut an operation: the call returns FLASH_E_POWER_LOST, and what
 *    the cut left in the device is not defined by this interface. The NOR model defines it
 *    (nor.h, POWER CUTS): code that has to survive a cut is tested against that model.
 *
 * WHAT A CALL CHECKS, in this order, the first failing check deciding the result:
 *   1. the supply: unpowered, every call returns FLASH_E_POWER_LOST;
 *   2. the range: FLASH_E_RANGE when any byte of the call lies outside the device, or len is 0;
 *   3. program and erase only, the alignment: FLASH_E_ALIGN for a program whose address or
 *      length is not a multiple of unit_size or that does not lie within one page, and for an
 *      erase whose address is not the start of a sector;
 *   4. program only, the write-once rule: FLASH_E_NOT_ERASED.
 * A call refused by checks 1 to 4 changes nothing. FLASH_E_POWER_LOST from a call that passed
 * them means that this call was cut.
 *
 * CONTEXT. None of the three operations may be called from an interrupt handler. A device does
 * not serialise its callers: one task at a time (on the node, the one task that writes the
 * flash). The buffers are the caller's and must hold `len` bytes.
 */
#ifndef FCN_FLASH_H
#define FCN_FLASH_H

#include <stdint.h>

/** The result of a flash operation. */
typedef enum {
    FLASH_OK = 0,       /**< done */
    FLASH_E_RANGE,      /**< outside the device or len 0 */
    FLASH_E_ALIGN,      /**< program not in whole units within one page; erase not at a sector
                             start */
    FLASH_E_NOT_ERASED, /**< program of a unit that is not all 0xFF: nothing changed */
    FLASH_E_POWER_LOST  /**< the supply is gone: this operation was cut (torn) or the device is
                             unpowered */
} flash_status_t;

/** The sizes of a device, in bytes. See GEOMETRY above. */
typedef struct {
    uint32_t size;        /**< bytes, a multiple of sector_size */
    uint32_t sector_size; /**< erase unit, power of two */
    uint32_t page_size;   /**< most bytes one program writes, power of two dividing sector_size */
    uint32_t unit_size;   /**< program unit, power of two dividing page_size */
} flash_geometry_t;

/**
 * A flash device. `ctx` is passed unchanged to each operation; what it points to is the
 * device's own business.
 *
 *  - read: copy `len` bytes from `addr` to `dst`. Any address and length within the device.
 *  - program: write the `len` bytes of `src` at `addr` under the rules above.
 *  - erase: set the sector that starts at `addr` to 0xFF.
 *
 * Each returns FLASH_OK or the first failing check, as described above.
 */
typedef struct {
    flash_geometry_t geometry;
    void *ctx;
    flash_status_t (*read)(void *ctx, uint32_t addr, uint8_t *dst, uint32_t len);
    flash_status_t (*program)(void *ctx, uint32_t addr, const uint8_t *src, uint32_t len);
    flash_status_t (*erase)(void *ctx, uint32_t addr);
} flash_dev_t;

#endif /* FCN_FLASH_H */
