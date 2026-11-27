/* SPDX-License-Identifier: MIT */
/**
 * @file
 * The CRCs used in this project. All are bit-exact with the catalogued algorithm named in
 * their comment; the unit tests check each against its published check value, the CRC of the
 * nine ASCII characters "123456789".
 *
 * Every function has the same shape: it takes the CRC register as it stands, feeds data in
 * and returns the register. Initial value and final XOR are applied by the caller, through
 * the constants below or the one-shot helpers, so that a CRC can be computed over several
 * separate pieces of data without copying them together first.
 */
#ifndef FCN_CRC_H
#define FCN_CRC_H

#include <stddef.h>
#include <stdint.h>

/* ------------------------------------------------------------------- CRC-8/SAE-J1850
 * Polynomial 0x1D, MSB first, not reflected. Catalogue parameters: init 0xFF, xorout 0xFF,
 * check value 0x4B. Used by AUTOSAR E2E Profile 1 and Profile 11. */
#define CRC8_J1850_INIT   0xFFU
#define CRC8_J1850_XOROUT 0xFFU

/** Feed `len` bytes into a CRC-8/SAE-J1850 register and return the new register value. */
uint8_t crc8_j1850_update(uint8_t crc, const uint8_t *data, size_t len);

/* ------------------------------------------------------------------- CRC-8/AUTOSAR (8H2F)
 * Polynomial 0x2F, MSB first, not reflected. Catalogue parameters: init 0xFF, xorout 0xFF,
 * check value 0xDF. Used by AUTOSAR E2E Profile 2. */
#define CRC8_H2F_INIT   0xFFU
#define CRC8_H2F_XOROUT 0xFFU

/** Feed `len` bytes into a CRC-8/AUTOSAR register and return the new register value. */
uint8_t crc8_h2f_update(uint8_t crc, const uint8_t *data, size_t len);

/* ------------------------------------------------------------------- CRC-15/CAN
 * Polynomial 0x4599 (x^15+x^14+x^10+x^8+x^7+x^4+x^3+1), init 0, no final XOR, not
 * reflected, check value 0x059E. This is the frame check sequence of classical CAN; it is
 * computed over the frame's bits from start of frame to the end of the data field, before
 * bit stuffing, which is why the update works on single bits. */
#define CRC15_CAN_INIT 0x0000U

/** Feed one bit (0 or 1) into a CRC-15/CAN register and return the new 15-bit value. */
uint16_t crc15_can_update_bit(uint16_t crc, uint8_t bit);

/** Feed `len` whole bytes, most significant bit first. */
uint16_t crc15_can_update(uint16_t crc, const uint8_t *data, size_t len);

/* ------------------------------------------------------------------- CRC-32/ISO-HDLC
 * The CRC-32 of Ethernet, zlib and PNG: polynomial 0x04C11DB7 reflected (0xEDB88320),
 * init 0xFFFFFFFF, xorout 0xFFFFFFFF, check value 0xCBF43926. Used for firmware images and
 * for the update metadata, so that Python's zlib.crc32 is an independent implementation. */
#define CRC32_INIT   0xFFFFFFFFU
#define CRC32_XOROUT 0xFFFFFFFFU

/** Feed `len` bytes into a CRC-32 register (the register, not the finished CRC). */
uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len);

/** One-shot CRC-32 of a buffer: init, update, final XOR. Equals zlib.crc32(data). */
uint32_t crc32_compute(const uint8_t *data, size_t len);

#endif /* FCN_CRC_H */
