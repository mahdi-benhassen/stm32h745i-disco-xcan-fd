/* SPDX-License-Identifier: MIT */
/**
 * @file
 * Compile-time configuration for the STM32H745I-DISCO port of leoembeddeder/can-node
 * (ISO 15765-2:2016 CAN-FD ISO-TP + ISO 14229-1 UDS server).
 *
 * Every macro is guarded with #ifndef so CMake presets or -D flags can switch cleanly between:
 *   - Option A (CAN_NODE_SINGLE_BOARD = 0): Standalone UDS/ISO-TP ECU on FDCAN1.
 *   - Option B (CAN_NODE_SINGLE_BOARD = 1): PCAN-USB Pro FD on FDCAN1 + UDS/ISO-TP ECU on FDCAN2.
 */
#ifndef STM32_CAN_NODE_CONFIG_H
#define STM32_CAN_NODE_CONFIG_H

#include "uds/uds.h"

/* ------------------------------------------------------------------------- board topology */

/** 0 = Option A (Standalone ECU on FDCAN1); 1 = Option B (PCAN on FDCAN1, ECU on FDCAN2). */
#ifndef CAN_NODE_SINGLE_BOARD
#define CAN_NODE_SINGLE_BOARD 0U
#endif

/* -------------------------------------------------------------------------- CAN / ISO-TP */

/** Physical UDS CAN identifiers (Normal Addressing, 11-bit standard by default). */
#ifndef CAN_NODE_RX_ID
#define CAN_NODE_RX_ID 0x7E0U
#endif

#ifndef CAN_NODE_TX_ID
#define CAN_NODE_TX_ID 0x7E8U
#endif

#ifndef CAN_NODE_EXTENDED_IDS
#define CAN_NODE_EXTENDED_IDS 0U
#endif

/** 1 = CAN-FD (ISO 11898-1), 0 = Classical CAN. */
#ifndef CAN_NODE_CAN_FD
#define CAN_NODE_CAN_FD 1U
#endif

/** 1 = Bit Rate Switch (requires CAN_NODE_CAN_FD = 1), 0 = BRS off. */
#ifndef CAN_NODE_CAN_FD_BRS
#define CAN_NODE_CAN_FD_BRS 1U
#endif

/** ISO 15765-2 TX_DL: 8 (classical) or {8, 12, 16, 20, 24, 32, 48, 64} (CAN-FD). */
#ifndef CAN_NODE_TX_DL
#if CAN_NODE_CAN_FD
#define CAN_NODE_TX_DL 64U
#else
#define CAN_NODE_TX_DL 8U
#endif
#endif

/** FDCAN bitrates derived from 80 MHz PLL1Q clock. */
#ifndef CAN_NODE_NOMINAL_BITRATE
#define CAN_NODE_NOMINAL_BITRATE 500000U
#endif

#ifndef CAN_NODE_DATA_BITRATE
#define CAN_NODE_DATA_BITRATE 2000000U
#endif

/** Receive buffer size in bytes (>= 4096 exercises 32-bit FF_DL escape sequences). */
#ifndef CAN_NODE_ISOTP_RX_BUF_SIZE
#define CAN_NODE_ISOTP_RX_BUF_SIZE 8192U
#endif

/** ISO 15765-2 protocol timing and FlowControl parameters. */
#ifndef CAN_NODE_ISOTP_TIMEOUT_US
#define CAN_NODE_ISOTP_TIMEOUT_US ISOTP_TIMEOUT_US
#endif

#ifndef CAN_NODE_ISOTP_BS
#define CAN_NODE_ISOTP_BS 16U
#endif

#ifndef CAN_NODE_ISOTP_STMIN
#define CAN_NODE_ISOTP_STMIN 0x00U
#endif

#ifndef CAN_NODE_ISOTP_TX_WFT_MAX
#define CAN_NODE_ISOTP_TX_WFT_MAX 4U
#endif

#ifndef CAN_NODE_ISOTP_N_CS_US
#define CAN_NODE_ISOTP_N_CS_US 150000U
#endif

/** 2 x N_As safety cap before an accepted ECUReset executes. */
#ifndef CAN_NODE_RESET_DEADLINE_US
#define CAN_NODE_RESET_DEADLINE_US (2U * CAN_NODE_ISOTP_TIMEOUT_US)
#endif

/* ------------------------------------------------------------------- virtual UDS slot & IDs */

/** Virtual flash slot address & size in AXI-SRAM D1 (.ram_d1_bss) for UDS download tests. */
#ifndef CAN_NODE_DL_SLOT_ADDR
#define CAN_NODE_DL_SLOT_ADDR 0x00010000U
#endif

#ifndef CAN_NODE_DL_SLOT_SIZE
#define CAN_NODE_DL_SLOT_SIZE 65536U
#endif

/** Simulated eraseMemory duration (60 ms -> exercises NRC 0x78 ResponsePending at 40 ms). */
#ifndef CAN_NODE_ERASE_SIM_US
#define CAN_NODE_ERASE_SIM_US 60000U
#endif

/** Reported application version (DID 0xF189 / DID 0xFD00). */
#ifndef CAN_NODE_SW_VER_MAJOR
#define CAN_NODE_SW_VER_MAJOR 1U
#endif

#ifndef CAN_NODE_SW_VER_MINOR
#define CAN_NODE_SW_VER_MINOR 2U
#endif

#ifndef CAN_NODE_SW_VER_PATCH
#define CAN_NODE_SW_VER_PATCH 3U
#endif

/* ---------------------------------------------------------------------- observability / UART */

#ifndef CAN_NODE_UART_TELEMETRY
#define CAN_NODE_UART_TELEMETRY 1U
#endif

#ifndef CAN_NODE_TELEMETRY_PERIOD_US
#define CAN_NODE_TELEMETRY_PERIOD_US 2000000U
#endif

#ifndef CAN_NODE_HEALTH_PERIOD_US
#define CAN_NODE_HEALTH_PERIOD_US 250000U
#endif

/* ------------------------------------------------------------------- compile-time invariants */

_Static_assert((CAN_NODE_SINGLE_BOARD == 0U) || (CAN_NODE_SINGLE_BOARD == 1U),
               "CAN_NODE_SINGLE_BOARD must be 0 (Option A) or 1 (Option B)");
_Static_assert((CAN_NODE_RX_ID != CAN_NODE_TX_ID), "UDS RX and TX CAN IDs must differ");
_Static_assert(CAN_NODE_ISOTP_RX_BUF_SIZE >= UDS_REQUEST_MAX,
               "ISO-TP rx_buffer must hold a complete UDS request");
_Static_assert(CAN_NODE_DL_SLOT_SIZE >= UDS_BLOCK_DATA_MAX,
               "Virtual download slot must fit at least one TransferData block");
_Static_assert((CAN_NODE_TX_DL == 8U) || (CAN_NODE_TX_DL == 12U) || (CAN_NODE_TX_DL == 16U) ||
                   (CAN_NODE_TX_DL == 20U) || (CAN_NODE_TX_DL == 24U) || (CAN_NODE_TX_DL == 32U) ||
                   (CAN_NODE_TX_DL == 48U) || (CAN_NODE_TX_DL == 64U),
               "CAN_NODE_TX_DL must be a valid ISO 15765-2 discrete payload length");
_Static_assert((CAN_NODE_CAN_FD != 0U) || (CAN_NODE_TX_DL == 8U),
               "Classical CAN requires CAN_NODE_TX_DL == 8");
_Static_assert((CAN_NODE_CAN_FD != 0U) || (CAN_NODE_CAN_FD_BRS == 0U),
               "CAN_NODE_CAN_FD_BRS requires CAN_NODE_CAN_FD == 1");
_Static_assert((CAN_NODE_ERASE_SIM_US == 0U) ||
                   (CAN_NODE_ERASE_SIM_US > ((uint32_t)UDS_PENDING_FIRST_MS * 1000U)),
               "CAN_NODE_ERASE_SIM_US should exceed 40 ms to exercise NRC 0x78 ResponsePending");

#endif /* STM32_CAN_NODE_CONFIG_H */
