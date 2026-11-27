# Upstream Attribution

The C11 protocol modules in this directory are vendored verbatim from
[`leoembeddeder/can-node`](https://github.com/leoembeddeder/can-node) (MIT License)
at commit `73f03801548265858883c35432aad4bea8bc9162` (merged in PR `#2`, `6601829`):

- `can/can_frame.h`, `can/can_frame.c` — Classical CAN & CAN-FD frame validation and DLC/length conversion helpers
- `isotp/isotp.h`, `isotp/isotp.c` — ISO 15765-2:2016 Classical CAN & CAN-FD ISO-TP transport (`SF`/`FF`/`CF`/`FC`, 32-bit `FF_DL`, sub-millisecond `STmin`, `N_As`/`N_Ar`/`N_Bs`/`N_Cr` timers)
- `dtc/dtc.h`, `dtc/dtc.c` — ISO 14229-1 Annex D & AUTOSAR Dem debounced DTC store
- `crc/crc.h`, `crc/crc.c` — CRC-8, CRC-15, and CRC-32/ISO-HDLC
- `uds/uds.h`, `uds/uds_internal.h`, `uds/uds.c`, `uds/uds_program.c` — ISO 14229-1 UDS server (`0x10`, `0x11`, `0x14`, `0x19`, `0x22`, `0x31`, `0x34`, `0x36`, `0x37`, `0x3E`, `NRC 0x78` ResponsePending, S3server)
- `boot/boot.h`, `flash/flash.h` — Header-only type dependencies (`boot_status_t`) included by `uds/uds.h`

## Local Modification

Only one preprocessor guard was added in `uds/uds.h` so `UDS_BLOCK_DATA_MAX` can be overridden at compile time (defaulting to `4096U` in the STM32H745 port to exercise 32-bit `FF_DL > 4095` frames natively, or `1024U` for upstream parity):

```c
#ifndef UDS_BLOCK_DATA_MAX
#define UDS_BLOCK_DATA_MAX 1024U
#endif
```
