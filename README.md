# XCAN-FD Firmware — HPMicro HPM5300EVK (`HPM5361`) High-Speed CherryUSB Port

This branch (`hpm5300evk`) ports the `PCAN-USB Pro FD` compatible firmware (`PCAN_PRO_FD=1`, `INCLUDE_LIN_INTERFACE=1`) to the **HPMicro `HPM5300EVK`** evaluation board (`HPM5361` RISC-V, **`hpm_sdk v1.13.0` + CherryUSB USB 2.0 High-Speed + Bosch MCAN3**).

> Looking for other targets? Switch branches:
> - `main`: STM32H745I-DISCO (`CMake` / ARM GCC, dual FDCAN)
> - `keil-mdk`: STM32H745I-DISCO (Keil MDK-ARM v5)
> - `at-start-f457`: Artery `AT-START-F457` (`AT32F457ZET7`, dual CAN-FD `CAN1`+`CAN2`, CMake + Keil MDK-ARM v5)

## Why HPM5300EVK + CherryUSB?

Unlike Full-Speed-only USB PHYs (12 Mbps / 64 B packets), the **`HPM5361`** integrates a **480 Mbps USB 2.0 High-Speed PHY (`USB0`)** backed by DMA + non-cacheable QHD/QTD descriptors in **CherryUSB**. This enables native **512-byte High-Speed Bulk** endpoints (`PCAN_HS_MAX_BULK_PACKET_SIZE = 512`) alongside hardware **Bosch MCAN (`MCAN3`)**, matching genuine PCAN-USB Pro FD USB High-Speed framing for heavy UDS / CAN-FD burst traffic and OTA flashing.

## Hardware & Clock Mapping (Verified Against HPM5300EVK Schematic)

| Resource | Pin / Instance | Notes |
|---|---|---|
| **MCU / Board** | `HPM5361` on `HPM5300EVK` | Built with `HPM_BUILD_TYPE=flash_xip` |
| **USB 2.0 High-Speed (`USB0`)** | Dedicated `USB_DP` (Pin 72), `USB_DM` (Pin 73), `USB_VBUS` (Pin 68) -> `J1` Type-C | LQFP100 uses dedicated USB PHY pins (`PA24`/`PA25` are only USB on QFN48/LQFP64; on HPM5300EVK `PA24` is `LIN 12V_EN`). 480 Mbps HS (`wMaxPacketSize = 512`, FS fallback `64`), `bInterfaceClass = 0x00`, 2 interfaces (CAN-FD `0x81..0x03`, LIN `0x84..0x06`) |
| **CAN-FD (`HPM_MCAN3`)** | `PY04` (`CAN3_RXD`), `PY05` (`CAN3_TXD`) -> `U10` (`TJA1042T/3`) -> `J9` (`Pin 1: CANL`, `Pin 3: CANH`, `Pin 2/4: GND`) | `TJA1042T/3` `STB` is tied to GND via `10 kOhm` (`R67`, active normal mode). Split `120 Ohm` termination onboard. Clocked at **`80.0 MHz`** (`clk_src_pll1_clk0`). |
| **LIN (`PLIN-USB`, `HPM_UART3`)** | `PA14` (`LIN3_RXD`), `PA15` (`LIN3_TXD`), `PA13` (`LIN3_SLP_N`), `PA24` (`12V_EN`) -> `U11` (`TJA1021T`) -> `J9` (`Pin 5: LIN1`) | Onboard `1 kOhm` Master pull-up (`R73`), wake push-button (`SW4`), and software-controlled `12V_EN` + `SLP_N`. Supports `PLIN_USB.sys` Master/Slave & scheduler. |
| **Status LED** | `PA23` (active-LOW: `0` = ON, `1` = OFF) | Slow/fast USB status blink + brief activity flicker on CAN/LIN RX/TX |
| **Timestamps** | RISC-V `mcycle` CSR | Monotonic `1 us` resolution derived from `clock_cpu0` |

## USB Identifiers Notice

Open-source development defaults (`0x0483` / `0x0011`, `"XCAN"` / `"XCAN-USB Pro FD"`) are isolated in [`app/pcan/pcan_usb_ids.h`](app/pcan/pcan_usb_ids.h).

## Building

### Option 1: CMake + RISC-V GCC (`hpm_sdk v1.13.0`)
Set `HPM_SDK_BASE` to your `hpm_sdk` v1.13.0 folder and ensure `riscv-none-elf-gcc` (or `riscv32-unknown-elf-gcc`) + Ninja are on `PATH`:
```bash
cmake --preset hpm5300evk-release
cmake --build --preset hpm5300evk-release --parallel
```
Outputs (`.elf`, `.hex`, `.bin`, `.map`) are written to `build/hpm5300evk-release/output/`.

### Option 2: HPM SDK Project Generator + Segger Embedded Studio (SES)
The root [`CMakeLists.txt`](CMakeLists.txt) invokes `generate_ide_projects()`, so you can also open the repository in **`HPM SDK Project Generator`**:
1. Set **SDK Boards** = `hpm5300evk`
2. Point **User Application Path** to this repo (`Enable User Application Path`), **Build Type** = `flash_xip`
3. Click **Generate**
4. Click **Open with IDE (SES)** to build and flash/debug in Segger Embedded Studio.
