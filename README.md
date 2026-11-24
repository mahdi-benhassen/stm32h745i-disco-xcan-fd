# XCAN-FD Firmware — Artery AT-START-F457 (`AT32F457ZET7`) Port

This branch (`at-start-f457`) ports the PCAN-USB Pro FD compatible firmware (`PCAN_PRO_FD=1`, `INCLUDE_LIN_INTERFACE=1`) to the **Artery `AT-START-F457`** evaluation board (`AT32F457ZET7`, Cortex-M4F, 512 KB Flash / 144 KB SRAM, single CAN-FD channel).

> Looking for the original dual-core STM32H745I-DISCO firmware? Switch to `main` (CMake/GCC) or `keil-mdk` (STM32CubeMX Keil MDK-ARM v5).

## Hardware & Clock Architecture

| Resource | Mapping / Configuration | Notes |
|---|---|---|
| **MCU** | `AT32F457ZET7` | 8 MHz HEXT crystal |
| **System Clock** | `160 MHz` (`PLL VCO = 960 MHz / 6`) | AHB = 160 MHz, APB1 = 160 MHz, APB2 = 160 MHz, APB3 = 80 MHz |
| **USB Clock** | `48.0 MHz` exact (`PLLU = 960 MHz / 20`) | Exact crystal-locked clock for OTGFS1 |
| **USB Device** | `OTGFS1` Full-Speed (`PA11` DM / `PA12` DP) | `USB_VBUS_IGNORE` enabled for Type-C device connector; `bInterfaceClass = 0x00` (Interface 0 CAN-FD `0x81..0x03`, Interface 1 LIN placeholder `0x84..0x06`) |
| **CAN-FD (`CAN1`)** | `PD0` (`CAN1_RX`, AF9), `PD1` (`CAN1_TX`, AF9), **`PB7` (`CAN_STB` LOW)** | `CAN1` clocked from `PCLK = 160 MHz` (`2 * 80 MHz` PCAN reference clock). **`PB7` is driven LOW** to keep the onboard CAN-FD transceiver active in normal mode. |
| **Timestamps** | `TMR2` (32-bit, `1 MHz` / `1 us` resolution) | Plus `1 ms` `SysTick` timebase for LEDs & bus-load timers |
| **Status LEDs** | `LED2` (`PD13`) USB Status, `LED3` (`PD14`) CAN1 RX, `LED4` (`PD15`) CAN1 TX | Active-LOW board LEDs |

## USB Identifiers Notice

Open-source development defaults (`0x0483` / `0x0011`, `"XCAN"` / `"XCAN-USB Pro FD"`) are isolated in [`app/pcan/pcan_usb_ids.h`](app/pcan/pcan_usb_ids.h).

## Building

### Option 1: Keil MDK-ARM v5 (uVision, Arm Compiler 6)
Open [`mdk_v5/at_start_f457_pcan_fd.uvprojx`](mdk_v5/at_start_f457_pcan_fd.uvprojx) in Keil uVision 5 (`ArteryTek.AT32F45x_DFP` 2.0.1+) and press **F7** to build `mdk_v5/objects/at_start_f457_pcan_fd.hex` (C11, Arm Compiler 6).

### Option 2: CMake + ARM GCC + Ninja
```bash
cmake --preset Release
cmake --build --preset Release --parallel
```
Outputs are written to `build/Release/AT_START_F457_PCAN_FD.{elf,hex,bin,map}`. Pre-built GitHub Actions artifacts (`.elf`, `.hex`, `.bin`, `.map`, `SHA256SUMS`) are also attached to every CI run.
