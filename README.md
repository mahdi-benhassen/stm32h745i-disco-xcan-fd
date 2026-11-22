# stm32h745i-disco-xcan-fd

Dual-channel USB-to-CAN-FD adapter firmware for the **STM32H745I-DISCO** board, porting [moonglow/pcan_pro_x](https://github.com/moonglow/pcan_pro_x) (`PCAN_PRO_FD=1`, `INCLUDE_LIN_INTERFACE=1`) to STM32H745 dual-core (`CM7` + `CM4`) with hardware `FDCAN1`/`FDCAN2`.

## Hardware Mapping (STM32H745I-DISCO / MB1381)

| Subsystem | Assignment | Notes |
|---|---|---|
| **Power & Clock** | `PWR_DIRECT_SMPS_SUPPLY`, `VOS1`, `25 MHz` HSE | `CM7` = `400 MHz`, `HCLK/CM4/TIM2` = `200 MHz`, `PLL1Q` (`FDCAN`) = `80.0 MHz`, `PLL3Q` (`USB`) = `48.0 MHz` |
| **FDCAN1 (Channel 0)** | `PH13` (TX), `PH14` (RX) | Onboard 5 Mbps transceiver, 1280-word Message RAM offset `0` |
| **FDCAN2 (Channel 1)** | `PB13` (TX), `PB5` (RX) | Onboard 5 Mbps transceiver, 1280-word Message RAM offset `1280` |
| **USB OTG FS** | `CN13` (`PA11` DM, `PA12` DP) | Full-Speed `64 B` bulk composite descriptor, polled via `pcan_usb_device_poll()` |
| **Timestamp** | `TIM2` (32-bit, `PSC = 200 - 1`) | Free-running `1.0 µs` tick extended to 64-bit timestamps |
| **LEDs (Active-LOW)** | `LED1` (`PJ2`, Green) / `LED2` (`PI13`, Red) | `LED1` = device status, `LED2` = dual-channel CAN RX/TX activity |
| **Cortex-M4** | `CM4` idle loop (`__WFI()`) | Required for `DUAL_CORE_BOOT_SYNC_SEQUENCE` (HSEM wakeup) |

## Building (CMake + Ninja + ARM GCC)

Configure and build both cores (`CM4` and `CM7`):

```bash
cmake --preset Release -S CM4
cmake --build --preset Release -S CM4 --parallel

cmake --preset Release -S CM7
cmake --build --preset Release -S CM7 --parallel
```

Outputs (`.elf`, `.hex`, `.bin`, `.map`) are generated in `CM4/build/Release/` and `CM7/build/Release/`.

## Flashing (Dual-Core)

Both binaries **must** be programmed so `CM7` and `CM4` complete dual-core HSEM boot synchronization:

* **CM7 (`PCAN_STM32H745I_CM7.bin`):** Flash Bank 1 `0x08000000`
* **CM4 (`PCAN_STM32H745I_CM4.bin`):** Flash Bank 2 `0x08100000`

## USB Identity Configuration

This repository ships with open-source default USB identifiers (`VID 0x0483`, `PID 0x0011`, manufacturer `"XCAN"`, product `"XCAN-USB Pro FD"`) isolated in `CM7/App/pcan/pcan_usb_ids.h`. You can customize `PCAN_USB_VID`, `PCAN_USB_PID`, and the descriptor strings locally via `CM7/App/pcan/pcan_usb_ids.h` or CMake `-D` compiler definitions without editing driver sources.

## Attribution

Based on the protocol and firmware architecture of [moonglow/pcan_pro_x](https://github.com/moonglow/pcan_pro_x), ported to STM32CubeH7 HAL `FDCAN` and USB Device Library v2.11 for STM32H745I-DISCO.
