# stm32h745i-disco-xcan-fd (`can-node-uds-isotp` branch)

Production port of [`leoembeddeder/can-node`](https://github.com/leoembeddeder/can-node) (**ISO 15765-2:2016** CAN-FD ISO-TP + **ISO 14229-1:2020** UDS server) onto **STM32H745I-DISCO** (`CM7 @ 400 MHz` + `CM4` HSEM companion) for hardware verification with **EcuBus + PCAN**.

---

## Architecture Highlights

| Subsystem | Implementation | Purpose |
|---|---|---|
| **ISO-TP (`lib/isotp`)** | Vendored from `can-node` (merged PR `#2`, `73f0380` / `6601829`) | Full CAN-FD (`CAN_DL = 8..64`), Table 12/13 SF escape PCI (`1..62 B`), Table 14 12-bit & 32-bit escape `FF_DL` (`> 4095 B`), `TX_DL - 1` CF chunking, microsecond `STmin` (`0xF1..0xF9` = `100..900 µs`), `N_As`/`N_Ar`/`N_Bs`/`N_Cr` |
| **UDS Server (`lib/uds`)** | Pure C11, static-allocation, zero OS/malloc | Sessions (`0x10`: `01`/`02`/`03`), `ECUReset` (`0x11 01`), `ClearDTC` (`0x14`), `ReadDTC` (`0x19`: `01`/`02`/`0A`), `ReadDID` (`0x22`: `0xF186`/`0xF189`/`0xFD00`), `RoutineControl` (`0x31`: `FF00`/`FF01`/`F000`), Download (`0x34`/`0x36`/`0x37`), `TesterPresent` (`0x3E`), `NRC 0x78` `ResponsePending`, `S3server` (`5000 ms`) |
| **1 µs Hardware Clock** | `TIM2` (`32-bit`, `PSC = 200 - 1` on `200 MHz` APB1) + `TIM2_IRQHandler` | Race-free, monotonic 64-bit microsecond timestamp (`can_node_time_us64()`) with `UIF` carry handling and Cortex-M7 `__DSB()` |
| **FDCAN Hardware Port** | STM32H7 HAL FDCAN + `TX Event FIFO` (`16` entries) | True wire TX confirmations (`FDCAN_STORE_TX_EVENTS`) via `MessageMarker` -> `isotp_on_tx_done()`, `80 MHz PLL1Q` `500k / 2M` timings, ISO CAN-FD mode, TDC (`TDCO = 30`), hardware acceptance filters, non-blocking Bus-Off recovery |
| **Memory Placement** | `DTCMRAM` (`128 KB`) + `RAM_D1` AXI-SRAM (`512 KB @ 0x24000000`) | `8192 B` ISO-TP `rx_buffer` + `64 KB` virtual download slot placed in `.ram_d1_bss`, leaving `DTCMRAM` free for stacks and PCAN USB buffers |
| **Non-Blocking Telemetry** | `USART3` ST-LINK VCP (`115200 8N1`), `1024 B` TX ring | Pumps max 1 byte/poll only on idle ticks so `STmin = 100 µs` (`0xF1`) is never delayed |

---

## Two Hardware Topologies on One Branch

| Topology | Preset | FDCAN Allocation | Bench Setup |
|---|---|---|---|
| **Option A (Default)** | `Release` / `Debug` | `FDCAN1` (`PH13`/`PH14`) runs the `can-node` UDS/ISO-TP ECU (`LED1` = heartbeat, `LED2` = CAN RX/TX) | Connect an external **PCAN-USB FD** adapter to the STM32H745I-DISCO `FDCAN1` connector (+ 120 Ω termination) and drive tests from EcuBus |
| **Option B (Single-Board)** | `SingleBoard-Release` / `SingleBoard-Debug` | `FDCAN1` runs **PCAN-USB Pro FD** USB bridge; `FDCAN2` (`PB13`/`PB5`) runs the `can-node` UDS/ISO-TP ECU | Connect a short 2-wire CAN cable between `FDCAN1` and `FDCAN2` (+ 120 Ω termination) on one STM32H745I-DISCO board; connect `CN13` (`USB_FS`) to host PC running EcuBus |

### Configurability (`UDS_REQUEST_MAX` & `rx_buffer`)

- Default configuration sets `CAN_NODE_UDS_BLOCK_DATA_MAX = 4096` (`UDS_REQUEST_MAX = 4098 B` in `0x36 TransferData`) and `CAN_NODE_ISOTP_RX_BUF_SIZE = 8192 B`. Because `4098 > 4095`, `0x36` uploads natively trigger **ISO 15765-2:2016 32-bit Escape FirstFrames (`FF_DL > 4095`)** end-to-end!
- For upstream `can-node` parity (`UDS_REQUEST_MAX = 1026 B`), configure with `-DCAN_NODE_UDS_BLOCK_DATA_MAX=1024` (retaining `8192 B` `rx_buffer`).

---

## Build & Flash (`CMake + Ninja + ARM GCC`)

```bash
# 1. Build CM4 companion core (always required for dual-core HSEM boot sync)
cmake --preset Release -S CM4
cmake --build CM4/build/Release --parallel

# 2A. Option A — Standalone UDS/ISO-TP ECU on FDCAN1
cmake --preset Release -S CM7
cmake --build CM7/build/Release --parallel

# 2B. Option B — Single-Board PCAN (FDCAN1) + UDS ECU (FDCAN2)
cmake --preset SingleBoard-Release -S CM7
cmake --build CM7/build/SingleBoard-Release --parallel
```

Flash **both** cores:
- **CM7:** `CM7/build/<preset>/PCAN_STM32H745I_CM7.bin` -> `0x08000000`
- **CM4:** `CM4/build/Release/PCAN_STM32H745I_CM4.bin` -> `0x08100000`

---

## EcuBus + PCAN Verification Matrix

Configure EcuBus / PCAN: **ISO CAN-FD**, `500 kbps` Nominal / `2.0 Mbps` Data, **BRS ON**, Physical TX `0x7E0`, RX `0x7E8`, padding `0xCC`.

| Test Case | EcuBus Request Payload | Expected Response / ISO-TP Frame Behavior |
|---|---|---|
| **Extended Session (`SF 2B`)** | `10 03` | `50 03 00 32 01 F4` (`CAN_DL = 8` classical-length SF on FD, `P2 = 50 ms`, `P2* = 5000 ms`) |
| **Read DID `0xF189` (SW Version)** | `22 F1 89` | `62 F1 89 31 2E 32 2E 33` (`8 B` -> `CAN_DL = 12` FD escape SF `00 08 ...`) |
| **Read DID `0xFD00` (Boot Info)** | `22 FD 00` | `62 FD 00 ...` (`15 B` -> `CAN_DL = 20` FD escape SF `00 0F ...`) |
| **ReadDTC Confirmed (`0x19 02`)** | `19 02 09` | `59 02 FF ...` (6 DTCs = `27 B` -> `CAN_DL = 32` FD escape SF) |
| **ReadDTC Pending (`0x19 02`)** | `19 02 04` | `59 02 FF ...` (10 DTCs = `43 B` -> `CAN_DL = 48` FD escape SF) |
| **ReadDTC Supported (`0x19 0A`)** | `19 0A` | `59 0A FF ...` (32 DTCs = `131 B` -> **Multi-frame CAN-FD `FF 64B + CF 64B + CF 8B`**) |
| **Erase + `NRC 0x78` Pending** | `10 03` -> `10 02` -> `31 01 FF 00 44 00 01 00 00 00 00 20 00` | `7F 31 78` (`ResponsePending` at `40 ms`), then `71 01 FF 00 00 00` (`60 ms`) |
| **Download + 32-bit `FF_DL` (`4098 B`)** | `34 00 44 00 01 00 00 00 00 20 00` -> `36 01 <4096 B>` -> `36 02 <4096 B>` -> `37` | `74 20 10 02` (`maxBlockLen = 4098`), `36` uses **32-bit Escape FF (`10 00 00 00 10 02 ...`)**, CRC-32 verified in `31 01 FF 01` |
| **Sub-ms `STmin` Stress (`0xF1..0xF9`)** | Configure EcuBus FC `STmin = 0xF1` (`100 µs`), request `19 0A` or `36` | Verify `CF` wire spacing `>= 100 µs` with zero missed TX confirmations |
| **`ECUReset` (`11 01`)** | `11 01` | `51 01` transmitted & TX-confirmed before `NVIC_SystemReset()` triggers |

---

## UART Telemetry (`USART3`, ST-LINK VCP @ `115200 8N1`)

Prints startup banner plus idle-time summary every 2 seconds (never during active ISO-TP transfers):

```text
[can-node] sess=0x01 loopMax=12us rxF=42 txEvt=18 maxRx=4098 req=12 rsp=12 pend=1 tec=0 rec=0
```
