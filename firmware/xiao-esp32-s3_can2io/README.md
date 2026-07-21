# xiao_esp32_s3_smd_serial_bridge

## 1. Overview

This firmware targets a XIAO ESP32-S3 based board (with an MCP2561 CAN transceiver) used as either:

- a **standalone serial<->GPIO bridge** (`MODE_IO`), or
- one **node** on a CAN bus (`MODE_CAN`), or
- the **host** that bridges a PC serial link to up to 3 other CAN nodes while also acting as node 0 itself (`MODE_CAN_HOST`), or
- a **read-only CAN sniffer** for bring-up/debugging (`MODE_CAN_MONITOR`), or
- a **dedicated DJI RoboMaster (M3508/M2006/GM6020) driver** for up to 4 motors on its own CAN bus (`MODE_ROBOMAS`, see section 9).

Each board exposes the same local I/O set (no DC motor driver on this board):

- 3x shared MULTI ports, each configurable per-port as either a digital switch input or a servo PWM output (`MULTI1..3` in `config.hpp`)
- 2x quadrature encoder inputs (ENC1, ENC2)

The CAN transport reuses the existing 24-slot int16 serial payload (`Tx_16Data` / `Rx_16Data`) as the common data model; it does not introduce a separate protocol.

---

## 2. Transport Modes

Select exactly one mode in `src/config.hpp`:

- `MODE_IO`: local GPIO/servo/encoder/switch handling over serial only. No CAN.
- `MODE_CAN`: CAN node mode. This board acts as one node on the CAN bus, driven entirely by CAN frames from the host (no serial link to a PC).
- `MODE_CAN_HOST`: CAN host mode. This board owns the PC serial link, relays data to/from up to 3 other CAN nodes, and additionally drives its own local I/O directly (see section 4).
- `MODE_CAN_MONITOR`: passive CAN sniffer. Starts the CAN driver and `canTask` only; no serial bridging and no IO task. Prints one summary line per node to `Serial` whenever all of that node's slots have been observed, for wiring/bring-up checks.
- `MODE_DEBUG`: development/debug mode (PID task).
- `MODE_ROBOMAS`: dedicated DJI RoboMaster driver. Does **not** use `canInit()`/`canTask()` or the node/slot protocol at all — it runs its own CAN bus at 1Mbps speaking DJI's native protocol directly. See section 9.

`main.cpp` enforces that exactly one of `MODE_IO`, `MODE_CAN`, `MODE_CAN_HOST`, `MODE_DEBUG`, `MODE_CAN_MONITOR`, `MODE_ROBOMAS` is defined; the build fails otherwise.

---

## 3. CAN Node Addressing

Each board's CAN node index is derived automatically from `CAN_ID`, not set as a separate constant:

```cpp
// CAN_ID is 3 digits: leading digit = bus number, last 2 digits = node number
#define CAN_ID 101
#define CAN_NODE_INDEX ((CAN_ID % 100U) - 1U)
```

So `CAN_ID = 101, 102, 103, 104` map to node index `0, 1, 2, 3`. The host is expected to use `CAN_ID = x01` (node 0); the three CAN node boards on the same bus use `x02`, `x03`, `x04`.

```cpp
#define CAN_NODE_COUNT 4      // max nodes on one bus (host + 3 nodes)
#define CAN_SLOTS_PER_NODE 5  // int16 slots owned by each node
```

Only set `CAN_ID` per board; `CAN_NODE_INDEX`, node addressing, and CAN frame IDs all follow from it.

---

## 4. CAN Slot Mapping

The 24-slot payload is divided into 4 node blocks of 5 slots each (20 of the 24 slots are used; the remaining 4 are unused headroom):

| Node | Slot range (of 24) | Board |
|:---|---:|:---|
| Node 0 | 0-4 | Host board itself |
| Node 1 | 5-9 | CAN node board 1 |
| Node 2 | 10-14 | CAN node board 2 |
| Node 3 | 15-19 | CAN node board 3 |

Each node's 5 slots are split into two dedicated I/O arrays (`src/frame_data.hpp`), not addressed directly by slot number:

**Command direction (host -> node), `CanIoRxData[5]`:**

| Index | Meaning |
|---:|:---|
| 0 | SERVO1 angle command (only used if `MULTI1 == 1`) |
| 1 | SERVO2 angle command (only used if `MULTI2 == 1`) |
| 2 | SERVO3 angle command (only used if `MULTI3 == 1`) |
| 3-4 | unused / reserved (no motor driver on this board) |

**Feedback direction (node -> host), `CanIoTxData[5]`:**

| Index | Meaning |
|---:|:---|
| 0-2 | SW1-3 switch state (`0` if the corresponding `MULTIx` port is configured as a servo) |
| 3-4 | ENC1-2 raw pulse counter value |

Each 5-slot block is transmitted as two CAN frames (`identifier = 0x100 + node_index*16 + chunk`): chunk 0 carries 4 int16 values, chunk 1 carries the remaining 1 value. `twai_message_t.data` holds each value big-endian.

---

## 5. How Data Flows

### Host mode (`MODE_CAN_HOST`)

1. `serialTask` decodes the 24-slot command payload from the PC into `Rx_16Data`.
2. `canTask` snapshots `Rx_16Data`, applies node 0's slot range directly to the host's own `CanIoRxData` (no CAN round-trip for its own outputs), and sends the other 3 node blocks out over CAN.
3. `canTask` drains CAN feedback frames from the 3 external nodes into a persistent buffer (slots are only overwritten when new frames arrive, so a node's last known value is retained until it reports again), and fills node 0's own slot range directly from the host's local `CanIoTxData` (its own switches/encoders).
4. That merged 24-slot buffer is published to `Tx_16Data` every host loop iteration.
5. `serialTask` sends `Tx_16Data` back to the PC every `CAN_TX_PERIOD_MS` (5 ms).
6. `IO_Task` runs locally on the host exactly as it would on a node, driving SERVO1-3 outputs from `CanIoRxData` and reading switch/encoder state into `CanIoTxData`.

### Node mode (`MODE_CAN`)

1. `canTask` receives only CAN frames addressed to `CAN_NODE_INDEX` and applies them to local `CanIoRxData`.
2. `IO_Task` drives SERVO1-3 outputs from `CanIoRxData` (for ports configured as servo via `MULTIx`) and writes SW1-3 / ENC1-2 into `CanIoTxData`.
3. Every `CAN_TX_PERIOD_MS` (5 ms), `CanIoTxData` is packed into this node's slot block and sent back to the host over CAN.

### CAN monitor mode (`MODE_CAN_MONITOR`)

1. `canTask` only receives CAN frames (no serial task, no IO task, no transmit).
2. Frame values are unpacked into a persistent per-node slot buffer, same layout as above.
3. Once every slot for a node has been seen at least once, a summary line (`SW1/SW2/SW3/ENC1/ENC2`) is printed to `Serial` for that node.

---

## 6. Configuration Workflow

1. Open `src/config.hpp`.
2. Set `DEVICE_ID` (serial frame ID, must match the PC-side config for this board).
3. Set `CAN_ID` (3-digit: bus digit + node number, e.g. `101`..`104`). This also determines `CAN_NODE_INDEX`.
4. Choose exactly one mode macro (`MODE_IO`, `MODE_CAN`, `MODE_CAN_HOST`, `MODE_CAN_MONITOR`, or `MODE_DEBUG`).
5. Set `MULTI1`/`MULTI2`/`MULTI3` per board (`0` = switch input, `1` = servo output) to match the wiring.
6. Adjust PWM, servo range, and pin settings if required.
7. Build and flash with PlatformIO.

---

## 7. Notes / Known Limitations

- The CAN transport intentionally reuses the serial slot model rather than a fully separate protocol.
- This board has no DC motor driver. Each node has exactly 2 encoder channels (ENC1, ENC2) and 3 MULTI ports (SW1-3 / SERVO1-3, pin-shared per port via `MULTI1..3`), all reachable over CAN.
- `MODE_CAN_MONITOR` is read-only and does not drive any outputs; use it to verify wiring/IDs before switching a board to `MODE_CAN` or `MODE_CAN_HOST`.

---

## 9. RoboMaster Driver Mode (`MODE_ROBOMAS`)

Unlike every other mode above, `MODE_ROBOMAS` does not participate in the node/slot
CAN protocol at all. DJI's C620 (M3508), C610 (M2006) and GM6020 controllers speak a
fixed protocol at a fixed **1Mbps** bitrate with fixed CAN IDs that cannot be changed
in firmware — this is incompatible with the 500kbps node/slot bus used by
`MODE_CAN`/`MODE_CAN_HOST`/`MODE_CAN_MONITOR`. A board in `MODE_ROBOMAS` therefore acts
as a **standalone device**: its own USB-serial link straight to the PC (own
`DEVICE_ID`), and its own dedicated CAN bus with up to `NUM_MOTOR` (4) RoboMaster
motors of a **single model** (mixing M3508/M2006/GM6020 on the same bus is not
supported). Do not put any other `ros2can` node board on this same physical CAN bus.

Select the motor model at compile time in `src/config.hpp`:

```cpp
#define ROBOMAS_MOTOR_TYPE ROBOMAS_MOTOR_M3508 // or ROBOMAS_MOTOR_M2006 / ROBOMAS_MOTOR_GM6020
```

Only velocity control is implemented. Velocity PID gains (`ROBOMAS_KP_VEL` /
`ROBOMAS_KI_VEL` / `ROBOMAS_KD_VEL`) are fixed compile-time constants in `config.hpp`;
they cannot be changed from `ros2can`/the PC side at runtime — tune them in firmware
and reflash.

Slot mapping reuses the standalone 24-slot `Tx_16Data`/`Rx_16Data` frame directly (no
node/slot chunking, since this board is not a node on the host's bus):

**Command (PC -> board), `Rx_16Data`:**

| Index | Meaning |
|---:|:---|
| 0-3 | target velocity for motor 1-4, raw rpm (output-shaft rpm), no scaling |
| 4-23 | unused |

**Feedback (board -> PC), `Tx_16Data`:**

| Index | Meaning |
|---:|:---|
| 0-3 | angle for motor 1-4, output-shaft degrees, scale 0.1 deg/LSB |
| 4-7 | velocity for motor 1-4, output-shaft rpm, no scaling |
| 8-11 | current for motor 1-4, milliamps, scale 0.001 A/LSB |
| 12-23 | unused |

CAN IDs used on the dedicated 1Mbps bus (all fixed by DJI, not configurable):

| Direction | M3508 / M2006 | GM6020 |
|:---|:---|:---|
| Command (group, IDs 1-4) | `0x200` | `0x1FE` |
| Feedback (per motor, ID n) | `0x200 + n` (`0x201`-`0x204`) | `0x204 + n` (`0x205`-`0x208`) |

---

## 10. Credits

Developed by NHK Project, RRST, Ritsumeikan University, Japan.
- Official Website: https://www.rrst.jp
- X (Twitter): https://x.com/RRST_BKC
