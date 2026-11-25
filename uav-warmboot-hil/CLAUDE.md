# UAV Warm-Boot HIL: PC side

Simulator + Mission Control for the STM32F446RE warm-boot project. Firmware: `../uav-warm-boot-recovery` (its README is the protocol source of truth).
The sim flies a quadcopter in closed loop through the MCU, on the wall clock, so a reset has a physical cost.

## Run (Windows PowerShell)
```powershell
cd sim-engine;        pip install -r requirements.txt; python server.py     # mock MCU, ws://127.0.0.1:8000/ws
cd mission-dashboard; npm install; npm run dev                              # http://localhost:5173
$env:HIL_PORT="COM5"; python server.py                                      # real STM32 instead of the mock
```
Flight logs: `sim-engine/logs/flight_<timestamp>.csv` (one row per 5 ms step).

## Architecture rules
- **`hil_bridge.py` is the only file that knows bytes.** Version/type ids, field order, units, endianness, CRC-32, COBS framing, serial transport. Everything else uses the neutral dataclasses (`SensorFrame`, `ActuatorFrame`, `CommandFrame`) in SI floats.
- `plant.py` is pure physics, `metrics.py` is pure scalars, neither imports the bridge or the MCU. `mock_mcu.py` talks wire bytes only (`write/read/tick`), same surface as `SerialDevice`, and never imports `plant.py`.
- `server.py` only composes: plant -> `HilBridge` -> device -> plant. No physics or protocol details in it.
- Constants live in `config.py`. No magic numbers elsewhere.

## Conventions
- World **NED** (x north, y east, z down; altitude = -z, gravity = +z). Body **FRD** (x forward, y right, z down).
- Quaternion `[w, x, y, z]` rotating body -> NED. Rates rad/s, accel is specific force (reads -g at rest), mag in uT (world field 22 N / 44 D), angles in degrees only at the UI/CSV edge. g = 9.80665.
- Motor order FR, RL, FL, RR (PX4 quad-X), commands normalised 0..1, ESC lag tau = 0.04 s.
- three.js mapping (dashboard only): X = east, Y = up, Z = -north; quaternion vector part (x,y,z)_ned -> (y, -z, -x).
- **dt = 0.005 s (200 Hz), fixed.** The physics loop spins on `time.perf_counter()` (no `sleep`) and replays missed steps, so sim time tracks wall time. Only the 60 Hz broadcaster sleeps.
- The ESC holds the last valid command while no `ctrl_active` frame arrives. A gust torque (`GUST_*` in config) is what the held command can't cancel.

## Metrics (metrics.py), all from fault instant t0
Downtime (first frame back), Time-to-Control (first `ctrl_active` frame; cold boot includes the 2.5 s gyro cal), Max Tilt, Settling Time (tilt < 2 deg for 0.5 s after control resumes), Attitude Drift (worst truth-vs-MCU-estimate angle over frames since t0).

## Firmware reconciliation status (vs uav-warm-boot-recovery @ 9ee6b3c)
The firmware currently runs only a USART1 DMA echo loopback (`hil_link.c`); no packet structs exist in C yet.
Its README fixes **HIL sensor protocol v1**, which `hil_bridge.py` now implements exactly:
`struct.pack("<BBHIQ9f", 1, 1, 36, seq, t_us, gyro, accel, mag)` + `zlib.crc32` LE, COBS-encoded, `0x00` delimiter,
USART1 921600 8N1, seq from 0 continuing across MCU resets, timestamps in us since sim start.

Bench check against the loopback firmware: `python hil_bridge.py COM5` (sends 1000 v1 frames, verifies the echo).
While the board runs the loopback, `server.py` with `HIL_PORT` just sees no actuator frames (echoed sensor frames are ignored).

| Message | Status | Notes |
|---|---|---|
| Sensor (type 1) | Reconciled | Byte-exact with the README example. |
| Actuator (type 2) | Provisional | README: carries the source sample sequence; command fields TBD. We send `sample_seq, motors[4], est_quat[4], boot_path, flags`. |
| Command (type 3) | Provisional | README: faults come from a test-only task; trigger undefined. We send `fault, extra_down_ms`. |

Open questions for the firmware team:
- v1 length field says **36** but the payload after the header is 48 bytes (seq + time + 9 floats). We send 36 as written; confirm.
- Actuator frame contents, especially exporting the estimate quaternion (needed for Attitude Drift) and boot path / `ctrl_active`.
- What the simulator should do while no actuator frames arrive (we hold the last valid ESC command).
- How the fault-injection task is triggered; numbering of `BootPath` / `Fault`.

## Mock MCU follows the firmware's planned recovery
Two alternating checkpoint slots; a torn write corrupts only the slot being written, so recovery uses the older intact slot (normally WARM-FULL).
After a watchdog reset the mock waits for a fresh sample, then accepts the checkpoint only if
age <= `CHECKPOINT_MAX_AGE` and its tilt agrees with the accelerometer within `ATTITUDE_REJECT_DEG`.
Otherwise WARM-PARTIAL: keep gyro bias (if under `BIAS_LIMIT`), seed attitude from accel + mag. No intact slot or bad bias -> COLD.
So long outages (slider >= ~150 ms on top of the IWDG timeout) now show WARM-PARTIAL. Timings in `config.py` are placeholders until DWT measurements exist.

## When the C structs land
Edit **only `sim-engine/hil_bridge.py`**: `_FMT`, `_LEN`, `encode`/`decode`, `T_*` ids, `BootPath`/`Fault` values.
A new *semantic* field (one the sim must produce or consume) also needs a change where it is produced or used: `plant.imu`, `mock_mcu`, or `server.step`.
