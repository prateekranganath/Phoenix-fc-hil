"""THE ONLY FILE TO EDIT when the firmware's packet format changes.

Translates between neutral Python frames (SI floats, numpy arrays) and UART bytes.
Everything wire-specific lives here: version/type ids, field order, units, endianness,
CRC, framing, and the serial transport. Nothing else in sim-engine knows what a byte looks like.

Source of truth: uav-warm-boot-recovery/README.md, "HIL sensor protocol v1".
    decoded frame = header <BBH> (version=1, type, length) + payload + CRC-32 LE (zlib.crc32 over header+payload)
    wire          = COBS(decoded frame) + b"\\x00"     (the delimiter is how the MCU resyncs after a reset)

Status per message type:
    SENSOR   (laptop -> MCU)  RECONCILED with firmware README v1.
    ACTUATOR (MCU -> laptop)  PROVISIONAL: firmware README only says it carries the source sample sequence;
                              command fields are an open decision there. Same v1 framing assumed.
    COMMAND  (laptop -> MCU)  PROVISIONAL: firmware injects faults from a test-only task; how that task is
                              triggered is not defined yet. Same v1 framing assumed.
"""
import struct
import zlib
from dataclasses import dataclass, field
from enum import IntEnum

import numpy as np
import config as C

# --- neutral frames: the contract the rest of sim-engine codes against ------------------
class BootPath(IntEnum):
    NONE = 0           # PROVISIONAL numbering
    COLD = 1
    WARM_FULL = 2      # newest intact checkpoint accepted
    WARM_PARTIAL = 3   # checkpoint attitude rejected; re-initialised from accel + mag ("warm fallback")


class Fault(IntEnum):  # PROVISIONAL numbering
    IWDG_HANG = 1
    WWDG_OVERRUN = 2
    TORN_WRITE = 3
    COLD_BOOT = 4


@dataclass
class SensorFrame:          # laptop -> MCU
    seq: int                # starts at 0, +1 per sample, continues across MCU resets
    t_us: int               # microseconds since simulation start
    gyro: np.ndarray        # rad/s, body FRD
    accel: np.ndarray       # m/s^2 specific force, body FRD (reads -g on z at rest)
    mag: np.ndarray         # uT, body FRD


@dataclass
class ActuatorFrame:        # MCU -> laptop
    sample_seq: int         # seq of the SensorFrame this output was computed from (firmware README)
    motors: np.ndarray      # 4 x normalised 0..1, order FR, RL, FL, RR
    est_quat: np.ndarray    # MCU attitude estimate [w,x,y,z] (needed for the Attitude Drift metric)
    boot_path: BootPath
    ctrl_active: bool       # False while MCU is alive but not yet flying (cal / re-init)


@dataclass
class CommandFrame:         # laptop -> MCU, test-only fault trigger
    fault: Fault
    extra_down_ms: int = 0


# --- wire formats ---------------------------------------------------------------------
VERSION = 1
T_SENSOR, T_ACTUATOR, T_COMMAND = 1, 2, 3

_FMT = {T_SENSOR: "<IQ9f",       # seq, t_us, gyro[3], accel[3], mag[3]          (v1, 48 bytes)
        T_ACTUATOR: "<I4f4fBB",  # sample_seq, motors[4], est_quat[4], boot_path, flags(bit0=ctrl_active)
        T_COMMAND: "<BH"}        # fault id, extra_down_ms
# Value of the header length field. v1 states 36 for the sensor frame, i.e. it counts only the
# nine float readings, not seq/time. FLAG TO FIRMWARE: confirm 36 vs 48. Provisional types use full size.
_LEN = {T_SENSOR: 36, T_ACTUATOR: struct.calcsize(_FMT[T_ACTUATOR]), T_COMMAND: struct.calcsize(_FMT[T_COMMAND])}


def cobs_encode(data):
    out, block = bytearray(), bytearray()
    for b in data:
        if b == 0:
            out += bytes([len(block) + 1]) + block
            block.clear()
        else:
            block.append(b)
            if len(block) == 254:
                out += b"\xff" + block
                block.clear()
    return bytes(out + bytes([len(block) + 1]) + block)


def cobs_decode(data):
    out, i = bytearray(), 0
    while i < len(data):
        code = data[i]
        if code == 0 or i + code > len(data):
            raise ValueError("bad COBS")
        out += data[i + 1:i + code]
        i += code
        if code < 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


def encode(f):
    """Frame dataclass -> wire bytes (COBS + delimiter)."""
    if isinstance(f, SensorFrame):
        t, body = T_SENSOR, struct.pack(_FMT[T_SENSOR], f.seq, f.t_us, *f.gyro, *f.accel, *f.mag)
    elif isinstance(f, ActuatorFrame):
        t, body = T_ACTUATOR, struct.pack(_FMT[T_ACTUATOR], f.sample_seq, *f.motors, *f.est_quat,
                                          int(f.boot_path), int(f.ctrl_active))
    else:
        t, body = T_COMMAND, struct.pack(_FMT[T_COMMAND], int(f.fault), f.extra_down_ms)
    frame = struct.pack("<BBH", VERSION, t, _LEN[t]) + body
    return cobs_encode(frame + struct.pack("<I", zlib.crc32(frame))) + b"\x00"


def decode(frame):
    """Decoded (un-COBSed) bytes -> frame dataclass, or None if version/type/length/CRC fail."""
    if len(frame) < 8:
        return None
    ver, t, n = struct.unpack_from("<BBH", frame)
    if ver != VERSION or t not in _FMT or n != _LEN[t] or len(frame) != 4 + struct.calcsize(_FMT[t]) + 4:
        return None
    (crc,) = struct.unpack_from("<I", frame, len(frame) - 4)
    if zlib.crc32(frame[:-4]) != crc:
        return None
    v = struct.unpack_from(_FMT[t], frame, 4)
    if t == T_SENSOR:
        return SensorFrame(v[0], v[1], np.array(v[2:5]), np.array(v[5:8]), np.array(v[8:11]))
    if t == T_ACTUATOR:
        return ActuatorFrame(v[0], np.array(v[1:5]), np.array(v[5:9]), BootPath(v[9]), bool(v[10] & 1))
    return CommandFrame(Fault(v[0]), v[1])


@dataclass
class FrameParser:
    """Byte-stream -> frames, split on 0x00. A partial or corrupt frame (e.g. cut by an MCU reset)
    fails COBS/CRC and is dropped; the next delimiter restores alignment."""
    buf: bytearray = field(default_factory=bytearray)
    errors: int = 0

    def feed(self, data):
        self.buf += data
        *chunks, self.buf = self.buf.split(b"\x00")
        out = []
        for c in chunks:
            if not c:
                continue
            try:
                f = decode(cobs_decode(bytes(c)))
            except (ValueError, struct.error):
                f = None
            if f is None:
                self.errors += 1
            else:
                out.append(f)
        return out


# --- transports -------------------------------------------------------------------------
class SerialDevice:
    """Real STM32 on USART1 (921600 8N1, no flow control). Same write/read/tick surface as MockMCU."""
    def __init__(self, port, baud=C.BAUD):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0)

    def write(self, data): self.ser.write(data)
    def read(self): return self.ser.read(self.ser.in_waiting)
    def tick(self, t): pass


class HilBridge:
    """What server.py talks to. `device` is a MockMCU or a SerialDevice."""
    def __init__(self, device):
        self.device, self.parser, self._seq = device, FrameParser(), 0

    def send_sensors(self, t, gyro, accel, mag):
        self.device.write(encode(SensorFrame(self._seq, int(round(t * 1e6)), gyro, accel, mag)))
        self._seq += 1

    def send_fault(self, fault, extra_down_ms=0):
        self.device.write(encode(CommandFrame(fault, extra_down_ms)))

    def poll(self, t):
        """Let the device run up to sim time `t`, return newly received ActuatorFrames.
        Other frame types are ignored (e.g. our own sensor frames echoed by the loopback firmware)."""
        self.device.tick(t)
        return [f for f in self.parser.feed(self.device.read()) if isinstance(f, ActuatorFrame)]


if __name__ == "__main__":
    # Bench check against the current loopback firmware:  python hil_bridge.py COM5
    # Sends 1000 v1 sensor frames and verifies they come back intact (exercises DMA ring wraps).
    import sys, time
    dev, parser, sent, got = SerialDevice(sys.argv[1]), FrameParser(), [], []
    for i in range(1000):
        f = SensorFrame(i, i * 5000, np.full(3, 0.01 * i), np.array([0, 0, -C.G]), np.array([22.0, 0, 44.0]))
        dev.write(encode(f)); sent.append(f.seq)
        got += [g.seq for g in parser.feed(dev.read()) if isinstance(g, SensorFrame)]
        time.sleep(C.DT)
    end = time.perf_counter() + 0.5
    while time.perf_counter() < end:
        got += [g.seq for g in parser.feed(dev.read()) if isinstance(g, SensorFrame)]
    print(f"sent {len(sent)}  echoed {len(got)}  in-order {got == sent[:len(got)]}  bad frames {parser.errors}")
