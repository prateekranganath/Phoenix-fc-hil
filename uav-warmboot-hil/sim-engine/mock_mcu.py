"""Software twin of the STM32: Mahony attitude estimator + cascaded PID + boot/fault state machine.

Speaks only wire bytes (write/read/tick), exactly like a SerialDevice, so server.py cannot tell it
from real hardware. It decodes/encodes through hil_bridge and never imports plant.py.

Follows the firmware README's planned recovery design (uav-warm-boot-recovery/README.md):
- Checkpoint written late in each control iteration, alternating between TWO slots, each with its own CRC.
  A torn write only corrupts the slot being written; the other slot is one iteration older and intact.
- Warm start (IWDG / WWDG): select the newest intact checkpoint, bring up USART, wait for a FRESH sample,
  then accept the saved attitude only if the checkpoint is young enough and its tilt agrees with the
  accelerometer. Otherwise fall back to re-initialising attitude from accel + mag, reusing the saved
  gyro bias only if it passes its own check. No intact checkpoint or bad bias -> cold path.

State machine (sim time):
    RUN --fault--> HUNG --(watchdog delay)--> BOOT --> DECIDE (warm only) --> ALIGN? --> RUN
    HUNG/BOOT/DECIDE: silent.  ALIGN: alive (frames out) but ctrl_active=0.

Boot paths reported to the laptop:
    COLD          zeroed state; ALIGN = gyro-bias calibration (GYRO_CAL s), then seed from accel + mag
    WARM_FULL     checkpoint accepted: quaternion + bias restored, control on the first fresh sample
    WARM_PARTIAL  checkpoint attitude rejected: bias kept, attitude seeded from accel + mag, ALIGN = REALIGN s
"""
import numpy as np
import config as C
from hil_bridge import (FrameParser, ActuatorFrame, SensorFrame, CommandFrame,
                        BootPath, Fault, encode)

RUN, HUNG, BOOT, DECIDE, ALIGN = "RUN", "HUNG", "BOOT", "DECIDE", "ALIGN"
_I4 = np.array([1.0, 0, 0, 0])
# fault -> time from the fault until the reset actually happens
_RESET_DELAY = {Fault.IWDG_HANG: C.IWDG_TIMEOUT, Fault.WWDG_OVERRUN: C.WWDG_WINDOW,
                Fault.TORN_WRITE: C.TORN_WRITE_DELAY, Fault.COLD_BOOT: 0.0}


def _qmul(a, b):
    return np.array([a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
                     a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
                     a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
                     a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]])


def _down(q):
    """World 'down' expressed in the body frame for attitude q."""
    w, x, y, z = q
    return np.array([2*(x*z - w*y), 2*(y*z + w*x), w*w - x*x - y*y + z*z])


class MockMCU:
    def __init__(self):
        self.parser, self.tx, self.latest, self.fresh = FrameParser(), bytearray(), None, False
        self.state, self.path, self.cold = RUN, BootPath.NONE, False
        self.t_now = self.t_until = 0.0
        # Pre-flight: calibrated on the ground, already flying.
        self.q, self.bias, self.integ = _I4.copy(), C.GYRO_BIAS.copy(), np.zeros(3)
        # Retained SRAM1: two checkpoint slots. Each is a dict, or None if its CRC would fail.
        self.slots, self.next_slot = [None, None], 0
        self.cal_sum, self.cal_n = np.zeros(3), 0

    # --- Device surface (same as SerialDevice) ------------------------------------
    def write(self, data):
        for f in self.parser.feed(data):
            if self.state in (HUNG, BOOT):
                continue                                # UART not listening: frame lost
            if isinstance(f, SensorFrame):
                self.latest, self.fresh = f, True
            elif isinstance(f, CommandFrame) and self.state == RUN:
                self._inject(f)

    def read(self):
        out, self.tx = bytes(self.tx), bytearray()
        return out

    def tick(self, t):
        self.t_now = t
        if self.state == HUNG and t >= self.t_until:
            self.state, self.t_until = BOOT, t + (C.BOOT_COLD if self.cold else C.BOOT_WARM)
            self.fresh = False                          # anything buffered before the reset is gone
        elif self.state == BOOT and t >= self.t_until:
            self.integ = np.zeros(3) if not C.PERSIST_INTEGRATOR else self.integ
            if self.cold or self._newest_checkpoint() is None:
                self._start_cold(t)
            else:
                self.state = DECIDE                     # warm: need a fresh sample before deciding
        elif self.state == ALIGN and t >= self.t_until:
            if self.path == BootPath.COLD:
                self.bias = self.cal_sum / max(self.cal_n, 1)          # calibration result
                self.q = self._seed(self.latest.accel, self.latest.mag)
            self.state = RUN

        if self.fresh and self.state in (DECIDE, ALIGN, RUN):
            self.fresh = False
            if self.state == DECIDE:
                self._warm_decide(self.latest, t)
            if self.state in (ALIGN, RUN):
                self._iterate(self.latest, flying=self.state == RUN)

    # --- fault + boot -------------------------------------------------------------
    def _inject(self, cmd):
        self.cold = cmd.fault == Fault.COLD_BOOT
        self.state, self.t_until = HUNG, self.t_now + _RESET_DELAY[cmd.fault] + cmd.extra_down_ms * 1e-3
        if cmd.fault == Fault.TORN_WRITE:
            self.slots[self.next_slot] = None          # reset lands mid-write: this slot's CRC fails

    def _newest_checkpoint(self):
        valid = [s for s in self.slots if s is not None]
        return max(valid, key=lambda s: s["t_us"]) if valid else None

    def _start_cold(self, t):
        self.path, self.q, self.bias = BootPath.COLD, _I4.copy(), np.zeros(3)
        self.slots, self.cal_sum, self.cal_n = [None, None], np.zeros(3), 0
        self.state, self.t_until = ALIGN, t + C.GYRO_CAL

    def _warm_decide(self, s, t):
        """Recovery rule: accept, correct (re-init), or reject the saved state."""
        ck = self._newest_checkpoint()
        if np.any(np.abs(ck["bias"]) > C.BIAS_LIMIT):
            return self._start_cold(t)                 # bias failed its own check
        age = (s.t_us - ck["t_us"]) * 1e-6
        meas_down = -s.accel / (np.linalg.norm(s.accel) + 1e-9)
        tilt_err = np.degrees(np.arccos(np.clip(meas_down @ _down(ck["q"]), -1, 1)))
        self.bias = ck["bias"].copy()
        if age <= C.CHECKPOINT_MAX_AGE and tilt_err <= C.ATTITUDE_REJECT_DEG:
            self.path, self.q, self.state = BootPath.WARM_FULL, ck["q"].copy(), RUN
            if C.PERSIST_INTEGRATOR:
                self.integ = ck["integ"].copy()
        else:
            self.path, self.q = BootPath.WARM_PARTIAL, self._seed(s.accel, s.mag)
            self.state, self.t_until = ALIGN, t + C.REALIGN

    # --- one control iteration ----------------------------------------------------
    def _iterate(self, s, flying):
        if self.state == ALIGN and self.path == BootPath.COLD:
            self.cal_sum, self.cal_n = self.cal_sum + s.gyro, self.cal_n + 1
        self._mahony(s.gyro, s.accel, C.REALIGN_KP if self.state == ALIGN else C.MAHONY_KP)

        motors = np.full(4, C.HOVER_THROTTLE)
        if flying:
            motors = self._pid_mix(s.gyro - self.bias)
            # Late in the loop body: commit a checkpoint to the alternate slot.
            self.slots[self.next_slot] = {"q": self.q.copy(), "bias": self.bias.copy(),
                                          "integ": self.integ.copy(), "t_us": s.t_us}
            self.next_slot ^= 1
        self.tx += encode(ActuatorFrame(s.seq, motors, self.q, self.path, flying))

    # --- estimator ----------------------------------------------------------------
    @staticmethod
    def _seed(accel, mag):
        """Attitude from gravity (roll, pitch) and tilt-compensated magnetometer (yaw). ZYX Euler -> quat."""
        d = -accel / (np.linalg.norm(accel) + 1e-9)                    # "down" in body frame
        r, p = np.arctan2(d[1], d[2]), np.arctan2(-d[0], np.hypot(d[1], d[2]))
        mx = mag[0]*np.cos(p) + mag[1]*np.sin(r)*np.sin(p) + mag[2]*np.cos(r)*np.sin(p)
        my = mag[1]*np.cos(r) - mag[2]*np.sin(r)
        y = np.arctan2(-my, mx)                                        # field points north: no declination
        cr, sr, cp, sp, cy, sy = (f(a / 2) for a in (r, p, y) for f in (np.cos, np.sin))
        return np.array([cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                         cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy])

    def _mahony(self, gyro, accel, kp):
        """Mahony complementary filter. Error = measured-down x estimated-down (body frame)."""
        n = np.linalg.norm(accel)
        e = np.cross(-accel / n, _down(self.q)) if n > 1e-3 else np.zeros(3)
        self.bias -= C.MAHONY_KI * e * C.DT                            # learns residual gyro bias
        omega = gyro - self.bias + kp * e
        self.q = self.q + 0.5 * _qmul(self.q, np.append(0.0, omega)) * C.DT
        self.q /= np.linalg.norm(self.q)

    # --- controller ---------------------------------------------------------------
    def _pid_mix(self, rate):
        """Angle P -> rate PI -> torque -> motor mixer. Target: level, yaw 0, hover throttle."""
        err = -2.0 * np.sign(self.q[0] or 1.0) * self.q[1:]            # body-frame rotation to level
        e_r = C.ANGLE_KP * err - rate
        self.integ = np.clip(self.integ + e_r * C.DT, -C.INT_LIMIT, C.INT_LIMIT)
        l, m, n = np.diag(C.J) * (C.RATE_KP * e_r + C.RATE_KI * self.integ)   # torques [N m]
        thrust = (C.MASS * C.G / 4
                  + (-C.MOTOR_Y * l + C.MOTOR_X * m) / (4 * C.ARM ** 2)
                  + C.MOTOR_SPIN * n / (4 * C.KQ))
        return np.clip(thrust / C.T_MAX, 0.0, 1.0)
